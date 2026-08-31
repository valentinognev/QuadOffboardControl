# region imports
# Standard library imports
import os
os.environ["GST_PLUGIN_FEATURE_RANK"] = "vaapidecodebin:NONE"

# Third-party imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import cv2
import psutil
import numpy as np
import cairo
import collections
import time
import serial
import threading
import queue
import json
import msgpack
import subprocess
import re
import zmq


# Local application-specific imports
import hailo
from hailo_apps.python.core.common.hailo_logger import get_logger
from hailo_apps.python.core.gstreamer.gstreamer_app import app_callback_class
#from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import DISPLAY_PIPELINE #can you import variables? #not ones from inside a function
#from hailo_apps.python.core.common.defines import GST_VIDEO_SINK
from hailo_apps.python.pipeline_apps.tiling.tiling_pipeline import GStreamerTilingApp

hailo_logger = get_logger(__name__)
# endregion imports

debug = False
debug_pred = False
debug_stat = False
cpu_interval = None #0.01 changed to none-blocking approach, hopefully works
listen_to_pose = "tcp://192.168.1.1:5556"
exact_name = 'iris'
pose_topic = "/gazebo/default/pose/info"
ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)
_serial_queue = queue.Queue(maxsize=10)
FPS_UPDATE_INTERVAL = 1.0
CPU_UPDATE_INTERVAL = 1.0

def _serial_worker():
    while True:
        message = _serial_queue.get()
        if message is None:
            break
        try:
            ser.write(message)
        except Exception as e:
            hailo_logger.warning("Serial write failed: %s", e)
        finally:
            _serial_queue.task_done()

_serial_thread = threading.Thread(target=_serial_worker, daemon=True)
_serial_thread.start()


class GazeboPoseReceiver:
    def __init__(
        self,
        endpoint = listen_to_pose,
        topic = pose_topic,
        pose_name = exact_name
    ):
        self.endpoint = endpoint
        self.topic = topic
        self.pose_name = pose_name

        self.x = None
        self.y = None
        self.z = None

        self.w = None
        self.qx = None
        self.qy = None
        self.qz = None

        self.lock = threading.Lock()

        self.thread = threading.Thread(
            target=self._receive,
            daemon=True
        )
        self.thread.start()

    def _receive(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)

        socket.connect(self.endpoint)
        socket.setsockopt_string(
            zmq.SUBSCRIBE,
            self.topic
        )

        hailo_logger.info(
            "Connected to Gazebo ZMQ at %s",
            self.endpoint
        )

        try:
            while True:
                parts = socket.recv_multipart()

                if len(parts) < 2:
                    continue

                try:
                    payload = json.loads(
                        parts[1].decode("utf-8")
                    )
                except Exception:
                    continue

                # Gazebo pose/info normally contains a list
                poses = (
                    payload.get("poses")
                    or payload.get("pose")
                    or []
                )

                # Find iris
                iris_pose = next(
                    (
                        p for p in poses
                        if isinstance(p, dict)
                        and p.get("name") == self.pose_name
                    ),
                    None
                )

                if iris_pose is None:
                    continue

                position = iris_pose.get("position", {})
                orientation = iris_pose.get("orientation", {})

                with self.lock:
                    self.x = position.get("x")
                    self.y = position.get("y")
                    self.z = position.get("z")

                    self.w = orientation.get("w")
                    self.qx = orientation.get("x")
                    self.qy = orientation.get("y")
                    self.qz = orientation.get("z")

        except Exception as e:
            hailo_logger.warning(
                "Gazebo ZMQ receiver stopped: %s",
                e
            )

        finally:
            socket.close()
            context.term()

    def get_position(self):
        """
        Returns the latest iris position as:
        (x, y, z)
        """
        with self.lock:
            return self.x, self.y, self.z

    def get_pose(self):
        """
        Returns the latest complete pose.
        """
        with self.lock:
            return {
                "position": {
                    "x": self.x,
                    "y": self.y,
                    "z": self.z
                },
                "orientation": {
                    "w": self.w,
                    "x": self.qx,
                    "y": self.qy,
                    "z": self.qz
                }
            }

# User-defined class to be used in the callback function: Inheritance from the app_callback_class
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()        
        self.use_frame = True

gazebo_pose = GazeboPoseReceiver()


def get_incrimination(detection):
    '''
    In the future, will check if a detection is a target. As of 31/03/2026 always returns true.
    '''
    return True


def serialize_predictions(detections):
    serialized = []
    for det in detections:
        serialized.append({
            "label": det.get_label(),
            "confidence": float(det.get_confidence()),
            "x_center": float((det.get_bbox().xmin() + det.get_bbox().xmax()) / 2),
            "y_center": float((det.get_bbox().ymin() + det.get_bbox().ymax()) / 2),
            "width": float(det.get_bbox().xmax() - det.get_bbox().xmin()),
            "height": float(det.get_bbox().ymax() - det.get_bbox().ymin()),
            "incrimination": get_incrimination(det),
        }) 
    return json.dumps(serialized).encode('utf-8')


def send_predictions(detections):
    payload = serialize_predictions(detections)
    # Frame the message: length-prefix + data + newline delimiter
    message = payload + b'\n'
    try:
        _serial_queue.put_nowait(message)
        if debug_pred:
            hailo_logger.info("predictions sent to queue")
        return
    except queue.Full:
        if debug_pred:
            hailo_logger.warning("Serial queue full, dropping frame predictions")
        return
    


def app_callback(element, buffer, user_data):
    # Note: Frame counting is handled automatically by the framework wrapper
    string_to_print = f"Frame count: {user_data.get_count()}\n"
    if buffer is None:
        hailo_logger.warning("Received None buffer at frame=%s", user_data.get_count())
        return
    detections = hailo.get_roi_from_buffer(buffer).get_objects_typed(hailo.HAILO_DETECTION)
    for detection in detections:
        string_to_print += (f"Detection: {detection.get_label()} Confidence: {detection.get_confidence():.2f}\n")
    if debug:
        print(string_to_print)
    if user_data.get_count() % 1 == 0:
        send_predictions(detections)
    return


#I think shutdown_serial() is never used
def shutdown_serial():
    _serial_queue.put(None)
    _serial_thread.join()


_overlay_state = {
    "last_update_ts": time.monotonic(),
    "frame_count": 0,
    "displayed_fps": 0.0,
    "displayed_cpu": 0.0,
}

# --- from here: addition that fetches the overlay data in the background rather than blocking the code ---
_system_state = {"hailo_text": "Hailo: Fetching..."}

def _poll_cpu_stats():
    """Polls psutil in the background every CPU_UPDATE_INTERVAL seconds."""
    # First call initializes the psutil baseline
    psutil.cpu_percent(interval=None)
    while True:
        # psutil with interval blocks ONLY this background thread, returning
        # the exact average CPU utilization over that timeframe.
        usage = psutil.cpu_percent(interval=CPU_UPDATE_INTERVAL)
        _overlay_state["displayed_cpu"] = usage

_cpu_thread = threading.Thread(target=_poll_cpu_stats, daemon=True)
_cpu_thread.start()

def _poll_hailo_stats():
    """Continuously reads hailortcli monitor locally and extracts table data."""
    # Give the pipeline a few seconds to start and lock the Hailo device
    time.sleep(3)
    
    # Regex to clean up terminal UI formatting codes
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    
    while True:
        try:
            process = subprocess.Popen(
                "stdbuf -o0 hailortcli monitor | tr '\\r' '\\n'", 
                shell=True,
                stdout=subprocess.PIPE, 
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1 
            )
            
            state = "SEEK_HEADER"
            
            for line in iter(process.stdout.readline, ''):
                if not line:
                    break
                    
                clean_line = ansi_escape.sub('', line).strip()
                
                # Skip empty lines
                if not clean_line:
                    continue
                
                # 1. Look for the Device table header
                if "Device ID" in clean_line and "Utilization" in clean_line:
                    state = "FOUND_HEADER"
                    continue
                
                # 2. Catch the dashed separator line
                if state == "FOUND_HEADER":
                    if clean_line.startswith("-"):
                        state = "EXPECT_DATA"
                    continue
                
                # 3. Parse the data row
                if state == "EXPECT_DATA":
                    tokens = clean_line.split()
                    # tokens[0] = Device ID (e.g. '0001:01:00.0')
                    # tokens[1] = Utilization (e.g. '91.2')
                    # tokens[2] = Architecture (e.g. 'HAILO8')
                    if len(tokens) >= 2:
                        util_percent = tokens[1]
                        _system_state["hailo_text"] = f"Hailo: {util_percent}%"
                    
                    # Reset state to seek the next monitor refresh cycle
                    state = "SEEK_HEADER"
                            
            process.wait()
            
        except Exception as e:
            _system_state["hailo_text"] = "Hailo: Monitor Error"
            
        time.sleep(2.0)
# Start the background thread once
_sys_thread = threading.Thread(target=_poll_hailo_stats, daemon=True)
_sys_thread.start()


def on_draw(overlay, cr, timestamp, duration, user_data):
    # --- FPS calculation (in-memory only)
    now = time.monotonic()
    _overlay_state["frame_count"] += 1
    elapsed = now - _overlay_state["last_update_ts"]

    if elapsed >= FPS_UPDATE_INTERVAL:
        _overlay_state["displayed_fps"] = _overlay_state["frame_count"] / elapsed
        _overlay_state["last_update_ts"] = now
        _overlay_state["frame_count"] = 0

    # --- Draw FPS
    cr.set_source_rgb(1, 1, 1)
    cr.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
    cr.set_font_size(20)
    cr.move_to(10, 25)
    cr.show_text(f"FPS: {_overlay_state['displayed_fps']:.1f}")

    # --- Draw CPU (instant memory read from background thread)
    cr.set_source_rgb(1, 1, 1)
    cr.move_to(10, 50)
    cr.show_text(f"CPU: {_overlay_state['displayed_cpu']}%")
    
    # --- Draw Hailo Utilization
    cr.set_source_rgb(1, 1, 1)
    cr.move_to(10, 75)          
    cr.show_text(_system_state["hailo_text"])
    
    # --- Draw number of tiles
    cr.set_source_rgb(1, 1, 1)
    cr.move_to(10, 100)
    cr.show_text(f"Tiles: {getattr(user_data, 'num_of_tiles', 'N/A')}")
    
    x, y, z = gazebo_pose.get_position()
    # --- Draw position
    cr.set_source_rgb(1, 1, 1)
    cr.move_to(10, 125)
    cr.show_text(f"Location: (x = {x}, y = {y}, z = {z})")
    

def main():
    """Main function for CLI entry point."""
    os.environ["HAILO_MONITOR"] = "1" #this needs to appear before gstreamer initializes
    #also this doesn't really work on its own and you gotta run "export HAILO_MONITOR=1" on the terminal 
    hailo_logger.info("Starting Tiling App.")
    user_data = user_app_callback_class()
    app = GStreamerTilingApp(app_callback, user_data)
    #---
    num_of_tiles = app.batch_size
    user_data.num_of_tiles = num_of_tiles
    #---

    
    #addition for presenting cpu usage and fps usage   
    cpu_overlay = app.pipeline.get_by_name("cpu_overlay")
    if cpu_overlay:
        cpu_overlay.connect("draw", on_draw, user_data)
    else:
        hailo_logger.warning("cpu_overlay element not found in pipeline")
    #end of addition
    
    app.run()


if __name__ == "__main__":
    main()
