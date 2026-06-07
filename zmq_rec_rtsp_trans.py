import argparse
import base64
import json
import shutil
import subprocess
import sys
import cv2
import numpy as np
import zmq

CAMERA_TOPIC = "/gazebo/default/iris/base_link/front_camera/image"
IP_TO_LISTEN_TO="192.168.1.1"
DESTINATION_IP="rtsp://127.0.0.1:8554/live"
DEFAULT_FPS=30
#"/iris/front_camera/image"
WINDOW_NAME = "Gazebo Camera Feed"

class ZmqWrapper:
	def __init__(self, ip=IP_TO_LISTEN_TO, port="5555"):
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.SUB)
		self.socket.connect(f"tcp://{ip}:{port}")
		self.socket.setsockopt(zmq.SUBSCRIBE, CAMERA_TOPIC.encode())

	def listen(self):
		try:
			print("in listen, trying to recive multipart")
			topic, data_json = self.socket.recv_multipart()
			print(f"Received topic: {topic}")
			topic = topic.decode()
			data = json.loads(data_json.decode())
			return topic, data
		except Exception as e:
			print(f"Error receiving/parsing: {e}")
			return None, None


class RtspPublisher:
	def __init__(self, rtsp_url=DESTINATION_IP, fps=DEFAULT_FPS):
		self.rtsp_url = rtsp_url
		self.fps = fps
		self.process = None
		self.width = None
		self.height = None
		self.ffmpeg_path = shutil.which("ffmpeg")
		if self.ffmpeg_path is None:
			raise RuntimeError("ffmpeg is required for RTSP publishing but was not found in PATH")

	def start(self, width, height):
		if self.process is not None:
			return

		self.width = width
		self.height = height
		cmd = [
			self.ffmpeg_path,
			"-hide_banner",
			"-loglevel",
			"warning",
			"-f",
			"rawvideo",
			"-pix_fmt",
			"bgr24",
			"-s",
			f"{self.width}x{self.height}",
			"-r",
			str(self.fps),
			"-i",
			"-",
			"-an",
			"-c:v",
			"libx264",
			"-preset",
			"veryfast",
			"-tune",
			"zerolatency",
			"-pix_fmt",
			"yuv420p",
			"-f",
			"rtsp",
			"-rtsp_transport",
			"tcp", # REMOVED: "-rtsp_flags", "listen"
			DESTINATION_IP, #self.rtsp_url,
		]

		print(f"Starting RTSP publisher on {self.rtsp_url}")
		self.process = subprocess.Popen(
			cmd,
			stdin=subprocess.PIPE,
			stdout=subprocess.DEVNULL,
			stderr=subprocess.DEVNULL,
		)

	def publish(self, frame):
		if frame is None:
			return

		if frame.ndim == 2:
			frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
		elif frame.shape[2] == 4:
			frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

		if self.process is None:
			self.start(frame.shape[1], frame.shape[0])

		try:
			self.process.stdin.write(frame.tobytes())
		except BrokenPipeError:
			print("FFmpeg pipe closed unexpectedly.")
			self.stop()
		except Exception as e:
			print(f"Error writing frame to FFmpeg: {e}")

	def stop(self):
		if self.process is None:
			return

		try:
			if self.process.stdin:
				self.process.stdin.close()
			self.process.terminate()
			self.process.wait(timeout=5)
		except Exception:
			self.process.kill()
		finally:
			self.process = None


def decode_image_payload(payload):
	image_b64 = payload.get("image_data")
	if image_b64 is None:
		raise ValueError("image_data missing from payload")

	image_bytes = base64.b64decode(image_b64)
	width = int(payload.get("width", 0))
	height = int(payload.get("height", 0))
	pixel_format = payload.get("pixel_format", "")
	if isinstance(pixel_format, int):
		pixel_format = str(pixel_format)
	else:
		pixel_format = str(pixel_format).upper()

	step = int(payload.get("step", 0))

	if width <= 0 or height <= 0:
		raise ValueError("Invalid image dimensions")

	if step <= 0:
		step = width

	if step == width:
		channels = 1
	elif step == width * 3:
		channels = 3
	elif step == width * 4:
		channels = 4
	else:
		if len(image_bytes) >= width * height * 3:
			channels = 3
		else:
			channels = 1

	expected_size = height * step
	if len(image_bytes) < expected_size:
		raise ValueError(f"Decoded image buffer is too small: {len(image_bytes)} < {expected_size}")

	image = np.frombuffer(image_bytes, dtype=np.uint8)
	if channels == 1:
		image = image.reshape((height, width))
	else:
		image = image.reshape((height, step))
		if step != width * channels:
			image = image[:, : width * channels]
		image = image.reshape((height, width, channels))

	if "RGB" in pixel_format and channels == 3:
		image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

	return image


def parse_args():
	parser = argparse.ArgumentParser(description="Listen to ZMQ Gazebo video and publish via RTSP")
	parser.add_argument("--zmq-ip", default=IP_TO_LISTEN_TO, help="ZMQ publisher IP")
	parser.add_argument("--zmq-port", default="5555", help="ZMQ publisher port")
	parser.add_argument("--rtsp-url", default=DESTINATION_IP, help="RTSP URL to publish")
	parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="RTSP stream framerate")
	parser.add_argument("--show-window", action="store_true", help="Show local OpenCV preview window")
	return parser.parse_args()


def main():
	args = parse_args()
	wrapper = ZmqWrapper(ip=args.zmq_ip, port=args.zmq_port)
	publisher = RtspPublisher(rtsp_url=args.rtsp_url, fps=args.fps)

	try:
		cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
	except Exception as e:
		print(f"exception when trying to skow window: {e}")

	print("Listening to Gazebo topics via ZMQ and publishing video to RTSP.")
	print(f"RTSP stream will be available at: {args.rtsp_url}")

	try:
		print("trying")
		while True:
			print("entered while loop in main")
			topic, msg = wrapper.listen()
			if topic is None or msg is None:
				print("topic is none or msg is none")
				continue

			try:
				print("in try block")
				frame = decode_image_payload(msg)
				print("managed to run decode_image_payload")
				publisher.publish(frame)
				print("managed to run publisher.publish")
				#if args.show_window:
				cv2.imshow(WINDOW_NAME, frame)
				if cv2.waitKey(1) & 0xFF == ord("q"):
					break
				#else:
				#	print("args.show_window in while is false")
			except Exception as e:
				print(f"Error decoding/publishing frame: {e}")

	except KeyboardInterrupt:
		pass
	finally:
		publisher.stop()
		if args.show_window:
			cv2.destroyAllWindows()


if __name__ == "__main__":
	main()


