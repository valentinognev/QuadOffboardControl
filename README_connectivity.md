# Raspbery pi connectivity fork
This fork is made for connecting a RP CM4 with hailo to a computer running the Gazebo simulation and streaming a video output from the simulation to the RP in a way that allows hailo-tiling to 
use the stream as input.

## Files changed:
- runSimNoetic.sh
- run_px4_sitl_docker.sh
- added model_zmq_bridge.cc
- added zmq_rec_rtsp_trans.py

## Initial Installation

#### For run_px4_sitl_docker.sh
- Must copy the directory in CONT_PX4_PATH='/home/valentin/PX4-Autopilot' to HOST_PX4_PATH before running runSimNoetic.sh, otherwise you will not be able to access any files you need.
- The addition of another volume allows us to edit /home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf. In iris.sdf, inside the 'base_link'
link tag, add the following sensor:

''' xml
      <sensor name="front_camera" type="camera">
        <gz_frame_id>base_link</gz_frame_id>
        <pose>0.5 0 0 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10000</far>
          </clip>
        </camera>
        <!-- This plugin is required for image publishing -->
        <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
          <ros>
            <remapping>image_raw:=/iris/front_camera/image</remapping>
          </ros>
          <frame_name>base_link</frame_name>
        </plugin>
        <topic>/iris/front_camera/image</topic>
      </sensor>
'''

### For model_zmq_bridge.cc
On your host (the one you will use to run the gazebo sim), run:
''' bash
cd <host path that '/home/valentin/PX4-Autopilot' was copied to>/Tools/simulation/gazebo-classic/sitl_gazebo-classic
mkdir -p build && cd build
cmake ..
make gazebo_zmq_bridge
'''

Relevant to transmitter and reciever:
### Configure wired connection
- On your host (the one you will use to run the gazebo sim), change your network settings and add to your ethernet configuration: ipv4 with IP 192.168.1.1 and subnet 255.255.255.0.
- On your Raspberry pi make a similar addition by running:
''' bash
interface eth0
static ip_address=192.168.1.2/24
static routers=192.168.1.1
'''
or by editing your connections through your internet settings.
- **Note:** the wifi connection will not work while the ethernet cable is connected.

### On the Raspberry pi
- Save zmq_rec_rtsp_trans.py
- Install mediamtx:
''' bash
mkdir mediamtx && cd mediamtx
wget https://github.com/bluenviron/mediamtx/releases/latest/download/mediamtx_v1.19.0_linux_arm64.tar.gz
tar -xvfz mediamtx_v1.19.0_linux_arm64.tar.gz
'''

## Running (after installation)
- Launch the simulation:
''' bash
bash runSimNoetic.sh
'''
- On the raspberry pi, launch mediamtx:
'''bash
cd mediamtx
./mediamtx
'''
- Still on the raspberry pi, start the listener:
'''bash
zmq_rec_rtsp_trans.py
'''
- Back on the device running the simulation connect to the container on another terminal and run, inside the container:
''' bash
./build/px4_sitl_default/build_gazebo-classic/gazebo_zmq_bridge /gazebo/default/iris/base_link/front_camera/image tcp://192.168.1.1:5555
'''
- After establishing all connections, on your Raspberry pi, run:
'''bash
hailo-tiling -i rtsp://127.0.0.1:8554/live
'''
