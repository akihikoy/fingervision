fv_demo
==========================
Demonstration videos of FingerVision sensors.

The videos in this directory are pairs of videos captured from two FingerVision sensors installed on a UR3 robot.

Robot Setup
==========================
- Universal Robots UR3 (6-joints robot arm).
- Dynamixel Gripper.
- Two FingerVision sensors on the fingers of the gripper.


Video Details
==========================

0001
-----------------------
Video files: cam0_0001.m4v, cam1_0001.m4v

cam0 denotes the camera on FingerVision of the left finger.
Cf. fingervision/config/fv_2_l.yaml for the configuration.

cam1 denotes the camera on FingerVision of the right finger.
Cf. fingervision/config/fv_2_r.yaml for the configuration.

Actual scene where the videos were recorded is available on:
https://youtu.be/TjK7gPZTqKE


0002
-----------------------
Video files: cam0_0002.m4v, cam1_0002.m4v

cam0 and cam1 denote the same as 0001.

Actual scene where the videos were recorded is available on:
https://youtu.be/EtCW8IDT_kM


0003
-----------------------
Video files: cam0_0003.m4v, cam1_0003.m4v

cam0 and cam1 denote the same as 0001.

Actual scene where the videos were recorded is available on:
https://youtu.be/uMXk2YXwrxg


How to Use the Videos?
==========================

Standalone
-----------------------
In the YAML file of camera configuration, we can use a video path as DevID as well as the video stream URL and a video device ID.
For example:

    DevID: "../data/fv_demo/cam0_0001.m4v"

Standalone programs can work with these configs.

    $ cd standalone/
    $ ./capture_mult.out file1.yaml
    $ ./blob_tracker2_test.out file1_r.yaml
    $ ./prox_vision_test.out file1_l.yaml

Note: Currently blob_tracker2_test and prox_vision_test cannot handle multiple cameras.
They cannot adjust FPS, so the FPS would be like 200 Hz.


Streaming Videos
-----------------------
Using MJPG Streamer, we can stream the videos from video files over the Ethernet network.

We need to install the following fork of MJPG Streamer:
https://github.com/akihikoy/mjpg-streamer

Installation:

    $ sudo apt-get install git cmake libjpeg8-dev uvcdynctrl v4l-utils
    $ mkdir ~/prg && cd ~/prg/
    $ git clone https://github.com/akihikoy/mjpg-streamer.git mjpg-streamer2
    $ cd mjpg-streamer2/mjpg-streamer-experimental
    $ make

Example use:

    Go to the fingervision directory.
    $ cd tools/
    $ ./stream_file1.sh

The videos 0001 are streamed over Ethernet.

cam0_0001.m4v is streamed on http://localhost:8080/ and
cam1_0001.m4v is streamed on http://localhost:8081/

MJPG Streamer provides an UI.
Go to the Stream or Javascript pages to see the videos.

For quitting the above program, just press the Enter key.

stream_file2.sh and stream_file3.sh are also provided for videos 0002 and 0003.

The equivalent scripts for ROS are stored in the fingervision package.
They can be executed by:

    $ rosrun fingervision stream_file1.sh

This can be executed anywhere.

stream_file2.sh and stream_file3.sh are also provided for videos 0002 and 0003.


Testing ROS Packages
-----------------------
For example, we can try:

    $ roslaunch fingervision fv_3.launch

fv_3.launch uses config/fv_3_l.yaml and config/fv_3_r.yaml as the configuration files where you will find that the DevID parameters are set as:

    DevID: "http://localhost:8080/?action=stream&dummy=file.mjpg"

Note that `dummy=file.mjpg` is not necessary for streaming the video, but it is necessary for OpenCV to recognize the given URL is a video stream.


