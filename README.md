fingervision
==================
Data processing programs for the vision-based tactile sensor FingerVision.
Two versions are provided: standalone, and ROS packages.  The standalone version would be good for making your own system, and the ROS version is good for a quick integration.

CAD models are also provided which are useful to reproduce FingerVision.

Refer to the documentation of this project:

http://akihikoy.net/p/fv.html

where you can find instructions of software, manufacturing, and integration.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Directory structure
==========================

fv_core/
-----------------------
Core programs of processing FingerVision data, that are shared among the standalone programs and the ROS package.

standalone/
-----------------------
Standalone programs of processing FingerVision data.

fingervision/
-----------------------
The main ROS package to process data of FingerVision.

fingervision_msgs/
-----------------------
The messages and services used in fingervision.

3rdparty/
-----------------------
Utility programs that are copied from other projects.

cad/
-----------------------
CAD models of FingerVision.

tools/
-----------------------
Useful tools, such as configuring a USB camera device, and a helper of mjpg-streamer.

