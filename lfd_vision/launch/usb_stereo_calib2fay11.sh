#!/bin/bash -x
#Use this script instead of launch/usb_stereo_calib2fay11.launch
#if it returns an error: Waiting for service .../set_camera_info - Service not found
rosrun lfd_vision bin/cv_usb_node config/usbcam2fay11.yaml &
sleep 5
#NOTE: USE SMALL CALIB BOARD
rosrun camera_calibration cameracalibrator.py  \
    --size 6x4 --square 0.0191 --approximate=0.1 -c ''  \
    left:=/cv_usb_node/usbcam2fay11_l/image_raw         \
    right:=/cv_usb_node/usbcam2fay11_r/image_raw        \
    left_camera:=/cv_usb_node/usbcam2fay11_l            \
    right_camera:=/cv_usb_node/usbcam2fay11_r
