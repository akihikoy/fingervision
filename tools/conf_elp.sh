#!/bin/bash
exposure=112
wb=3850
./conf_cam.sh -e $exposure -wb $wb /dev/video0
./conf_cam.sh -e $exposure -wb $wb /dev/video1
