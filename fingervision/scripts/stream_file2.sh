#!/bin/bash
# Streaming demo video files with MJPG-streamer.
# They can be accessed from
#   http://localhost:8080/
#   http://localhost:8081/
options="-f 60"
files=(  \
$(rospack find fingervision)/../data/fv_demo/cam0_0002.m4v   \
$(rospack find fingervision)/../data/fv_demo/cam1_0002.m4v   \
)
source $(rospack find fingervision)/../tools/mjpg_stream_file.sh
