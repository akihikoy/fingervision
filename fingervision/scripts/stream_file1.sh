#!/bin/bash
options="-f 60"
files=(  \
$(rospack find fingervision)/../data/fv_demo/cam0_0001.m4v   \
$(rospack find fingervision)/../data/fv_demo/cam1_0001.m4v   \
)
source $(rospack find fingervision)/../tools/mjpg_stream_file.sh
