#!/bin/bash
#\file    sync.sh
#\brief   Sync files from original (private use).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.06, 2017

base=~/ros_ws/lfd_vision
files=(
Makefile
# blob_usbcam2fay12_l.yaml
# blob_usbcam2fay12_r.yaml
# blob_usbcam2fay22_l.yaml
# blob_usbcam2fay22_r.yaml
# config/usbcam2fay12.yaml
# config/usbcam2fay22.yaml
include/lfd_vision/blob_tracker2.h
include/lfd_vision/geom_util.h
include/lfd_vision/pcl_util.h
include/lfd_vision/prox_vision.h
include/lfd_vision/usb_stereo.h
include/lfd_vision/vision_util.h
launch/usb_stereo_calib2fay11.sh
launch/visual_skin_2fay12.launch
launch/visual_skin_2fay22.launch
mainpage.dox
# manifest.xml
msg/BlobMove.msg
msg/BlobMoves.msg
msg/Int32Array.msg
msg/ProxVision.msg
scripts/cameracalibrator.py
src/blob_tracker2.cpp
src/cv_usb_node.cpp
src/disp_rostime.cpp
src/pcl_util.cpp
src/prox_vision.cpp
src/usb_stereo.cpp
src/vision_util.cpp
src/visual_skin_node2.cpp
srv/SetFrameRate.srv
srv/SetInt32.srv
)
files2=""
for ((i=0; i<$((${#files[@]})); i++)) ; do
  files2="$files2 ${base}/./${files[i]}"
done
# echo $files2
rsync -azv -R -L ${files2} lfd_vision/

base=~/prg/testl/cv
files=(
cv2-videoout2.h
rotate90n.h
cap_open.h
simple_blob_tracker4.cpp
obj_det_track3.cpp
capture.cpp
)
files2=""
for ((i=0; i<$((${#files[@]})); i++)) ; do
  files2="$files2 ${base}/./${files[i]}"
done
# echo $files2
rsync -azv -R -L ${files2} standalone/

base=~/things
#Use `git ls-files` to list files.
files=(
bxgr_optskin01/bxgr_optskin02a-v3.1.fcstd
bxgr_optskin01/bxgr_optskin02a-v3.1.step
bxgr_optskin01/bxgr_optskin02a-v3.1.stl
bxgr_optskin01/bxgr_optskin02b-v3.1.fcstd
bxgr_optskin01/bxgr_optskin02b-v3.1.stl
bxgr_optskin01/bxgr_optskin02b-v3.fcstd
bxgr_optskin01/bxgr_optskin02b-v3.stl
bxgr_optskin01/bxgr_optskin02c-v3.1.fcstd
bxgr_optskin01/bxgr_optskin02c-v3.1.stl
optskin_mold01/bxgr_optskin02_mold_a-v3.1-p1.stp
optskin_mold01/bxgr_optskin02_mold_a-v3.1.fcstd
optskin_mold01/bxgr_optskin02_mold_a-v3.1.stl
optskin_mold01/bxgr_optskin02_mold_a-v3.1.stp
optskin_mold01/bxgr_optskin02_mold_b.fcstd
optskin_mold01/bxgr_optskin02_mold_b.stl
optskin_mold01/bxgr_optskin02_mold_c-v3.fcstd
optskin_mold01/bxgr_optskin02_mold_c-v3.stl
optskin_mold01/bxgr_optskin02_mold_d-v3.fcstd
optskin_mold01/bxgr_optskin02_mold_d-v3.stl
optskin_mold01/rq_optskin01_mold_a.fcstd
optskin_mold01/rq_optskin01_mold_a.stl
optskin_mold01/rq_optskin01_mold_b.fcstd
optskin_mold01/rq_optskin01_mold_b.stl
rq_optskin01/rq_optskin01a.fcstd
rq_optskin01/rq_optskin01a.stl
)
files2=""
for ((i=0; i<$((${#files[@]})); i++)) ; do
  files2="$files2 ${base}/./${files[i]}"
done
# echo $files2
rsync -azv -R -L ${files2} cad/
