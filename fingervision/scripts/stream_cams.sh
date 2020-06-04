#!/bin/bash
# Streaming specified video devices with MJPG-streamer.
# $ rosrun fingervision stream_cams.sh [-o OPTIONS] -d DEV1 [-d DEV2 [...]]
#   OPTIONS: Options of MJPG-streamer.
#   DEV?: Camera device ID.
# e.g.  $ rosrun fingervision stream_cams.sh -d 1 -d 2 -o "-f -1 -r 640x480 -n"
# They can be accessed from
#   http://localhost:8080/
#   http://localhost:8081/
#   ...
uvc_opt="-f -1 -r 320x240 -n"
devs=()
while true; do
  case "$1" in
    -o) uvc_opt=$2; shift 2 ;;
    -d) devs+=($2); shift 2 ;;
    ''|--) shift; break ;;
    *)  shift 1 ;;
  esac
done
echo "uvc_opt: $uvc_opt"
echo "devs: ${devs[@]}"
source $(rospack find fingervision)/../tools/mjpg_stream.sh
