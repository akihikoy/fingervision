#!/bin/bash
#\file    ln_cams.sh
#\brief   Find camera devices with a device type name and create symbolic links.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.17, 2024
#--------------------------------------------------
TARGET_DEVICE_NAME="VGA Camera: VGA Camera"
FVDEV_PREFIX="/media/video_fv"
FVDEV_NUM=2
FVDEV_IDX_START=1
#--------------------------------------------------

usage="`basename $0` [OPTIONS]
Find camera devices with a device type name and create symbolic links.
  OPTIONS:
    [-devn STR]   : set TARGET_DEVICE_NAME (default: $TARGET_DEVICE_NAME)
    [-pre STR]  : set FVDEV_PREFIX (default: $FVDEV_PREFIX)
    [-num INT]  : set FVDEV_NUM (default: $FVDEV_NUM)
    [-ist INT]  : set FVDEV_IDX_START (default: $FVDEV_IDX_START)
    [-help]    : show help
"
#--------------------------------------------------

while true; do
  case "$1" in
    -help|--help) echo "usage: $usage"; exit 0 ;;
    -devn) TARGET_DEVICE_NAME=$2; shift 2 ;;
    -pre) FVDEV_PREFIX=$2; shift 2 ;;
    -num) FVDEV_NUM=$2; shift 2 ;;
    -ist) FVDEV_IDX_START=$2; shift 2 ;;
    -*) echo "invalid option: $1"; echo ""; echo "usage: $usage"; exit 1 ;;
    '') break ;;
    *) echo "invalid option: $1"; echo ""; echo "usage: $usage"; exit 1 ;;
  esac
done
#--------------------------------------------------

echo "`basename $0` configuration:
  TARGET_DEVICE_NAME=$TARGET_DEVICE_NAME
  FVDEV_PREFIX=$FVDEV_PREFIX
  FVDEV_NUM=$FVDEV_NUM
  FVDEV_IDX_START=$FVDEV_IDX_START
"

# List all devices and their paths
DEVICES=$(v4l2-ctl --list-devices 2>&1)
DEVICE_STATUS=$?

# Initialize an empty array to hold matching device paths
MATCHING_PATHS=()

# Read through the devices output
while IFS= read -r line; do
# echo "debug,${line}"
  if [[ "$line" == *"$TARGET_DEVICE_NAME"* ]]; then
    # Get the next line for the path
    read -r path_line
    # Extract the path
    DEVICE_PATH=$(echo $path_line | grep -oP '(?<=/dev/).*')

    # Check if the device path exists in /dev/v4l/by-path/
    for by_path in /dev/v4l/by-path/*; do
      if [ "$(readlink -f $by_path)" == "/dev/$DEVICE_PATH" ]; then
        MATCHING_PATHS+=("$by_path")
      fi
    done
  fi
done <<< "$DEVICES"

# List the matched paths
echo "[$TARGET_DEVICE_NAME] Camera device found:"
for path in "${MATCHING_PATHS[@]}"; do
  echo "  $path"
done
echo ""

echo "Following symbolic links are created:"
num_devices=$(( ${#MATCHING_PATHS[@]} < $FVDEV_NUM ? ${#MATCHING_PATHS[@]} : $FVDEV_NUM ))
for (( i=0; i<num_devices; i++ )); do
  echo "  $FVDEV_PREFIX$((i+FVDEV_IDX_START)) --> ${MATCHING_PATHS[$i]}"
done
echo ""

#--------------------------------------------------

for (( i=0; i<num_devices; i++ )); do
  echo "$FVDEV_PREFIX$((i+FVDEV_IDX_START)) --> ${MATCHING_PATHS[$i]}"
  sudo ln -is ${MATCHING_PATHS[$i]} $FVDEV_PREFIX$((i+FVDEV_IDX_START))
done

echo "Finished."

