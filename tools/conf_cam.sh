#!/bin/bash
#--------------------------------------------------
dev=''
exposure=123
wb=3500
#--------------------------------------------------

usage="`basename $0` [OPTIONS] DEV
Configure camera parameters of device DEV.
  OPTIONS:
    [-e INT]   : set exposure (default: $exposure)
    [-wb INT]  : set white balance temperature (default: $wb)
    [-help]    : show help
  Example:  `basename $0` /dev/video0
"
#--------------------------------------------------

while true; do
  case "$1" in
    -help|--help) echo "usage: $usage"; exit 0 ;;
    -e) exposure=$2; shift 2 ;;
    -wb) wb=$2; shift 2 ;;
    -*) echo "invalid option: $1"; echo ""; echo "usage: $usage"; exit 0 ;;
    '') break ;;
    *)
      if [ "$dev" == "" ];then dev="$1"; shift 1
      else echo "invalid option: $1"; echo ""; echo "usage: $usage"; exit 1
      fi
      ;;
  esac
done

if ! [ -e "$dev" ];then
  echo "Device not found or specified: $dev"
  echo ""
  echo "usage: $usage"
  exit 1
fi
#--------------------------------------------------

uvcdynctrl -d $dev -s 'Exposure, Auto' 1
uvcdynctrl -d $dev -s 'Exposure (Absolute)' $exposure
uvcdynctrl -d $dev -s 'White Balance Temperature, Auto' 0
uvcdynctrl -d $dev -s 'White Balance Temperature' $wb
