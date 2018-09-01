#!/bin/bash
#--------------------------------------------------
basedir=~/
user=$(whoami)
#--------------------------------------------------

usage="`basename $0` [OPTIONS]
Setting up a Raspberry Pi to work with FingerVision.
We assume:
- Raspbian or Ubuntu Mate is installed.
OPTIONS:
  [-b STR]   : Set the base directory (default: $basedir)
  [-u USER]  : Set the user (default: $user)
  [-help]    : Show help
Example (on Raspberry Pi):
  `basename $0`
Alternatively (on Raspberry Pi):
  bash <(curl -s https://raw.githubusercontent.com/akihikoy/fingervision/master/tools/setup_raspi.sh )
"
#--------------------------------------------------

while true; do
  case "$1" in
    -help|--help) echo "Usage: $usage"; exit 0 ;;
    -b) basedir=$2; shift 2 ;;
    -u) user=$2; shift 2 ;;
    -*) echo "Invalid option: $1"; echo ""; echo "Usage: $usage"; exit 1 ;;
    '') break ;;
    *) echo "Invalid option: $1"; echo ""; echo "Usage: $usage"; exit 1 ;;
  esac
done
#--------------------------------------------------

echo "basedir= $basedir"
echo "user= $user"
echo "#-----------------------"

# Install common packages.
echo "Installing packages..."
sudo apt-get install git cmake libjpeg8-dev uvcdynctrl v4l-utils htop dstat

# Gain the permissions of the user.
if [ -z "$(sudo grep $user /etc/sudoers)" ];then
  echo "Adding $user to /etc/sudoers..."
  echo "$user ALL = NOPASSWD: /sbin/halt, /sbin/reboot, /sbin/poweroff, /sbin/rmmod, /sbin/modprobe" | sudo EDITOR='tee -a' visudo
fi

# Install MJPG-Streamer.
mkdir -p $basedir/prg
cd $basedir/prg/
if [ -d mjpg-streamer2 ];then
  cd mjpg-streamer2/mjpg-streamer-experimental
  git merge --ff-only origin/master
  make
else
  git clone https://github.com/akihikoy/mjpg-streamer.git mjpg-streamer2
  cd mjpg-streamer2/mjpg-streamer-experimental
  make
fi

# Install scripts.
cd $basedir
wget https://raw.githubusercontent.com/akihikoy/fingervision/master/tools/mjpg_stream.sh -O mjpg_stream.sh
wget https://raw.githubusercontent.com/akihikoy/fingervision/master/tools/stream.sh  -O stream.sh
wget https://raw.githubusercontent.com/akihikoy/fingervision/master/tools/stream1.sh -O stream1.sh
wget https://raw.githubusercontent.com/akihikoy/fingervision/master/tools/stream4.sh -O stream4.sh
wget https://raw.githubusercontent.com/akihikoy/fingervision/master/tools/conf_cam.sh -O conf_cam.sh
wget https://raw.githubusercontent.com/akihikoy/fingervision/master/tools/conf_elp.sh -O conf_elp.sh
chmod 755 stream*.sh conf_*.sh

