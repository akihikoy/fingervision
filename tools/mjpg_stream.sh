#!/bin/bash
#\file    mjpg_stream.sh
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.09, 2018
#\brief   Utility of MJPG Streamer.
#         Multiple processes of handling specified devices are launched at the same time,
#         and they are watched if running.
#         If some processes died, all process are relaunched.
#         Press Enter to quit all processes.
#         A practical usage:
#           Put this script on a Raspberry Pi, and create a file:
#           stream.sh
#
#             #!/bin/bash
#             uvc_opt="-f -1 -r 320x240 -n"
#             devs=(1)
#             source mjpg_stream.sh
#
#           and change the file mode of stream.sh to 755.
#           Run ./stream.sh, and press Enter to quit.

if [ -z "$uvc_opt" ];then
  uvc_opt=-f -1 -r 320x240 -n
fi
if [ -z "$devs" ];then
  devs=(0 1)
fi
mjpg_streamer_dir=~/prg/mjpg-streamer2/mjpg-streamer-experimental

ids=()
function run()
{
  cd $mjpg_streamer_dir
  ids=()
  for((i=0;i<${#devs[@]};i+=1)); do
    ./mjpg_streamer -i "./input_uvc.so $uvc_opt -d /dev/video${devs[$i]}" -o "./output_http.so -w ./www -p $((8080+i))" &
    ids=(${ids[@]} $!)
  done
  sleep 1
}

restart=true
while true; do
  if $restart; then
    run
    echo "Running ${ids[@]}; Press enter to quit"
    restart=false
  fi

  #Check a keyboard input.
  if read -t 0.5 s; then
    break
  fi

  #Check if the processes are running.
  for((i=0;i<${#ids[@]};i+=1)); do
    if [ -z "$(ps -p ${ids[$i]} -o %cpu --noheader)" ]; then
      echo "Process ${ids[$i]} died."
      restart=true
    fi
  done
  if $restart; then
    echo "Killing ${ids[@]}"
    kill -SIGINT ${ids[@]}
    #Wait all processes are killed:
    running=true
    while $running; do
      sleep 0.2
      running=false
      for((i=0;i<${#ids[@]};i+=1)); do
        if [ -n "$(ps -p ${ids[$i]} -o %cpu --noheader)" ]; then
          echo "Process ${ids[$i]} is still running..."
          running=true
        fi
      done
    done
  fi
done
echo "Killing ${ids[@]}"
kill -SIGINT ${ids[@]}
sleep 2
