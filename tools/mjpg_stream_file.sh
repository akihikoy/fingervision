#!/bin/bash
#\file    mjpg_stream_file.sh
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.11, 2018
#\brief   Utility of MJPG Streamer.
#         Multiple processes of handling specified video-files are launched at the same time.
#         Press Enter to quit all processes.
#         An example of usage:
#           Create a file:
#           stream_file1.sh
#
#             #!/bin/bash
#             options="-f 60"
#             files=(../data/cam0_0001.m4v ../data/cam1_0001.m4v)
#             source mjpg_stream_file.sh
#
#           and change the file mode of stream.sh to 755.
#           Run ./stream_file1.sh, and press Enter to quit.
#
#           NOTE: We need to use the following fork of MJPG Streamer:
#           https://github.com/akihikoy/mjpg-streamer

if [ -z "$options" ];then
  options="-f 60"
fi
if [ -z "$files" ];then
  files=(../data/cam0_0001.m4v ../data/cam1_0001.m4v)
fi
mjpg_streamer_dir=~/prg/mjpg-streamer2/mjpg-streamer-experimental

# Convert file paths
for((i=0;i<${#files[@]};i+=1)); do
  if [ "${files[$i]:0:1}" != "/" ];then
    files[$i]=`pwd`/${files[$i]}
  fi
done
echo ${#files[@]}

ids=()
function run()
{
  cd $mjpg_streamer_dir
  ids=()
  for((i=0;i<${#files[@]};i+=1)); do
    ./mjpg_streamer -i "./input_opencv_file.so $options -d ${files[$i]}" -o "./output_http.so -w ./www -p $((8080+i))" &
    ids=(${ids[@]} $!)
  done
  sleep 1
}

run
echo "Running ${ids[@]}; Press enter to quit"
read s
echo "Killing ${ids[@]}"
kill -SIGINT ${ids[@]}
sleep 2
