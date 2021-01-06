//-------------------------------------------------------------------------------------------
/*! \file    capture_mult.cpp
    \brief   General capture tool (multiple cameras).
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Aug.09, 2018

g++ -g -Wall -O2 -o capture_mult.out capture_mult.cpp -I../3rdparty -I../fv_core -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_highgui

Run:
  $ ./capture_mult.out [CAMERA_NUMBER [WIDTH HEIGHT [CAMERA_NUMBER [WIDTH HEIGHT ...]]]]
    CAMERA_NUMBER: Camera device number (e.g. 1).
    To capture from multiple cameras, repeat like:
    $ ./capture_mult.out 0 320 240 1 320 240
  $ ./capture_mult.out CONFIG_FILE
    CONFIG_FILE: Configuration file in YAML (e.g. fv_1.yaml).
    The configuration file may include camera configuration (CameraInfo).
    Put configurations of multiple cameras in the same CONFIG_FILE
    to capture from multiple cameras (e.g. cam_2.yaml).
Usage:
  Press 'q' or Esc: Exit the program.
  Press 'W' (shift+'w'): Start/stop video recording.
*/
//-------------------------------------------------------------------------------------------
#include "ay_vision/vision_util.h"
#include "ay_cpp/cpp_util.h"
#include "ay_cpp/sys_util.h"
//-------------------------------------------------------------------------------------------
#include "ay_vision/vision_util.cpp"
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace trick;
//-------------------------------------------------------------------------------------------

cv::Mat Capture(cv::VideoCapture &cap, TCameraInfo &info, TCameraRectifier *pcam_rectifier=NULL)
{
  cv::Mat frame;
  while(!cap.read(frame))
  {
    if(CapWaitReopen(info,cap)) continue;
    else  return cv::Mat();
  }
  if(info.CapWidth!=info.Width || info.CapHeight!=info.Height)
    cv::resize(frame,frame,cv::Size(info.Width,info.Height));
  if(info.HFlip)  cv::flip(frame, frame, /*horizontal*/1);
  Rotate90N(frame,frame,info.NRotate90);
  if(info.Rectification && pcam_rectifier)
    pcam_rectifier->Rectify(frame, /*border=*/cv::Scalar(0,0,0));
  return frame;
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  std::string cam("0");
  if(argc>1)  cam= argv[1];

  std::vector<TCameraInfo> cam_info;
  // If cam exists as a file, it is considered as a configuration YAML file.
  if(FileExists(cam))
  {
    ReadFromYAML(cam_info, cam);
  }
  else
  {
    for(int i_arg(0); argc>i_arg+1; i_arg+=3)
    {
      cam_info.push_back(TCameraInfo());
      cam_info.back().DevID= argv[i_arg+1];  // cam
      cam_info.back().Width= ((argc>i_arg+2)?atoi(argv[i_arg+2]):0);
      cam_info.back().Height= ((argc>i_arg+3)?atoi(argv[i_arg+3]):0);
    }
  }
  std::vector<cv::VideoCapture> cap(cam_info.size());
  for(int i_cam(0),i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
    if(!CapOpen(cam_info[i_cam], cap[i_cam]))  return -1;

  std::vector<TCameraRectifier> cam_rectifier(cam_info.size());
  for(int i_cam(0),i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
    if(cam_info[i_cam].Rectification)
    {
      const TCameraInfo &info(cam_info[i_cam]);
      cv::Size size_in(info.Width,info.Height), size_out(info.Width,info.Height);
      cam_rectifier[i_cam].Setup(info.K, info.D, info.R, size_in, info.Alpha, size_out);
    }

  for(int i_cam(0),i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
    cv::namedWindow(ToString("camera",i_cam),1);

  std::vector<TEasyVideoOut> vout(cam_info.size());
  for(int i_cam(0),i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
    vout[i_cam].SetfilePrefix(ToString("/tmp/cam",i_cam,"_"));

  int show_fps(0);
  std::vector<cv::Mat> frame(cam_info.size());
  for(;;)
  {
    for(int i_cam(0),i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
      frame[i_cam]= Capture(cap[i_cam], cam_info[i_cam], &cam_rectifier[i_cam]);
    for(int i_cam(0),i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
    {
      vout[i_cam].Step(frame[i_cam]);
      vout[i_cam].VizRec(frame[i_cam]);
    }
    for(int i_cam(0),i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
      cv::imshow(ToString("camera",i_cam), frame[i_cam]);
    char c(cv::waitKey(10));
    if(c=='\x1b'||c=='q') break;
    else if(char(c)=='W')
    {
      for(int i_cam(0),i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
        vout[i_cam].Switch();
    }
    // usleep(10000);
    if(show_fps==0)
    {
      std::cerr<<"FPS: "<<vout[0].FPS()<<std::endl;
      show_fps= vout[0].FPS()*4;
    }
    --show_fps;
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
