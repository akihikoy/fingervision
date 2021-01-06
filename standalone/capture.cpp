//-------------------------------------------------------------------------------------------
/*! \file    capture.cpp
    \brief   General capture tool.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.15, 2017
    \date    Aug.9, 2018
             Using the same core programs as the ROS version.
             Supporting to load YAML configuration.
             Supporting camera rectification.

g++ -g -Wall -O2 -o capture.out capture.cpp -I../3rdparty -I../fv_core -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_highgui

Run:
  $ ./capture.out [CAMERA_NUMBER [WIDTH HEIGHT]]
    CAMERA_NUMBER: Camera device number (e.g. 1).
  $ ./capture.out CONFIG_FILE
    CONFIG_FILE: Configuration file in YAML (e.g. fv_1.yaml).
    The configuration file may include camera configuration (CameraInfo).
Usage:
  Press 'q' or Esc: Exit the program.
  Press 'W' (shift+'w'): Start/stop video recording.
*/
//-------------------------------------------------------------------------------------------
#include "ay_vision/vision_util.h"
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
    cam_info.push_back(TCameraInfo());
    cam_info[0].DevID= cam;
    cam_info[0].Width= ((argc>2)?atoi(argv[2]):0);
    cam_info[0].Height= ((argc>3)?atoi(argv[3]):0);
  }
  cv::VideoCapture cap;
  if(!CapOpen(cam_info[0], cap))  return -1;

  TCameraRectifier cam_rectifier;
  if(cam_info[0].Rectification)
  {
    cv::Size size_in(cam_info[0].Width,cam_info[0].Height), size_out(cam_info[0].Width,cam_info[0].Height);
    cam_rectifier.Setup(cam_info[0].K, cam_info[0].D, cam_info[0].R, size_in, cam_info[0].Alpha, size_out);
  }

  std::string win("camera");
  cv::namedWindow(win,1);

  TEasyVideoOut vout;
  vout.SetfilePrefix("/tmp/cam");

  int show_fps(0);
  cv::Mat frame;
  for(;;)
  {
    frame= Capture(cap, cam_info[0], &cam_rectifier);
    vout.Step(frame);
    vout.VizRec(frame);
    cv::imshow("camera", frame);
    char c(cv::waitKey(10));
    if(c=='\x1b'||c=='q') break;
    else if(char(c)=='W')  vout.Switch();
    // usleep(10000);
    if(show_fps==0)
    {
      std::cerr<<"FPS: "<<vout.FPS()<<std::endl;
      show_fps= vout.FPS()*4;
    }
    --show_fps;
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
