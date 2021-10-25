//-------------------------------------------------------------------------------------------
/*! \file    prox_vision_test.cpp
    \brief   Object detection and tracking (proximity vision) of FingerVision.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.17, 2017
    \version 0.2
    \date    Aug.9, 2018
             Using the same core programs as the ROS version.
             Supporting to load YAML configuration.
             Supporting camera rectification.

g++ -g -Wall -O2 -o prox_vision_test.out prox_vision_test.cpp -I../3rdparty -I../fv_core -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_video -lopencv_highgui -lopencv_videoio

Run:
  $ ./prox_vision_test.out [CAMERA_NUMBER [WIDTH HEIGHT]]
    CAMERA_NUMBER: Camera device number (e.g. 1).
  $ ./prox_vision_test.out CONFIG_FILE
    CONFIG_FILE: Configuration file in YAML (e.g. fv_1.yaml).
    The configuration file may include camera configuration (CameraInfo) and
    the configuration of proximity vision (ObjDetTrack).
Usage:
  Press 'q' or Esc: Exit the program.
  Press 'c': Calibrate the tracker (constructing the background color model).
  Press 'C': Show/hide the parameter configuration trackbars.
  Press 'W' (shift+'w'): Start/stop video recording.
  Press 'r': Reset (clear) the tracking object.
  Press 'd': On/off the object detection mode (default=on).
  Press 'm': Change the dimming-level of the displayed image (0.3 -> 0.7 -> 1.0 -> 0.3 -> ...).
  Shift+Click a point on the image: Add the color of the point to the object color model.
          Tips: This is useful when making an object model manually.
*/
//-------------------------------------------------------------------------------------------
#include "prox_vision.h"
#include "ay_vision/vision_util.h"
#include "ay_cpp/cpp_util.h"
#include "ay_cpp/sys_util.h"
//-------------------------------------------------------------------------------------------
#include "prox_vision.cpp"
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

struct TMouseEventData
{
  cv::Mat &frame;
  TObjDetTrackBSP &tracker;
  TMouseEventData(cv::Mat &f, TObjDetTrackBSP &t) : frame(f), tracker(t) {}
};
void OnMouse(int event, int x, int y, int flags, void *data)
{
  if(event == cv::EVENT_LBUTTONDOWN && (flags & cv::EVENT_FLAG_SHIFTKEY))
  {
    TMouseEventData &d(*reinterpret_cast<TMouseEventData*>(data));
    d.tracker.AddToModel(d.frame, cv::Point(x,y));
  }
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  std::string cam("0"), config_file;
  if(argc>1)  cam= argv[1];

  std::vector<TCameraInfo> cam_info;
  // If cam exists as a file, it is considered as a configuration YAML file.
  if(FileExists(cam))
  {
    ReadFromYAML(cam_info, cam);
    config_file= cam;
  }
  else
  {
    cam_info.push_back(TCameraInfo());
    cam_info[0].DevID= cam;
    cam_info[0].Width= ((argc>2)?atoi(argv[2]):320);
    cam_info[0].Height= ((argc>3)?atoi(argv[3]):240);
  }
  cv::VideoCapture cap;
  if(!CapOpen(cam_info[0], cap))  return -1;

  TCameraRectifier cam_rectifier;
  if(cam_info[0].Rectification)
  {
    cv::Size size_in(cam_info[0].Width,cam_info[0].Height), size_out(cam_info[0].Width,cam_info[0].Height);
    cam_rectifier.Setup(cam_info[0].K, cam_info[0].D, cam_info[0].R, size_in, cam_info[0].Alpha, size_out);
  }

  TObjDetTrackBSP tracker;
  if(config_file=="")
  {
    tracker.Init();
  }
  else
  {
    std::vector<TObjDetTrackBSPParams> objdettrack_info;
    ReadFromYAML(objdettrack_info, config_file);
    tracker.Params()= objdettrack_info[0];
    tracker.Init();
  }

  cv::Mat frame, frame_src;

  cv::namedWindow("camera",1);
  TMouseEventData mouse_data(frame_src,tracker);
  cv::setMouseCallback("camera", OnMouse, &mouse_data);
  bool trackbar_visible(false);

  TEasyVideoOut vout;
  vout.SetfilePrefix("/tmp/objtr");

  int show_fps(0);
  double dim_levels[]={0.0,0.3,0.7,1.0};  int dim_idx(1);
  for(int f(0);;++f)
  {
    frame= Capture(cap, cam_info[0], &cam_rectifier);
    frame.copyTo(frame_src);

    if(f>0)
    {
      tracker.Step(frame);
      frame*= dim_levels[dim_idx];
      tracker.Draw(frame);
    }

    vout.Step(frame);
    vout.VizRec(frame);
    cv::imshow("camera", frame);
    char c(cv::waitKey(1));
    if(c=='\x1b'||c=='q') break;
    else if(char(c)=='W')  vout.Switch();
    else if(c=='m')
    {
      dim_idx++;
      if(dim_idx>=int(sizeof(dim_levels)/sizeof(dim_levels[0])))  dim_idx=0;
    }
    else if(c=='r')
    {
      tracker.ClearObject();
    }
    else if(c=='d')
    {
      if(tracker.ModeDetect())  tracker.StopDetect();
      else                      tracker.StartDetect();
      std::cerr<<"Object detection mode is: "<<(tracker.ModeDetect()?"on":"off")<<std::endl;
    }
    else if(c=='C')
    {
      trackbar_visible= !trackbar_visible;
      if(trackbar_visible)
      {
        CreateTrackbar<float>("History:", "camera", &tracker.Params().BS_History, 0.0, 100.0, 0.1, &TrackbarPrintOnTrack);
        CreateTrackbar<float>("Fbg:",     "camera", &tracker.Params().Fbg, 0.0, 10.0, 0.01, &TrackbarPrintOnTrack);
        CreateTrackbar<float>("Fgain:",   "camera", &tracker.Params().Fgain, 0.0, 10.0, 0.01, &TrackbarPrintOnTrack);
        CreateTrackbar<int>("N-Erode(1):",   "camera", &tracker.Params().NErode1,     0, 10, 1, &TrackbarPrintOnTrack);
        CreateTrackbar<int>("N-Erode(2):",   "camera", &tracker.Params().NErode2,     0, 20, 1, &TrackbarPrintOnTrack);
        CreateTrackbar<int>("N-Dilate(2):",  "camera", &tracker.Params().NDilate2,    0, 20, 1, &TrackbarPrintOnTrack);
        CreateTrackbar<int>("Threshold(2):", "camera", &tracker.Params().NThreshold2, 0, 255, 1, &TrackbarPrintOnTrack);
      }
      else
      {
        // Remove trackbars from window.
        cv::destroyWindow("camera");
        cv::namedWindow("camera",1);
        cv::setMouseCallback("camera", OnMouse, &mouse_data);
      }
    }
    else if(c=='c' || f==0)
    {
      std::vector<cv::Mat> frames;
      for(int i(0); i<tracker.Params().NCalibBGFrames; ++i)
        frames.push_back(Capture(cap, cam_info[0], &cam_rectifier));
      tracker.CalibBG(frames);
    }
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
