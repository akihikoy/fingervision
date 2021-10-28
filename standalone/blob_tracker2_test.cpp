//-------------------------------------------------------------------------------------------
/*! \file    blob_tracker2_test.cpp
    \brief   Marker tracker (blob tracker) of FingerVision.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.10, 2016
    \version 0.2
    \date    Aug.9, 2018
             Using the same core programs as the ROS version.
             Supporting to load YAML configuration.
             Supporting camera rectification.
    \version 0.3
    \date    May.20, 2019
             Modified the trackbars.

g++ -g -Wall -O2 -o blob_tracker2_test.out blob_tracker2_test.cpp -I../3rdparty -I../fv_core -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_highgui -lopencv_videoio

Run:
  $ ./blob_tracker2_test.out [CAMERA_NUMBER [WIDTH HEIGHT]]
    CAMERA_NUMBER: Camera device number (e.g. 1).
  $ ./blob_tracker2_test.out CONFIG_FILE
    CONFIG_FILE: Configuration file in YAML (e.g. fv_1.yaml).
    The configuration file may include camera configuration (CameraInfo) and
    the configuration of blob tracker (BlobTracker2).
Usage:
  Press 'q' or Esc: Exit the program.
  Press 'c': Calibrate the tracker (detecting markers and storing the initial positions and sizes).
             Tips: Show a white paper or white wall during the calibration.
  Press 'C': Switch the parameter configuration trackbar mode.
  Press 'P': Save the tracker parameter into a file in /tmp.
  Press 'W' (shift+'w'): Start/stop video recording.
  Press 'p': Print the calibration result.
  Press 's': Save the calibration result to file "blob_calib.yaml".
  Press 'l': Load a calibration from file "blob_calib.yaml".
*/
//-------------------------------------------------------------------------------------------
#include "blob_tracker2.h"
#include "ay_vision/vision_util.h"
#include "ay_cpp/cpp_util.h"
#include "ay_cpp/sys_util.h"
//-------------------------------------------------------------------------------------------
#include "blob_tracker2.cpp"
#include "ay_vision/vision_util.cpp"
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace trick;
//-------------------------------------------------------------------------------------------

TBlobTracker2 *PTracker(NULL);
template<typename T>
void OnTrack2(const TExtendedTrackbarInfo<T> &info, void*)
{
  TrackbarPrintOnTrack(info, NULL);
  if(PTracker!=NULL)  PTracker->Init();
}
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
  TBlobTracker2 &tracker;
  TMouseEventData(cv::Mat &f, TBlobTracker2 &t) : frame(f), tracker(t) {}
};
void OnMouse(int event, int x, int y, int flags, void *data)
{
  if(event == cv::EVENT_RBUTTONDOWN && (flags & cv::EVENT_FLAG_SHIFTKEY))
  {
    TMouseEventData &d(*reinterpret_cast<TMouseEventData*>(data));
    d.tracker.RemovePointAt(cv::Point2f(x,y));
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

  TBlobTracker2 tracker;
  if(config_file=="")
  {
    tracker.Init();
  }
  else
  {
    std::vector<TBlobTracker2Params> blobtrack_info;
    ReadFromYAML(blobtrack_info, config_file);
    tracker.Params()= blobtrack_info[0];
    tracker.Init();
  }

  bool calib_request(true);
  std::string blob_calib_yaml("blob_calib.yaml");
  if(FileExists(blob_calib_yaml))
  {
    tracker.LoadCalib(blob_calib_yaml);
    std::cerr<<"Loaded calibration data from "<<blob_calib_yaml<<std::endl;
    calib_request= false;
  }

  cv::Mat frame;

  std::string win("camera");
  cv::namedWindow("camera",1);
  TMouseEventData mouse_data(frame,tracker);
  cv::setMouseCallback("camera", OnMouse, &mouse_data);
  int trackbar_mode(0);

  TEasyVideoOut vout;
  vout.SetfilePrefix("/tmp/blobtr");

  int show_fps(0);
  double dim_levels[]={0.0,0.3,0.7,1.0};  int dim_idx(3);
  for(int f(0);;++f)
  {
    frame= Capture(cap, cam_info[0], &cam_rectifier);

    tracker.Step(frame);
    frame*= dim_levels[dim_idx];
    tracker.Draw(frame);

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
    else if(c=='p')  tracker.SaveCalib("/dev/stdout");
    else if(c=='s')
    {
      tracker.SaveCalib(blob_calib_yaml);
      std::cerr<<"Saved calibration data to "<<blob_calib_yaml<<std::endl;
    }
    else if(c=='l')
    {
      tracker.LoadCalib(blob_calib_yaml);
      std::cerr<<"Loaded calibration data from "<<blob_calib_yaml<<std::endl;
    }
    else if(c=='P')
    {
      std::vector<TBlobTracker2Params> p;
      p.push_back(tracker.Params());
      WriteToYAML(p,"/tmp/blobtr_params.yaml");
      std::cerr<<"Parameters of the tracker are saved into /tmp/blobtr_params.yaml"<<std::endl;
    }
    else if(c=='C')
    {
      ++trackbar_mode;
      if(trackbar_mode==1)
      {
        CreateTrackbar<int>("ThreshV", win, &tracker.Params().ThreshV, 0, 255, 1, &TrackbarPrintOnTrack);
        CreateTrackbar<int>("NDilate1:", win, &tracker.Params().NDilate1, 0, 10, 1, &TrackbarPrintOnTrack);
        CreateTrackbar<int>("NErode1:", win, &tracker.Params().NErode1, 0, 10, 1, &TrackbarPrintOnTrack);
        CreateTrackbar<float>("SWidth:", win, &tracker.Params().SWidth, 0.0, 100.0, 0.1, &TrackbarPrintOnTrack);
        CreateTrackbar<float>("NonZeroMin:", win, &tracker.Params().NonZeroMin, 0.0, 20.0, 0.01, &TrackbarPrintOnTrack);
        CreateTrackbar<float>("NonZeroMax:", win, &tracker.Params().NonZeroMax, 0.0, 20.0, 0.01, &TrackbarPrintOnTrack);
        CreateTrackbar<float>("VPMax:", win, &tracker.Params().VPMax, 0.0, 20.0, 0.1, &TrackbarPrintOnTrack);
        CreateTrackbar<float>("VSMax:", win, &tracker.Params().VSMax, 0.0, 20.0, 0.1, &TrackbarPrintOnTrack);
        CreateTrackbar<int>("NReset:", win, &tracker.Params().NReset, 0, 20, 1, &TrackbarPrintOnTrack);
      }
      else if(trackbar_mode==2)
      {
        // Remove trackbars from window.
        cv::destroyWindow(win);
        cv::namedWindow(win,1);
        PTracker= &tracker;
        CreateTrackbar<float>("SBDParams.minArea", win, &tracker.Params().SBDParams.minArea, 0.0, 20000.0, 1.0, &OnTrack2);
        CreateTrackbar<float>("SBDParams.maxArea", win, &tracker.Params().SBDParams.maxArea, 0.0, 20000.0, 1.0, &OnTrack2);
        CreateTrackbar<float>("SBDParams.minCircularity:", win, &tracker.Params().SBDParams.minCircularity, 0.0, 1.0, 0.01, &OnTrack2);
        CreateTrackbar<float>("SBDParams.minConvexity:", win, &tracker.Params().SBDParams.minConvexity, 0.0, 1.0, 0.01, &OnTrack2);
        CreateTrackbar<float>("SBDParams.minInertiaRatio:", win, &tracker.Params().SBDParams.minInertiaRatio, 0.0, 1.0, 0.01, &OnTrack2);
      }
      else
      {
        trackbar_mode= 0;
        // Remove trackbars from window.
        cv::destroyWindow(win);
        cv::namedWindow(win,1);
      }
    }
    else if(c=='c' || calib_request)
    {
      std::vector<cv::Mat> frames;
      for(int i(0); i<tracker.Params().NCalibPoints; ++i)
        frames.push_back(Capture(cap, cam_info[0], &cam_rectifier));
      tracker.Calibrate(frames);
      calib_request= false;
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
