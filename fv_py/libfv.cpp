//-------------------------------------------------------------------------------------------
/*! \file    libfv.cpp
    \brief   Standalone FingerVision module.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Aug.27, 2022
*/
//-------------------------------------------------------------------------------------------
#include "blob_tracker2.h"
#include "prox_vision.h"
#include "ay_vision/vision_util.h"
#include "ay_cpp/geom_util.h"
#include "ay_cpp/sys_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
//-------------------------------------------------------------------------------------------
#include "libfv.h"
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

bool Running(true), Shutdown(false), CalibrationRequest(false), TrackerInitRequest(false);
std::string *CurrentWin(NULL);

/*FPS control parameters.
  FrameSkip: Video images is processed every 1+FrameSkip frames.
             FrameSkip=0: processing every frame.
  TargetFPS: Video image is processed at this FPS.
             TargetFPS=0: processing every frame.
  CaptureFPS: Image is captured at this FPS.
             CaptureFPS=0: capture as fast as possible.
  NOTE: Do not set FrameSkip and TargetFPS simultaneously, which would be confusing.
  Pseudo code:
    for each frame f:
      Capture frame;
      if(FrameSkip<=0 || f%(FrameSkip+1)==0)  Process;
    In Process:
      TRateAdjuster rate(TargetFPS)
      for each frame:
        rate.Sleep()
  TODO:FIXME: Currently TargetFPS is not configurable during run time, but
    changing TargetFPS in execution is useful.
*/
int FrameSkip(0);  // 0: no skip
double TargetFPS(0);  // 0: no FPS control
double CaptureFPS(0);  // 0: no FPS control

std::string BlobCalibPrefix("blob_");
std::vector<TCameraInfo> CamInfo;
std::vector<TBlobTracker2> BlobTracker;  // Marker tracker
std::vector<TObjDetTrackBSP> ObjDetTracker;  // Proximity vision
std::vector<TCameraRectifier> SingleCamRectifier;
std::vector<boost::function<void(cv::Mat&)> > CamRectifier;  // Functions to rectify camera images.
void DummyRectify(cv::Mat&) {}  // Do nothing function

std::map<std::string, TEasyVideoOut> VideoOut;
struct TShowTrackbars
{
  // bool Enabled;
  std::string Kind;
  int Mode;
  TShowTrackbars() : /*Enabled(false),*/ Mode(0) {}
};
std::map<std::string, TShowTrackbars> ShowTrackbars;

// std::vector<ros::Publisher> BlobPub;
// std::vector<ros::Publisher> PXVPub;
// std::vector<image_transport::Publisher> ImgPub;  // Image publisher [i_cam]
std::vector<cv::Mat> Frame;
std::vector<int64_t> CapTime;
std::vector<boost::shared_ptr<boost::mutex> > MutCamCapture;  // Mutex for capture
std::vector<boost::shared_ptr<boost::mutex> > MutFrameCopy;  // Mutex for Frame
struct TIMShowStuff
{
  boost::shared_ptr<boost::mutex> Mutex;
  cv::Mat Frame;
};
std::map<std::string, TIMShowStuff> IMShowStuff;

struct TWindowInfo
{
  int CamIdx;  // Camera index.
  std::string Kind;  // Kind of computer vision.
  int Index;  // Index of computer vision object (BlobTracker, ObjDetTracker).
  TWindowInfo()
    : CamIdx(-1), Kind(), Index(-1)  {}
  TWindowInfo(int cam_idx, const std::string &k, int idx)
    : CamIdx(cam_idx), Kind(k), Index(idx)  {}
};
std::map<std::string, TWindowInfo> WindowInfo;

bool WindowsHidden(false);

// Dim-levels.
double DimLevels[]={0.0,0.3,0.7,1.0};
int DimIdxBT(3),DimIdxPV(1);

// Threads
std::vector<boost::shared_ptr<boost::thread> > ThBlobTrack;
std::vector<boost::shared_ptr<boost::thread> > ThObjDetTrack;
// std::vector<boost::shared_ptr<boost::thread> > ThImgPub;
std::vector<boost::shared_ptr<boost::thread> > ThImgCap;
//-------------------------------------------------------------------------------------------

bool IsShutdown()
{
  // return Shutdown || ros::isShuttingDown() || !ros::ok();
  return Shutdown;
}
//-------------------------------------------------------------------------------------------

void SetShutdown()
{
  Shutdown= true;
}
//-------------------------------------------------------------------------------------------

/*
  Right click: pause/resume
*/
void OnMouse(int event, int x, int y, int flags, void *data)
{
  if(event!=0)
  {
    CurrentWin= reinterpret_cast<std::string*>(data);
    std::cerr<<"CurrentWin: "<<*CurrentWin<<std::endl;
  }

  if(event == cv::EVENT_RBUTTONDOWN && flags == 0)
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  if(event == cv::EVENT_LBUTTONDOWN && (flags & cv::EVENT_FLAG_SHIFTKEY))
  {
    if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
    {
      int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
      cv::Mat frame;
      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
      }
      ObjDetTracker[idx].AddToModel(frame, cv::Point(x,y));
    }
  }
  if(event == cv::EVENT_RBUTTONDOWN && (flags & cv::EVENT_FLAG_SHIFTKEY))
  {
    if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="BlobTracker")
    {
      int idx(WindowInfo[*CurrentWin].Index);
      BlobTracker[idx].RemovePointAt(cv::Point2f(x,y));
    }
  }
}
//-------------------------------------------------------------------------------------------

// return if continue
bool HandleKeyEvent()
{
  // keyboard interface:
  char c(cv::waitKey(1));
  if(c=='\x1b'||c=='q') return false;
  else if(c=='W')
  {
    for(std::map<std::string, TEasyVideoOut>::iterator itr(VideoOut.begin()),itr_end(VideoOut.end()); itr!=itr_end; ++itr)
      itr->second.Switch();
  }
  else if(c=='m' && CurrentWin!=NULL)
  {
    if(WindowInfo[*CurrentWin].Kind=="BlobTracker")
    {
      DimIdxBT++;
      if(DimIdxBT>=int(sizeof(DimLevels)/sizeof(DimLevels[0])))  DimIdxBT=0;
    }
    else if(WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
    {
      DimIdxPV++;
      if(DimIdxPV>=int(sizeof(DimLevels)/sizeof(DimLevels[0])))  DimIdxPV=0;
    }
  }
  else if(c==' ')
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  else if(c=='c')
  {
    CalibrationRequest= true;
  }
  else if(c=='r')
  {
    if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
    {
      int idx(WindowInfo[*CurrentWin].Index);
      ObjDetTracker[idx].ClearObject();
    }
  }
  else if(c=='d')
  {
    if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
    {
      int idx(WindowInfo[*CurrentWin].Index);
      if(ObjDetTracker[idx].ModeDetect())  ObjDetTracker[idx].StopDetect();
      else                                 ObjDetTracker[idx].StartDetect();
      std::cerr<<"Object detection mode is: "<<(ObjDetTracker[idx].ModeDetect()?"on":"off")<<std::endl;
    }
  }
  else if(c=='s')
  {
    if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="BlobTracker")
    {
      int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
      BlobTracker[idx].SaveCalib(BlobCalibPrefix+CamInfo[i_cam].Name+".yaml");
      std::cerr<<"Saved calibration data of "<<BlobTracker[idx].Name<<" to "<<BlobCalibPrefix+CamInfo[i_cam].Name+".yaml"<<std::endl;
    }
  }
  else if(c=='l')
  {
    if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="BlobTracker")
    {
      int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
      BlobTracker[idx].LoadCalib(BlobCalibPrefix+CamInfo[i_cam].Name+".yaml");
      std::cerr<<"Loaded calibration data of "<<BlobTracker[idx].Name<<" from "<<BlobCalibPrefix+CamInfo[i_cam].Name+".yaml"<<std::endl;
    }
  }
  else if(c=='C' && CurrentWin!=NULL && !WindowsHidden)
  {
    cv::destroyWindow(*CurrentWin);
    cv::namedWindow(*CurrentWin,1);
    if(ShowTrackbars[*CurrentWin].Kind=="BlobTracker")
    {
      int idx(WindowInfo[*CurrentWin].Index);
      ++ShowTrackbars[*CurrentWin].Mode;
      CreateTrackbars(*CurrentWin, BlobTracker[idx].Params(), ShowTrackbars[*CurrentWin].Mode, TrackerInitRequest);
    }
    else if(ShowTrackbars[*CurrentWin].Kind=="ObjDetTracker")
    {
      int idx(WindowInfo[*CurrentWin].Index);
      ++ShowTrackbars[*CurrentWin].Mode;
      CreateTrackbars(*CurrentWin, ObjDetTracker[idx].Params(), ShowTrackbars[*CurrentWin].Mode, TrackerInitRequest);
    }
    cv::setMouseCallback(*CurrentWin, OnMouse, CurrentWin);
  }

  return true;
}
//-------------------------------------------------------------------------------------------

void Pause()
{
  std::cerr<<"Paused..."<<std::endl;
  Running= false;
}
//-------------------------------------------------------------------------------------------

void Resume()
{
  std::cerr<<"Resumed..."<<std::endl;
  Running= true;
}
//-------------------------------------------------------------------------------------------

void ShowWindows()
{
  std::cerr<<"Showing windows..."<<std::endl;
  for(std::map<std::string, TIMShowStuff>::const_iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
  {
    std::string &window_name(const_cast<std::string&>(itr->first));
    cv::namedWindow(window_name,1);
    cv::setMouseCallback(window_name, OnMouse, &window_name);
  }
  WindowsHidden= false;
}
//-------------------------------------------------------------------------------------------

void HideWindows()
{
  std::cerr<<"Hiding windows..."<<std::endl;
  for(std::map<std::string, TIMShowStuff>::const_iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
    cv::destroyWindow(itr->first);
  WindowsHidden= true;
}
//-------------------------------------------------------------------------------------------

void StartRecord()
{
  for(std::map<std::string, TEasyVideoOut>::iterator itr(VideoOut.begin()),itr_end(VideoOut.end()); itr!=itr_end; ++itr)
    itr->second.Rec();
}
//-------------------------------------------------------------------------------------------

void StopRecord()
{
  for(std::map<std::string, TEasyVideoOut>::iterator itr(VideoOut.begin()),itr_end(VideoOut.end()); itr!=itr_end; ++itr)
    itr->second.Stop();
}
//-------------------------------------------------------------------------------------------

void SetVideoPrefix(const std::string &file_prefix)
{
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  {
    VideoOut[BlobTracker[j].Name].SetfilePrefix(file_prefix+BlobTracker[j].Name);
    VideoOut[BlobTracker[j].Name+"-orig"].SetfilePrefix(file_prefix+BlobTracker[j].Name+"-orig");
  }
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  {
    VideoOut[ObjDetTracker[j].Name].SetfilePrefix(file_prefix+ObjDetTracker[j].Name);
    VideoOut[ObjDetTracker[j].Name+"-orig"].SetfilePrefix(file_prefix+ObjDetTracker[j].Name+"-orig");
  }
}
//-------------------------------------------------------------------------------------------

void SetFrameSkip(int frame_skip)
{
  std::cerr<<"Setting frame skip as "<<frame_skip<<"..."<<std::endl;
  FrameSkip= frame_skip;
}
//-------------------------------------------------------------------------------------------

/*Take a snapshot.
Filename(camera) = [prefix]-CameraName-TimeStamp[ext].
ext: File extension (.png, .jpg, etc.).
return: files (Snapshot file names). */
std::list<std::string> TakeSnapshot(const std::string &prefix, const std::string &ext)
{
  std::list<std::string> files;
  std::string filename, timestamp;
  cv::Mat frame;
  int64_t t_cap(0);
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    {
      boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
      Frame[i_cam].copyTo(frame);
      t_cap= CapTime[i_cam];
    }
    std::stringstream ss;
    ss<<t_cap;
    timestamp= ss.str();
    {
      filename= prefix+"-"+CamInfo[i_cam].Name+"-"+timestamp+ext;
      cv::imwrite(filename, frame);
      files.push_back(filename);
      std::cerr<<"Saved a snapshot: "<<filename<<std::endl;
    }
  }
  return files;
}
//-------------------------------------------------------------------------------------------

void StopDetectObj()
{
  std::cerr<<"Stopping object detection..."<<std::endl;
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    ObjDetTracker[j].StopDetect();
}
//-------------------------------------------------------------------------------------------

void StartDetectObj()
{
  std::cerr<<"Starting object detection..."<<std::endl;
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    ObjDetTracker[j].StartDetect();
}
//-------------------------------------------------------------------------------------------

void ClearObj()
{
  std::cerr<<"Clearing object models..."<<std::endl;
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    ObjDetTracker[j].ClearObject();
}
//-------------------------------------------------------------------------------------------

void ExecBlobTrack(int i_cam)
{
  TCameraInfo &info(CamInfo[i_cam]);
  TBlobTracker2 &tracker(BlobTracker[i_cam]);
  cv::Mat frame;
  int64_t t_cap(0);
  TRateAdjuster rate(TargetFPS>0.0?TargetFPS:1);
  for(int seq(0); !Shutdown; ++seq)
  {
    if(Running)
    {
      if(TrackerInitRequest && CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="BlobTracker")
      {
        tracker.Init();
        TrackerInitRequest= false;
      }

      if(CapTime[i_cam]==t_cap)
      {
        usleep(10*1000);
        continue;
      }
      if(TargetFPS>0.0)  rate.Sleep();

      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
        t_cap= CapTime[i_cam];
      }
      VideoOut[tracker.Name+"-orig"].Step(frame);  // Record original video

      CamRectifier[i_cam](frame);
      tracker.Step(frame);
      frame*= DimLevels[DimIdxBT];
      tracker.Draw(frame);

      VideoOut[tracker.Name].Step(frame);
      VideoOut[tracker.Name].VizRec(frame);

      {
        // cv::imshow(info.Name, frame);
        boost::mutex::scoped_lock lock(*IMShowStuff[tracker.Name].Mutex);
        frame.copyTo(IMShowStuff[tracker.Name].Frame);
      }

      // Publish as BlobMoves
//       {
//         const std::vector<TPointMove2> &data(tracker.Data());
//         fingervision_msgs::BlobMoves blob_moves;
//         blob_moves.header.seq= seq;
//         blob_moves.header.stamp= ros::Time::now();
//         blob_moves.header.frame_id= info.Name;
//         blob_moves.camera_index= i_cam;
//         blob_moves.camera_name= info.Name;
//         blob_moves.width= info.Width;
//         blob_moves.height= info.Height;
//         blob_moves.data.resize(data.size());
//         int i(0);
//         for(std::vector<TPointMove2>::const_iterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr,++i)
//         {
//           fingervision_msgs::BlobMove &m(blob_moves.data[i]);
//           m.Pox= itr->Po.x;
//           m.Poy= itr->Po.y;
//           m.So = itr->So;
//           m.DPx= itr->DP.x;
//           m.DPy= itr->DP.y;
//           m.DS = itr->DS;
//         }
//         BlobPub[i_cam].publish(blob_moves);
//       }
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

void ExecObjDetTrack(int i_cam)
{
  TCameraInfo &info(CamInfo[i_cam]);
  TObjDetTrackBSP &tracker(ObjDetTracker[i_cam]);
  cv::Mat frame;
  int64_t t_cap(0);
  TRateAdjuster rate(TargetFPS>0.0?TargetFPS:1);
  for(int seq(0); !Shutdown; ++seq)
  {
    if(Running)
    {
      if(TrackerInitRequest && CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
      {
        tracker.Init();
        TrackerInitRequest= false;
      }

      if(CapTime[i_cam]==t_cap)
      {
        usleep(10*1000);
        continue;
      }
      if(TargetFPS>0.0)  rate.Sleep();

      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
        t_cap= CapTime[i_cam];
      }
      VideoOut[tracker.Name+"-orig"].Step(frame);  // Record original video

      CamRectifier[i_cam](frame);
      tracker.Step(frame);
      frame*= DimLevels[DimIdxPV];
      tracker.Draw(frame);

      VideoOut[tracker.Name].Step(frame);
      VideoOut[tracker.Name].VizRec(frame);

      {
        // cv::imshow(info.Name, frame);
        boost::mutex::scoped_lock lock(*IMShowStuff[tracker.Name].Mutex);
        frame.copyTo(IMShowStuff[tracker.Name].Frame);
      }

      // Publish as PXVPub
//       {
//         const cv::Mat &objs(tracker.ObjS()), &mvs(tracker.MvS());
//         const cv::Moments &om(tracker.ObjMoments());
//         fingervision_msgs::ProxVision prox_vision;
//         prox_vision.header.seq= seq;
//         prox_vision.header.stamp= ros::Time::now();
//         prox_vision.header.frame_id= info.Name;
//         prox_vision.camera_index= i_cam;
//         prox_vision.camera_name= info.Name;
//         prox_vision.width= info.Width;
//         prox_vision.height= info.Height;
//
//         double m[]= {om.m00, om.m10, om.m01, om.m20, om.m11, om.m02, om.m30, om.m21, om.m12, om.m03};
//         std::vector<float> vm(m,m+10);
//         prox_vision.ObjM_m= vm;
//
//         double mu[]= {om.mu20, om.mu11, om.mu02, om.mu30, om.mu21, om.mu12, om.mu03};
//         std::vector<float> vmu(mu,mu+7);
//         prox_vision.ObjM_mu= vmu;
//
//         double nu[]= {om.nu20, om.nu11, om.nu02, om.nu30, om.nu21, om.nu12, om.nu03};
//         std::vector<float> vnu(nu,nu+7);
//         prox_vision.ObjM_nu= vnu;
//
//         prox_vision.ObjS.resize(objs.rows*objs.cols);
//         for(int r(0),rend(objs.rows),i(0);r<rend;++r) for(int c(0),cend(objs.cols);c<cend;++c,++i)
//           prox_vision.ObjS[i]= objs.at<float>(r,c);
//
//         prox_vision.MvS.resize(mvs.rows*mvs.cols);
//         for(int r(0),rend(mvs.rows),i(0);r<rend;++r) for(int c(0),cend(mvs.cols);c<cend;++c,++i)
//           prox_vision.MvS[i]= mvs.at<float>(r,c);
//
//         PXVPub[i_cam].publish(prox_vision);
//       }
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

cv::Mat Capture(cv::VideoCapture &cap, int i_cam, bool rectify, bool auto_reopen=true)
{
  cv::Mat frame;
  {
    boost::mutex::scoped_lock lock(*MutCamCapture[i_cam]);
    while(!cap.read(frame))
    {
      if(!auto_reopen || IsShutdown()
        || !CapWaitReopen(CamInfo[i_cam],cap,/*ms_wait=*/1000,/*max_count=*/0,/*check_to_stop=*/IsShutdown))
        return cv::Mat();
    }
  }
  // TODO:Define a PreProc function in ay_vision and replace the following code.
  if(CamInfo[i_cam].CapWidth!=CamInfo[i_cam].Width || CamInfo[i_cam].CapHeight!=CamInfo[i_cam].Height)
    cv::resize(frame,frame,cv::Size(CamInfo[i_cam].Width,CamInfo[i_cam].Height));
  if(CamInfo[i_cam].HFlip)  cv::flip(frame, frame, /*horizontal*/1);
  Rotate90N(frame,frame,CamInfo[i_cam].NRotate90);
  if(rectify)  CamRectifier[i_cam](frame);
  return frame;
}
//-------------------------------------------------------------------------------------------

// Capture a sequence of images.
std::vector<cv::Mat> CaptureSeq(cv::VideoCapture &cap, int i_cam, int num)
{
  std::vector<cv::Mat> frames;
  for(int i(0); i<num; ++i)
    frames.push_back(Capture(cap, i_cam, /*rectify=*/true));
  return frames;
}
//-------------------------------------------------------------------------------------------

void ExecImgCapture(std::vector<cv::VideoCapture> &cap, bool camera_auto_reopen)
{
  int show_fps(0);
  TRateAdjuster rate(CaptureFPS>0.0?CaptureFPS:1);
  for(int f(0);!IsShutdown();++f)
  {
    if(Running)
    {
      // Capture from cameras:
      for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
      {
        cv::Mat frame= Capture(cap[i_cam], i_cam, /*rectify=*/false, camera_auto_reopen);
        if(frame.empty())  Shutdown=true;
        if(FrameSkip<=0 || f%(FrameSkip+1)==0)
        {
          boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
          Frame[i_cam]= frame;
          CapTime[i_cam]= GetCurrentTimeL();
        }
      }
      if(IsShutdown())  break;

      // Handle blob tracker calibration request
      if(CalibrationRequest && BlobTracker.size()>0 && CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="BlobTracker")
      {
        Running= false;
        int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
        std::vector<cv::Mat> frames= CaptureSeq(cap[i_cam], i_cam, BlobTracker[idx].Params().NCalibPoints);
        BlobTracker[idx].Calibrate(frames);
        Running= true;
      }
      if(CalibrationRequest && ObjDetTracker.size()>0 && CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
      {
        Running= false;
        int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
        std::vector<cv::Mat> frames= CaptureSeq(cap[i_cam], i_cam, ObjDetTracker[idx].Params().NCalibBGFrames);
        ObjDetTracker[idx].CalibBG(frames);
        Running= true;
      }
      CalibrationRequest= false;

      // usleep(10*1000);
      if(show_fps==0)
      {
        std::cerr<<"FPS: "<<VideoOut.begin()->second.FPS()<<std::endl;
        show_fps=VideoOut.begin()->second.FPS()*4;
      }
      --show_fps;

      if(CaptureFPS>0.0)  rate.Sleep();

    }  // Running
    else
    {
      usleep(200*1000);
    }

//     ros::spinOnce();
  }
  Shutdown= true;
}
//-------------------------------------------------------------------------------------------

// Display images with imshow and run the key event handler.
// return: false if shutdown is requested.
bool DisplayImages()
{
  // Show windows
  for(std::map<std::string, TIMShowStuff>::iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
  {
    boost::mutex::scoped_lock lock(*itr->second.Mutex);
    if(itr->second.Frame.total()>0)
      cv::imshow(itr->first, itr->second.Frame);
  }
  if(!HandleKeyEvent())
  {
    Shutdown= true;
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

// Display an image (window) specified by name.
void DisplayImage(const std::string &name)
{
  if(IMShowStuff.count(name)==0)
  {
    std::cerr<<"DisplayImage: No image named "<<name<<std::endl;
    return;
  }
  boost::mutex::scoped_lock lock(*IMShowStuff[name].Mutex);
  if(IMShowStuff[name].Frame.total()>0)
    cv::imshow(name, IMShowStuff[name].Frame);
}
//-------------------------------------------------------------------------------------------

// Return a list of image (window) names to be displayed.
std::list<std::string> GetDisplayImageList()
{
  std::list<std::string> res;
  for(std::map<std::string, TIMShowStuff>::const_iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
    res.push_back(itr->first);
  return res;
}
//-------------------------------------------------------------------------------------------

inline std::string CheckYAMLExistence(const std::string &filename)
{
  std::cerr<<"Loading from YAML: "<<filename<<std::endl;
  if(!FileExists(filename))
    std::cerr<<"!!!File not found: "<<filename<<std::endl;
  return filename;
}
//-------------------------------------------------------------------------------------------

void StopThreads()
{
  for(int j(0),j_end(ThBlobTrack.size());j<j_end;++j)
    ThBlobTrack[j]->join();
  ThBlobTrack.clear();
  for(int j(0),j_end(ThObjDetTrack.size());j<j_end;++j)
    ThObjDetTrack[j]->join();
  ThObjDetTrack.clear();
  // for(int j(0),j_end(ThImgPub.size());j<j_end;++j)
  //   ThImgPub[j]->join();
  // ThImgPub.clear();
  for(int j(0),j_end(ThImgCap.size());j<j_end;++j)
    ThImgCap[j]->join();
  ThImgCap.clear();
  // usleep(500*1000);
}
//-------------------------------------------------------------------------------------------

void StartThreads(
    const std::string &pkg_dir,
    const std::string &config,
    const std::string &blob_calib_prefix,
    int frame_skip, int target_fps, int capture_fps, bool windows_hidden, bool camera_auto_reopen)
{
//   ros::init(argc, argv, "fv_core_node");
//   ros::NodeHandle node("~");
//   ros::Duration(0.1).sleep();
  const std::string &cam_config(config);
  const std::string &blobtrack_config(config);
  const std::string &objdettrack_config(config);
  std::string vout_base("/tmp/vout-");
//   bool publish_image(false);
  FrameSkip= frame_skip;
  TargetFPS= target_fps;
  CaptureFPS= capture_fps;
  WindowsHidden= windows_hidden;

  std::cerr<<"pkg_dir: "<<pkg_dir<<std::endl;
  std::cerr<<"cam_config: "<<cam_config<<std::endl;
  std::cerr<<"blobtrack_config: "<<blobtrack_config<<std::endl;
  std::cerr<<"objdettrack_config: "<<objdettrack_config<<std::endl;
  std::cerr<<"blob_calib_prefix: "<<blob_calib_prefix<<std::endl;

  std::vector<TBlobTracker2Params> blobtrack_info;
  std::vector<TObjDetTrackBSPParams> objdettrack_info;
  ReadFromYAML(CamInfo, CheckYAMLExistence(pkg_dir+"/"+cam_config));
  ReadFromYAML(blobtrack_info, CheckYAMLExistence(pkg_dir+"/"+blobtrack_config));
  ReadFromYAML(objdettrack_info, CheckYAMLExistence(pkg_dir+"/"+objdettrack_config));
  BlobCalibPrefix= pkg_dir+"/"+blob_calib_prefix;

  std::vector<cv::VideoCapture> cap(CamInfo.size());
  SingleCamRectifier.resize(CamInfo.size());
  CamRectifier.resize(CamInfo.size());
  Frame.resize(CamInfo.size());
  CapTime.resize(CamInfo.size());
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    TCameraInfo &info(CamInfo[i_cam]);
    if(!CapOpen(info, cap[i_cam]))  return;
    MutCamCapture.push_back(boost::shared_ptr<boost::mutex>(new boost::mutex));
    MutFrameCopy.push_back(boost::shared_ptr<boost::mutex>(new boost::mutex));

    if(info.Rectification)
    {
      // Setup rectification
      // NOTE: The rectification of StereoInfo overwrites this rectification.
      cv::Size size_in(info.Width,info.Height), size_out(info.Width,info.Height);
      SingleCamRectifier[i_cam].Setup(info.K, info.D, info.R, size_in, info.Alpha, size_out);
      CamRectifier[i_cam]= boost::bind(&TCameraRectifier::Rectify, SingleCamRectifier[i_cam], _1, /*border=*/cv::Scalar(0,0,0));
    }
    else
    {
      CamRectifier[i_cam]= &DummyRectify;
    }
  }
  std::cerr<<"Opened camera(s)"<<std::endl;

  BlobTracker.resize(CamInfo.size());
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  {
    BlobTracker[j].Name= CamInfo[j].Name+"-blob";
    BlobTracker[j].Params()= blobtrack_info[j];
    BlobTracker[j].Init();
    if(FileExists(BlobCalibPrefix+CamInfo[j].Name+".yaml"))
      BlobTracker[j].LoadCalib(BlobCalibPrefix+CamInfo[j].Name+".yaml");
    WindowInfo[BlobTracker[j].Name]= TWindowInfo(j, "BlobTracker", j);
    if(!WindowsHidden)
    {
      cv::namedWindow(BlobTracker[j].Name,1);
      cv::setMouseCallback(BlobTracker[j].Name, OnMouse, &BlobTracker[j].Name);
    }
    IMShowStuff[BlobTracker[j].Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
    ShowTrackbars[BlobTracker[j].Name].Kind= "BlobTracker";
  }

  ObjDetTracker.resize(CamInfo.size());
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  {
    ObjDetTracker[j].Name= CamInfo[j].Name+"-pxv";
    ObjDetTracker[j].Params()= objdettrack_info[j];
    ObjDetTracker[j].Init();
    WindowInfo[ObjDetTracker[j].Name]= TWindowInfo(j, "ObjDetTracker", j);
    if(!WindowsHidden)
    {
      cv::namedWindow(ObjDetTracker[j].Name,1);
      cv::setMouseCallback(ObjDetTracker[j].Name, OnMouse, &ObjDetTracker[j].Name);
    }
    IMShowStuff[ObjDetTracker[j].Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
    ShowTrackbars[ObjDetTracker[j].Name].Kind= "ObjDetTracker";
  }

  SetVideoPrefix(vout_base);

//   BlobPub.resize(BlobTracker.size());
//   for(int j(0),j_end(BlobTracker.size());j<j_end;++j)
//     BlobPub[j]= node.advertise<fingervision_msgs::BlobMoves>(ros::this_node::getNamespace()+"/"+CamInfo[j].Name+"/blob_moves", 1);

//   PXVPub.resize(ObjDetTracker.size());
//   for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
//     PXVPub[j]= node.advertise<fingervision_msgs::ProxVision>(ros::this_node::getNamespace()+"/"+CamInfo[j].Name+"/prox_vision", 1);

//   ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
//   ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);
//   ros::ServiceServer srv_start_record= node.advertiseService("start_record", &StartRecord);
//   ros::ServiceServer srv_stop_record= node.advertiseService("stop_record", &StopRecord);
//   ros::ServiceServer srv_set_video_prefix= node.advertiseService("set_video_prefix", &SetVideoPrefix);
//   ros::ServiceServer srv_set_frame_skip= node.advertiseService("set_frame_skip", &SetFrameSkip);
//   ros::ServiceServer srv_take_snapshot= node.advertiseService("take_snapshot", &TakeSnapshot);
//   ros::ServiceServer srv_stop_detect_obj= node.advertiseService("stop_detect_obj", &StopDetectObj);
//   ros::ServiceServer srv_start_detect_obj= node.advertiseService("start_detect_obj", &StartDetectObj);
//   ros::ServiceServer srv_clear_obj= node.advertiseService("clear_obj", &ClearObj);

  // Dummy capture.
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    cap[i_cam] >> Frame[i_cam];
    CapTime[i_cam]= GetCurrentTimeL();
  }

  // Calibrate ObjDetTracker
  if(ObjDetTracker.size()>0)
  {
    std::vector<std::vector<cv::Mat> > frames(CamInfo.size());
    for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
      frames[i_cam]= CaptureSeq(cap[i_cam], i_cam, ObjDetTracker[i_cam].Params().NCalibBGFrames);
    for(int j(0),j_end(ObjDetTracker.size()); j<j_end; ++j)
      ObjDetTracker[j].CalibBG(frames[j]);
  }

  // Start threads:

  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
    ThBlobTrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecBlobTrack,j))));

  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
    ThObjDetTrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecObjDetTrack,j))));

  // if(publish_image)
  //   for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  //     ThImgPub.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecImgPublisher,j))));

  ThImgCap.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecImgCapture,cap,camera_auto_reopen))));
}
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

