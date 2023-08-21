//-------------------------------------------------------------------------------------------
/*! \file    fv_core_node.cpp
    \brief   Computer vision for FingerVision.
             Improved marker (blob) tracking and proximity vision.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.19, 2017
    \version 0.2
    \date    Apr.1, 2018
             Removed stereo vision (independent from PCL; no point cloud output).
*/
//-------------------------------------------------------------------------------------------
#include "blob_tracker2.h"
#include "prox_vision.h"
#include "ay_vision/vision_util.h"
#include "ay_cpp/geom_util.h"
#include "ay_cpp/sys_util.h"
#include "ay_cpp/cpp_util.h"
//-------------------------------------------------------------------------------------------
#include "fingervision_msgs/BlobMoves.h"
#include "fingervision_msgs/ProxVision.h"
#include "fingervision_msgs/SetInt32.h"
#include "fingervision_msgs/SetString.h"
#include "fingervision_msgs/TakeSnapshot.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
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
namespace trick
{
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
      ros::Rate rate(TargetFPS)
      for each frame:
        rate.sleep()
  TODO:FIXME: Currently TargetFPS is not configurable during run time, but
    changing TargetFPS in execution is useful.
*/
int FrameSkip(0);  // 0: no skip
double TargetFPS(0);  // 0: no FPS control
double CaptureFPS(0);  // 0: no FPS control

std::string BlobCalibPrefix("blob_");
std::string ObjDetModelPrefix("objdet_");
std::vector<TCameraInfo> CamInfo;
std::vector<TBlobTracker2> BlobTracker;  // Marker tracker
std::vector<TObjDetTrackBSP> ObjDetTracker;  // Proximity vision
std::vector<TCameraRectifier> CamRectifier;

std::map<std::string, TEasyVideoOut> VideoOut;
struct TShowTrackbars
{
  // bool Enabled;
  std::string Kind;
  int Mode;
  TShowTrackbars() : /*Enabled(false),*/ Mode(0) {}
};
std::map<std::string, TShowTrackbars> ShowTrackbars;

std::vector<ros::Publisher> BlobPub;
std::vector<ros::Publisher> PXVPub;
std::vector<image_transport::Publisher> ImgPub;  // Image publisher [i_cam]
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
enum {
  wvrNone=0,
  wvrHide,
  wvrShow
} WindowVisibilityRequest(wvrNone);

// Dim-levels.
double DimLevels[]={0.0,0.3,0.7,1.0};
int DimIdxBT(3),DimIdxPV(1);

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

bool IsShutdown()
{
  return Shutdown || ros::isShuttingDown() || !ros::ok();
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
    std::cerr<<"CurrentWin: "<<*CurrentWin<<" (clicked at "<<x<<", "<<y<<")"<<std::endl;
  }

  if(event == cv::EVENT_RBUTTONDOWN && flags == 0)
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  if((event == cv::EVENT_LBUTTONDOWN && (flags & cv::EVENT_FLAG_SHIFTKEY))
    || (event == cv::EVENT_RBUTTONDOWN && (flags & cv::EVENT_FLAG_SHIFTKEY)) )
  {
    bool to_add= (event == cv::EVENT_LBUTTONDOWN);
    if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
    {
      int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
      cv::Mat frame;
      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
      }
      Preprocess(frame, CamInfo[i_cam], &CamRectifier[i_cam]);
      std::cerr<<"ObjDetTracker#"<<idx<<".";
      std::cerr<<(to_add ? "AddToModel" : "RemoveFromModel");
      std::cerr<<": Cam#"<<i_cam<<" "<<frame.size()<<" clicked("<<x<<","<<y<<") bgr="<<frame.at<cv::Vec3b>(y,x)<<std::endl;
      if(to_add)  ObjDetTracker[idx].AddToModel(frame, cv::Point(x,y));
      else        ObjDetTracker[idx].RemoveFromModel(frame, cv::Point(x,y));
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
  else if(c=='i')
  {
    TrackerInitRequest= true;
  }
  else if(c=='p' || c=='P')
  {
    if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="BlobTracker")
    {
      int idx(WindowInfo[*CurrentWin].Index);
      std::vector<TBlobTracker2Params> p;
      p.push_back(BlobTracker[idx].Params());
      std::string file_name((c=='p') ? "/dev/stdout" : "/tmp/blobtr_params.yaml");
      WriteToYAML(p,file_name.c_str());
      std::cerr<<"Parameters of the tracker are saved into "<<file_name<<std::endl;
    }
    else if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
    {
      int idx(WindowInfo[*CurrentWin].Index);
      std::vector<TObjDetTrackBSPParams> p;
      p.push_back(ObjDetTracker[idx].Params());
      std::string file_name((c=='p') ? "/dev/stdout" : "/tmp/objtr_params.yaml");
      WriteToYAML(p,file_name.c_str());
      std::cerr<<"Parameters of the tracker are saved into "<<file_name<<std::endl;
    }
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
    else if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
    {
      int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
      ObjDetTracker[idx].SaveBGObjModels(ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml");
      std::cerr<<"Saved BG+Object models of "<<ObjDetTracker[idx].Name<<" to "<<ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml"<<std::endl;
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
    else if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
    {
      int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
      ObjDetTracker[idx].LoadBGObjModels(ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml");
      std::cerr<<"Loaded BG+Object models of "<<ObjDetTracker[idx].Name<<" from "<<ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml"<<std::endl;
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

bool Pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Paused..."<<std::endl;
  Running= false;
  return true;
}
//-------------------------------------------------------------------------------------------

bool Resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resumed..."<<std::endl;
  Running= true;
  return true;
}
//-------------------------------------------------------------------------------------------

bool ShowWindows(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Showing windows..."<<std::endl;
  WindowVisibilityRequest= wvrShow;
  return true;
}
//-------------------------------------------------------------------------------------------

bool HideWindows(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Hiding windows..."<<std::endl;
  WindowVisibilityRequest= wvrHide;
  return true;
}
//-------------------------------------------------------------------------------------------

bool StartRecord(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  for(std::map<std::string, TEasyVideoOut>::iterator itr(VideoOut.begin()),itr_end(VideoOut.end()); itr!=itr_end; ++itr)
    itr->second.Rec();
  return true;
}
//-------------------------------------------------------------------------------------------

bool StopRecord(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  for(std::map<std::string, TEasyVideoOut>::iterator itr(VideoOut.begin()),itr_end(VideoOut.end()); itr!=itr_end; ++itr)
    itr->second.Stop();
  return true;
}
//-------------------------------------------------------------------------------------------

void SetVideoPrefix(const std::string &file_prefix)
{
  for(int j(0),j_end(BlobTracker.size());j<j_end;++j)
  {
    VideoOut[BlobTracker[j].Name].SetfilePrefix(file_prefix+BlobTracker[j].Name);
    VideoOut[BlobTracker[j].Name+"-orig"].SetfilePrefix(file_prefix+BlobTracker[j].Name+"-orig");
  }
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
  {
    VideoOut[ObjDetTracker[j].Name].SetfilePrefix(file_prefix+ObjDetTracker[j].Name);
    VideoOut[ObjDetTracker[j].Name+"-orig"].SetfilePrefix(file_prefix+ObjDetTracker[j].Name+"-orig");
  }
}
//-------------------------------------------------------------------------------------------

bool SetVideoPrefix(fingervision_msgs::SetString::Request &req, fingervision_msgs::SetString::Response &res)
{
  std::cerr<<"Setting video prefix as "<<req.data<<"..."<<std::endl;
  SetVideoPrefix(req.data);
  res.result= true;
  return true;
}
//-------------------------------------------------------------------------------------------

bool SetFrameSkip(fingervision_msgs::SetInt32::Request &req, fingervision_msgs::SetInt32::Response &res)
{
  std::cerr<<"Setting frame skip as "<<req.data<<"..."<<std::endl;
  FrameSkip= req.data;
  res.result= true;
  return true;
}
//-------------------------------------------------------------------------------------------

bool TakeSnapshot(fingervision_msgs::TakeSnapshot::Request &req, fingervision_msgs::TakeSnapshot::Response &res)
{
  res.files.clear();
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
    Preprocess(frame, CamInfo[i_cam], &CamRectifier[i_cam]);
    std::stringstream ss;
    ss<<t_cap;
    timestamp= ss.str();
    {
      filename= req.prefix+"-"+CamInfo[i_cam].Name+"-"+timestamp+req.ext;
      cv::imwrite(filename, frame);
      res.files.push_back(filename);
      std::cerr<<"Saved a snapshot: "<<filename<<std::endl;
    }
  }
  return true;
}
//-------------------------------------------------------------------------------------------

bool StopDetectObj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Stopping object detection..."<<std::endl;
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    ObjDetTracker[j].StopDetect();
  return true;
}
//-------------------------------------------------------------------------------------------

bool StartDetectObj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Starting object detection..."<<std::endl;
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    ObjDetTracker[j].StartDetect();
  return true;
}
//-------------------------------------------------------------------------------------------

bool ClearObj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Clearing object models..."<<std::endl;
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    ObjDetTracker[j].ClearObject();
  return true;
}
//-------------------------------------------------------------------------------------------

void ExecBlobTrack(int i_cam)
{
  TCameraInfo &info(CamInfo[i_cam]);
  TBlobTracker2 &tracker(BlobTracker[i_cam]);
  cv::Mat frame;
  int64_t t_cap(0);
  ros::Rate rate(TargetFPS>0.0?TargetFPS:1);
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
      if(TargetFPS>0.0)  rate.sleep();

      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
        t_cap= CapTime[i_cam];
      }
      VideoOut[tracker.Name+"-orig"].Step(frame);  // Record original video

      Preprocess(frame, info, &CamRectifier[i_cam]);
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
      {
        const std::vector<TPointMove2> &data(tracker.Data());
        fingervision_msgs::BlobMoves blob_moves;
        blob_moves.header.seq= seq;
        blob_moves.header.stamp= ros::Time::now();
        blob_moves.header.frame_id= info.Name;
        blob_moves.camera_index= i_cam;
        blob_moves.camera_name= info.Name;
        blob_moves.width= info.Width;
        blob_moves.height= info.Height;
        blob_moves.data.resize(data.size());
        int i(0);
        for(std::vector<TPointMove2>::const_iterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr,++i)
        {
          fingervision_msgs::BlobMove &m(blob_moves.data[i]);
          m.Pox= itr->Po.x;
          m.Poy= itr->Po.y;
          m.So = itr->So;
          m.DPx= itr->DP.x;
          m.DPy= itr->DP.y;
          m.DS = itr->DS;
        }
        BlobPub[i_cam].publish(blob_moves);
      }
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
  ros::Rate rate(TargetFPS>0.0?TargetFPS:1);
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
      if(TargetFPS>0.0)  rate.sleep();

      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
        t_cap= CapTime[i_cam];
      }
      VideoOut[tracker.Name+"-orig"].Step(frame);  // Record original video

      Preprocess(frame, info, &CamRectifier[i_cam]);
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
      {
        const cv::Mat &objs(tracker.ObjS()), &mvs(tracker.MvS());
        const cv::Moments &om(tracker.ObjMoments());
        fingervision_msgs::ProxVision prox_vision;
        prox_vision.header.seq= seq;
        prox_vision.header.stamp= ros::Time::now();
        prox_vision.header.frame_id= info.Name;
        prox_vision.camera_index= i_cam;
        prox_vision.camera_name= info.Name;
        prox_vision.width= info.Width;
        prox_vision.height= info.Height;

        double m[]= {om.m00, om.m10, om.m01, om.m20, om.m11, om.m02, om.m30, om.m21, om.m12, om.m03};
        std::vector<float> vm(m,m+10);
        prox_vision.ObjM_m= vm;

        double mu[]= {om.mu20, om.mu11, om.mu02, om.mu30, om.mu21, om.mu12, om.mu03};
        std::vector<float> vmu(mu,mu+7);
        prox_vision.ObjM_mu= vmu;

        double nu[]= {om.nu20, om.nu11, om.nu02, om.nu30, om.nu21, om.nu12, om.nu03};
        std::vector<float> vnu(nu,nu+7);
        prox_vision.ObjM_nu= vnu;

        prox_vision.ObjS.resize(objs.rows*objs.cols);
        for(int r(0),rend(objs.rows),i(0);r<rend;++r) for(int c(0),cend(objs.cols);c<cend;++c,++i)
          prox_vision.ObjS[i]= objs.at<float>(r,c);

        prox_vision.MvS.resize(mvs.rows*mvs.cols);
        for(int r(0),rend(mvs.rows),i(0);r<rend;++r) for(int c(0),cend(mvs.cols);c<cend;++c,++i)
          prox_vision.MvS[i]= mvs.at<float>(r,c);

        PXVPub[i_cam].publish(prox_vision);
      }
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

void ExecImgPublisher(int i_cam)
{
  TCameraInfo &info(CamInfo[i_cam]);
  cv::Mat frame;
  int64_t t_cap(0);
  ros::Rate rate(TargetFPS>0.0?TargetFPS:1);
  for(int seq(0); !Shutdown; ++seq)
  {
    if(Running)
    {
      if(CapTime[i_cam]==t_cap)
      {
        usleep(10*1000);
        continue;
      }
      if(TargetFPS>0.0)  rate.sleep();

      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
        t_cap= CapTime[i_cam];
      }

      Preprocess(frame, info, &CamRectifier[i_cam]);

      // Publish image
      {
        std_msgs::Header header;
        header.stamp= ros::Time::now();
        header.frame_id= info.Name;
        ImgPub[i_cam].publish( cv_bridge::CvImage(header, "bgr8", frame).toImageMsg() );
      }
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

cv::Mat Capture(cv::VideoCapture &cap, int i_cam, bool auto_reopen=true)
{
  TCameraInfo &info(CamInfo[i_cam]);
  cv::Mat frame;
  {
    boost::mutex::scoped_lock lock(*MutCamCapture[i_cam]);
    while(!cap.read(frame))
    {
      if(!auto_reopen || IsShutdown()
        || !CapWaitReopen(info,cap,/*ms_wait=*/1000,/*max_count=*/0,/*check_to_stop=*/IsShutdown))
        return cv::Mat();
    }
  }
  return frame;
}
//-------------------------------------------------------------------------------------------

// Capture a sequence of images.
std::vector<cv::Mat> CaptureSeq(cv::VideoCapture &cap, int i_cam, int num)
{
  std::vector<cv::Mat> frames;
  for(int i(0); i<num; ++i)
    frames.push_back(Capture(cap, i_cam));
  return frames;
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

inline std::vector<std::string> CheckYAMLExistence(const std::vector<std::string> &filenames)
{
  for(std::vector<std::string>::const_iterator fitr(filenames.begin()),fitr_end(filenames.end());
      fitr!=fitr_end; ++fitr)
    CheckYAMLExistence(*fitr);
  return filenames;
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "fv_core_node");
  ros::NodeHandle node("~");
  ros::Duration(0.1).sleep();
  std::string pkg_dir(".");
  std::string cam_config("config/cam1.yaml");
  std::string blobtrack_config("config/cam1.yaml");
  std::string objdettrack_config("config/cam1.yaml");
  std::string blob_calib_prefix("blob_");
  std::string objdet_model_prefix("objdet_");
  std::string vout_base("/tmp/vout-");
  bool camera_auto_reopen(true), publish_image(false), initial_obj_detect(true);

  node.param("pkg_dir",pkg_dir,pkg_dir);
  node.param("cam_config",cam_config,cam_config);
  node.param("blobtrack_config",blobtrack_config,blobtrack_config);
  node.param("objdettrack_config",objdettrack_config,objdettrack_config);
  node.param("blob_calib_prefix",blob_calib_prefix,blob_calib_prefix);
  node.param("objdet_model_prefix",objdet_model_prefix,objdet_model_prefix);
  node.param("vout_base",vout_base,vout_base);
  node.param("frame_skip",FrameSkip,FrameSkip);
  node.param("target_fps",TargetFPS,TargetFPS);
  node.param("capture_fps",CaptureFPS,CaptureFPS);
  node.param("camera_auto_reopen",camera_auto_reopen,camera_auto_reopen);
  node.param("windows_hidden",WindowsHidden,WindowsHidden);
  node.param("publish_image",publish_image,publish_image);
  node.param("initial_obj_detect",initial_obj_detect,initial_obj_detect);
  std::cerr<<"pkg_dir: "<<pkg_dir<<std::endl;
  std::cerr<<"cam_config: "<<cam_config<<std::endl;
  std::cerr<<"blobtrack_config: "<<blobtrack_config<<std::endl;
  std::cerr<<"objdettrack_config: "<<objdettrack_config<<std::endl;
  std::cerr<<"blob_calib_prefix: "<<blob_calib_prefix<<std::endl;
  std::cerr<<"objdet_model_prefix: "<<objdet_model_prefix<<std::endl;

  std::vector<TBlobTracker2Params> blobtrack_info;
  std::vector<TObjDetTrackBSPParams> objdettrack_info;
  ReadFromYAML(CamInfo, CheckYAMLExistence(PathJoin(pkg_dir,SplitString(cam_config))));
  ReadFromYAML(blobtrack_info, CheckYAMLExistence(PathJoin(pkg_dir,SplitString(blobtrack_config))));
  ReadFromYAML(objdettrack_info, CheckYAMLExistence(PathJoin(pkg_dir,SplitString(objdettrack_config))));
  BlobCalibPrefix= pkg_dir+"/"+blob_calib_prefix;
  ObjDetModelPrefix= pkg_dir+"/"+objdet_model_prefix;

  std::vector<cv::VideoCapture> cap(CamInfo.size());
  CamRectifier.resize(CamInfo.size());
  Frame.resize(CamInfo.size());
  CapTime.resize(CamInfo.size());
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    TCameraInfo &info(CamInfo[i_cam]);
    if(!CapOpen(info, cap[i_cam]))  return -1;
    MutCamCapture.push_back(boost::shared_ptr<boost::mutex>(new boost::mutex));
    MutFrameCopy.push_back(boost::shared_ptr<boost::mutex>(new boost::mutex));

    if(info.Rectification)
    {
      // Setup rectification
      cv::Size size_in(info.Width,info.Height), size_out(info.Width,info.Height);
      CamRectifier[i_cam].Setup(info.K, info.D, info.R, size_in, info.Alpha, size_out);
    }
  }
  std::cerr<<"Opened camera(s)"<<std::endl;

  BlobTracker.resize(blobtrack_info.size());
  for(int j(0),j_end(blobtrack_info.size());j<j_end;++j)
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

  ObjDetTracker.resize(objdettrack_info.size());
  bool objdet_model_loaded(false);
  for(int j(0),j_end(objdettrack_info.size());j<j_end;++j)
  {
    ObjDetTracker[j].Name= CamInfo[j].Name+"-pxv";
    ObjDetTracker[j].Params()= objdettrack_info[j];
    ObjDetTracker[j].Init();
    if(initial_obj_detect)  ObjDetTracker[j].StartDetect();
    else                    ObjDetTracker[j].StopDetect();
    if(FileExists(ObjDetModelPrefix+CamInfo[j].Name+".yaml"))
    {
      ObjDetTracker[j].LoadBGObjModels(ObjDetModelPrefix+CamInfo[j].Name+".yaml");
      objdet_model_loaded= true;
    }
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

  BlobPub.resize(BlobTracker.size());
  for(int j(0),j_end(BlobTracker.size());j<j_end;++j)
    BlobPub[j]= node.advertise<fingervision_msgs::BlobMoves>(ros::this_node::getNamespace()+"/"+CamInfo[j].Name+"/blob_moves", 1);

  PXVPub.resize(ObjDetTracker.size());
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    PXVPub[j]= node.advertise<fingervision_msgs::ProxVision>(ros::this_node::getNamespace()+"/"+CamInfo[j].Name+"/prox_vision", 1);

  image_transport::ImageTransport imgtr(node);
  typedef boost::shared_ptr<camera_info_manager::CameraInfoManager> CamInfoMngrPtr;
  std::vector<CamInfoMngrPtr> info_manager;
  if(publish_image)
  {
    ImgPub.resize(CamInfo.size());
    info_manager.resize(CamInfo.size());
    for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
    {
      ImgPub[i_cam]= imgtr.advertise(ros::this_node::getNamespace()+"/"+CamInfo[i_cam].Name, 1);
      info_manager[i_cam]= CamInfoMngrPtr(new camera_info_manager::CameraInfoManager(ros::NodeHandle("/"+CamInfo[i_cam].Name), CamInfo[i_cam].Name, /*camera_info_url=*/""));
    }
  }

  ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
  ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);
  ros::ServiceServer srv_show_windows= node.advertiseService("show_windows", &ShowWindows);
  ros::ServiceServer srv_hide_windows= node.advertiseService("hide_windows", &HideWindows);
  ros::ServiceServer srv_start_record= node.advertiseService("start_record", &StartRecord);
  ros::ServiceServer srv_stop_record= node.advertiseService("stop_record", &StopRecord);
  ros::ServiceServer srv_set_video_prefix= node.advertiseService("set_video_prefix", &SetVideoPrefix);
  ros::ServiceServer srv_set_frame_skip= node.advertiseService("set_frame_skip", &SetFrameSkip);
  ros::ServiceServer srv_take_snapshot= node.advertiseService("take_snapshot", &TakeSnapshot);
  ros::ServiceServer srv_stop_detect_obj= node.advertiseService("stop_detect_obj", &StopDetectObj);
  ros::ServiceServer srv_start_detect_obj= node.advertiseService("start_detect_obj", &StartDetectObj);
  ros::ServiceServer srv_clear_obj= node.advertiseService("clear_obj", &ClearObj);

  int show_fps(0);

  // Dummy capture.
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    cap[i_cam] >> Frame[i_cam];
    CapTime[i_cam]= GetCurrentTimeL();
  }

  // Calibrate ObjDetTracker
  if(ObjDetTracker.size()>0 && !objdet_model_loaded)
  {
    std::vector<std::vector<cv::Mat> > frames(ObjDetTracker.size());
    for(int i_cam(0), i_cam_end(ObjDetTracker.size()); i_cam<i_cam_end; ++i_cam)
    {
      frames[i_cam]= CaptureSeq(cap[i_cam], i_cam, ObjDetTracker[i_cam].Params().NCalibBGFrames);
      for(int i_frame(0),i_frame_end(frames[i_cam].size()); i_frame<i_frame_end; ++i_frame)
        Preprocess(frames[i_cam][i_frame], CamInfo[i_cam], &CamRectifier[i_cam]);
    }
    for(int j(0),j_end(ObjDetTracker.size()); j<j_end; ++j)
      ObjDetTracker[j].CalibBG(frames[j]);
  }

  std::vector<boost::shared_ptr<boost::thread> > th_blobtrack;
  for(int j(0),j_end(BlobTracker.size());j<j_end;++j)
    th_blobtrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecBlobTrack,j))));

  std::vector<boost::shared_ptr<boost::thread> > th_objdettrack;
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    th_objdettrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecObjDetTrack,j))));

  std::vector<boost::shared_ptr<boost::thread> > th_imgpub;
  if(publish_image)
    for(int j(0),j_end(CamInfo.size());j<j_end;++j)
      th_imgpub.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecImgPublisher,j))));

  ros::Rate rate(CaptureFPS>0.0?CaptureFPS:1);
  for(int f(0);!IsShutdown();++f)
  {
    if(Running)
    {
      // Capture from cameras:
      for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
      {
        cv::Mat frame= Capture(cap[i_cam], i_cam, camera_auto_reopen);
        if(frame.empty())  Shutdown=true;
        if(FrameSkip<=0 || f%(FrameSkip+1)==0)
        {
          boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
          Frame[i_cam]= frame;
          CapTime[i_cam]= GetCurrentTimeL();
        }
      }
      if(IsShutdown())  break;

      // Show windows
      if(!WindowsHidden)
      {
        bool hide_req= (WindowVisibilityRequest==wvrHide);
        for(std::map<std::string, TIMShowStuff>::iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
        {
          boost::mutex::scoped_lock lock(*itr->second.Mutex);
          if(itr->second.Frame.total()>0 && !hide_req)
            cv::imshow(itr->first, itr->second.Frame);
          if(hide_req)
            cv::destroyWindow(itr->first);
        }
        if(hide_req)
        {
          WindowVisibilityRequest= wvrNone;
          WindowsHidden= true;
        }
      }
      if(WindowsHidden && WindowVisibilityRequest==wvrShow)
      {
        for(std::map<std::string, TWindowInfo>::const_iterator itr(WindowInfo.begin()),itr_end(WindowInfo.end()); itr!=itr_end; ++itr)
        {
          std::string &window_name(const_cast<std::string&>(itr->first));
          cv::namedWindow(window_name,1);
          cv::setMouseCallback(window_name, OnMouse, &window_name);
        }
        WindowVisibilityRequest= wvrNone;
        WindowsHidden= false;
      }

      // Handle blob tracker calibration request
      if(CalibrationRequest && BlobTracker.size()>0 && CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="BlobTracker")
      {
        Running= false;
        int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
        std::vector<cv::Mat> frames= CaptureSeq(cap[i_cam], i_cam, BlobTracker[idx].Params().NCalibPoints);
        for(int i_frame(0),i_frame_end(frames.size()); i_frame<i_frame_end; ++i_frame)
          Preprocess(frames[i_frame], CamInfo[i_cam], &CamRectifier[i_cam]);
        BlobTracker[idx].Calibrate(frames);
        Running= true;
      }
      if(CalibrationRequest && ObjDetTracker.size()>0 && CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
      {
        Running= false;
        int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
        std::vector<cv::Mat> frames= CaptureSeq(cap[i_cam], i_cam, ObjDetTracker[idx].Params().NCalibBGFrames);
        for(int i_frame(0),i_frame_end(frames.size()); i_frame<i_frame_end; ++i_frame)
          Preprocess(frames[i_frame], CamInfo[i_cam], &CamRectifier[i_cam]);
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

      if(CaptureFPS>0.0)  rate.sleep();

    }  // Running
    else
    {
      usleep(200*1000);
    }

    if(!HandleKeyEvent())  break;

    ros::spinOnce();
  }
  Shutdown= true;
  for(int j(0),j_end(th_blobtrack.size());j<j_end;++j)
    th_blobtrack[j]->join();
  for(int j(0),j_end(th_objdettrack.size());j<j_end;++j)
    th_objdettrack[j]->join();
  if(publish_image)
    for(int j(0),j_end(th_imgpub.size());j<j_end;++j)
      th_imgpub[j]->join();

  usleep(500*1000);

  return 0;
}
//-------------------------------------------------------------------------------------------
