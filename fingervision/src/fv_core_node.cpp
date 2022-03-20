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
#ifdef WITH_STEREO
  #include "ay_vision/usb_stereo.h"
  #include "ay_3dvision/pcl_util.h"
#endif
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

// Dim-levels.
double DimLevels[]={0.0,0.3,0.7,1.0};
int DimIdxBT(3),DimIdxPV(1);

#ifdef WITH_STEREO
std::vector<TStereoInfo> StereoInfo;
std::vector<TStereo> Stereo;  // Standard stereo
std::vector<TStereo> StereoB;  // Stereo for blob
std::vector<ros::Publisher> CloudPub;
#endif
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
  else if(c=='C' && CurrentWin!=NULL)
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
  #ifdef WITH_STEREO
  for(int j(0),j_end(Stereo.size());j<j_end;++j)
  {
    VideoOut[info.Name].SetfilePrefix(file_prefix+info.Name);
  }
  #endif
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

#ifdef WITH_STEREO
// Get point cloud by stereo
void ExecStereo(int i_stereo)
{
  TStereo &stereo(Stereo[i_stereo]);
  TStereoInfo &info(StereoInfo[i_stereo]);
  cv::Mat frame[2];
  cv::Mat disparity;
  ros::Rate rate(TargetFPS>0.0?TargetFPS:1);
  while(!Shutdown)
  {
    if(Running)
    {
      if(TargetFPS>0.0)  rate.sleep();
      {
        boost::mutex::scoped_lock lock1(*MutFrameCopy[info.CamL]);
        boost::mutex::scoped_lock lock2(*MutFrameCopy[info.CamR]);
        Frame[info.CamL].copyTo(frame[0]);
        Frame[info.CamR].copyTo(frame[1]);
      }
      stereo.Proc(frame[0],frame[1]);
      cv::normalize(stereo.Disparity(), disparity, 0, 255, CV_MINMAX, CV_8U);

      // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2
      // if(ImgWin[2]=='1')  cv::imshow("stereo_frame_l", stereo.FrameL());
      // if(ImgWin[3]=='1')  cv::imshow("stereo_frame_r", stereo.FrameR());
      // if(ImgWin[4]=='1')  cv::imshow("stereo_disparity", disparity);
      {
        // cv::imshow(info.Name, disparity);
        boost::mutex::scoped_lock lock(*IMShowStuff[info.Name].Mutex);
        disparity.copyTo(IMShowStuff[info.Name].Frame);
      }

      // Publish as point cloud.
      stereo.ReprojectTo3D();
      sensor_msgs::PointCloud2 cloud_msg;
      ConvertPointCloudToROSMsg<pcl::PointXYZRGB>(cloud_msg,
          ConvertXYZImageToPointCloud(stereo.XYZ(),stereo.RGB()),
          /*frame_id=*/CamInfo[info.CamL].Name);
      CloudPub[i_stereo].publish(cloud_msg);
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
#endif // WITH_STEREO
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

      CamRectifier[i_cam](frame);

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

inline std::string CheckYAMLExistence(const std::string &filename)
{
  std::cerr<<"Loading from YAML: "<<filename<<std::endl;
  if(!FileExists(filename))
    std::cerr<<"!!!File not found: "<<filename<<std::endl;
  return filename;
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
  std::string vout_base("/tmp/vout-");
  bool camera_auto_reopen(true), publish_image(false);

  node.param("pkg_dir",pkg_dir,pkg_dir);
  node.param("cam_config",cam_config,cam_config);
  node.param("blobtrack_config",blobtrack_config,blobtrack_config);
  node.param("objdettrack_config",objdettrack_config,objdettrack_config);
  node.param("blob_calib_prefix",blob_calib_prefix,blob_calib_prefix);
  node.param("vout_base",vout_base,vout_base);
  node.param("frame_skip",FrameSkip,FrameSkip);
  node.param("target_fps",TargetFPS,TargetFPS);
  node.param("capture_fps",CaptureFPS,CaptureFPS);
  node.param("camera_auto_reopen",camera_auto_reopen,camera_auto_reopen);
  node.param("publish_image",publish_image,publish_image);
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

  #ifdef WITH_STEREO
  std::string stereo_config("config/cam1.yaml");
  node.param("stereo_config",stereo_config,stereo_config);
  std::cerr<<"stereo_config: "<<stereo_config<<std::endl;
  ReadFromYAML(StereoInfo, CheckYAMLExistence(pkg_dir+"/"+stereo_config));
  #endif

  std::vector<cv::VideoCapture> cap(CamInfo.size());
  SingleCamRectifier.resize(CamInfo.size());
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

  #ifdef WITH_STEREO
  Stereo.resize(StereoInfo.size());
  StereoB.resize(StereoInfo.size());
  for(int j(0),j_end(Stereo.size());j<j_end;++j)
  {
    const TStereoInfo &info(StereoInfo[j]);
    Stereo[j].LoadCameraParametersFromYAML(pkg_dir+"/"+info.StereoParam);
    Stereo[j].SetImageSize(
        cv::Size(CamInfo[info.CamL].Width,CamInfo[info.CamL].Height),
        cv::Size(info.Width,info.Height) );
    Stereo[j].SetRecommendedStereoParams();
    Stereo[j].LoadConfigurationsFromYAML(pkg_dir+"/"+info.StereoConfig);
    Stereo[j].Init();
    WindowInfo[info.Name]= TWindowInfo(-1, "Stereo", j);
    cv::namedWindow(info.Name,1);
    cv::setMouseCallback(info.Name, OnMouse, const_cast<std::string*>(&info.Name));
    IMShowStuff[info.Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
    ShowTrackbars[info.Name].Kind= "Stereo";

    StereoB[j].LoadCameraParametersFromYAML(pkg_dir+"/"+info.StereoParam);
    StereoB[j].SetImageSize(
        cv::Size(CamInfo[info.CamL].Width,CamInfo[info.CamL].Height),
        cv::Size(CamInfo[info.CamL].Width,CamInfo[info.CamL].Height) );
    StereoB[j].SetRecommendedStereoParams();
    StereoB[j].LoadConfigurationsFromYAML(pkg_dir+"/"+info.StereoConfig);
    StereoB[j].Init();
    CamRectifier[info.CamL]= boost::bind(&TStereo::RectifyL, StereoB[j], _1, /*gray_scale=*/false);
    CamRectifier[info.CamR]= boost::bind(&TStereo::RectifyR, StereoB[j], _1, /*gray_scale=*/false);
  }
  #endif

  BlobTracker.resize(CamInfo.size());
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  {
    BlobTracker[j].Name= CamInfo[j].Name+"-blob";
    BlobTracker[j].Params()= blobtrack_info[j];
    BlobTracker[j].Init();
    if(FileExists(BlobCalibPrefix+CamInfo[j].Name+".yaml"))
      BlobTracker[j].LoadCalib(BlobCalibPrefix+CamInfo[j].Name+".yaml");
    WindowInfo[BlobTracker[j].Name]= TWindowInfo(j, "BlobTracker", j);
    cv::namedWindow(BlobTracker[j].Name,1);
    cv::setMouseCallback(BlobTracker[j].Name, OnMouse, &BlobTracker[j].Name);
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
    cv::namedWindow(ObjDetTracker[j].Name,1);
    cv::setMouseCallback(ObjDetTracker[j].Name, OnMouse, &ObjDetTracker[j].Name);
    IMShowStuff[ObjDetTracker[j].Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
    ShowTrackbars[ObjDetTracker[j].Name].Kind= "ObjDetTracker";
  }

  SetVideoPrefix(vout_base);

  #ifdef WITH_STEREO
  CloudPub.resize(Stereo.size());
  for(int j(0),j_end(Stereo.size());j<j_end;++j)
    CloudPub[j]= node.advertise<sensor_msgs::PointCloud2>(ros::this_node::getNamespace()+"/"+StereoInfo[j].Name+"/point_cloud", 1);
  #endif

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
  if(ObjDetTracker.size()>0)
  {
    std::vector<std::vector<cv::Mat> > frames(CamInfo.size());
    for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
      frames[i_cam]= CaptureSeq(cap[i_cam], i_cam, ObjDetTracker[i_cam].Params().NCalibBGFrames);
    for(int j(0),j_end(ObjDetTracker.size()); j<j_end; ++j)
      ObjDetTracker[j].CalibBG(frames[j]);
  }

  std::vector<boost::shared_ptr<boost::thread> > th_blobtrack;
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
    th_blobtrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecBlobTrack,j))));

  std::vector<boost::shared_ptr<boost::thread> > th_objdettrack;
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
    th_objdettrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecObjDetTrack,j))));

  std::vector<boost::shared_ptr<boost::thread> > th_imgpub;
  if(publish_image)
    for(int j(0),j_end(CamInfo.size());j<j_end;++j)
      th_imgpub.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecImgPublisher,j))));

  #ifdef WITH_STEREO
  std::vector<boost::shared_ptr<boost::thread> > th_stereo;
  for(int j(0),j_end(StereoInfo.size());j<j_end;++j)
    th_stereo.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecStereo,j))));
  #endif

  ros::Rate rate(CaptureFPS>0.0?CaptureFPS:1);
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

      // Show windows
      for(std::map<std::string, TIMShowStuff>::iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
      {
        boost::mutex::scoped_lock lock(*itr->second.Mutex);
        if(itr->second.Frame.total()>0)
          cv::imshow(itr->first, itr->second.Frame);
      }

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
  #ifdef WITH_STEREO
  for(int j(0),j_end(th_stereo.size());j<j_end;++j)
    th_stereo[j]->join();
  #endif

  usleep(500*1000);

  return 0;
}
//-------------------------------------------------------------------------------------------
