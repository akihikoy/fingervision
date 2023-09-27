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
#include "ay_cpp/cpp_util.h"
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

bool Running(true), Shutdown(false);
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

std::string PkgDir(".");  // Base directory of this node.
std::string BlobCalibPrefix("blob_");
std::string ObjDetModelPrefix("objdet_");
std::string ConfigOut("out1.yaml");  // Default file to save the configuration.
std::vector<TCameraInfo> CamInfo;
std::vector<TBlobTracker2> BlobTracker;  // Marker tracker
std::vector<TObjDetTrackBSP> ObjDetTracker;  // Proximity vision
std::vector<TCameraRectifier> CamRectifier;

std::map<std::string, boost::shared_ptr<boost::mutex> > MutTracker;  // tracker: Mutex for tracker

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
std::vector<TBlobMoves> BlobMovesData;
std::vector<TProxVision> ProxVisionData;
std::map<std::string, boost::shared_ptr<boost::mutex> > DataCopyMutex;
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

// Calibration and initialization request.
bool CalibrationRequest(false), TrackerInitRequest(false), TrackerInitReqFromTrackbar(false);
TWindowInfo CalibrationReqWinInfo, TrackerInitReqWinInfo;

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
    if(CurrentWin!=NULL)
    {
      CalibrationRequest= true;
      CalibrationReqWinInfo= WindowInfo[*CurrentWin];
    }
  }
  else if(c=='i')
  {
    if(CurrentWin!=NULL)
    {
      TrackerInitRequest= true;
      TrackerInitReqWinInfo= WindowInfo[*CurrentWin];
    }
  }
  else if(c=='p')  // || c=='P'
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
  else if(c=='P')  // Save all parameters into ConfigOut.
  {
    std::vector<TBlobTracker2Params> blobtrack_info(BlobTracker.size());
    for(int idx(0),idx_end(BlobTracker.size()); idx<idx_end; ++idx)
      blobtrack_info[idx]= BlobTracker[idx].Params();

    std::vector<TObjDetTrackBSPParams> objdettrack_info(ObjDetTracker.size());
    for(int idx(0),idx_end(ObjDetTracker.size()); idx<idx_end; ++idx)
      objdettrack_info[idx]= ObjDetTracker[idx].Params();

    cv::FileStorage fs(PathJoin(PkgDir,ConfigOut), cv::FileStorage::WRITE);
    WriteToYAML(blobtrack_info, "", &fs);
    WriteToYAML(objdettrack_info, "", &fs);
    fs.release();
    std::cerr<<"Parameters of the trackers are saved into "<<PathJoin(PkgDir,ConfigOut)<<std::endl;
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
      std::cerr<<"Saving calibration data of "<<BlobTracker[idx].Name<<" to "<<PathJoin(PkgDir,BlobCalibPrefix+CamInfo[i_cam].Name+".yaml")<<std::endl;
      BlobTracker[idx].SaveCalib(PathJoin(PkgDir,BlobCalibPrefix+CamInfo[i_cam].Name+".yaml"));
    }
    else if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
    {
      int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
      std::cerr<<"Saving BG+Object models of "<<ObjDetTracker[idx].Name<<" to "<<PathJoin(PkgDir,ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml")<<std::endl;
      ObjDetTracker[idx].SaveBGObjModels(PathJoin(PkgDir,ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml"));
    }
  }
  else if(c=='l')
  {
    if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="BlobTracker")
    {
      int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
      std::cerr<<"Loading calibration data of "<<BlobTracker[idx].Name<<" from "<<PathJoin(PkgDir,BlobCalibPrefix+CamInfo[i_cam].Name+".yaml")<<std::endl;
      BlobTracker[idx].LoadCalib(PathJoin(PkgDir,BlobCalibPrefix+CamInfo[i_cam].Name+".yaml"));
    }
    else if(CurrentWin!=NULL && WindowInfo[*CurrentWin].Kind=="ObjDetTracker")
    {
      int i_cam(WindowInfo[*CurrentWin].CamIdx), idx(WindowInfo[*CurrentWin].Index);
      std::cerr<<"Loading BG+Object models of "<<ObjDetTracker[idx].Name<<" from "<<PathJoin(PkgDir,ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml")<<std::endl;
      ObjDetTracker[idx].LoadBGObjModels(PathJoin(PkgDir,ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml"));
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
      CreateTrackbars(*CurrentWin, BlobTracker[idx].Params(), ShowTrackbars[*CurrentWin].Mode, TrackerInitReqFromTrackbar);
    }
    else if(ShowTrackbars[*CurrentWin].Kind=="ObjDetTracker")
    {
      int idx(WindowInfo[*CurrentWin].Index);
      ++ShowTrackbars[*CurrentWin].Mode;
      CreateTrackbars(*CurrentWin, ObjDetTracker[idx].Params(), ShowTrackbars[*CurrentWin].Mode, TrackerInitReqFromTrackbar);
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
  WindowVisibilityRequest= wvrShow;
}
//-------------------------------------------------------------------------------------------

void HideWindows()
{
  std::cerr<<"Hiding windows..."<<std::endl;
  WindowVisibilityRequest= wvrHide;
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
    Preprocess(frame, CamInfo[i_cam], &CamRectifier[i_cam]);
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

// Set the calibration flag with the specified name of a window.
void SetCalibrationRequest(const std::string &name)
{
  // Search for a tracker whose name matches with the given name.
  for(int it(0),it_end(BlobTracker.size()); it!=it_end; ++it)
  {
    if(BlobTracker[it].Name==name)
    {
      ReqCalibrate("BlobTracker", it);
      return;
    }
  }
  for(int it(0),it_end(ObjDetTracker.size()); it!=it_end; ++it)
  {
    if(ObjDetTracker[it].Name==name)
    {
      ReqCalibrate("ObjDetTracker", it);
      return;
    }
  }
}
//-------------------------------------------------------------------------------------------

bool ReqCalibrate(const std::string &kind, const int &idx)
{
  const int i_cam(idx);
  if(kind=="BlobTracker"
    || kind=="ObjDetTracker")
  {
    std::cerr<<"Calibration requested: "<<kind<<"-"<<idx<<std::endl;
    CalibrationRequest= true;
    CalibrationReqWinInfo.Kind= kind;
    CalibrationReqWinInfo.CamIdx= i_cam;
    CalibrationReqWinInfo.Index= idx;
    return true;
  }
  else
  {
    std::cerr<<"Invalid kind of the calibration requested: "<<kind<<"-"<<idx<<std::endl;
    return false;
  }
}
//-------------------------------------------------------------------------------------------

bool ReqInitialize(const std::string &kind, const int &idx)
{
  const int i_cam(idx);
  if(kind=="BlobTracker"
    || kind=="ObjDetTracker")
  {
    std::cerr<<"Initialization requested: "<<kind<<"-"<<idx<<std::endl;
    TrackerInitRequest= true;
    TrackerInitReqWinInfo.Kind= kind;
    TrackerInitReqWinInfo.CamIdx= i_cam;
    TrackerInitReqWinInfo.Index= idx;
    return true;
  }
  else
  {
    std::cerr<<"Invalid kind of the initialization requested: "<<kind<<"-"<<idx<<std::endl;
    return false;
  }
}
//-------------------------------------------------------------------------------------------

bool SetDimLevel(const std::string &kind, const int &dim_idx_in)
{
  int dim_level_len(sizeof(DimLevels)/sizeof(DimLevels[0]));
  int dim_idx= dim_idx_in;
  if(dim_idx<0)  dim_idx= 0;
  if(dim_idx>=dim_level_len)  dim_idx= dim_level_len-1;
  std::cerr<<"SetDimLevel: "<<kind<<":"<<dim_idx<<std::endl;
  if(kind=="BlobTracker")
  {
    DimIdxBT= dim_idx;
    return true;
  }
  else if(kind=="ObjDetTracker")
  {
    DimIdxPV= dim_idx;
    return true;
  }
  else
  {
    std::cerr<<"Invalid kind: "<<kind<<std::endl;
    return false;
  }
}
//-------------------------------------------------------------------------------------------

bool SetTrackbarMode(const std::string &kind, const int &mode)
{
  std::cerr<<"SetTrackbarMode: "<<kind<<":"<<mode<<std::endl;
  if(kind=="BlobTracker")
  {
    for(int it(0),it_end(BlobTracker.size()); it!=it_end; ++it)
    {
      std::string &win_name(BlobTracker[it].Name);
      if(ShowTrackbars[win_name].Mode!=mode)
      {
        cv::destroyWindow(win_name);
        cv::namedWindow(win_name,1);
        ShowTrackbars[win_name].Mode= mode;
        CreateTrackbars(win_name, BlobTracker[it].Params(), ShowTrackbars[win_name].Mode, TrackerInitReqFromTrackbar);
        cv::setMouseCallback(win_name, OnMouse, &win_name);
      }
    }
    return true;
  }
  else if(kind=="ObjDetTracker")
  {
    for(int it(0),it_end(ObjDetTracker.size()); it!=it_end; ++it)
    {
      std::string &win_name(ObjDetTracker[it].Name);
      if(ShowTrackbars[win_name].Mode!=mode)
      {
        cv::destroyWindow(win_name);
        cv::namedWindow(win_name,1);
        ShowTrackbars[win_name].Mode= mode;
        CreateTrackbars(win_name, ObjDetTracker[it].Params(), ShowTrackbars[win_name].Mode, TrackerInitReqFromTrackbar);
        cv::setMouseCallback(win_name, OnMouse, &win_name);
      }
    }
    return true;
  }
  else
  {
    std::cerr<<"Invalid kind: "<<kind<<std::endl;
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

void SaveParameters(const std::string &file_name_in)
{
  std::string file_name;
  if(file_name_in=="")  file_name= PathJoin(PkgDir,ConfigOut);
  else                  file_name= PathJoin(PkgDir,file_name_in);

  std::vector<TBlobTracker2Params> blobtrack_info(BlobTracker.size());
  for(int idx(0),idx_end(BlobTracker.size()); idx<idx_end; ++idx)
    blobtrack_info[idx]= BlobTracker[idx].Params();

  std::vector<TObjDetTrackBSPParams> objdettrack_info(ObjDetTracker.size());
  for(int idx(0),idx_end(ObjDetTracker.size()); idx<idx_end; ++idx)
    objdettrack_info[idx]= ObjDetTracker[idx].Params();

  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  WriteToYAML(blobtrack_info, "", &fs);
  WriteToYAML(objdettrack_info, "", &fs);
  fs.release();
  std::cerr<<"Parameters of the trackers are saved into "<<file_name<<std::endl;
}
//-------------------------------------------------------------------------------------------

void LoadParameters(const std::string &file_names_in)
{
  std::vector<std::string> file_names;
  file_names= CheckYAMLExistence(PathJoin(PkgDir,SplitString(file_names_in)));
  std::cerr<<"Loading parameters of the trackers from:"<<std::endl;
  for(std::vector<std::string>::const_iterator itr(file_names.begin()),itr_end(file_names.end());
      itr!=itr_end; ++itr)
    std::cerr<<"  - "<<*itr<<std::endl;

  std::vector<TBlobTracker2Params> blobtrack_info;
  std::vector<TObjDetTrackBSPParams> objdettrack_info;
  // ReadFromYAML(CamInfo, file_names);
  ReadFromYAML(blobtrack_info, file_names);
  ReadFromYAML(objdettrack_info, file_names);
  for(int idx(0),idx_end(std::min(blobtrack_info.size(),BlobTracker.size())); idx<idx_end; ++idx)
  {
    boost::mutex::scoped_lock lock(*MutTracker[BlobTracker[idx].Name]);
    BlobTracker[idx].Params()= blobtrack_info[idx];
    BlobTracker[idx].Init();
  }
  for(int idx(0),idx_end(std::min(objdettrack_info.size(),ObjDetTracker.size())); idx<idx_end; ++idx)
  {
    boost::mutex::scoped_lock lock(*MutTracker[ObjDetTracker[idx].Name]);
    ObjDetTracker[idx].Params()= objdettrack_info[idx];
    ObjDetTracker[idx].Init();
  }
}
//-------------------------------------------------------------------------------------------

bool SaveCalibration(const std::string &kind, const int &idx)
{
  const int i_cam(idx);
  if(kind=="BlobTracker")
  {
    std::cerr<<"Saving calibration data of "<<BlobTracker[idx].Name<<" to "<<PathJoin(PkgDir,BlobCalibPrefix+CamInfo[i_cam].Name+".yaml")<<std::endl;
    BlobTracker[idx].SaveCalib(PathJoin(PkgDir,BlobCalibPrefix+CamInfo[i_cam].Name+".yaml"));
    return true;
  }
  else if(kind=="ObjDetTracker")
  {
    std::cerr<<"Saving BG+Object models of "<<ObjDetTracker[idx].Name<<" to "<<PathJoin(PkgDir,ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml")<<std::endl;
    ObjDetTracker[idx].SaveBGObjModels(PathJoin(PkgDir,ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml"));
    return true;
  }
  else
  {
    std::cerr<<"Invalid kind: "<<kind<<std::endl;
    return false;
  }
}
//-------------------------------------------------------------------------------------------

bool LoadCalibration(const std::string &kind, const int &idx)
{
  const int i_cam(idx);
  if(kind=="BlobTracker")
  {
    std::cerr<<"Loading calibration data of "<<BlobTracker[idx].Name<<" from "<<PathJoin(PkgDir,BlobCalibPrefix+CamInfo[i_cam].Name+".yaml")<<std::endl;
    boost::mutex::scoped_lock lock(*MutTracker[BlobTracker[idx].Name]);
    BlobTracker[idx].LoadCalib(PathJoin(PkgDir,BlobCalibPrefix+CamInfo[i_cam].Name+".yaml"));
    return true;
  }
  else if(kind=="ObjDetTracker")
  {
    std::cerr<<"Loading BG+Object models of "<<ObjDetTracker[idx].Name<<" from "<<PathJoin(PkgDir,ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml")<<std::endl;
    boost::mutex::scoped_lock lock(*MutTracker[ObjDetTracker[idx].Name]);
    ObjDetTracker[idx].LoadBGObjModels(PathJoin(PkgDir,ObjDetModelPrefix+CamInfo[i_cam].Name+".yaml"));
    return true;
  }
  else
  {
    std::cerr<<"Invalid kind: "<<kind<<std::endl;
    return false;
  }
}
//-------------------------------------------------------------------------------------------

void ExecBlobTrack(int i_cam)
{
  TCameraInfo &info(CamInfo[i_cam]);
  TBlobTracker2 &tracker(BlobTracker[i_cam]);
  boost::mutex &mut_tracker(*MutTracker[tracker.Name]);
  cv::Mat frame;
  int64_t t_cap(0);
  TRateAdjuster rate(TargetFPS>0.0?TargetFPS:1);
  for(int seq(0); !Shutdown; ++seq)
  {
    if(Running)
    {
      boost::mutex::scoped_lock lock(mut_tracker);

      if(TrackerInitRequest
        && TrackerInitReqWinInfo.Kind=="BlobTracker"
        && TrackerInitReqWinInfo.CamIdx==i_cam
        && TrackerInitReqWinInfo.Index==i_cam)
      {
        tracker.Init();
        TrackerInitRequest= false;
      }
      if(TrackerInitReqFromTrackbar && BlobTracker.size()>0 && CurrentWin!=NULL
        && WindowInfo[*CurrentWin].Kind=="BlobTracker"
        && WindowInfo[*CurrentWin].CamIdx==i_cam
        && WindowInfo[*CurrentWin].Index==i_cam)
      {
        tracker.Init();
        TrackerInitReqFromTrackbar= false;
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

      // Copy to the data buffer.
      {
        const std::vector<TPointMove2> &data(tracker.Data());
        TBlobMoves blob_moves;
        blob_moves.time_stamp= t_cap*1.0e-6;
        blob_moves.frame_id= info.Name;
        blob_moves.camera_index= i_cam;
        blob_moves.camera_name= info.Name;
        blob_moves.width= info.Width;
        blob_moves.height= info.Height;
        blob_moves.data.resize(data.size());
        int i(0);
        for(std::vector<TPointMove2>::const_iterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr,++i)
        {
          TBlobMove &m(blob_moves.data[i]);
          m.Pox= itr->Po.x;
          m.Poy= itr->Po.y;
          m.So = itr->So;
          m.DPx= itr->DP.x;
          m.DPy= itr->DP.y;
          m.DS = itr->DS;
        }

        {
          boost::mutex::scoped_lock lock(*DataCopyMutex[tracker.Name]);
          BlobMovesData[i_cam]= blob_moves;
        }
      }
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
    usleep(100);  // short wait for mutex(mut_tracker).
  }
}
//-------------------------------------------------------------------------------------------

void ExecObjDetTrack(int i_cam)
{
  TCameraInfo &info(CamInfo[i_cam]);
  TObjDetTrackBSP &tracker(ObjDetTracker[i_cam]);
  boost::mutex &mut_tracker(*MutTracker[tracker.Name]);
  cv::Mat frame;
  int64_t t_cap(0);
  TRateAdjuster rate(TargetFPS>0.0?TargetFPS:1);
  for(int seq(0); !Shutdown; ++seq)
  {
    if(Running)
    {
      boost::mutex::scoped_lock lock(mut_tracker);

      if(TrackerInitRequest
        && TrackerInitReqWinInfo.Kind=="ObjDetTracker"
        && TrackerInitReqWinInfo.CamIdx==i_cam
        && TrackerInitReqWinInfo.Index==i_cam)
      {
        tracker.Init();
        TrackerInitRequest= false;
      }
      if(TrackerInitReqFromTrackbar && ObjDetTracker.size()>0 && CurrentWin!=NULL
        && WindowInfo[*CurrentWin].Kind=="ObjDetTracker"
        && WindowInfo[*CurrentWin].CamIdx==i_cam
        && WindowInfo[*CurrentWin].Index==i_cam)
      {
        tracker.Init();
        TrackerInitReqFromTrackbar= false;
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

      // Copy to the data buffer
      {
        const cv::Mat &objs(tracker.ObjS()), &mvs(tracker.MvS());
        const cv::Moments &om(tracker.ObjMoments());
        TProxVision prox_vision;
        prox_vision.time_stamp= t_cap*1.0e-6;
        prox_vision.frame_id= info.Name;
        prox_vision.camera_index= i_cam;
        prox_vision.camera_name= info.Name;
        prox_vision.width= info.Width;
        prox_vision.height= info.Height;

        double m[]= {om.m00, om.m10, om.m01, om.m20, om.m11, om.m02, om.m30, om.m21, om.m12, om.m03};
        std::vector<double> vm(m,m+10);
        prox_vision.ObjM_m= vm;

        double mu[]= {om.mu20, om.mu11, om.mu02, om.mu30, om.mu21, om.mu12, om.mu03};
        std::vector<double> vmu(mu,mu+7);
        prox_vision.ObjM_mu= vmu;

        double nu[]= {om.nu20, om.nu11, om.nu02, om.nu30, om.nu21, om.nu12, om.nu03};
        std::vector<double> vnu(nu,nu+7);
        prox_vision.ObjM_nu= vnu;

        prox_vision.ObjS.resize(objs.rows*objs.cols);
        for(int r(0),rend(objs.rows),i(0);r<rend;++r) for(int c(0),cend(objs.cols);c<cend;++c,++i)
          prox_vision.ObjS[i]= objs.at<float>(r,c);

        prox_vision.MvS.resize(mvs.rows*mvs.cols);
        for(int r(0),rend(mvs.rows),i(0);r<rend;++r) for(int c(0),cend(mvs.cols);c<cend;++c,++i)
          prox_vision.MvS[i]= mvs.at<float>(r,c);

        {
          boost::mutex::scoped_lock lock(*DataCopyMutex[tracker.Name]);
          ProxVisionData[i_cam]= prox_vision;
        }
      }
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
    usleep(100);  // short wait for mutex(mut_tracker).
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

void ExecImgCapture(std::vector<cv::VideoCapture> &cap, bool camera_auto_reopen, bool disp_images, bool handle_key_event)
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

      // NOTE: In case ExecImgCapture is executed as a thread,
      // DisplayImages / cv::imshow should not be performed here but be called from the main thread.
      if(disp_images)  DisplayImages();


      // Handle blob tracker calibration request
      if(CalibrationRequest && BlobTracker.size()>0 && CalibrationReqWinInfo.Kind=="BlobTracker")
      {
        int i_cam(CalibrationReqWinInfo.CamIdx), idx(CalibrationReqWinInfo.Index);
        boost::mutex::scoped_lock lock(*MutTracker[BlobTracker[idx].Name]);
        std::vector<cv::Mat> frames= CaptureSeq(cap[i_cam], i_cam, BlobTracker[idx].Params().NCalibPoints);
        for(int i_frame(0),i_frame_end(frames.size()); i_frame<i_frame_end; ++i_frame)
          Preprocess(frames[i_frame], CamInfo[i_cam], &CamRectifier[i_cam]);
        BlobTracker[idx].Calibrate(frames);
      }
      if(CalibrationRequest && ObjDetTracker.size()>0 && CalibrationReqWinInfo.Kind=="ObjDetTracker")
      {
        int i_cam(CalibrationReqWinInfo.CamIdx), idx(CalibrationReqWinInfo.Index);
        boost::mutex::scoped_lock lock(*MutTracker[ObjDetTracker[idx].Name]);
        std::vector<cv::Mat> frames= CaptureSeq(cap[i_cam], i_cam, ObjDetTracker[idx].Params().NCalibBGFrames);
        for(int i_frame(0),i_frame_end(frames.size()); i_frame<i_frame_end; ++i_frame)
          Preprocess(frames[i_frame], CamInfo[i_cam], &CamRectifier[i_cam]);
        ObjDetTracker[idx].CalibBG(frames);
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

    if(handle_key_event && !HandleKeyEvent())  break;

//     ros::spinOnce();
  }
  Shutdown= true;
}
//-------------------------------------------------------------------------------------------

// Handle the window visibility request and return if the windows are visible.
bool HandleWindowVisibilityRequest()
{
  if(!WindowsHidden && WindowVisibilityRequest==wvrHide)
  {
    for(std::map<std::string, TIMShowStuff>::const_iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
      cv::destroyWindow(itr->first);
    WindowsHidden= true;
  }
  if(WindowsHidden && WindowVisibilityRequest==wvrShow)
  {
    for(std::map<std::string, TIMShowStuff>::const_iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
    {
      std::string &window_name(const_cast<std::string&>(itr->first));
      cv::namedWindow(window_name,1);
      cv::setMouseCallback(window_name, OnMouse, &window_name);
    }
    WindowsHidden= false;
  }
  WindowVisibilityRequest= wvrNone;
  return !WindowsHidden;
}
//-------------------------------------------------------------------------------------------

// Handle the window visibility request, display images with imshow, and run the key event handler.
// return: false if shutdown is requested.
bool DisplayImages()
{
  HandleWindowVisibilityRequest();

  // Show windows
  if(!WindowsHidden)
  {
    for(std::map<std::string, TIMShowStuff>::iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
    {
      boost::mutex::scoped_lock lock(*itr->second.Mutex);
      if(itr->second.Frame.total()>0)
        cv::imshow(itr->first, itr->second.Frame);
    }
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
  HandleWindowVisibilityRequest();
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

// Copy an image for display specified by the window name to dst.
void CopyDispImage(const std::string &name, cv::Mat &dst)
{
  if(IMShowStuff.count(name)==0)
  {
    std::cerr<<"CopyDispImage: No image named "<<name<<std::endl;
    return;
  }
  boost::mutex::scoped_lock lock(*IMShowStuff[name].Mutex);
  if(IMShowStuff[name].Frame.total()>0)
    IMShowStuff[name].Frame.copyTo(dst);
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

// Return the number of cameras.
int GetNumCameras()
{
  return CamInfo.size();
}
//-------------------------------------------------------------------------------------------

// Return the latest BlobMoves data.
TBlobMoves GetBlobMoves(int i_cam)
{
  TBlobTracker2 &tracker(BlobTracker[i_cam]);
  boost::mutex::scoped_lock lock(*DataCopyMutex[tracker.Name]);
  TBlobMoves res= BlobMovesData[i_cam];  // Copy the data.
  return res;
}
//-------------------------------------------------------------------------------------------

// Return the latest ProxVision data.
TProxVision GetProxVision(int i_cam)
{
  TObjDetTrackBSP &tracker(ObjDetTracker[i_cam]);
  boost::mutex::scoped_lock lock(*DataCopyMutex[tracker.Name]);
  TProxVision res= ProxVisionData[i_cam];  // Copy the data.
  return res;
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
    const std::string &config_out,
    const std::string &blob_calib_prefix,
    const std::string &objdet_model_prefix,
    int frame_skip, int target_fps, int capture_fps,
    bool windows_hidden, bool camera_auto_reopen, bool initial_obj_detect)
{
//   ros::init(argc, argv, "fv_core_node");
//   ros::NodeHandle node("~");
//   ros::Duration(0.1).sleep();
  PkgDir= pkg_dir;
  const std::string &cam_config(config);
  const std::string &blobtrack_config(config);
  const std::string &objdettrack_config(config);
  ConfigOut= config_out;
  BlobCalibPrefix= blob_calib_prefix;
  ObjDetModelPrefix= objdet_model_prefix;
  std::string vout_base("/tmp/vout-");
//   bool publish_image(false);
  FrameSkip= frame_skip;
  TargetFPS= target_fps;
  CaptureFPS= capture_fps;
  WindowsHidden= windows_hidden;

  std::cerr<<"pkg_dir: "<<PkgDir<<std::endl;
  std::cerr<<"cam_config: "<<cam_config<<std::endl;
  std::cerr<<"blobtrack_config: "<<blobtrack_config<<std::endl;
  std::cerr<<"objdettrack_config: "<<objdettrack_config<<std::endl;
  std::cerr<<"config_out: "<<ConfigOut<<std::endl;
  std::cerr<<"blob_calib_prefix: "<<BlobCalibPrefix<<std::endl;
  std::cerr<<"objdet_model_prefix: "<<ObjDetModelPrefix<<std::endl;

  std::vector<TBlobTracker2Params> blobtrack_info;
  std::vector<TObjDetTrackBSPParams> objdettrack_info;
  ReadFromYAML(CamInfo, CheckYAMLExistence(PathJoin(PkgDir,SplitString(cam_config))));
  ReadFromYAML(blobtrack_info, CheckYAMLExistence(PathJoin(PkgDir,SplitString(blobtrack_config))));
  ReadFromYAML(objdettrack_info, CheckYAMLExistence(PathJoin(PkgDir,SplitString(objdettrack_config))));

  std::vector<cv::VideoCapture> cap(CamInfo.size());
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
    if(FileExists(PathJoin(PkgDir,BlobCalibPrefix+CamInfo[j].Name+".yaml")))
      BlobTracker[j].LoadCalib(PathJoin(PkgDir,BlobCalibPrefix+CamInfo[j].Name+".yaml"));
    MutTracker[BlobTracker[j].Name]= boost::shared_ptr<boost::mutex>(new boost::mutex);
    WindowInfo[BlobTracker[j].Name]= TWindowInfo(j, "BlobTracker", j);
    if(!WindowsHidden)
    {
      cv::namedWindow(BlobTracker[j].Name,1);
      cv::setMouseCallback(BlobTracker[j].Name, OnMouse, &BlobTracker[j].Name);
    }
    DataCopyMutex[BlobTracker[j].Name]= boost::shared_ptr<boost::mutex>(new boost::mutex);
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
    if(FileExists(PathJoin(PkgDir,ObjDetModelPrefix+CamInfo[j].Name+".yaml")))
    {
      ObjDetTracker[j].LoadBGObjModels(PathJoin(PkgDir,ObjDetModelPrefix+CamInfo[j].Name+".yaml"));
      objdet_model_loaded= true;
    }
    MutTracker[ObjDetTracker[j].Name]= boost::shared_ptr<boost::mutex>(new boost::mutex);
    WindowInfo[ObjDetTracker[j].Name]= TWindowInfo(j, "ObjDetTracker", j);
    if(!WindowsHidden)
    {
      cv::namedWindow(ObjDetTracker[j].Name,1);
      cv::setMouseCallback(ObjDetTracker[j].Name, OnMouse, &ObjDetTracker[j].Name);
    }
    DataCopyMutex[ObjDetTracker[j].Name]= boost::shared_ptr<boost::mutex>(new boost::mutex);
    IMShowStuff[ObjDetTracker[j].Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
    ShowTrackbars[ObjDetTracker[j].Name].Kind= "ObjDetTracker";
  }

  SetVideoPrefix(vout_base);

  BlobMovesData.resize(BlobTracker.size());
//   BlobPub.resize(BlobTracker.size());
//   for(int j(0),j_end(BlobTracker.size());j<j_end;++j)
//     BlobPub[j]= node.advertise<fingervision_msgs::BlobMoves>(ros::this_node::getNamespace()+"/"+CamInfo[j].Name+"/blob_moves", 1);

  ProxVisionData.resize(ObjDetTracker.size());
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

  // Start threads:

  for(int j(0),j_end(BlobTracker.size());j<j_end;++j)
    ThBlobTrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecBlobTrack,j))));

  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    ThObjDetTrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecObjDetTrack,j))));

  // if(publish_image)
  //   for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  //     ThImgPub.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecImgPublisher,j))));

  ThImgCap.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecImgCapture,cap,camera_auto_reopen,/*disp_images=*/false,/*handle_key_event=*/false))));
}
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

