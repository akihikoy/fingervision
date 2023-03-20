//-------------------------------------------------------------------------------------------
/*! \file    prox_vision.cpp
    \brief   Slip and deformation detection by detecting and tracking a proximity object.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.19, 2017
    \version 0.2
    \date    Sep.26, 2018
             Improved the detection of darker objects.

cf. testl/cv/obj_det_track3.cpp
*/
//-------------------------------------------------------------------------------------------
#include "prox_vision.h"
#include "ay_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/imgproc/imgproc.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;


// Shrink image by area-based interpolation.
// Similar to cv::resize with INTER_AREA, but this function returns data as a float matrix.
static void Shrink(const cv::Mat &src, cv::Mat &dst, const cv::Size &size)
{
  dst.create(size, CV_32FC1);
  int dx(float(src.cols)/float(dst.cols));
  int dy(float(src.rows)/float(dst.rows));
  float area(dx*dy);
  for(int r(0),rend(dst.rows);r<rend;++r)
    for(int c(0),cend(dst.cols);c<cend;++c)
      dst.at<float>(r,c)= cv::sum(src(cv::Rect(c*dx,r*dy,dx,dy)))[0]/area;
}
//-------------------------------------------------------------------------------------------


TObjDetTrackBSPParams::TObjDetTrackBSPParams()
{
  // Resize image for speed up
  Width= 160;
  Height= 120;

  // Remove black background
  ThreshBlkH= 180;
  ThreshBlkS= 255;
  ThreshBlkV= 1;

  // Parameters of BackgroundSubtractorMOG2
  BS_History= 30.0;
  BS_VarThreshold= 10.0;
  BS_DetectShadows= true;

  // Background subtraction and post process
  NErode1= 0;  // Size of erode applied to the background subtraction result.

  // Histogram bins (Hue, Saturation)
  BinsH= 50 ;
  BinsS= 10 ;
  BinsV= 10 ;

  // Object detection parameters
  Fbg= 1.0  ;  // Degree to remove background model (histogram). Larger value removes more.
  Fgain= 2.5;  // Learning rate.

  NumModel= 5         ;  // Number of object models to be maintained.
  NumFramesInHist= 200;  // Number of frames to make an object histogram.

  NoNewModel= false;  // Even when ModeDetect==True, object models are not added if their number is NumModel.

  BackProjScale= 5.0;  // Scale parameter of calcBackProject

  // Object tracking parameters
  NThreshold2= 150 ; // Threshold applied to back projection.
  NErode2= 2       ;
  NDilate2= 7      ;
  NCalibBGFrames= 3;  // Number of frames to make a background histogram.
  NUpDiff= 20;  // Upper color diff of floodFill in AddToModel.
  NLoDiff= 20;  // Lower color diff of floodFill in AddToModel.

  // Simplifying detected object and movement for easy use
  ObjSW= 3;  ObjSH= 3;  // Size of shrunken object data
  MvSW= 3;  MvSH= 3;  // Size of shrunken movement data
}
//-------------------------------------------------------------------------------------------

void WriteToYAML(const std::vector<TObjDetTrackBSPParams> &params, const std::string &file_name)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs<<"ObjDetTrack"<<"[";
  for(std::vector<TObjDetTrackBSPParams>::const_iterator itr(params.begin()),itr_end(params.end()); itr!=itr_end; ++itr)
  {
    fs<<"{";
    #define PROC_VAR(x)  fs<<#x<<itr->x;
    PROC_VAR(Width              );
    PROC_VAR(Height             );
    PROC_VAR(ThreshBlkH         );
    PROC_VAR(ThreshBlkS         );
    PROC_VAR(ThreshBlkV         );
    PROC_VAR(BS_History         );
    PROC_VAR(BS_VarThreshold    );
    PROC_VAR(BS_DetectShadows   );
    PROC_VAR(NErode1            );
    PROC_VAR(BinsH              );
    PROC_VAR(BinsS              );
    PROC_VAR(BinsV              );
    PROC_VAR(Fbg                );
    PROC_VAR(Fgain              );
    PROC_VAR(NumModel           );
    PROC_VAR(NumFramesInHist    );
    PROC_VAR(NoNewModel         );
    PROC_VAR(BackProjScale      );
    PROC_VAR(NThreshold2        );
    PROC_VAR(NErode2            );
    PROC_VAR(NDilate2           );
    PROC_VAR(NCalibBGFrames     );
    PROC_VAR(NUpDiff            );
    PROC_VAR(NLoDiff            );
    PROC_VAR(ObjSW              );
    PROC_VAR(ObjSH              );
    PROC_VAR(MvSW               );
    PROC_VAR(MvSH               );
    fs<<"}";
    #undef PROC_VAR
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<TObjDetTrackBSPParams> &params, const std::string &file_name)
{
  params.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs["ObjDetTrack"];
  if(data.empty())
  {
    fs.release();
    return;
  }
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    TObjDetTrackBSPParams cf;
    #define PROC_VAR(x)  if(!(*itr)[#x].empty())  (*itr)[#x]>>cf.x;
    PROC_VAR(Width              );
    PROC_VAR(Height             );
    PROC_VAR(ThreshBlkH         );
    PROC_VAR(ThreshBlkS         );
    PROC_VAR(ThreshBlkV         );
    PROC_VAR(BS_History         );
    PROC_VAR(BS_VarThreshold    );
    PROC_VAR(BS_DetectShadows   );
    PROC_VAR(NErode1            );
    PROC_VAR(BinsH              );
    PROC_VAR(BinsS              );
    PROC_VAR(BinsV              );
    PROC_VAR(Fbg                );
    PROC_VAR(Fgain              );
    PROC_VAR(NumModel           );
    PROC_VAR(NumFramesInHist    );
    PROC_VAR(NoNewModel         );
    PROC_VAR(BackProjScale      );
    PROC_VAR(NThreshold2        );
    PROC_VAR(NErode2            );
    PROC_VAR(NDilate2           );
    PROC_VAR(NCalibBGFrames     );
    PROC_VAR(NUpDiff            );
    PROC_VAR(NLoDiff            );
    PROC_VAR(ObjSW              );
    PROC_VAR(ObjSH              );
    PROC_VAR(MvSW               );
    PROC_VAR(MvSH               );
    #undef PROC_VAR
    params.push_back(cf);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// class TObjDetTrackBSP
//-------------------------------------------------------------------------------------------

void TObjDetTrackBSP::Init()
{
  bkg_sbtr_= cv::createBackgroundSubtractorMOG2(params_.BS_History, params_.BS_VarThreshold, params_.BS_DetectShadows);
  i_frame_= 0;
}
//-------------------------------------------------------------------------------------------

void TObjDetTrackBSP::Step(const cv::Mat &frame)
{
  float h_range[]= {0, 180};
  float s_range[]= {0, 256};
  float v_range[]= {0, 256};
  const float* ranges[]= {h_range, s_range, v_range};
  int channels[]= {0, 1, 2}, n_channels= 3;

  cv::Mat frame_s;
  if(params_.Width*params_.Height>0)
    cv::resize(frame, frame_s, cv::Size(params_.Width,params_.Height), 0, 0, cv::INTER_NEAREST);
  else
    frame_s= frame;

  cv::Mat frame_hsv, mask_nonblack;
  cv::cvtColor(frame_s, frame_hsv, cv::COLOR_BGR2HSV);
  cv::inRange(frame_hsv, cv::Scalar(0, 0, 0), cv::Scalar(params_.ThreshBlkH, params_.ThreshBlkS, params_.ThreshBlkV), mask_nonblack);
  std::cerr<<"debug/mask_nonblack/nonzero:"<<cv::countNonZero(mask_nonblack)<<std::endl;
  mask_nonblack= 255-mask_nonblack;

  bkg_sbtr_->apply(frame_s, mask_bs_, 1./float(params_.BS_History));
  mask_bs_&= mask_nonblack;  // Remove black background
  cv::erode(mask_bs_, mask_bser_, cv::Mat(), cv::Point(-1,-1), params_.NErode1);
  mask_bser_&= mask_nonblack;  // Remove black background

  // Object detection mode:
  if(mode_detect_ && !hist_bg_.empty())
  {
    // Get the Histogram and normalize it
    cv::Mat hist_tmp;
    int hist_size[]= {params_.BinsH, params_.BinsS, params_.BinsV};
    cv::calcHist(&frame_hsv, 1, channels, mask_bser_, hist_tmp, n_channels, hist_size, ranges, true, false);

    // Add/delete model
    if(params_.NoNewModel && int(hist_obj_.size())==params_.NumModel)
    {
      // No more new models are added to hist_obj_
    }
    else
    {
      int interval= params_.NumFramesInHist / params_.NumModel;
      if(hist_obj_.size()==0 || i_frame_%interval==0)
      {
        cv::Mat hist(n_channels, hist_size, hist_tmp.type());
        hist.setTo(0);
        hist_obj_.push_front(hist);
        if(int(hist_obj_.size()) > params_.NumModel)  hist_obj_.pop_back();
      }
    }
    // Update model
    if(params_.Fgain>0.0)
    {
      for(std::list<cv::Mat>::iterator ho_itr(hist_obj_.begin()),ho_itr_end(hist_obj_.end());
          ho_itr!=ho_itr_end; ++ho_itr)
      {
        cv::Mat &hist(*ho_itr);
        // Add current histogram to the model, and
        // Normalize the histogram
        for(int h(0);h<params_.BinsH;++h) for(int s(0);s<params_.BinsS;++s) for(int v(0);v<params_.BinsV;++v)
        {
          hist.at<float>(h,s,v)+= params_.Fgain*std::max(0.0f,hist_tmp.at<float>(h,s,v)-params_.Fbg*hist_bg_.at<float>(h,s,v));
          if(hist.at<float>(h,s,v)>255)  hist.at<float>(h,s,v)= 255;
        }
      }
    }
  }

  // Track object:
  if(hist_obj_.size()>0)
  {
    cv::Mat &hist(hist_obj_.back());
    cv::Mat backproj;
    cv::calcBackProject(&frame_hsv, 1, channels, hist, backproj, ranges, params_.BackProjScale, true);
    backproj.copyTo(mask_obj1_);
    mask_obj1_&= mask_nonblack;  // Remove black background
    // std::cerr<<mask_obj1_<<std::endl;
    cv::threshold(mask_obj1_, mask_obj1_, params_.NThreshold2, 0, cv::THRESH_TOZERO);
    cv::erode(mask_obj1_,mask_obj1_,cv::Mat(),cv::Point(-1,-1), params_.NErode2);
    cv::dilate(mask_obj1_,mask_obj2_,cv::Mat(),cv::Point(-1,-1), params_.NDilate2);
    mask_mv_.setTo(0);

    mask_bs_.copyTo(mask_mv_, mask_obj2_);

    obj_mu_= cv::moments(mask_obj1_, /*binaryImage=*/true);

    // Shrink object and movement data
    // cv::resize(mask_obj1_, obj_, cv::Size(3,3), 0, 0, cv::INTER_AREA);
    // cv::resize(mask_mv_, mv_, cv::Size(3,3), 0, 0, cv::INTER_AREA);
    Shrink(mask_obj1_, obj_, cv::Size(params_.ObjSW, params_.ObjSH));
    Shrink(mask_mv_, mv_, cv::Size(params_.MvSW, params_.MvSH));
  }
  else
  {
    mask_mv_= cv::Mat();
    mask_obj1_= cv::Mat();
    mask_obj2_= cv::Mat();
  }

  ++i_frame_;
}
//-------------------------------------------------------------------------------------------

void TObjDetTrackBSP::Draw(cv::Mat &frame)
{
  // cv::Mat masks1[3]= {mask_bser_, 0.0*mask_bs_, mask_bs_}, cmask1;
  // cv::merge(masks1,3,cmask1);
  // frame+= cmask1;

  if(!mask_obj1_.empty() && !mask_bs_.empty() && !mask_mv_.empty())
  {
    cv::Mat mask_obj1_l, mask_obj2_l, mask_bs_l, mask_mv_l/*, mask_bser_l*/;
    if(params_.Width*params_.Height>0)
    {
      cv::resize(mask_obj1_, mask_obj1_l, frame.size(), 0, 0, cv::INTER_NEAREST);
      cv::resize(mask_obj2_, mask_obj2_l, frame.size(), 0, 0, cv::INTER_NEAREST);
      cv::resize(mask_bs_, mask_bs_l, frame.size(), 0, 0, cv::INTER_NEAREST);
      cv::resize(mask_mv_, mask_mv_l, frame.size(), 0, 0, cv::INTER_NEAREST);
      // cv::resize(mask_bser_, mask_bser_l, frame.size(), 0, 0, cv::INTER_NEAREST);
    }
    else
    {
      mask_obj1_l= mask_obj1_;
      mask_obj2_l= mask_obj2_;
      mask_bs_l= mask_bs_;
      mask_mv_l= mask_mv_;
      // mask_bser_l= mask_bser_;
    }

    cv::Mat masks2[3]= {0.5*mask_obj1_l+0.5*mask_obj2_l, 0.2*mask_bs_l, 0.5*mask_mv_l}, cmask2;
    cv::merge(masks2,3,cmask2);
    frame+= cmask2;

    // Visualize momenr (obj_mu_):
    // const cv::Moments &mu(obj_mu_);
    // if(mu.m00>0.0)
    // {
      // cv::Point2f center(mu.m10/mu.m00, mu.m01/mu.m00);
      // float angle= 0.5 * std::atan2(2.0*mu.mu11/mu.m00, (mu.mu20/mu.m00 - mu.mu02/mu.m00));
      // std::cerr<<mu.m00<<"  "<<angle*180.0/M_PI<<"  "<<center<<std::endl;
      // cv::line(frame, center, center+100.0*cv::Point2f(std::cos(angle),std::sin(angle)), cv::Scalar(0,0,255,128),10,8,0);
    // }

    // Visualize shrunken object and movement data (obj_, mv_):
    // std::cerr<<"obj="<<obj_.reshape(1,params_.ObjSW*params_.ObjSH)<<std::endl;
    // std::cerr<<"mv="<<mv_.reshape(1,params_.MvSW*params_.MvSH)<<std::endl;
    cv::Mat obj2, mv2;
    cv::resize(obj_, obj2, frame.size(), 0, 0, cv::INTER_NEAREST);
    cv::resize(mv_, mv2, frame.size(), 0, 0, cv::INTER_NEAREST);
    obj2.convertTo(obj2, CV_8UC1);
    mv2.convertTo(mv2, CV_8UC1);
    cv::Mat obj2s[3]= {0.5*obj2, 0.0*mv2, 0.5*mv2}, obj2c;
    cv::merge(obj2s,3,obj2c);
    frame+= obj2c;
  }
}
//-------------------------------------------------------------------------------------------

void TObjDetTrackBSP::CalibBG(const std::vector<cv::Mat> &frames)
{
  hist_bg_= cv::Mat();
  ClearObject();
  if(frames.size()==0)  return;

  std::cerr<<"Calibrating BG..."<<std::endl;
  float h_range[]= {0, 180};
  float s_range[]= {0, 255};
  float v_range[]= {0, 255};
  const float* ranges[]= {h_range, s_range, v_range};
  int channels[]= {0, 1, 2}, n_channels= 3;
  int hist_size[]= {params_.BinsH, params_.BinsS, params_.BinsV};

  cv::Mat frame_s, frame_hsv;
  cv::Mat hist_tmp;
  for(int i(0),i_end(frames.size()); i<i_end; ++i)
  {
    if(params_.Width*params_.Height>0)
      cv::resize(frames[i], frame_s, cv::Size(params_.Width,params_.Height));
    else
      frame_s= frames[i];
    cv::cvtColor(frame_s, frame_hsv, cv::COLOR_BGR2HSV);
    cv::calcHist(&frame_hsv, 1, channels, cv::Mat(), hist_tmp, n_channels, hist_size, ranges, true, false);
    if(i==0)
    {
      hist_tmp.copyTo(hist_bg_);
    }
    else
    {
      for(int h(0);h<params_.BinsH;++h) for(int s(0);s<params_.BinsS;++s) for(int v(0);v<params_.BinsV;++v)
        hist_bg_.at<float>(h,s,v)+= hist_tmp.at<float>(h,s,v);
    }
  }
  float f= 1.0/float(frames.size());
  for(int h(0);h<params_.BinsH;++h) for(int s(0);s<params_.BinsS;++s) for(int v(0);v<params_.BinsV;++v)
    hist_bg_.at<float>(h,s,v)*= f;
}
//-------------------------------------------------------------------------------------------

// Add area around a point p to the object models.
// The background model is NOT subtracted.
void TObjDetTrackBSP::AddToModel(const cv::Mat &frame, const cv::Point &p)
{
  if(hist_obj_.size()==0)  return;

  std::cerr<<"Modifying object model..."<<std::endl;

  // Get mask of points around the point
  cv::Mat mask= cv::Mat::zeros(frame.rows+2, frame.cols+2, CV_8UC1);
  cv::Scalar new_val(120,120,120);
  int lo(params_.NLoDiff), up(params_.NUpDiff);
  int new_mask_val= 255;
  int connectivity= 8;
  int flags= connectivity + (new_mask_val << 8 ) + cv::FLOODFILL_FIXED_RANGE + cv::FLOODFILL_MASK_ONLY;
  cv::floodFill(frame, mask, p, new_val, 0, cv::Scalar(lo,lo,lo), cv::Scalar(up,up,up), flags);
  mask= mask(cv::Range(1, mask.rows-1), cv::Range(1, mask.cols-1));
  // std::cerr<<"AddToModel: mask: non-zero-points: "<<cv::countNonZero(mask)<<std::endl;
  // cv::imshow("debug/mask",mask);

  // Get histogram
  float h_range[]= {0, 180};
  float s_range[]= {0, 256};
  float v_range[]= {0, 256};
  const float* ranges[]= {h_range, s_range, v_range};
  int channels[]= {0, 1, 2}, n_channels= 3;
  int hist_size[]= {params_.BinsH, params_.BinsS, params_.BinsV};
  cv::Mat frame_s, frame_hsv, mask_s;
  cv::Mat hist_tmp;
  if(params_.Width*params_.Height>0)
  {
    cv::resize(frame, frame_s, cv::Size(params_.Width,params_.Height));
    cv::resize(mask, mask_s, cv::Size(params_.Width,params_.Height));
  }
  else
  {
    frame_s= frame;
    mask_s= mask;
  }
  cv::cvtColor(frame_s, frame_hsv, cv::COLOR_BGR2HSV);
  cv::calcHist(&frame_hsv, 1, channels, mask_s, hist_tmp, n_channels, hist_size, ranges, true, false);
  // cv::imshow("debug/frame_hsv",frame_hsv);
  // std::cerr<<"hist_tmp: "<<hist_tmp.size()<<std::endl;

  // Update model
  for(std::list<cv::Mat>::iterator ho_itr(hist_obj_.begin()),ho_itr_end(hist_obj_.end());
      ho_itr!=ho_itr_end; ++ho_itr)
  {
    cv::Mat &hist(*ho_itr);
    // Add current histogram to the model, and
    // Normalize the histogram
    for(int h(0);h<params_.BinsH;++h) for(int s(0);s<params_.BinsS;++s) for(int v(0);v<params_.BinsV;++v)
    {
      hist.at<float>(h,s,v)+= hist_tmp.at<float>(h,s,v);
      if(hist.at<float>(h,s,v)>255)  hist.at<float>(h,s,v)= 255;
    }
  }
  // std::cerr<<"hist_obj_.back(): "<<hist_obj_.back().size()<<std::endl;
}
//-------------------------------------------------------------------------------------------

void TObjDetTrackBSP::SaveBGObjModels(const std::string &file_name) const
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs<<"BackgroundModel"<<hist_bg_;

  fs<<"ObjectModel"<<"[";
  for(std::list<cv::Mat>::const_iterator itr(hist_obj_.begin()),itr_end(hist_obj_.end()); itr!=itr_end; ++itr)
  {
    fs<<(*itr);
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void TObjDetTrackBSP::LoadBGObjModels(const std::string &file_name)
{
  hist_bg_.release();
  hist_obj_.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data;
  data= fs["BackgroundModel"];
  if(!data.empty())  data>>hist_bg_;

  data= fs["ObjectModel"];
  if(!data.empty())
  {
    for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
    {
      cv::Mat obj;
      (*itr)>>obj;
      hist_obj_.push_back(obj);
    }
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
// Utility
//-------------------------------------------------------------------------------------------

void CreateTrackbars(const std::string &window_name, TObjDetTrackBSPParams &params, int &trackbar_mode, bool &init_request)
{
  const std::string &win(window_name);
  if(trackbar_mode==1)
  {
    CreateTrackbar<int>("ThreshBlkH:", win, &params.ThreshBlkH, -1, 255, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("ThreshBlkS:", win, &params.ThreshBlkS, -1, 255, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("ThreshBlkV:", win, &params.ThreshBlkV, -1, 255, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<float>("BS_History:", win, &params.BS_History, 0.0, 100.0, 0.1, &TrackbarPrintOnTrack);
    CreateTrackbar<float>("BS_VarThreshold:", win, &params.BS_VarThreshold, 0.0, 100.0, 0.1, &TrackbarPrintOnTrack);
    CreateTrackbar<bool>("BS_DetectShadows:", win, &params.BS_DetectShadows, &TrackbarPrintOnTrack);
  }
  else if(trackbar_mode==2)
  {
    CreateTrackbar<int>("NErode1:",   win, &params.NErode1,     0, 10, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("BinsH:",   win, &params.BinsH,     1, 100, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("BinsS:",   win, &params.BinsS,     1, 100, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("BinsV:",   win, &params.BinsV,     1, 100, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<float>("Fbg:",     win, &params.Fbg, 0.0, 10.0, 0.01, &TrackbarPrintOnTrack);
    CreateTrackbar<float>("Fgain:",   win, &params.Fgain, 0.0, 10.0, 0.01, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("NumModel:",   win, &params.NumModel, 1, 20, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("NumFramesInHist:", win, &params.NumFramesInHist, 0, 1000, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<bool>("NoNewModel:", win, &params.NoNewModel, &TrackbarPrintOnTrack);
  }
  else if(trackbar_mode==3)
  {
    CreateTrackbar<double>("BackProjScale:", win, &params.BackProjScale, 0.0, 20.0, 0.1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("NErode2:",   win, &params.NErode2,     0, 20, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("NDilate2:",  win, &params.NDilate2,    0, 20, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("NThreshold2:", win, &params.NThreshold2, 0, 255, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("NCalibBGFrames:", win, &params.NCalibBGFrames, 1, 50, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("NUpDiff:", win, &params.NUpDiff, 0, 50, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("NLoDiff:", win, &params.NLoDiff, 0, 50, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("ObjSW:", win, &params.ObjSW, 0, 20, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("ObjSH:", win, &params.ObjSH, 0, 20, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("MvSW:", win, &params.MvSW, 0, 20, 1, &TrackbarPrintOnTrack);
    CreateTrackbar<int>("MvSH:", win, &params.MvSH, 0, 20, 1, &TrackbarPrintOnTrack);
  }
  else
  {
    trackbar_mode= 0;
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
