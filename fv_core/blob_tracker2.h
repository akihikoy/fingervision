//-------------------------------------------------------------------------------------------
/*! \file    blob_tracker2.h
    \brief   Blob tracker version 2.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.19, 2017

cf. testl/cv/simple_blob_tracker4.cpp
*/
//-------------------------------------------------------------------------------------------
#ifndef blob_tracker2_h
#define blob_tracker2_h
//-------------------------------------------------------------------------------------------
#include "ay_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <boost/function.hpp>
#include <iostream>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

struct TPointMove2
{
  cv::Point2f Po;  // Original position
  float So;        // Original size
  cv::Point2f DP;  // Displacement of position
  float DS;        // Displacement of size
  int NTrackFailed; // Number of traking failures in a raw
};
void DrawPointMoves2(cv::Mat &img, const std::vector<TPointMove2> &move,
    const cv::Scalar &col1, const cv::Scalar &col2,
    const float &ds_emp,  // Emphasize (scale) ratio of DS
    const float &dp_emp,  // Emphasize (scale) ratio of DP
    const cv::Mat &img_th,  // Processed image
    bool is_thresholded,  // If img_th is thresholded
    const float &s_width  // Width of search ROI of each keypoint
  );
// Track individual blobs.
void TrackKeyPoints2(
    const cv::Mat &img_th,  // Preprocessed image
    bool is_thresholded,   // If img_th is thresholded
    cv::SimpleBlobDetector &detector,  // Blob detector
    const std::vector<cv::KeyPoint> &orig,  // Original keypoints
    std::vector<TPointMove2> &move,  // Must be previous movement
    const float &s_width,  // Width of search ROI of each keypoint
    const float &nonzero_min,  // Minimum ratio of nonzero pixels in ROI over original keypoint size.
    const float &nonzero_max,  // Maximum ratio of nonzero pixels in ROI over original keypoint size.
    const float &vp_max,  // Maximum position change (too large one might be noise)
    const float &vs_max,  // Maximum size change (too large one might be noise)
    const int   &n_reset  // When number of tracking failure in a row exceeds this, tracking is reset
  );
//-------------------------------------------------------------------------------------------

struct TBlobTracker2Params
{
  // For blob detection:
  cv::SimpleBlobDetector::Params SBDParams;

  // For preprocessing:
  bool ThresholdingImg;  // Thresholding the input image where Thresh*, NDilate1, NErode1 are used.
  int ThreshH;
  int ThreshS;
  int ThreshV;
  int NDilate1;
  int NErode1;
  // For keypoint tracking;
  float SWidth;  // Width of search ROI of each keypoint
  float NonZeroMin; // Minimum ratio of nonzero pixels in ROI over original keypoint size (used only when ThresholdingImg==true).
  float NonZeroMax; // Maximum ratio of nonzero pixels in ROI over original keypoint size (used only when ThresholdingImg==true).
  float VPMax;  // Maximum position change (too large one might be noise)
  float VSMax;  // Maximum size change (too large one might be noise)
  int   NReset;  // When number of tracking failure in a row exceeds this, tracking is reset
  // For calibration:
  float DistMaxCalib;
  float DSMaxCalib;

  // For visualization:
  float DSEmp;  // Emphasize (scale) ratio of DS to draw
  float DPEmp;  // Emphasize (scale) ratio of DP to draw
  // For calibration:
  int NCalibPoints;  // Number of points for calibration

  TBlobTracker2Params();
};
void WriteToYAML(const std::vector<TBlobTracker2Params> &blob_params, const std::string &file_name, cv::FileStorage *pfs=NULL);
void ReadFromYAML(std::vector<TBlobTracker2Params> &blob_params, const std::string &file_name);
//-------------------------------------------------------------------------------------------

class TBlobTracker2
{
public:
  // User defined identifier.
  std::string Name;

  void Init();
  void Preprocess(const cv::Mat &img, cv::Mat &img_th);
  void Step(const cv::Mat &img);
  void Draw(cv::Mat &img);
  void Calibrate(const std::vector<cv::Mat> &images);

  // Remove a keypoint around p.
  void RemovePointAt(const cv::Point2f &p, const float &max_dist=30.0);

  void SaveCalib(const std::string &file_name) const;
  void LoadCalib(const std::string &file_name);

  TBlobTracker2Params& Params()  {return params_;}
  const TBlobTracker2Params& Params() const {return params_;}

  const std::vector<TPointMove2>& Data() const {return keypoints_move_;}

private:
  TBlobTracker2Params params_;
  cv::Ptr<cv::SimpleBlobDetector> detector_;

  std::vector<cv::KeyPoint> keypoints_orig_;
  std::vector<TPointMove2> keypoints_move_;

  cv::Mat img_th_;
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// Utility
//-------------------------------------------------------------------------------------------

void CreateTrackbars(const std::string &window_name, TBlobTracker2Params &params, int &trackbar_mode, bool &init_request);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // blob_tracker2_h
//-------------------------------------------------------------------------------------------
