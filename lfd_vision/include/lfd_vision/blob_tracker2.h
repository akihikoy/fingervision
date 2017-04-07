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
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/function.hpp>
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
    const float &ds_emp=4.0,   // Emphasize (scale) ratio of DS to draw
    const float &dp_emp=10.0,  // Emphasize (scale) ratio of DP to draw
    const cv::Mat &img_th=cv::Mat()  // Processed image
  );
// Track individual blobs.  prev: base, curr: current.
void TrackKeyPoints2(
    const std::vector<cv::KeyPoint> &prev,
    const std::vector<cv::KeyPoint> &curr,
    std::vector<TPointMove2> &move,
    const float &dist_min,  // Minimum distance change (i.e. sensitivity)
    const float &dist_max,  // Maximum distance change (too large might be noise)
    const float &ds_min,  // Minimum size change (i.e. sensitivity)
    const float &ds_max  // Maximum size change (too large might be noise)
  );
//-------------------------------------------------------------------------------------------

struct TBlobTracker2Params
{
  // For blob detection:
  cv::SimpleBlobDetector::Params SBDParams;

  // For preprocessing:
  int ThreshH;
  int ThreshS;
  int ThreshV;
  int NDilate1;
  int NErode1;
  // For keypoint tracking;
  float SWidth;  // Width of search ROI of each keypoint
  float NonZeroMin; // Minimum ratio of nonzero pixels in ROI over original keypoint size.
  float NonZeroMax; // Maximum ratio of nonzero pixels in ROI over original keypoint size.
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
void WriteToYAML(const std::vector<TBlobTracker2Params> &blob_params, const std::string &file_name);
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
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // blob_tracker2_h
//-------------------------------------------------------------------------------------------
