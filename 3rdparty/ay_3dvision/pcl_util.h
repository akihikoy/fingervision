//-------------------------------------------------------------------------------------------
/*! \file    pcl_util.h
    \brief   PCL utility
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Oct.17, 2014
    \version 0.2
    \date    Feb.10, 2015
*/
//-------------------------------------------------------------------------------------------
#ifndef pcl_util_h
#define pcl_util_h
//-------------------------------------------------------------------------------------------
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// #include <pcl/ros/conversions.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/Pose.h>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

template<typename t_point>
inline bool IsValid(const t_point &pt)
{
  // pcl_isfinite is better?
  return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
}
template<>
inline bool IsValid<float>(const float &x)
{
  return std::isfinite(x);
}
template<>
inline bool IsValid<double>(const double &x)
{
  return std::isfinite(x);
}
//-------------------------------------------------------------------------------------------

template<typename t_point>
inline bool IsInvalid(const t_point &pt)
{
  // pcl_isfinite is better?
  return !std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z);
}
template<>
inline bool IsInvalid<float>(const float &x)
{
  return !std::isfinite(x);
}
template<>
inline bool IsInvalid<double>(const double &x)
{
  return !std::isfinite(x);
}
//-------------------------------------------------------------------------------------------


pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ConvertROSMsgToPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg, bool no_rgb=false);

template<typename t_point>
void ConvertPointCloudToROSMsg(
    sensor_msgs::PointCloud2 &msg,
    const typename pcl::PointCloud<t_point>::ConstPtr &cloud,
    const std::string &frame_id, const ros::Time &time=ros::Time::now());

/* Convert an XYZ image to a point cloud.  */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ConvertXYZImageToPointCloud(const cv::Mat &xyz_img, const cv::Mat &rgb_img);

/* Convert 4D points (e.g. result of triangulatePoints) to a point cloud.  */
pcl::PointCloud<pcl::PointXYZ>::Ptr
Convert4DPointsToPointCloud(const cv::Mat &points4d);

/* Get RGB image and depth image from a Point Cloud.
    cf. pcl::PointXYZRGB Struct Reference
    http://docs.pointclouds.org/1.7.1/structpcl_1_1_point_x_y_z_r_g_b.html  */
void ConvertPointCloudToRGBDImages(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    cv::Mat &rgb_img, cv::Mat &depth_img);

/* Estimate normal and store it as an image.
    FS: Window size for computing normal (should be odd).  */
void ConvertPointCloudToNormalImage(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    cv::Mat &normal_img, int FS=9);
//-------------------------------------------------------------------------------------------

// Compute 3x4 projection/camera matrix from point cloud data.
std::vector<double> GetCameraProjFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
//-------------------------------------------------------------------------------------------

// Get an error map of camera projection matrix over point cloud data.
void CameraProjErrorImgFromCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    cv::Mat &err_img, const double &Fx, const double &Fy, const double &Cx, const double &Cy);
//-------------------------------------------------------------------------------------------

bool AssignInfToPlane(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_io,
    const double &ransac_dist_thresh,
    int ransac_max_iterations);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // pcl_util_h
//-------------------------------------------------------------------------------------------
