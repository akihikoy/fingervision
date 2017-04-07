//-------------------------------------------------------------------------------------------
/*! \file    pcl_util.cpp
    \brief   PCL utility
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Oct.17, 2014
    \version 0.2
    \date    Feb.10, 2015
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/pcl_util.h"
#include "lfd_vision/geom_util.h"
//-------------------------------------------------------------------------------------------
#include <pcl/io/io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <limits>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ConvertROSMsgToPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg, bool no_rgb)
{
  //Convert the point cloud to depth image and rgb image
  // pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  if(!no_rgb)  pcl::fromROSMsg(*msg, *cloud);
  else
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_g(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_g);
    pcl::copyPointCloud(*cloud_g,*cloud);
  }

  // // Remove plains
  // bool res= RemovePlanes<pcl::PointXYZRGB>(cloud,
                // /*non_planar_points_ratio=*/0.9,
                // /*ransac_dist_thresh=*/0.01,
                // /*ransac_max_iterations=*/100);

  // ref.  template<typename CloudT> void toROSMsg(const CloudT& cloud, sensor_msgs::Image& msg)
  // http://docs.pointclouds.org/1.1.0/conversions_8h_source.html#l00254
  if(cloud->width == 0 && cloud->height == 0)
    throw std::runtime_error("Needs to be a dense like cloud!!");
  else
  {
    if(cloud->points.size() != cloud->width * cloud->height)
      throw std::runtime_error("The width and height do not match the cloud size!");
  }

  return cloud;
}
//-------------------------------------------------------------------------------------------

template<typename t_point>
void ConvertPointCloudToROSMsg(
    sensor_msgs::PointCloud2 &msg,
    const typename pcl::PointCloud<t_point>::ConstPtr &cloud,
    const std::string &frame_id, const ros::Time &time)
{
  pcl::toROSMsg(*cloud,msg);
  msg.header.frame_id= frame_id;
  msg.header.stamp= time;
}
// instantiation:
template
void ConvertPointCloudToROSMsg<pcl::PointXYZRGB>(
    sensor_msgs::PointCloud2 &msg,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    const std::string &frame_id, const ros::Time &time);
template
void ConvertPointCloudToROSMsg<pcl::PointXYZ>(
    sensor_msgs::PointCloud2 &msg,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
    const std::string &frame_id, const ros::Time &time);
//-------------------------------------------------------------------------------------------

/* Convert an XYZ image to a point cloud.  */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ConvertXYZImageToPointCloud(const cv::Mat &xyz_img, const cv::Mat &rgb_img)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->height= xyz_img.rows;
  cloud->width= xyz_img.cols;
  cloud->is_dense= true;
  cloud->points.resize(xyz_img.rows*xyz_img.cols);
  for(int y(0),i(0); y<xyz_img.rows; ++y)
  {
    const cv::Point3f *xyz_ptr= xyz_img.ptr<cv::Point3f>(y);
    const cv::Vec3b   *rgb_ptr= rgb_img.ptr<cv::Vec3b>(y);
    for(int x(0); x<xyz_img.cols; ++x,++i)
    {
      // const cv::Point3f &xyz(xyz_ptr[x]);
      // const cv::Vec3b   &rgb(rgb_ptr[x]);
      const cv::Point3f &xyz(xyz_img.at<cv::Point3f>(y,x));
      const cv::Vec3b   &rgb(rgb_img.at<cv::Vec3b>(y,x));
      pcl::PointXYZRGB &point(cloud->points[i]);
      // point.y= -xyz.x;
      // point.z= -xyz.y;
      // if(xyz.z>0.0 && !isinf(xyz.z))  point.x= xyz.z;
      point.x= xyz.x;
      point.y= xyz.y;
      if(xyz.z>0.0 && !isinf(xyz.z))  point.z= xyz.z;
      // if(!isinf(xyz.z))  point.z= xyz.z;
      else  point.z= nanf("");
      point.r= rgb[2];
      point.g= rgb[1];
      point.b= rgb[0];
    }
  }
  return cloud;
}
//-------------------------------------------------------------------------------------------

/* Convert 4D points (e.g. result of triangulatePoints) to a point cloud.  */
pcl::PointCloud<pcl::PointXYZ>::Ptr
Convert4DPointsToPointCloud(const cv::Mat &points4d)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->height= 1;
  cloud->width= points4d.cols;
  cloud->is_dense= false;
  cloud->points.resize(points4d.cols);
  for(int i(0); i<points4d.cols; ++i)
  {
    const cv::Vec4f &xyz(points4d.col(i));
    pcl::PointXYZ &point(cloud->points[i]);
    // point.y= -xyz[0];
    // point.z= -xyz[1];
    // if(xyz[2]>0.0 && !isinf(xyz[2]))  point.x= xyz[2];
    point.x= xyz[0];
    point.y= xyz[1];
    if(xyz[2]>0.0 && !isinf(xyz[2]))  point.z= xyz[2];
    // if(!isinf(xyz[2]))  point.z= xyz[2];
    else  point.z= nanf("");
  }
  return cloud;
}
//-------------------------------------------------------------------------------------------

/* Get RGB image and depth image from a Point Cloud.
    cf. pcl::PointXYZRGB Struct Reference
    http://docs.pointclouds.org/1.7.1/structpcl_1_1_point_x_y_z_r_g_b.html  */
void ConvertPointCloudToRGBDImages(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    cv::Mat &rgb_img, cv::Mat &depth_img)
{
  rgb_img.create(cloud->height, cloud->width, CV_8UC3);
  depth_img.create(cloud->height, cloud->width, CV_32FC1);
  // xyz_img.create(cloud->height, cloud->width, CV_32FC3);
  for(size_t y(0); y<cloud->height; ++y)
  {
    for(size_t x(0); x<cloud->width; ++x)
    {
      const pcl::PointXYZRGB &pt(cloud->at(x,y));
      if(IsInvalid(pt))
      {
        rgb_img.at<cv::Vec3b>(y,x)= cv::Vec3b(0.0,0.0,0.0);
        depth_img.at<float>(y,x)= 0.0;
      }
      else
      {
        uint32_t rgb = *reinterpret_cast<const int*>(&pt.rgb);
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8)  & 0x0000ff;
        uint8_t b = (rgb)       & 0x0000ff;
        rgb_img.at<cv::Vec3b>(y,x)= cv::Vec3b(b,g,r);
        depth_img.at<float>(y,x)= pt.z;
        // xyz_img.at<cv::Vec3f>(y,x)= cv::Vec3f(pt.x+1.0,pt.y+1.0,pt.z*5.0);
      }
    }
  }
}
//-------------------------------------------------------------------------------------------

/* Estimate normal and store it as an image.
    FS: Window size for computing normal (should be odd).  */
void ConvertPointCloudToNormalImage(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    cv::Mat &normal_img, int FS)
{
  int FSh((FS-1)/2);
  normal_img.create(cloud->height, cloud->width, CV_32FC3);
  for(int y(0); y<cloud->height; ++y)
  {
    for(int x(0); x<cloud->width; ++x)
    {
      bool no_data(false);
      if(x<FSh || cloud->width-FSh<=x || y<FSh || cloud->height-FSh<=y)  no_data= true;
      else if(IsInvalid(cloud->at(x-FSh,y)) || IsInvalid(cloud->at(x+FSh,y))
            || IsInvalid(cloud->at(x,y-FSh)) || IsInvalid(cloud->at(x,y+FSh)))  no_data= true;
      if(no_data)
      {
        normal_img.at<cv::Vec3f>(y,x)= cv::Vec3f(0.0,0.0,0.0);
        continue;
      }
      #if 1  // Simple calculation (fast)
      Eigen::Vector3f ax1= cloud->at(x+FSh,y).getVector3fMap() - cloud->at(x-FSh,y).getVector3fMap();
      Eigen::Vector3f ax2= cloud->at(x,y+FSh).getVector3fMap() - cloud->at(x,y-FSh).getVector3fMap();
      #endif
      #if 0  // Complicated calculation (slow, but robust for noise)
      int num1(0), num2(0);
      Eigen::Vector3f ax1(0.0,0.0,0.0);
      Eigen::Vector3f ax2(0.0,0.0,0.0);
      int xs(x-FSh), ys(y-FSh);
      for(int fs(1); fs<=FS; fs+=2)
      {
        for(int i(0); i<FS-fs; ++i)
          for(int j(0); j<FS; ++j)
            if(IsValid(cloud->at(xs+i,ys+j)) && IsValid(cloud->at(xs+i+fs,ys+j)))
            {
              ax1+= cloud->at(xs+i+fs,ys+j).getVector3fMap() - cloud->at(xs+i,ys+j).getVector3fMap();
              ++num1;
            }
        for(int i(0); i<FS; ++i)
          for(int j(0); j<FS-fs; ++j)
            if(IsValid(cloud->at(xs+i,ys+j)) && IsValid(cloud->at(xs+i,ys+j+fs)))
            {
              ax2+= cloud->at(xs+i,ys+j+fs).getVector3fMap() - cloud->at(xs+i,ys+j).getVector3fMap();
              ++num2;
            }
      }
      ax1/= (float)num1;
      ax2/= (float)num2;
      #endif

      // Eigen::Vector3f normal= ax1.cross(ax2);
      Eigen::Vector3f normal= ax2.cross(ax1);
      if(normal.norm()>1.0e-6)
      {
        normal.normalize();
        // normal_img.at<cv::Vec3f>(y,x)= cv::Vec3f(normal[0],normal[1],normal[2]);
        cv::Vec3f col;
        GetVisualNormal(normal[0],normal[1],normal[2], col(0),col(1),col(2));
        //*DEBUG*/GetVisualNormal(normal[1],normal[0],normal[2], col(0),col(1),col(2));
        normal_img.at<cv::Vec3f>(y,x)= col;
      }
      else
      {
        normal_img.at<cv::Vec3f>(y,x)= cv::Vec3f(0.0,0.0,0.0);
      }
    }
  }
}
//-------------------------------------------------------------------------------------------

// Compute 3x4 projection/camera matrix from point cloud data.
std::vector<double> GetCameraProjFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  int n_samples(0);
  for(size_t y(0); y<cloud->height; ++y)
  {
    for(size_t x(0); x<cloud->width; ++x)
    {
      const pcl::PointXYZRGB &pt(cloud->at(x,y));
      if(IsValid(pt) && std::fabs(pt.z)>0.2)
        ++n_samples;
    }
  }
  Eigen::MatrixXd X_xz(n_samples,2), Y_u(n_samples,1);
  Eigen::MatrixXd X_yz(n_samples,2), Y_v(n_samples,1);
  int i(0);
  for(size_t y(0); y<cloud->height; ++y)
  {
    for(size_t x(0); x<cloud->width; ++x)
    {
      const pcl::PointXYZRGB &pt(cloud->at(x,y));
      if(IsValid(pt) && std::fabs(pt.z)>0.2)
      {
        X_xz(i,0)= pt.x/pt.z;
        X_xz(i,1)= 1.0;
        Y_u(i,0)= x;
        X_yz(i,0)= pt.y/pt.z;
        X_yz(i,1)= 1.0;
        Y_v(i,0)= y;
        ++i;
      }
    }
  }
  Eigen::MatrixXd W_u= LeastSq(X_xz, Y_u, /*lambda=*/0.01);  // Fx, Cx
  Eigen::MatrixXd W_v= LeastSq(X_yz, Y_v, /*lambda=*/0.01);  // Fy, Cy
  std::vector<double> cam_proj(12,0.0);  // 3x4 projection/camera matrix
  double tmp[]={
      W_u(0),    0.0, W_u(1), 0.0,
      0.0,    W_v(0), W_v(1), 0.0,
      0.0,       0.0,    1.0, 0.0};
  for(int i(0);i<12;++i) cam_proj[i]= tmp[i];
  return cam_proj;
}
//-------------------------------------------------------------------------------------------

// Get an error map of camera projection matrix over point cloud data.
void CameraProjErrorImgFromCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    cv::Mat &err_img, const double &Fx, const double &Fy, const double &Cx, const double &Cy)
{
  err_img.create(cloud->height, cloud->width, CV_8UC3);
  for(size_t y(0); y<cloud->height; ++y)
  {
    for(size_t x(0); x<cloud->width; ++x)
    {
      const pcl::PointXYZRGB &pt(cloud->at(x,y));
      if(!(IsValid(pt) && std::fabs(pt.z)>0.2))
      {
        err_img.at<cv::Vec3b>(y,x)= cv::Vec3b(0.0,0.0,0.0);
      }
      else
      {
        int u= Fx * pt.x/pt.z + Cx;
        int v= Fy * pt.y/pt.z + Cy;
        err_img.at<cv::Vec3b>(y,x)= cv::Vec3b(0.0,std::fabs(u-x),std::fabs(v-y));
      }
    }
  }
}
//-------------------------------------------------------------------------------------------

bool AssignInfToPlane(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_io,
    const double &ransac_dist_thresh,
    int ransac_max_iterations)
{
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB>  seg;
  pcl::PointIndices::Ptr  inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr  coefficients(new pcl::ModelCoefficients);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(ransac_max_iterations);
  seg.setDistanceThreshold(ransac_dist_thresh);

  // int nr_points= cloud_io->points.size();
  // while(cloud_io->points.size() > non_planar_points_ratio*nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_io);
    seg.segment(*inliers, *coefficients); //*
    if(inliers->indices.size() == 0)
    {
      // std::cerr<<"Error: Could not estimate a planar model for the given dataset."<<std::endl;
      return false;
    }

    for(int i(0); i<inliers->indices.size(); ++i)
      cloud_io->points[inliers->indices[i]].z= std::numeric_limits<float>::infinity();
  }
  return true;
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

