//-------------------------------------------------------------------------------------------
/*! \file    disp_rostime.cpp
    \brief   Display ROS time
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.24, 2017
*/
//-------------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstdio>
#include <sys/time.h>  // gettimeofday
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "disp_rostime");
  ros::NodeHandle node("~");
  cv::namedWindow("time", CV_WINDOW_AUTOSIZE);
  cv::Mat frame(cv::Size(320,50),CV_8UC3);
  while(ros::ok())
  {
    double time= ros::Time::now().toSec();
    std::stringstream ss;
    ss<<std::setprecision(14)<<time;
    frame.setTo(0);
    cv::putText(frame, ss.str(), cv::Point(10,35), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 1, CV_AA);
    cv::imshow("time", frame);
    char c(cv::waitKey(100));
    if(c=='\x1b'||c=='q') break;
    ros::spinOnce();
  }
  return 0;
}
//-------------------------------------------------------------------------------------------
