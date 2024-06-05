/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include "opencv2/opencv.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)

int w=500;
int h=500;

double fps = 10;
int delay = cvRound(1000 / fps);

int fourcc = cv::VideoWriter::fourcc('X', '2', '6', '4');

cv::VideoWriter outputVideo("lidarsave.mp4", fourcc, fps,cv::Size(w, h));

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  int x,y;
  cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
    x=250+scan->ranges[i]*50*sin(degree*M_PI/180);
    y=250+scan->ranges[i]*50*cos(degree*M_PI/180);
    cv::circle(img,cv::Point(x,y),1,cv::Scalar(0,0,255),1);
  }
  outputVideo << img;
  cv::imshow("img",img);
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();


  return 0;
}
