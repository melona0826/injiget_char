#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
<<<<<<< HEAD

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <chrono>
#include <thread>
=======
#include <opencv2/oepncv.hpp>
#include <cv_brdige/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
>>>>>>> 30531393cab546a6c417a767c67c5ba65382bd42

using namespace std;
using namespace cv;

<<<<<<< HEAD
int h_min = 45;
int h_max = 65;

int s_min = 40;
int s_max = 100;

int v_min = 40;
int v_max = 100;

int terminate_count = 0;

void callTerminate(const std_msgs::String& msg)
{
  if(msg.data == "Terminate")
    ros::shutdown();
}
=======
// Constraint Range of Pallet

int pal_h_min = 40;
int pal_h_max = 60;

int pal_s_min = 17;
int pal_s_max = 100;

int pal_v_min = 30;
int pal_v_max = 100;

Scalar pal_hsv_min(pal_h_min, pal_s_min, pal_v_min);
Scalar pal_hsv_max(pal_h_max, pal_s_max, pal_v_max);
>>>>>>> 30531393cab546a6c417a767c67c5ba65382bd42

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pallet_det");
<<<<<<< HEAD
  ros::NodeHandle nh;

  ros::Publisher pub_tilt = nh.advertise<std_msgs::String>("/tilt/mode", 100, true);
  ros::Publisher pub_fork = nh.advertise<std_msgs::String>("/fork/mode", 100, true);
  std_msgs::String tilt_mode;
  std_msgs::String fork_mode;
  tilt_mode.data = "front";
  fork_mode.data = "down";
  pub_tilt.publish(tilt_mode);
  pub_fork.publish(fork_mode);
  std::this_thread::sleep_for(1s);

  ros::Subscriber terminate_sub = nh.subscribe("/pallet_pick/terminate", 1, callTerminate);

  geometry_msgs::Pose2D fitLine_msg;
  ros::Publisher pub_fitLine = nh.advertise<geometry_msgs::Pose2D>("/pallet_det/pallet_pos",1000);

  h_min = (int)(h_min/2);
  h_max = (int)(h_max/2);

  s_min = (int)(s_min * 255 / 100);
  s_max = (int)(s_max * 255 / 100);

  v_min = (int)(v_min * 255 / 100);
  v_max = (int)(v_max * 255 / 100);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/pallet_det/edge_image", 1);
  image_transport::Publisher pub2 = it.advertise("/pallet_det/det_image", 1);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1,
  [&](const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge Erorr ! : %s", e.what());
      return;
    }

    Mat frame = cv_ptr->image;
    flip(frame, frame, -1);
    frame = frame * 0.8;

    Mat img_hsv, img_pallet, img_edge, img_recog;
    Mat pallet_mask;
    vector<Vec4i> linesP;
    vector<Point> pts;
    Vec4d fit_line;
    Point pt1, pt2;
    double slope_treshold = (90 - 45) * CV_PI / 180.0;

    frame.copyTo(img_recog);

    cvtColor(frame, img_hsv, COLOR_BGR2HSV);
    inRange(img_hsv, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), pallet_mask);
    bitwise_and(frame, frame, img_pallet,  pallet_mask);

    Canny(img_pallet, img_edge, 80, 100);

    HoughLinesP(img_edge, linesP, 1, (CV_PI/180), 10, 18, 10);
    double m = 0;
    int count = 0;
    for(size_t i = 0; i < linesP.size(); i++)
    {
      Vec4i l = linesP[i];
      pt1 = Point(l[0], l[1]);
      pt2 = Point(l[2], l[3]);

      double slope = (static_cast<double>(pt1.y) - static_cast<double>(pt2.y)) / (static_cast<double>(pt1.x) - static_cast<double>(pt2.x) );

      if(abs(slope) >= slope_treshold)
      {
        line(frame, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(0,0,255), 2, 8);
        count ++;
        m = slope;
        pts.push_back(pt1);
        pts.push_back(pt2);
      }

    }
    if(pts.size() > 0)
    {
      fitLine(pts, fit_line, DIST_L2, 0, 0.01, 0.01);

      double m = fit_line[1] / fit_line[0];
      Point b = Point(fit_line[2], fit_line[3]);

      int pt1_y = frame.rows;
      int pt2_y = 0;

      double pt1_x = ((pt1_y - b.y) / m) + b.x;
      double pt2_x = ((pt2_y - b.y) / m) + b.x;

      cout << "slope : " << (static_cast<double>(pt1_y) - static_cast<double>(pt2_y)) / (static_cast<double>(pt1_x) - static_cast<double>(pt2_x)) << endl;

      line(frame, Point(pt1_x, pt1_y) , Point(pt2_x , pt2_y) , Scalar(255,0,0) , 2 , 8);

      fitLine_msg.x = b.x;
      fitLine_msg.y = b.y;
      fitLine_msg.theta = m;

      cout << "x : " << b.x << endl;;
      cout << "y : " << b.y << endl;;
      terminate_count = 0;
    }

    else if(terminate_count < 5)
    {
      terminate_count += 1;
    }

    else
    {
      fitLine_msg.x = -333;
      fitLine_msg.y = -333;
      fitLine_msg.theta = -333;
    }

    sensor_msgs::ImageConstPtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_edge).toImageMsg();
    sensor_msgs::ImageConstPtr pub_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    pub.publish(pub_msg);
    pub2.publish(pub_msg2);
    pub_fitLine.publish(fitLine_msg);
=======

  ros::NodeHandle nh;

  geometry_msgs::Pose2D pallet_pos;
  ros::Publisher pub_pallet_pos = nh.advertise<geometry_msgs::Pose2D>("/pallet_det/pallet_pos", 1000);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher mask_pub = it.advertise("/pallet_det/mask_img", 1);
  image_transport::Publisher det_pub = it.advertise("/pallet_det/det_img", 1);
  image_transport::Subscriber img_sub = it.subscriber("/cam/raw_img", 1,
  [&](const sensor_msgs::IamgeConstPtr& msg){
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::iamge_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge Error ! : %s" , e.what());
      return;
    }

    Mat img_hsv, edge_img, img_pallet;
    Mat pallet_mask;
    Mat frame = cv_ptr -> image;
    Mat grayImg, blurImg, edgeImg, copyImg;

    cvtColor(frame, img_hsv, COLOR_BGR2HSV);
    inRange(img_hsv, pal_hsv_min, pal_hsv_max, pallet_mask);
    bitwise_and(frame, frame, img_pallet, pallet_mask);

    sensor_msgs::ImagePtr mask_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", copyImg).toImageMsg();
    mask_pub.publish(mask_msg);



>>>>>>> 30531393cab546a6c417a767c67c5ba65382bd42

  });

  ros::spin();
<<<<<<< HEAD

  return 0;

=======
  return 0;
>>>>>>> 30531393cab546a6c417a767c67c5ba65382bd42
}

