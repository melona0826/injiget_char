#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>

using namespace std;
using namespace cv;

int h_min = 45;
int h_max = 65;

int s_min = 20;
int s_max = 100;

int v_min = 55;
int v_max = 100;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pallet_det");

  ros::NodeHandle nh;

  h_min = (int)(h_min/2);
  h_max = (int)(h_max/2);

  s_min = (int)(s_min * 255 / 100);
  s_max = (int)(s_max * 255 / 100);

  v_min = (int)(v_min * 255 / 100);
  v_max = (int)(v_max * 255 / 100);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/pallet_det/ver_edge", 1);
  image_transport::Publisher pub2 = it.advertise("/pallet_det/image_dark", 1);
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
    frame = frame * 0.7;

    Mat img_hsv, img_pallet, img_edge, img_recog;
    Mat pallet_mask;
    vector<Vec4i> linesP;

    frame.copyTo(img_recog);

    cvtColor(frame, img_hsv, COLOR_BGR2HSV);
    inRange(img_hsv, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), pallet_mask);
    bitwise_and(frame, frame, img_pallet,  pallet_mask);

    Canny(img_pallet, img_edge, 30, 70);

    HoughLinesP(img_edge, linesP, 1, (CV_PI/180), 50, 50, 10);

    for(size_t i = 0; i < linesP.size(); i++)
    {
      Vec4i l = linesP[i];
      line(img_pallet, Point(3,3), Point(50,3), Scalar(0,0,255), 2, 8);
    }

    sensor_msgs::ImageConstPtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_pallet).toImageMsg();
    sensor_msgs::ImageConstPtr pub_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_pallet).toImageMsg();

    pub.publish(pub_msg);
    pub2.publish(pub_msg2);

  });

  ros::spin();

  return 0;

}

