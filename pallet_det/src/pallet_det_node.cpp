#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/oepncv.hpp>
#include <cv_brdige/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>

using namespace std;
using namespace cv;

// Constraint Range of Pallet

int pal_h_min = 40;
int pal_h_max = 60;

int pal_s_min = 17;
int pal_s_max = 100;

int pal_v_min = 30;
int pal_v_max = 100;

Scalar pal_hsv_min(pal_h_min, pal_s_min, pal_v_min);
Scalar pal_hsv_max(pal_h_max, pal_s_max, pal_v_max);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pallet_det");

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




  });

  ros::spin();
  return 0;
}

