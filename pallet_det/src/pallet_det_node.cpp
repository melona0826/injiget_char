#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pallet_det");

  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/pallet_det/ver_edge", 1);
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
    cvtColor(frame, frame, COLOR_BGR2GRAY);

    Mat ref_img = imread("/home/jetson/catkin_ws/src/injiget_char/pallet_det/src/refer.png", IMREAD_GRAYSCALE);
    if(ref_img.empty())
      ROS_ERROR("Reference Img Not Loaded !");

    Ptr<SURF> detector = SURF::create(500);
    Ptr<SurfDescriptorExtractor> extractor = SurfDescriptorExtractor::create();
    FlannBasedMatcher matcher;

    Mat descrip_obj;

    vector<KeyPoint> key_obj;

    detector->detect(ref_img, key_obj);
    extractor->compute(ref_img, key_obj, descrip_obj);

    //detector->detectAndCompute(ref_img, noArray(), key_obj, descrip_obj);

    // detector->detectAndCompute(ref_img, noArray(), key_obj, descrip_obj);

    // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    // vector<vector<DMatch>> knn_matches;
    // matcher->knnMatch(descrip_obj, descrip_scene, knn_matches, 2);

    // const float threshold = 0.75;
    // vector<DMatch> good_matches;

    // for(size_t i = 0; i < knn_matches.size(); i++)
    // {
    //   if(knn_matches[i][0].distance < threshold * knn_matches[i][1].distance)
    //     good_matches.push_back(knn_matches[i][0]);
    // }

    // Mat img_mat;
    // drawMatches(ref_img, key_obj, frame, key_scene, good_matches, img_mat, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // vector<Point2f> obj;
    // vector<Point2f> scene;

    // for(size_t i = 0; i < good_matches.size(); i++)
    // {
    //   obj.push_back(key_obj[good_matches[i].queryIdx].pt);
    //   scene.push_back(key_scene[good_matches[i].trainIdx].pt);
    // }

    // Mat H = findHomography(obj, scene, RANSAC);

    // vector<Point2f> obj_corners(4);
    // obj_corners[0] = Point2f(0,0);
    // obj_corners[1] = Point2f((float)ref_img.cols,0);
    // obj_corners[2] = Point2f((float)ref_img.cols, (float)ref_img.rows);
    // obj_corners[3] = Point2f(0,(float)ref_img.rows);
    // vector<Point2f> scene_corners(4);

    // perspectiveTransform(obj_corners, scene_corners, H);

    // line(img_mat, scene_corners[0] + Point2f((float)ref_img.cols,0) , scene_corners[1] + Point2f((float)ref_img.cols,0), Scalar(0, 255,0), 4);
    // line(img_mat, scene_corners[1] + Point2f((float)ref_img.cols,0) , scene_corners[2] + Point2f((float)ref_img.cols,0), Scalar(0, 255,0), 4);
    // line(img_mat, scene_corners[2] + Point2f((float)ref_img.cols,0) , scene_corners[3] + Point2f((float)ref_img.cols,0), Scalar(0, 255,0), 4);
    // line(img_mat, scene_corners[3] + Point2f((float)ref_img.cols,0) , scene_corners[0] + Point2f((float)ref_img.cols,0), Scalar(0, 255,0), 4);

    sensor_msgs::ImageConstPtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", ref_img).toImageMsg();

    pub.publish(pub_msg);

  });

  ros::spin();

  return 0;

}

