#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <vector>

using namespace std;
using namespace cv;

int frame_wid = 1600;
int frame_hei = 1200;

int roi_x = 80;
int roi_y = 240;
int roi_width = frame_wid - roi_x;
int roi_height = frame_hei - roi_y;

int black_hue_low = 0;
int black_hue_high = 255;
int black_sat_low = 0;
int black_sat_high = 255;
int black_val_low = 0;
int black_val_high =80;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_detect");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/line_detect/detect_img", 1);
  image_transport::Publisher pub2 = it.advertise("/line_detect/black_img", 1);
  image_transport::Subscriber sub = it.subscribe("/cam_pub/raw_img", 1,
  [&](const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge Error ! : %s", e.what());
      return;
    }

    Mat frame = cv_ptr->image;
    Mat grayImg, blurImg, edgeImg;


    Rect bounds(0, 0, frame.cols, frame.rows);
    Rect roi(roi_x, roi_y, roi_width, roi_height);
    frame = frame(bounds & roi);

    cout << "row : " << frame.rows << endl;
    cout << "cols : " << frame.cols << endl;

    Mat img_hsv, black_mask, img_black, img_edge;
    cvtColor(frame, img_hsv, COLOR_BGR2HSV);

    //inRange(img_hsv, Scalar(10, 71, 76) , Scalar(30, 255, 255), black_mask);
    inRange(img_hsv, Scalar(black_hue_low, black_sat_low, black_val_low) , Scalar(black_hue_high, black_sat_high, black_val_high), black_mask);

    bitwise_and(frame, frame, img_black, black_mask);

    Mat copyImg;
    img_black.copyTo(copyImg);

    cvtColor(img_black, img_black, COLOR_BGR2GRAY);
    Canny(img_black, img_edge, 50, 450);

    vector<Vec4i> lines;
    HoughLinesP(img_edge, lines, 1, CV_PI / 180 , 50 ,20, 10);


    Point pt1, pt2;
    vector<double> slopes;
    vector<Vec4i> selected_lines;
    vector<Point> pts;
    Vec4d fit_line;
    int slope_tor = 45;
    double slope_treshol = (90 - slope_tor) * CV_PI / 180.0;



    for(size_t i = 0; i < lines.size(); i++)
    {
      Vec4i line = lines[i];
      pt1 = Point(line[0] , line[1]);
      pt2 = Point(line[2], line[3]);

      double slope = (static_cast<double>(pt1.y) - static_cast<double>(pt2.y)) / (static_cast<double>(pt1.x) - static_cast<double>(pt2.x) );
      //cout << slope << endl;

      cv::line(frame, Point(pt1.x, pt1.y) , Point(pt2.x , pt2.y) , Scalar(0,255,0) , 2 , 8);
      if(abs(slope) >= slope_treshol)
      {
        selected_lines.push_back(line);
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

      //cout << "slope : " << (static_cast<double>(pt1_y) - static_cast<double>(pt2_y)) / (static_cast<double>(pt1_x) - static_cast<double>(pt2_x)) << endl;

      line(frame, Point(pt1_x, pt1_y) , Point(pt2_x , pt2_y) , Scalar(0,0,255) , 2 , 8);
    }



   for(size_t i = 0; i < selected_lines.size(); i++)
    {
      //cout << "i : " << i << endl;
      Vec4i I = selected_lines[i];
      line(frame, Point(I[0], I[1]), Point(I[2], I[3]) , Scalar(255,0,0) , 2 , 8);
    }





    //ROS_INFO("cols : %d , rows : %d" , frame.cols, frame.rows);

    sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    sensor_msgs::ImagePtr pub_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", copyImg).toImageMsg();
    pub.publish(pub_msg);
    pub2.publish(pub_msg2);


  });

  ros::spin();

  return 0;

}
