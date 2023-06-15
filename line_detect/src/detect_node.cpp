/***********************************************************************
 * detect_node.cpp
 *
 * Author : Kwon Jin (School of Computer, KAIST)
 * Date (Last Modify):2023.06.15
 *
 * Line detecting Node.
 * Detect line with hough line detection.
 *
 **********************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <vector>
#include <chrono>
#include <thread>

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

/***********************************************************************
* Tunning Parameters.
***********************************************************************/

// Parameters that crop roi images from the row image
int frame_wid = 640;
int frame_hei = 300;

// ROI image size ( (raw_image width - roi_x*2) X (frame_hei - roi_y) )
int roi_x = 10;
int roi_y = 200;
int roi_width = frame_wid - roi_x*2;
int roi_height = frame_hei - roi_y;

// Line color parameters (HSV Color Space)
int black_hue_low = 0;
int black_hue_high = 180;
int black_sat_low = (int)(20 * 255 / 100);
int black_sat_high = (int)(50 * 255 / 100);
int black_val_low = (int)(10 * 255 / 100);
int black_val_high = (int)(35 * 255 / 100);

int toggle = 0;

/* callTerminate function
Parameters : (const std_msgs::String& msg)
Return : void

The callback function of the terminate_sub subscriber.
If the std_msgs::String topic message data equal to "Terminate",
it terminate the node.
In addtion, it means injiget_char success to line driving and aligned.
*/
void callTerminate(const std_msgs::String& msg)
{
  if(msg.data == "Terminate")
    ros::shutdown();
}

/* toggleCallback function
Parameters : (const std_msgs::String& msg)
Return : void

The callback function of the start_sub subscriber.
If the std_msgs::String topic message data equal to "Start",
it set to toggle as 1 to start the detection.
In addtion, it means injiget_char success to pick the pallet.
*/
void toggleCallback(const std_msgs::String& msg)
{
  if(msg.data == "Start")
    toggle = 1;
}

/* main function
Parameters : (int) argc, (char**) argv
Return : void

Init the node name as "line_detect" and detect the line by using hough
line detection.
*/
int main(int argc, char** argv)
{
  // Node Name : line_detect
  ros::init(argc, argv, "line_detect");

  ros::NodeHandle nh;

  /*  fitLine_msg
    Type : geometry_msgs/Pose2D

    msg.x = b.x   (x value of point on line)
    msg.y = b.y   (y value of point on line)
    msg.theta = m (Radian degree of line)
  */
  geometry_msgs::Pose2D fitLine_msg;

  /*********************************************************************
   * Notation)
   *   Node Name = NN , Message Type = MT , Callback Function = CF
   *
   * (Publishers)
   * pub_fitLine      :
   *   detected line information topic publisher.
   *   (NN : "/line_detect/line_pos")
   *
   * pub      :
   *   detected image message publisher (image_transport Publisher)
   *   (NN : "/line_detect/detect_img")
   *
   * pub2      :
   *   detected line mask image publisher (image_transport Publisher)
   *   (NN : "/line_detect/line_img")
   *
   * (Subscribers)
   * terminate_sub :
   *   terminate toggle topic subscriber
   *   (NN : "/line_moving/terminate" , CF : callTerminate)
   *
   * start_sub :
   *   start toggle topic subscriber
   *   (NN : "/drive_start/toggle", CF : toggleCallback)
   *
   * sub (image_transport Subscriber):
   *   raw_image from the usb web cam subscriber
   *   (NN : "/usb_cam/image_raw",
   *    CF : (Direct implemetation callback function))
   *
  */
  ros::Publisher pub_fitLine = nh.advertise<geometry_msgs::Pose2D>("/line_detect/line_pos", 1);
  ros::Subscriber terminate_sub = nh.subscribe("/line_moving/terminate", 1, callTerminate);
  ros::Subscriber start_sub = nh.subscribe("/drive_start/toggle", 1, toggleCallback);

  // Set image_transport Publishers & Sublscribers
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/line_detect/detect_img", 1);
  image_transport::Publisher pub2 = it.advertise("/line_detect/line_img", 1);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1,
  [&](const sensor_msgs::ImageConstPtr& msg){
    if (toggle)
    {
      // Convert ros topic message to opencv mat type by uisng cv_bridge
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

      // Recognize Slope angle tolerance
      int slope_tor = 70;

      // Recognize Slope angle treshold
      double slope_treshold = (90 - slope_tor) * CV_PI / 180.0;

      Mat img_hsv, line_mask, img_line, img_edge;
      Mat test;
      Mat frame = cv_ptr->image;

      // PreProcessing (Adjust brightness of image)
      flip(frame, frame, -1);
      frame = (frame * 0.8);

      Mat grayImg, blurImg, edgeImg, copyImg;

      Point pt1, pt2;
      vector<Vec4i> lines, selected_lines;
      vector<double> slopes;
      vector<Point> pts;
      Vec4d fit_line;

      // Crop the ROI
      Rect bounds(0, 0, frame.cols, frame.rows);
      Rect roi(roi_x, roi_y, roi_width, roi_height);
      frame = frame(bounds & roi);

      // GaussianBlur for remove noise in frame
      GaussianBlur(frame, frame, Size(5,5), 15);

      // Color Filtering
      cvtColor(frame, img_hsv, COLOR_BGR2HSV);
      inRange(img_hsv, Scalar(black_hue_low, black_sat_low, black_val_low) , Scalar(black_hue_high, black_sat_high, black_val_high), line_mask);
      bitwise_and(frame, frame, img_line, line_mask);
      img_line.copyTo(copyImg);

      // Canny Edge Detection
      cvtColor(img_line, grayImg, COLOR_BGR2GRAY);
      Canny(grayImg, img_edge, 100, 170);

      // Line Dtection with probablistic houghline detection
      HoughLinesP(img_edge, lines, 1, CV_PI / 180 , 6 , 15, 12);

      //cout << "slope treshol : " << slope_treshold << endl;

      // Select lines in detected lines by slope threshold
      for(size_t i = 0; i < lines.size(); i++)
      {
        Vec4i line = lines[i];
        pt1 = Point(line[0] , line[1]);
        pt2 = Point(line[2], line[3]);

        // slope = (y increment) / (x increment)
        double slope = (static_cast<double>(pt1.y) - static_cast<double>(pt2.y)) / (static_cast<double>(pt1.x) - static_cast<double>(pt2.x) );
        //cout << slope << endl;

        // Draw the detected line on frame as green
        cv::line(frame, Point(pt1.x, pt1.y) , Point(pt2.x , pt2.y) , Scalar(0,255,0) , 2 , 8);

        // select line only over the slope treshold
        if(abs(slope) >= slope_treshold)
        {
          selected_lines.push_back(line);
          pts.push_back(pt1);
          pts.push_back(pt2);
        }
      }

      // If selected line is exist.
      if(pts.size() > 0)
      {
        // Compute the line that fit to set of points which on the
        // selected lines.
        fitLine(pts, fit_line, DIST_L2, 0, 0.01, 0.01);

        double m = fit_line[1] / fit_line[0];
        Point b = Point(fit_line[2], fit_line[3]);

        /*
         * We know the slope of the line and one point on the line.
         * Therefore if we set the two points that y is min and max in
         * frame, we can compute x coordinate of the two point by
         * x = (y - y') / m + x' where x' and y' is coordinate of x and
         * y on line.
         *
        */

        int pt1_y = frame.rows;
        int pt2_y = 0;

        double pt1_x = ((pt1_y - b.y) / m) + b.x;
        double pt2_x = ((pt2_y - b.y) / m) + b.x;

        //cout << "slope : " << (static_cast<double>(pt1_y) - static_cast<double>(pt2_y)) / (static_cast<double>(pt1_x) - static_cast<double>(pt2_x)) << endl;

        // Draw fit line as red
        line(frame, Point(pt1_x, pt1_y) , Point(pt2_x , pt2_y) , Scalar(0,0,255) , 2 , 8);

        // Assign the information of the line to fitline_msgs
        fitLine_msg.x = b.x;
        fitLine_msg.y = b.y;
        fitLine_msg.theta = m;
      }


    // Draw selected line as blue
    for(size_t i = 0; i < selected_lines.size(); i++)
    {
      //cout << "i : " << i << endl;
      Vec4i I = selected_lines[i];
      line(frame, Point(I[0], I[1]), Point(I[2], I[3]) , Scalar(255,0,0) , 2 , 8);
    }


    //ROS_INFO("X : %d , Y : %d" , fitLine_msg.x, fitLine_msg.y);

    // Publish line drawed image, detected line mask image and line
    // information
    sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    sensor_msgs::ImagePtr pub_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", copyImg).toImageMsg();
    pub.publish(pub_msg);
    pub2.publish(pub_msg2);
    pub_fitLine.publish(fitLine_msg);
  }

  });



  ros::spin();

  return 0;

}
