/***********************************************************************
 * pallet_det_node.cpp
 *
 * Author : Kwon Jin (School of Computer, KAIST)
 * Date (Last Modify):2023.06.15
 *
 * Pallet detecting Node.
 * Detect pallet with hough line detection.
 *
 **********************************************************************/

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <chrono>
#include <thread>

using namespace std;
using namespace cv;

/***********************************************************************
* Tunning Parameters.
***********************************************************************/

// hsv for pallet (yellow)
int h_min = 40;
int h_max = 70;

int s_min = 40;
int s_max = 100;

int v_min = 40;
int v_max = 100;

int terminate_count = 0;

/* callTerminate function
Parameters : (const std_msgs::String& msg)
Return : void

If msg.data is "Terminate",
terminate node.
*/
void callTerminate(const std_msgs::String& msg)
{
  if(msg.data == "Terminate")
    ros::shutdown();
}

/* main function
Parameters : (int) argc, (char**) argv
Return : void

Init the node name as "line_detect" and detect the line by using hough
line detection.
*/
int main(int argc, char** argv)
{
  // Init node name as "pallet_det"
  ros::init(argc, argv, "pallet_det");
  ros::NodeHandle nh;

  /*********************************************************************
   * Notation)
   *   Node Name = NN , Message Type = MT , Callback Function = CF
   *
   * (Publishers)
   * pub_tilt      :
   *   tilt mode publisher
   *   (NN : "/tilt/mode")
   *
   * pub_fork      :
   *   fork mode publisher
   *   (NN : "/fork/mode")
   *
   * pub_fitLine      :
   *   detected pallet position publisher
   *   (NN : "/pallet_det/pallet_pos")
   *
   * (Subscribers)
   * terminate_sub :
   *   terminate toggle topic subscriber
   *   (NN : "/pallet_pick/terminate" , CF : callTerminate)
   *
  */
  ros::Publisher pub_tilt = nh.advertise<std_msgs::String>("/tilt/mode", 1, true);
  ros::Publisher pub_fork = nh.advertise<std_msgs::String>("/fork/mode", 1, true);
  ros::Publisher pub_fitLine = nh.advertise<geometry_msgs::Pose2D>("/pallet_det/pallet_pos",1);
  ros::Subscriber terminate_sub = nh.subscribe("/pallet_pick/terminate", 1, callTerminate);

  /*
  tilt_mode msg
    Type : std_msgs/String
    mode list : ("front", "line" , "object")

  fork_mode msg
    Type : std_msgs/String
    mode list : ("up", "down" )
  */
  std_msgs::String tilt_mode;
  std_msgs::String fork_mode;

  // See front and fork down
  tilt_mode.data = "front";
  fork_mode.data = "down";
  pub_tilt.publish(tilt_mode);
  pub_fork.publish(fork_mode);
  std::this_thread::sleep_for(1s);

  /*  fitLine_msg
    Type : geometry_msgs/Pose2D

    msg.x = b.x   (x value of point on line)
    msg.y = b.y   (y value of point on line)
    msg.theta = m (Radian degree of line)
  */
  geometry_msgs::Pose2D fitLine_msg;

  // Convert hsv value to opencv format
  // (cv hsv range  h : 0~180 | s : 0~100 | v: 0~100)
  h_min = (int)(h_min/2);
  h_max = (int)(h_max/2);

  s_min = (int)(s_min * 255 / 100);
  s_max = (int)(s_max * 255 / 100);

  v_min = (int)(v_min * 255 / 100);
  v_max = (int)(v_max * 255 / 100);

  image_transport::ImageTransport it(nh);
  /*********************************************************************
   * Notation)
   *   Node Name = NN , Message Type = MT , Callback Function = CF
   *
   * (image_transport Publishers)
   * pub      :
   *   edge image publisher
   *   (NN : "/pallet_det/edge_image")
   *
   * pub2      :
   *   mask detected line image publisher
   *   (NN : "/pallet_det/det_image")
   *
   * (image_transportSubscribers)
   * sub :
   *   raw image from usb webcam subscriber
   *   (NN : "/usb_cam/image_raw" , CF : dirctly implment)
   *
  */
  image_transport::Publisher pub = it.advertise("/pallet_det/edge_image", 1);
  image_transport::Publisher pub2 = it.advertise("/pallet_det/det_image", 1);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1,
  [&](const sensor_msgs::ImageConstPtr& msg) {
    // Convert image message to cv Mat type
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

    // Preprocess (Flip , brightness)
    flip(frame, frame, -1);
    frame = frame * 0.8;

    Mat img_hsv, img_pallet, img_edge, img_recog;
    Mat pallet_mask;
    vector<Vec4i> linesP;
    vector<Point> pts;
    Vec4d fit_line;
    Point pt1, pt2;

    // Recognize Slope angle treshold
    double slope_treshold = (90 - 60) * CV_PI / 180.0;

    frame.copyTo(img_recog);

    // Color filtering with hsv
    cvtColor(frame, img_hsv, COLOR_BGR2HSV);
    inRange(img_hsv, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), pallet_mask);
    bitwise_and(frame, frame, img_pallet,  pallet_mask);

    // Canny edge detectioin
    Canny(img_pallet, img_edge, 80, 100);

    // Find lines with probablitic hough line detection
    HoughLinesP(img_edge, linesP, 1, (CV_PI/180), 10, 18, 10);
    double m = 0;
    int count = 0;

    // Select lines in detected lines by slope threshold
    for(size_t i = 0; i < linesP.size(); i++)
    {
      Vec4i l = linesP[i];
      pt1 = Point(l[0], l[1]);
      pt2 = Point(l[2], l[3]);

      // slope = (y increment) / (x increment)
      double slope = (static_cast<double>(pt1.y) - static_cast<double>(pt2.y)) / (static_cast<double>(pt1.x) - static_cast<double>(pt2.x) );

      // select line only over the slope treshold
      if(abs(slope) >= slope_treshold)
      {
        line(frame, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(0,0,255), 2, 8);
        count ++;
        m = slope;
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

      int pt1_y = frame.rows;
      int pt2_y = 0;

      double pt1_x = ((pt1_y - b.y) / m) + b.x;
      double pt2_x = ((pt2_y - b.y) / m) + b.x;

      // cout << "slope : " << (static_cast<double>(pt1_y) - static_cast<double>(pt2_y)) / (static_cast<double>(pt1_x) - static_cast<double>(pt2_x)) << endl;

      // Draw fit line as red
      line(frame, Point(pt1_x, pt1_y) , Point(pt2_x , pt2_y) , Scalar(255,0,0) , 2 , 8);

      // Assign the information of the line to fitline_msgs

      fitLine_msg.x = b.x;
      fitLine_msg.y = b.y;
      fitLine_msg.theta = m;

      // cout << "x : " << b.x << endl;;
      // cout << "y : " << b.y << endl;;
      // cout << "m : " << m * CV_PI / 180 << endl;

      // Initalize termintate_count to 0
      terminate_count = 0;
    }

    // terminate_count count for 5 times.
    else if(terminate_count < 5)
    {
      terminate_count += 1;
    }

    // If detection node not find pallet 5 times,
    // publish speical message.
    else
    {
      fitLine_msg.x = -333;
      fitLine_msg.y = -333;
      fitLine_msg.theta = -333;
    }

    // Publish edge image, mask image, detected position
    sensor_msgs::ImageConstPtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_edge).toImageMsg();
    sensor_msgs::ImageConstPtr pub_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(pub_msg);
    pub2.publish(pub_msg2);
    pub_fitLine.publish(fitLine_msg);

  });

  ros::spin();

  return 0;

}

