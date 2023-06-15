/***********************************************************************
 * place_node.cpp
 *
 * Author : Kwon Jin (School of Computer, KAIST)
 * Date (Last Modify):2023.06.15
 *
 * Line tracking node cpp code.
 *
 * Determines robot turns right or turns left by the x coordinate and
 * slope of the topic messages from detection nodes.
 *
 * Detection Node cpp File :
 *   line_detect/detect_node.cpp
 *   line_detect/finish_line_detect_node.cpp
 *
 **********************************************************************/

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <vector>
#include <chrono>
#include <thread>
#include <math.h>

using namespace std;
using namespace std::chrono_literals;

class SubAndPub
{
  public:
    SubAndPub()
    {
      /*****************************************************************
       * Notation)
       *   Node Name = NN , Message Type = MT , Callback Function = CF
       *
       * (Publishers)
       * pub_      :
       *   turtlebot control topic publisher
       *   (NN : "cmd_vel")
       *
       * pub_reset_ :
       *   reset OCR toggle publisher
       *   (NN : "/ocr/rest")
       *
       * pub_tilt_ :
       *   tilt motor control topic publisher
       *   (NN : "/tilt/mode")
       *
       * pub_term_ :
       *   terminate toggle of OCR publisher
       *   (NN : "/ocr/terminate")
       *
       * (Subscribers)
       * sub_start_ :
       *   place start toggle topic subscriber
       *   (NN : "/place/toggle" , CF : toggleCallback)
       *
       * sub_imu_ :
       *   imu sensor data subscriber
       *   (NN : "/imu" , CF : imuCallback)
       *
       * sub_odom_ :
       *   odometry data subscriber
       *   (NN : "/odom" , CF : odomCallback)
       *
       * sub_ :
       *   detected place position subscriber
       *   (NN : "/ocr/place_pos" , CF : callback)
      */
      pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" ,1 , true);
      pub_reset_ = nh_.advertise<std_msgs::String>("/ocr/reset", 1, true);
      pub_tilt_ = nh_.advertise<std_msgs::String>("/tilt/mode", 1, true);
      pub_fork_ = nh_.advertise<std_msgs::String>("/fork/mode", 1, true);
      pub_term_ = nh_.advertise<std_msgs::String>("/ocr/terminate", 1, true);

      sub_start_ = nh_.subscribe("/place/toggle", 1,  &SubAndPub::toggleCallback, this);
      sub_imu_ = nh_.subscribe("/imu", 1, &SubAndPub::imuCallback, this);
      sub_odom_ = nh_.subscribe("/odom", 1, &SubAndPub::odomCallback, this);
      sub_ = nh_.subscribe("/ocr/place_pos", 1,  &SubAndPub::callback, this);
    }

    /* resetCallback Function
    Params : (const std_msgs::String& msg)
    Return : void

    If msg.data is "Reset", set the value of the final_place_toggle as 0
    to start.
    */
    void resetCallback(const std_msgs::String& msg)
    {
      if(msg.data == "Reset")
        final_place_toggle = 0;
    }

    /* odomCallback Function
    Params : (nav_msgs::Odometry& msg)
    Return : void

    Move forward as much as distance that from robot to station by
    using odometry information
    */
    void odomCallback(const nav_msgs::Odometry& msg)
    {
      if(start_toggle)
      {

        // While turning, set start position as current position
        if(turn_toggle)
        {
          start_pos = msg.pose.pose.position.y;
          ROS_INFO("Start pos : %f", start_pos);
        }

        // Forward Moving
        if(forward_toggle)
        {
          ROS_INFO("ref_dist : %f" , ref_dist);
          ROS_INFO("dist : %f" , abs(msg.pose.pose.position.y - start_pos));
          ROS_INFO("start pos : %f" , start_pos);
          ROS_INFO("cur pos : %f", msg.pose.pose.position.y);

          // Forward as much as computed distance (from robot to station)
          if(abs(msg.pose.pose.position.y - start_pos) < ref_dist)
          {
            cmd_vel.linear.x = 0.2;
          }

          else
          {
            cmd_vel.linear.x = 0.0;
            forward_toggle = 0;
            recover_toggle = 1;
          }
        }
      }
    }

    /* quart2euler Function
    Params : (double) x, (double) y, (double) z, (double) w
    Return : convert value from quaternion to euler angle of z-axis

    convert value of quaternion to euler angle of z-aixs
    */
    double quart2euler(double x, double y, double z, double w)
    {
      return atan((2 * (w*z + x*y) / (1 - 2 * (y * y + z * z)))) * 180.0 / CV_PI;
    }

    /* abs Function
    Params : (double) ang
    Return : absolute value of angle

    Return absolute value of give angle
    */
    double abs(double ang)
    {
      if(ang < 0.0)
        ang *= -1.0;

      return ang;
    }
    /* imuCallback function
    Params : (const sensor_msgs::Imu&) msg
    Return : void

    Turn the robot 90 degree by using imu sensor data
    */
    void imuCallback(const sensor_msgs::Imu& msg)
    {
      if(first_toggle)
      {
        // Assign current euler angle to start_ang
        start_ang = quart2euler(msg.orientation.x, msg.orientation.y, msg.orientation.z , msg.orientation.w);
      }

      if(turn_toggle)
      {
        // Update current angle
        angle = quart2euler(msg.orientation.x, msg.orientation.y, msg.orientation.z , msg.orientation.w);

        // Turn while turning angle over 89.7
        if(abs(angle - start_ang) < 89.7)
        {
          ROS_INFO("Ang : %f", angle);
          cmd_vel.angular.z = turn_spd;
          ROS_INFO("Move !");
        }

        // Over 89.7, stop.
        // Then off turn_toggle and on the forward_toggle
        else
        {
          cmd_vel.angular.z = 0;
          turn_toggle = 0;
          forward_toggle = 1;
        }
      }

      // Turn back to start angle
      else if(recover_toggle)
      {
        angle = quart2euler(msg.orientation.x, msg.orientation.y, msg.orientation.z , msg.orientation.w);

        if(abs(angle - start_ang) > 0.15)
        {
          ROS_INFO("recov Ang : %f", angle);
          cmd_vel.angular.z = -1 * turn_spd;
          ROS_INFO("Move !");
        }

        else
        {
          cmd_vel.angular.z = 0;
          turn_toggle = 0;
          forward_toggle = 0;
          recover_toggle = 0;
          final_place_toggle = 1;
        }
      }
    }

    /* toggleCallback Function
    Params : (const std_msgs::String& msg)
    Return : void

    If msg.data is "Start", set the value of the start_toggle as 1 to
    start.
    */
    void toggleCallback(const std_msgs::String& msg)
    {
      if(msg.data == "Start")
        start_toggle = 1;
    }

    /* callback Function
    Params : (const geometry_msgs::Pose2D& msg)
    Return : void

    Get Pose2D message from the OCR node, and
    determins the angular and linear speed of injiget_char by the Pose2D
    information and drive to station
    */
    void callback(const geometry_msgs::Pose2D& msg)
    {
      if(start_toggle)
      {
        if(first_toggle)
        {
          ROS_INFO("X : %f" , msg.x);
          ROS_INFO("Y : %f" , msg.y);
          turn_toggle = 1;

          // (difference in pixel) * (1m per pixel)
          ref_dist = (abs(center - msg.x) ) * ( height / ( msg.y * 1000));
          first_toggle = 0;

          // Determin turn direction
          if(msg.x > center + tor)
            turn_spd *= -1;
        }

        else if(final_place_toggle)
        {
          // Publish reset_msg to restart ocr node
          reset_msg.data = "Reset";
          pub_reset_.publish(reset_msg);

          if(!reset_toggle)
          {
            reset_toggle = 1;
          }


          else
          {
            ROS_INFO("X : %f, Y : %f", msg.x, msg.y);

            // Robot come close enough to station
            if(msg.y >= 200)
            {
              // Stop robot
              cmd_vel.linear.x = 0.0;
              cmd_vel.linear.y= 0.0;
              cmd_vel.angular.z = 0.0;
              pub_.publish(cmd_vel);
              std::this_thread::sleep_for(2s);

              // Forward more to station
              for(int i = 0; i < 30; i++)
              {
                cmd_vel.linear.x = 0.3;
                cmd_vel.linear.y= 0.0;
                cmd_vel.angular.z = 0.0;
                pub_.publish(cmd_vel);
              }
              std::this_thread::sleep_for(2s);

              // Down the pallet at station
              tilt_mode.data = "line";
              fork_mode.data = "down";
              pub_tilt_.publish(tilt_mode);
              pub_fork_.publish(fork_mode);
              std::this_thread::sleep_for(1s);

              // Backward to escape station
              for(int i = 0; i < 20; i++)
              {
                cmd_vel.linear.x = -0.3;
                cmd_vel.linear.y= 0.0;
                cmd_vel.angular.z = 0.0;
                pub_.publish(cmd_vel);
              }
              std::this_thread::sleep_for(2s);

              // Stop
              cmd_vel.linear.x = -0.0;
              cmd_vel.linear.y= 0.0;
              cmd_vel.angular.z = 0.0;
              pub_.publish(cmd_vel);
              std::this_thread::sleep_for(1s);

              // Pubilsh terminate msg to ocr node and
              // terminate node.
              term_msg.data = "Terminate";
              pub_term_.publish(term_msg);
              std::this_thread::sleep_for(2s);

              ros::shutdown();

            }

            // Drive to object station by using position information from
            // OCR node.
            if(msg.x < center-tor)
            {
              cmd_vel.linear.x = 0.0;
              cmd_vel.angular.z = 0.08;
              ROS_INFO("LEFT !");
            }

            else if(msg.x > center+tor)
            {
              cmd_vel.linear.x = 0.0;
              cmd_vel.angular.z = -0.08;
              ROS_INFO("RIGHT !");
            }

            else
            {
              cmd_vel.angular.z = 0.0;
              cmd_vel.linear.x = 0.13;
              ROS_INFO("GOGOYA !");
            }

          }


        }

        pub_.publish(cmd_vel);
      }


    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_reset_;
    ros::Publisher pub_tilt_;
    ros::Publisher pub_fork_;
    ros::Publisher pub_term_;
    ros::Subscriber sub_;
    ros::Subscriber sub_start_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_odom_;
    std_msgs::String tilt_mode;
    std_msgs::String fork_mode;
    std_msgs::String terminate_msg;
    std_msgs::String drive_toggle_msg;
    std_msgs::String reset_msg;
    std_msgs::String term_msg;
    geometry_msgs::Twist cmd_vel;

    // toggles
    int start_toggle = 0;
    int first_toggle = 1;
    int turn_toggle = 0;
    int forward_toggle = 0;
    int recover_toggle = 0;
    int final_place_toggle = 0;
    int toggle = 0;
    int reset_toggle = 0;

    // Tuning parameters
    int center = 320;
    int tor = 20;
    double height = 117;
    double turn_spd = 0.2;

    double ref_dist = 0;
    double start_pos;
    double start_ang;
    double angle = 0;


};

/* main function
Params : (int) argc, (char**) argv
Return : 0 (Success)

Init the node name as "place_node".
Make a object of SubAndPub class and publish and subscribe by ros spin().
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "place_node");
  SubAndPub subPub;
  ros::spin();
  return 0;
}

