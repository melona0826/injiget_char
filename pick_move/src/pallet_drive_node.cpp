/***********************************************************************
 * pallet_drive_node.cpp
 *
 * Author : Kwon Jin (School of Computer, KAIST)
 * Date (Last Modify):2023.06.15
 *
 * Pallet pick Node
 * Pick the pallet with detection nodes.
 *
 * Detection Node cpp File :
 *   pallet_det/pallet_det_node.cpp
 *
 **********************************************************************/

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <vector>
#include <chrono>
#include <thread>

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
       * pub_tilt_ :
       *   tilt motor control topic publisher
       *   (NN : "/tilt/mode")
       *
       * pub_fork_ :
       *   fork control topic publisher
       *   (NN : "/fork/mode")
       *
       * pub_terminate_ :
       *   terminate toggle of line driving node &
       *   line detection node topic publisher
       *   (NN : "/drive_start/toggle")
       *
       * pub_drive_toggle_ :
       *   start line dirving toggle publisher
       *   (NN : "/drive_start/toggle")
       *
       *
       * (Subscribers)
       * sub_start_ :
       *   pick start toggle topic subscriber
       *   (NN : "/pallet_det/toggle" , CF : toggleCallback)
       *
       * sub_ :
       *   detected pallet position subscriber
       *   (NN : "/pallet_det/pallet_pos" , CF : callback)
       *
      */
      pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" ,1 , true);
      pub_tilt_ = nh_.advertise<std_msgs::String>("/tilt/mode", 1, true);
      pub_fork_ = nh_.advertise<std_msgs::String>("/fork/mode", 1, true);
      pub_terminate_ = nh_.advertise<std_msgs::String>("/pallet_pick/terminate", 1, true);
      pub_drive_toggle_ = nh_.advertise<std_msgs::String>("/drive_start/toggle", 1, true);
      sub_start_ = nh_.subscribe("/pallet_det/toggle", 1,  &SubAndPub::toggleCallback, this);
      sub_ = nh_.subscribe("/pallet_det/pallet_pos", 1,  &SubAndPub::callback, this);

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

    Get Pose2D message from the pallet detection node, and
    determins the angular and linear speed of injiget_char by the Pose2D
    information and pick pallet.
    */
    void callback(const geometry_msgs::Pose2D& msg)
    {
      if(start_toggle)
      {
        ROS_INFO("X : %f" , msg.x);
        ROS_INFO("Y : %f" , msg.y);
        ROS_INFO("Theta : %f" , msg.theta);

        // If pallet is close, down the tilt motor and change the center
        // and torlance more thight.
        if(msg.y > 380 && toggle == 0 && tilt_down_toggle == 0)
        {
          tilt_mode.data = "object";
          toggle += 1;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = 0.0;
          pub_.publish(cmd_vel);
          pub_tilt_.publish(tilt_mode);
          std::this_thread::sleep_for(3s);
          tilt_down_toggle = 1;
          turn_f_speed = 0.0;
          turn_speed = 0.02;
          center = down_center;
          tor = down_tor;
        }

        // If pallet position is special message or pallet is close,
        // pick the pallet
        else if(tilt_down_toggle == 1 && (msg.y > 450 || msg.y == -333))
        {
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = 0.0;
          pub_.publish(cmd_vel);
          std::this_thread::sleep_for(2s);

          cmd_vel.linear.x = 0.3;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = 0.0;
          for(int i = 0; i < 1; i++)
            pub_.publish(cmd_vel);
          std::this_thread::sleep_for(2s);

          tilt_mode.data = "line";
          fork_mode.data = "up";
          pub_tilt_.publish(tilt_mode);
          pub_fork_.publish(fork_mode);
          terminate_msg.data = "Terminate";
          pub_terminate_.publish(terminate_msg);

          std::this_thread::sleep_for(1s);

          drive_toggle_msg.data = "Start";
          pub_drive_toggle_.publish(drive_toggle_msg);
          ros::shutdown();
        }

        // If pallet position is tilted left or right,
        // then turn the robot to pick pallet.

        else if(msg.x > center + tor)
        {
          cmd_vel.linear.x = turn_f_speed;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = -turn_speed ;
          ROS_INFO(" RIGHT ! ");
        }

        else if(msg.x < center - tor)
        {
          cmd_vel.linear.x = turn_f_speed;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = turn_speed ;
          ROS_INFO(" LEFT ! ");
        }

        else
        {
          cmd_vel.linear.x = 0.07;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = 0.0;
        }

        pub_.publish(cmd_vel);
        pub_tilt_.publish(tilt_mode);
      }
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_tilt_;
    ros::Publisher pub_fork_;
    ros::Publisher pub_terminate_;
    ros::Publisher pub_drive_toggle_;
    ros::Subscriber sub_;
    ros::Subscriber sub_start_;
    std_msgs::String tilt_mode;
    std_msgs::String fork_mode;
    std_msgs::String terminate_msg;
    std_msgs::String drive_toggle_msg;
    geometry_msgs::Twist cmd_vel;

    // Toggles
    int start_toggle = 0;
    int toggle = 0;
    int tilt_down_toggle = 0;

    // Tuining parameters
    int down_center = 325;
    int down_tor = 7;
    float turn_f_speed = 0.0;
    float turn_speed = 0.05;
    int center = 320;
    int tor = 10;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pallet_drive_node");
  SubAndPub subPub;
  ros::spin();
  return 0;
}

