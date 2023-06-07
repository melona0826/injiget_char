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
      pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" ,100 , true);
      pub_tilt_ = nh_.advertise<std_msgs::String>("/tilt/mode", 100, true);
      pub_fork_ = nh_.advertise<std_msgs::String>("/fork/mode", 100, true);
      pub_terminate_ = nh_.advertise<std_msgs::String>("/pallet_pick/terminate", 1, true);
      sub_ = nh_.subscribe("/pallet_det/pallet_pos", 1,  &SubAndPub::callback, this);
    }

    void callback(const geometry_msgs::Pose2D& msg)
    {
      ROS_INFO("X : %f" , msg.x);
      ROS_INFO("Y : %f" , msg.y);
      ROS_INFO("Theta : %f" , msg.theta);

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
        center = 320;
        tor = 7;
      }

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

        ros::shutdown();
      }

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
        cmd_vel.linear.x = 0.1;
        cmd_vel.linear.y= 0.0;
        cmd_vel.angular.z = 0.0;
      }

      pub_.publish(cmd_vel);
      pub_tilt_.publish(tilt_mode);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_tilt_;
    ros::Publisher pub_fork_;
    ros::Publisher pub_terminate_;
    ros::Subscriber sub_;
    std_msgs::String tilt_mode;
    std_msgs::String fork_mode;
    std_msgs::String terminate_msg;
    geometry_msgs::Twist cmd_vel;
    float turn_f_speed = 0.1;
    float turn_speed = 0.1;
    int center = 320;
    int tor = 7;
    int toggle = 0;
    int tilt_down_toggle = 0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pallet_drive_node");
  SubAndPub subPub;
  ros::spin();
  return 0;
}

