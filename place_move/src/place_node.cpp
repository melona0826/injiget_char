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
      pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" ,1 , true);
      pub_reset_ = nh_.advertise<std_msgs::String>("ocr/reset", 1, true);
      sub_start_ = nh_.subscribe("/place/toggle", 1,  &SubAndPub::toggleCallback, this);
      sub_imu_ = nh_.subscribe("/imu", 1, &SubAndPub::imuCallback, this);
      sub_odom_ = nh_.subscribe("/odom", 1, &SubAndPub::odomCallback, this);
      sub_ = nh_.subscribe("/ocr/place_pos", 1,  &SubAndPub::callback, this);
    }

    void resetCallback(const std_msgs::String& msg)
    {
      if(msg.data == "Reset")
        final_place_toggle = 0;
    }

    void odomCallback(const nav_msgs::Odometry& msg)
    {
      if(start_toggle)
      {

        if(turn_toggle)
        {
          start_pos = msg.pose.pose.position.y;
          ROS_INFO("Start pos : %f", start_pos);
        }


        if(forward_toggle)
        {
          ROS_INFO("ref_dist : %f" , ref_dist);
          ROS_INFO("dist : %f" , abs(msg.pose.pose.position.y - start_pos));
          ROS_INFO("start pos : %f" , start_pos);
          ROS_INFO("cur pos : %f", msg.pose.pose.position.y);
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

    double quart2euler(double x, double y, double z, double w)
    {
      return atan((2 * (w*z + x*y) / (1 - 2 * (y * y + z * z)))) * 180.0 / CV_PI;
    }

    double abs(double ang)
    {
      if(ang < 0.0)
        ang *= -1.0;

      return ang;
    }

    void imuCallback(const sensor_msgs::Imu& msg)
    {
      if(first_toggle)
      {
        start_ang = quart2euler(msg.orientation.x, msg.orientation.y, msg.orientation.z , msg.orientation.w);
      }

      if(turn_toggle)
      {
        angle = quart2euler(msg.orientation.x, msg.orientation.y, msg.orientation.z , msg.orientation.w);

        if(abs(angle - start_ang) < 89.8)
        {
          ROS_INFO("Ang : %f", angle);
          cmd_vel.angular.z = turn_spd;
          ROS_INFO("Move !");
        }

        else
        {
          cmd_vel.angular.z = 0;
          turn_toggle = 0;
          forward_toggle = 1;
        }
      }

      else if(recover_toggle)
      {
        angle = quart2euler(msg.orientation.x, msg.orientation.y, msg.orientation.z , msg.orientation.w);

        if(abs(angle - start_ang) > 0.1)
        {
          ROS_INFO("Ang : %f", angle);
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

    void toggleCallback(const std_msgs::String& msg)
    {
      if(msg.data == "Start")
        start_toggle = 1;
    }

    void callback(const geometry_msgs::Pose2D& msg)
    {
      if(start_toggle)
      {
        if(first_toggle)
        {
          ROS_INFO("X : %f" , msg.x);
          ROS_INFO("Y : %f" , msg.y);
          turn_toggle = 1;
          ref_dist = (abs(center - msg.x)) * (height / (msg.y * 1000));
          first_toggle = 0;

          if(msg.x > center + tor)
            turn_spd *= -1;
        }

        else if(final_place_toggle)
        {
          reset_msg.data = "Reset";
          pub_reset_.publish(reset_msg);
        }



        pub_.publish(cmd_vel);
      }
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_reset_;
    ros::Subscriber sub_;
    ros::Subscriber sub_start_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_odom_;
    std_msgs::String terminate_msg;
    std_msgs::String drive_toggle_msg;
    std_msgs::String reset_msg;
    geometry_msgs::Twist cmd_vel;
    int start_toggle = 0;
    int first_toggle = 1;
    int turn_toggle = 0;
    int forward_toggle = 0;
    int recover_toggle = 0;
    int final_place_toggle = 0;
    int center = 320;
    int tor = 10;
    int toggle = 0;
    double ref_dist = 0;
    double start_pos;
    double start_ang;
    double height = 115;
    double angle = 0;
    double scale_factor = 0;
    double turn_spd = 0.2;


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "place_node");
  SubAndPub subPub;
  ros::spin();
  return 0;
}

