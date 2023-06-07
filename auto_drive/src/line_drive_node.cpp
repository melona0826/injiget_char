#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <vector>
#include <chrono>
#include <thread>

using namespace std;

class SubAndPub
{
  public:
    SubAndPub()
    {
      pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" ,100 , true);
      pub_tilt_ = nh_.advertise<std_msgs::String>("/tilt/mode", 100, true);
      pub_terminate_ = nh_.advertise<std_msgs::String>("/line_moving/terminate", 100, true);
      pub_ocr_start_ = nh_.advertise<std_msgs::String>("/ocr/toggle" , 100, true);
      pub_obj_ = nh_.advertise<std_msgs::String>("/object/name" , 100 , true);
      sub_start_ = nh_.subscribe("/drive_start/toggle", 1, &SubAndPub::toggleCallback, this);
      sub_ = nh_.subscribe("/line_detect/line_pos", 1,  &SubAndPub::callback, this);
      sub_obj_name = nh_.subscribe("/object/name", 1, &SubAndPub::objCallBack, this);
      sub_finish_ = nh_.subscribe("/finish_line_detect/line_pos" , 1, &SubAndPub::finsihCallback, this);
    }

    void objCallBack(const std_msgs::String& msg)
    {
      obj_name_msg.data = msg.data;
    }

    void finsihCallback(const geometry_msgs::Pose2D& msg)
    {
      if(!finish_toggle && msg.y > 80)
      {
        finish_toggle = 1;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y= 0.0;
        cmd_vel.angular.z = 0.0;
        pub_.publish(cmd_vel);
      }
      else if(finish_toggle && msg.x < finish_center - finishi_tor)
      {
        finish_toggle = 1;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y= 0.0;
        cmd_vel.angular.z = 0.02;
        pub_.publish(cmd_vel);
      }

      else if(finish_toggle && msg.x > finish_center + finishi_tor)
      {
        finish_toggle = 1;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y= 0.0;
        cmd_vel.angular.z = -0.02;
        pub_.publish(cmd_vel);
      }

      else if(finish_toggle)
      {
        ROS_INFO("DDD");
        tilt_mode.data = "front";
        pub_tilt_.publish(tilt_mode);
        terminate_msg.data = "Terminate";
        pub_terminate_.publish(terminate_msg);
        std::this_thread::sleep_for(1s);

        ocr_toggle_msg.data = "Start";
        pub_obj_.publish(obj_name_msg);
        pub_ocr_start_.publish(ocr_toggle_msg);

        ros::shutdown();
      }

      ROS_INFO("[FINISH LINE] X : %f  |  Y : %f" , msg.x, msg.y);

    }

    void callback(const geometry_msgs::Pose2D& msg)
    {
      if(!finish_toggle)
      {
        ROS_INFO("X : %f" , msg.x);
        ROS_INFO("Y : %f" , msg.y);
        ROS_INFO("Theta : %f" , msg.theta);

        if(msg.x > center + tor)
        {
          cmd_vel.angular.z = -0.15;
          ROS_INFO(" RIGHT ! ");
        }

        else if(msg.x < center - tor)
        {
          cmd_vel.angular.z = 0.15;
          ROS_INFO(" LEFT ! ");
        }

        else
        {
          cmd_vel.angular.z = 0.0;
        }

        cmd_vel.linear.x = 0.1;
        cmd_vel.linear.y= 0.0;
        pub_.publish(cmd_vel);
      }

    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_tilt_;
    ros::Publisher pub_terminate_;
    ros::Publisher pub_ocr_start_;
    ros::Publisher pub_obj_;
    ros::Subscriber sub_start_;
    ros::Subscriber sub_;
    ros::Subscriber sub_finish_;
    ros::Subscriber sub_obj_name;
    geometry_msgs::Twist cmd_vel;
    std_msgs::String tilt_mode;
    std_msgs::String terminate_msg;
    std_msgs::String ocr_toggle_msg;
    std_msgs::String obj_name_msg;
    std_msgs::String start_toggle;
    int finish_toggle = 0;
    int center = 320;
    int finish_center = 180;
    int finishi_tor = 35;
    int tor = 60;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_drive_node");
  SubAndPub subPub;
  ros::spin();
  return 0;
}

