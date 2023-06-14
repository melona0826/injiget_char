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
      pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" ,1 , true);
      pub_tilt_ = nh_.advertise<std_msgs::String>("/tilt/mode", 1, true);
      pub_terminate_ = nh_.advertise<std_msgs::String>("/line_moving/terminate", 1, true);
      pub_ocr_start_ = nh_.advertise<std_msgs::String>("/ocr/toggle" , 1, true);
      // pub_obj_ = nh_.advertise<std_msgs::String>("/object/name" , 1 , true);
      sub_start_ = nh_.subscribe("/drive_start/toggle", 1, &SubAndPub::toggleCallback, this);
      sub_ = nh_.subscribe("/line_detect/line_pos", 1,  &SubAndPub::callback, this);
      // sub_obj_name = nh_.subscribe("/object/name", 1, &SubAndPub::objCallBack, this);
      sub_finish_ = nh_.subscribe("/finish_line_detect/line_pos" , 1, &SubAndPub::finsihCallback, this);
    }

    void toggleCallback(const std_msgs::String& msg)
    {
      if(msg.data == "Start")
        start_toggle = 1;
    }

    // void objCallBack(const std_msgs::String& msg)
    // {
    //   obj_name_msg.data = msg.data;
    // }

    void finsihCallback(const geometry_msgs::Pose2D& msg)
    {
      if(start_toggle)
      {
        ROS_INFO("[FINISH LINE] y : %f" , msg.y);
        if(!finish_toggle && msg.y > 30)
        {
          finish_toggle = 1;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = 0.0;
          pub_.publish(cmd_vel);
        }
        else if(finish_toggle && msg.theta < finish_center - finishi_tor)
        {
          finish_toggle = 1;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = 0.02;
          pub_.publish(cmd_vel);
          ROS_INFO("LEFT !");
        }

        else if(finish_toggle && msg.theta > finish_center + finishi_tor)
        {
          finish_toggle = 1;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = -0.02;
          pub_.publish(cmd_vel);
          ROS_INFO("RIGHT !");
        }

        else if(finish_toggle)
        {
          ROS_INFO("DDD");
          tilt_mode.data = "front";
          pub_tilt_.publish(tilt_mode);
          terminate_msg.data = "Terminate";
          pub_terminate_.publish(terminate_msg);
          std::this_thread::sleep_for(2s);

          ocr_toggle_msg.data = "Start";
          // pub_obj_.publish(obj_name_msg);
          pub_ocr_start_.publish(ocr_toggle_msg);

          ros::shutdown();
        }

        ROS_INFO("[FINISH LINE] m : %f" , msg.theta);

      }
    }

    void callback(const geometry_msgs::Pose2D& msg)
    {
      if(start_toggle && !finish_toggle)
      {
        ROS_INFO("X : %f" , msg.x);
        ROS_INFO("Y : %f" , msg.y);
        ROS_INFO("Theta : %f" , msg.theta);

        if(msg.x > center + tor)
        {
          cmd_vel.angular.z = -0.18;
          ROS_INFO(" RIGHT ! ");
        }

        else if(msg.x < center - tor)
        {
          cmd_vel.angular.z = 0.18;
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
    // ros::Publisher pub_obj_;
    ros::Subscriber sub_start_;
    ros::Subscriber sub_;
    ros::Subscriber sub_finish_;
    ros::Subscriber sub_obj_name;
    geometry_msgs::Twist cmd_vel;
    std_msgs::String tilt_mode;
    std_msgs::String terminate_msg;
    std_msgs::String ocr_toggle_msg;
    std_msgs::String obj_name_msg;
    int start_toggle = 0;
    int finish_toggle = 0;
    int center = 320;
    double finish_center = 0.0;
    double finishi_tor = 0.8;
    int tor = 60;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_drive_node");
  SubAndPub subPub;
  ros::spin();
  return 0;
}

