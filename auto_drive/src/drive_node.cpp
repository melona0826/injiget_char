#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <vector>

using namespace std;

class SubAndPub
{
  public:
    SubAndPub()
    {
      pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" ,100 , true);
      sub_ = nh_.subscribe("/line_detect/line_pos", 1,  &SubAndPub::callback, this);
    }

    void callback(const geometry_msgs::Pose2D& msg)
    {
      ROS_INFO("X : %f" , msg.x);
      ROS_INFO("Y : %f" , msg.y);
      ROS_INFO("Theta : %f" , msg.theta);

      if(msg.x > 400)
      {
        cmd_vel.angular.z = -0.2;
        ROS_INFO(" RIGHT ! ");
      }

      else if(msg.x < 300)
      {
        cmd_vel.angular.z = 0.2;
        ROS_INFO(" LEFT ! ");
      }

      else
      {
        cmd_vel.angular.z = 0.0;
      }

      cmd_vel.linear.x = 0.05;
      cmd_vel.linear.y= 0.0;
      pub_.publish(cmd_vel);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    geometry_msgs::Twist cmd_vel;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_drive");
  SubAndPub subPub;
  ros::spin();
  return 0;
}

