/***********************************************************************
 * line_drive_node.cpp
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
       * pub_terminate_ :
       *   terminate toggle of line driving node &
       *   line detection node topic publisher
       *   (NN : "/line_moving/terminate")
       *
       * pub_ocr_start : ocr start toggle topic publisher
       *   (NN : "/ocr/toggle")
       *
       *
       * (Subscribers)
       * sub_start_ :
       *   line driving start toggle topic subscriber
       *   (NN : "/drive_start/toggle" , CF : toggleCallback)
       *
       * sub_ :
       *   line position information topic subscriber
       *   (NN : "/line_detect/line_pos" , CF : callback)
       *
       * sub_finish_ :
       *   finish toggle of line driving topic subscriber
       *   (NN : "/finish_line_detect/line_pos" , CF : finisihCallback)
       *
       *
      */
      pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" ,1 , true);
      pub_tilt_ = nh_.advertise<std_msgs::String>("/tilt/mode", 1, true);
      pub_terminate_ = nh_.advertise<std_msgs::String>("/line_moving/terminate", 1, true);
      pub_ocr_start_ = nh_.advertise<std_msgs::String>("/ocr/toggle" , 1, true);
      sub_start_ = nh_.subscribe("/drive_start/toggle", 1, &SubAndPub::toggleCallback, this);
      sub_ = nh_.subscribe("/line_detect/line_pos", 1,  &SubAndPub::callback, this);
      sub_finish_ = nh_.subscribe("/finish_line_detect/line_pos" , 1, &SubAndPub::finsihCallback, this);
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

    /* finsihCallback Function
    Params : (const geometry_msgs::Pose2D& msg)
    Return : void

    Get Pose2D message from the finish line detection node, and
    determins the angular and linear speed of injiget_char by the Pose2D
    information and align robot at finish line.
    */
    void finsihCallback(const geometry_msgs::Pose2D& msg)
    {
      if(start_toggle)
      {
        ROS_INFO("[FINISH LINE] y : %f" , msg.y);

        // Meet the end horizontal blue line
        if(!finish_toggle && msg.y > 30)
        {
          // Stop and set finish_toggle as 1
          finish_toggle = 1;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = 0.0;
          pub_.publish(cmd_vel);
        }

        // At finish line and robot tilted right.
        else if(finish_toggle && msg.theta < finish_center - finishi_tor)
        {
          // Turn left and set finish_toggle as 1
          finish_toggle = 1;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = 0.02;
          pub_.publish(cmd_vel);
          ROS_INFO("LEFT !");
        }

        // At finish line and robot tilted left.
        else if(finish_toggle && msg.theta > finish_center + finishi_tor)
        {
          // Turn right and set finish_toggle as 1
          finish_toggle = 1;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y= 0.0;
          cmd_vel.angular.z = -0.02;
          pub_.publish(cmd_vel);
          ROS_INFO("RIGHT !");
        }

        // Robot at finish line and alinged.
        else if(finish_toggle)
        {
          ROS_INFO("DDD");

          // Set tilt motor mode as front and terminate line driving and
          // detection nodes.
          tilt_mode.data = "front";
          pub_tilt_.publish(tilt_mode);
          terminate_msg.data = "Terminate";
          pub_terminate_.publish(terminate_msg);
          std::this_thread::sleep_for(2s);

          // Start publish message for start ocr
          ocr_toggle_msg.data = "Start";
          // pub_obj_.publish(obj_name_msg);
          pub_ocr_start_.publish(ocr_toggle_msg);

          // Terminate currnet node
          ros::shutdown();
        }

        ROS_INFO("[FINISH LINE] m : %f" , msg.theta);

      }
    }

    /* callback Function
    Params : (const geometry_msgs::Pose2D& msg)
    Return : void

    Get Pose2D message from the line detection node, and
    determins the angular and linear speed of injiget_char by the Pose2D
    information and drive to finish line.
    */
    void callback(const geometry_msgs::Pose2D& msg)
    {
      // Start toggle is on and not yet meet finish line
      if(start_toggle && !finish_toggle)
      {
        ROS_INFO("X : %f" , msg.x);
        ROS_INFO("Y : %f" , msg.y);
        ROS_INFO("Theta : %f" , msg.theta);

        // Robot needs to turn right to follow line
        if(msg.x > center + tor)
        {
          cmd_vel.angular.z = -0.18;
          ROS_INFO(" RIGHT ! ");
        }

        // Robot needs to turn left to follow line
        else if(msg.x < center - tor)
        {
          cmd_vel.angular.z = 0.18;
          ROS_INFO(" LEFT ! ");
        }

        // Otherwise, remove angular move (i.e, move forward)
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

    // Toggles
    int start_toggle = 0;
    int finish_toggle = 0;

    // Tunning Parameters.
    int center = 320;
    double finish_center = 0.0;
    double finishi_tor = 0.8;
    int tor = 60;
};
/* main function
Params : (int) argc, (char**) argv
Return : 0 (Success)

Init the node name as "line_derive_node".
Make a object of SubAndPub class and publish and subscribe by ros spin().
*/
int main(int argc, char** argv)
{
  // Node Name : "line_drive_node"
  ros::init(argc, argv, "line_drive_node");
  SubAndPub subPub;
  ros::spin();
  return 0;
}

