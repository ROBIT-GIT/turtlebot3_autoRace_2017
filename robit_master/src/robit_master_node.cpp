#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt64.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "../include/robit_master/robitautodv.h"
#include "../include/robit_master/turtlevision_msg.h"
#include "../include/robit_master/driving_msg.h"


using namespace std;

geometry_msgs::Twist      cmd_vel_msg;
robit_master::driving_msg driving_mode_msg;

RobitAutoDV               auto_driving;
ros::Publisher            driving_mode_msg_pub;
ros::Publisher            driving_msg_pub;
ros::Subscriber           driving_msg_sub;
ros::Subscriber           psd_val_sub;
ros::Subscriber           exit_tunnel_sub;

// call back function
inline void   msgCallBack(const turtlevision::turtlevision_msg::ConstPtr& msg);
void          psdCallback(const std_msgs::UInt64::ConstPtr& msg);
void          publishVelMsg(double linear_x, double angular_z);
void          callback1(const ros::TimerEvent&);
void          exit_tunnel_callback(const robit_master::driving_msg::ConstPtr& msg);
void          timer_parking_callback(const ros::TimerEvent);

// initialize variables
double  degree_left          = 0.0;
double  degree_right         = 0.0;
int     notice_information   = 0;
int     in_line_mpt_x        = 0;
int     out_line_mpt_x       = 0;
int     psd_val              = 0;
int     is_in_tunnel_        = 0;

bool    parking_mark_labeling   = false;
bool    tunnel_mark_labeling    = false;
bool    parking_flag            = false;
bool    parking_complete        = false;
bool    tunnel_flag             = false;
bool    tunnel_msg_flag         = false;


enum{NOTHING , GATE_BAR_MARK , TRAFFIC_MARK_RED, TRAFFIC_MARK_YELLOW, PARKING_MARK , TUNNEL_MARK};
enum{PARKING_MODE = 1 , TUNNEL_MODE};


int RobitAutoDV::number        = 0;
int RobitAutoDV::parking_count = 0;
int RobitAutoDV::tunnel_count  = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robit_master_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(200);

  // set timer
  ros::Timer timer1 = n.createTimer(ros::Duration(0.05), callback1);

  // message
  driving_msg_sub = n.subscribe("turtle_bot", 1, msgCallBack);
  psd_val_sub = n.subscribe("psd_val", 1 , psdCallback);
  exit_tunnel_sub = n.subscribe("/tunnel_state", 1, exit_tunnel_callback);
  driving_mode_msg_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  driving_msg_pub = n.advertise<robit_master::driving_msg>("/driving_mode",1);

  while(ros::ok())
  {
    if(is_in_tunnel_ != 1)
    {
      auto_driving.lineTracing();
      publishVelMsg(auto_driving.linear_x, auto_driving.angular_z);
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  ros::spin();
  return 0;
}

void psdCallback(const std_msgs::UInt64::ConstPtr& msg)
{
  // get psd data
  psd_val = msg->data;
}

inline void msgCallBack(const turtlevision::turtlevision_msg::ConstPtr& msg)
{
  // set vision message
  degree_left           = msg->degree_left;
  degree_right          = msg->degree_right;
  in_line_mpt_x         = msg->in_line_mpt_x;
  out_line_mpt_x        = msg->out_line_mpt_x;



  if((auto_driving.number == 0 && is_in_tunnel_ == 0) || (auto_driving.number >200 && is_in_tunnel_ == 0) )
  {
    notice_information    = msg->notice_information;
    parking_mark_labeling = msg->parking_mark_labeling;
    tunnel_mark_labeling  = msg->tunnel_mark_labeling;
    if(notice_information == PARKING_MARK)
    {
      parking_flag = true;
    }

    if(notice_information == TUNNEL_MARK)
    {
      tunnel_flag = true;
    }
  }

  auto_driving.setMsgData(msg);
}

void publishVelMsg(double linear_x, double angular_z)
{
  // publish turtlebot_3 velocity
  cmd_vel_msg.linear.x = linear_x;
  cmd_vel_msg.angular.z = angular_z;

  if(is_in_tunnel_ != 1)
  {
    driving_mode_msg_pub.publish(cmd_vel_msg);
  }
}

void exit_tunnel_callback(const robit_master::driving_msg::ConstPtr& msg)
{
  // subscribe exit tunnel message
  if(msg->is_in_tunnel == 1)
  {
    is_in_tunnel_ = 0;
  }
}

void callback1(const ros::TimerEvent&)
{
  // tunnel mode timer
  if(tunnel_flag == true && tunnel_mark_labeling == false)
  {
    auto_driving.tunnel_count ++;

    if(auto_driving.tunnel_count < 200 && auto_driving.tunnel_count > 60 && psd_val > 450)
    {
      is_in_tunnel_ = 1;

      auto_driving.setVel(0.0, 0.0);
      cmd_vel_msg.linear.x = 0;
      cmd_vel_msg.angular.z = 0;
      driving_mode_msg_pub.publish(cmd_vel_msg);

      driving_mode_msg.is_in_tunnel = is_in_tunnel_;
      driving_msg_pub.publish(driving_mode_msg);

      tunnel_flag = false;
      tunnel_msg_flag = true;
    }
  }

  // parking mode timer
  if(parking_flag == true && parking_mark_labeling == false)
  {
    if(parking_mark_labeling == true)
      auto_driving.parking_count = 0;

    auto_driving.parking_count ++;

    if(auto_driving.parking_count >= 90 && auto_driving.parking_count <= 150 && parking_complete == false)
    {
      if(psd_val > 300)
      {
        auto_driving.parking_mode = auto_driving.PARKING_REAR_MODE;
        parking_complete = true;
      }

      else
      {
          auto_driving.parking_mode = auto_driving.PARKING_FRONT_MODE;
      }
    }

    if(auto_driving.parking_count > 150 && auto_driving.parking_mode == auto_driving.PARKING_FRONT_MODE)
    {
      auto_driving.number ++;

      parking_complete = true;

      if(auto_driving.number > 210)
        parking_flag = false;
    }

    else if(auto_driving.parking_count > 210 && auto_driving.parking_mode == auto_driving.PARKING_REAR_MODE)
    {
      auto_driving.number ++;

      if(auto_driving.number > 210)
        parking_flag = false;
    }

    else
    {

    }
  }
}

