#ifndef TURTLEBOT3_TUNNEL_H
#define TURTLEBOT3_TUNNEL_H

#include <ros/ros.h>
#include <ros/time.h>

#include <math.h>
#include <limits.h>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "driving_msg.h"

#include <iostream>
#include <queue>
#include <stack>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>

#include <cstdlib>
#include <cstdio>

#include <string>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>


#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5
#define ANGULAR_GAIN 0.15
#define TURN_GAIN 2

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

#define POS_X   0
#define POS_Y   1

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define DIR   8
#define INF   20000

#define XFIN  62
#define YFIN  66

class Turtlebot3_Tunnel
{
 public:
  Turtlebot3_Tunnel();
  ~Turtlebot3_Tunnel();

  bool init();
  bool controlLoop();
  int is_in_tunnel;
  robit_master::driving_msg driving_msg;

  struct Move {
    int x,y;
    int g_cost;
    int f_cost;
  };


 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters
  bool is_debug_;

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;
  ros::Publisher exit_tunnel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber tf_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber driving_mode_sub;


  enum{FIRST_QUADRANT, SECOND_QUADRANT, THIRD_QUADRANT, FOURTH_QUADRANT};
  enum{NONE, OBSTACLE , POS_ROBOT, PATH};
  enum{WEST, NORTH_WEST, NORTH, NORTH_EAST, EAST, SOUTH_EAST, SOUTH, SOUTH_WEST};


  cv::Mat cv_grid;
  cv::Mat cv_grid_clone;
  cv::Mat robot_pos_in_grid;


  int    print_count;
  int    find_count;
  int    first_top_count;
  bool   find_path;

  int    top_data_x;
  int    top_data_y;
  int    grid[80][80];
  int    grid_copy[80][80];
  int    grid_pos_x;
  int    grid_pos_y;
  int    grid_pos_robot_x;
  int    grid_pos_robot_y;

  int    direction;
  double direction_angle;
  bool   is_obstacle;

  bool   position_flag;

  double turning_radius_;
  double rotate_angle_;
  double front_distance_limit_;
  double side_distance_limit_;

  double distance_obstacle[360] = {0.0, };
  double position_obstacle[360][2];

  double right_joint_encoder_;
  double priv_right_joint_encoder_;

  double robot_position_x;
  double robot_position_y;
  double robot_position_x_init;
  double robot_position_y_init;

  double imu_w;
  double imu_x;
  double imu_y;
  double imu_z;
  double imu_fin;

  int        goal_x, goal_y;
  bool       closed_nodes_[80][80];
  bool       open_nodes_[80][80];
  int        dir_map[80][80];
  double     cost[80][80];

  std::pair<int, int> path[80][80];


  int dx[DIR] = {1, 1, 0, -1, -1, -1, 0, 1};
  int dy[DIR] = {1, 0, 1, 1, 0, -1, -1, -1};

  int robot_navigation_x[DIR] = {1, 1, 0, -1, -1, -1, 0, 1};
  int robot_navigation_y[DIR] = {0, 1, 1, 1, 0, -1, -1, -1};

  int xFinish;
  int yFinish;

  // Callback Function prototypes

  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &scan);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &odom);
  void imuMsgCallBack(const sensor_msgs::Imu::ConstPtr &imu);
  void drivingModeMsgCallBack(const robit_master::driving_msg::ConstPtr &dm_msg);

  // Function prototypes

  int  estimate_H(int xStart, int xFinish, int yStart, int yFinish);
  void pathFinding();
  void printing_path();
  void find_direction(bool find_path);
  void updatecommandVelocity(double linear, double angular);

};
#endif // TURTLEBOT3_TUNNEL_H
