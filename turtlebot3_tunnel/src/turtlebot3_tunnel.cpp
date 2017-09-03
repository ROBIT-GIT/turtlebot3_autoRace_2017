/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */
/* Maintainer : Youngjoon Jang */


#include "turtlebot3_tunnel/turtlebot3_tunnel.h"

using namespace std;

Turtlebot3_Tunnel::Turtlebot3_Tunnel()
  : nh_priv_("~")
{
  // initialize tunnel node
  ROS_INFO("TurtleBot3 Tunnel Node Init");
  ROS_ASSERT(init());
}

Turtlebot3_Tunnel::~Turtlebot3_Tunnel()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3_Tunnel::init()
{
  turning_radius_ = 0.08;
  rotate_angle_ = 50 * DEG2RAD;
  front_distance_limit_ = 0.4;
  side_distance_limit_  = 0.3;

  ROS_INFO("robot_model : turtlebot3_burger");
  ROS_INFO("turning_radius_ : %lf", turning_radius_);
  ROS_INFO("front_distance_limit_ = %lf", front_distance_limit_);
  ROS_INFO("side_distance_limit_ = %lf", side_distance_limit_);

  // initialize variables
  right_joint_encoder_ = 0.0;
  priv_right_joint_encoder_ = 0.0;
  robot_position_y = 0.0;
  robot_position_x = 0.0;

  imu_w = 0.0;
  imu_x = 0.0;
  imu_y = 0.0;
  imu_z = 0.0;

  cv_grid = cv::Mat::zeros(80,80,CV_8UC1);
  robot_pos_in_grid = cv::Mat::zeros(80,80,CV_8UC1);
  print_count = 0;
  first_top_count = 0;
  find_count = 0;

  top_data_x = 0;
  top_data_y = 0;

  position_flag = false;

  // initialize grid

  for(int grid_row = 0 ; grid_row < 80 ; grid_row ++)
  {
    for(int grid_col = 0 ; grid_col < 80 ; grid_col ++)
    {
      grid[grid_col][grid_row] = 0;
      grid_copy[grid_col][grid_row] = 0;
      closed_nodes_[grid_col][grid_row] = 0;
      open_nodes_[grid_col][grid_row] = 0;
    }
  }

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  exit_tunnel_pub_ = nh_.advertise<robit_master::driving_msg>("/tunnel_state", 1);

  // initialize subscribers

  driving_mode_sub = nh_.subscribe("/driving_mode", 1, &Turtlebot3_Tunnel::drivingModeMsgCallBack, this);

  return true;
}

/*******************************************************************************
* Callback function
*******************************************************************************/
void Turtlebot3_Tunnel::imuMsgCallBack(const sensor_msgs::Imu::ConstPtr &imu)
{
    imu_w = imu->orientation.w;
    imu_z = imu->orientation.z;
    imu_x = imu->orientation.x;
    imu_y = imu->orientation.y;

    double yaw = atan2(2*imu_x*imu_y + 2*imu_w*imu_z, imu_w*imu_w + imu_x*imu_x - imu_y*imu_y - imu_z*imu_z);

    // get yaw angle of the turtlebot
    double yaw_angle = yaw * RAD2DEG;

    if(yaw_angle > -90.0 && yaw_angle <= 180.0)
      direction_angle = yaw_angle + 90.0;
    else if(yaw_angle > -180.0 && yaw_angle <= -90.0)
      direction_angle = yaw_angle + 450.0;
}

void Turtlebot3_Tunnel::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &odom)
{
  if(position_flag == false)
  {
    robot_position_y_init = odom->pose.pose.position.x;
    robot_position_x_init = odom->pose.pose.position.y;

    position_flag = true;
  }

  else if(position_flag == true)
  {
    robot_position_y = odom->pose.pose.position.x - robot_position_y_init + 0.09;
    robot_position_x = odom->pose.pose.position.y - robot_position_x_init + 0.05;
  }
}

void Turtlebot3_Tunnel::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  {
    for (int scan_angle = 0; scan_angle < 360; scan_angle++)
    {
      if (std::isinf(scan->ranges.at(scan_angle))) // when the scan value is infinity
      {
        distance_obstacle[scan_angle] = scan->range_max; // range_max = 3.5
        is_obstacle = false;
      }
      else if(scan->ranges.at(scan_angle) == 0.0)
      {
        distance_obstacle[scan_angle] = 3.0;
        is_obstacle = false;
      }
      else
      {
        if(scan->ranges.at(scan_angle) < 0.7 && scan->intensities.at(scan_angle) > 3000)
        {
          distance_obstacle[scan_angle] = scan->ranges.at(scan_angle);
          is_obstacle = true;
        }
        else
          is_obstacle = false;
      }

      if(is_obstacle == true)
      {
        position_obstacle[scan_angle][POS_X] = (( robot_position_x) + (-1)*(distance_obstacle[scan_angle] * cos((scan_angle + direction_angle) * DEG2RAD)));
        position_obstacle[scan_angle][POS_Y] = (robot_position_y + (distance_obstacle[scan_angle] * sin((scan_angle + direction_angle) * DEG2RAD)));

        grid_pos_x = position_obstacle[scan_angle][POS_X] * 40;
        grid_pos_y = position_obstacle[scan_angle][POS_Y] * 40;

        grid_pos_robot_x = robot_position_x * 40;
        grid_pos_robot_y = robot_position_y * 40;

        if(grid_pos_x >= 0 && grid_pos_y >= 0 && grid_pos_x < 80 && grid_pos_y < 80)
        {
          cv_grid.at<uchar>(grid_pos_y , grid_pos_x) = OBSTACLE;
          closed_nodes_[grid_pos_x][grid_pos_y] = OBSTACLE;
        }

        if(grid_pos_robot_x >= 0 && grid_pos_robot_y >= 0 && grid_pos_robot_x < 80 && grid_pos_robot_y < 80)
        {
          robot_pos_in_grid.at<uchar>(grid_pos_robot_y , grid_pos_robot_x) = POS_ROBOT;
        }
      }


      cv_grid_clone  = cv_grid.clone();

      cv::Mat mask_erode     = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
      cv::Mat mask_dilate    = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));

      erode(cv_grid,
      cv_grid_clone,
      mask_erode,
      cv::Point(-1, -1),
      1);

      dilate(cv_grid,
      cv_grid_clone,
      mask_dilate,
      cv::Point(-1, -1),
      4);

      cv::add(cv_grid_clone, robot_pos_in_grid, cv_grid_clone);

      if(!(XFIN == grid_pos_robot_x && YFIN == grid_pos_robot_y))
      {
        pathFinding();
        find_path = false;
        find_direction(find_path);
      }

      else
      {
        find_path = true;
        find_direction(find_path);
      }
  }
}
}

void Turtlebot3_Tunnel::drivingModeMsgCallBack(const robit_master::driving_msg::ConstPtr &dm_msg)
{
  is_in_tunnel = dm_msg->is_in_tunnel;

  if(is_in_tunnel == 1)
  {
    laser_scan_sub_  = nh_.subscribe("/scan", 10, &Turtlebot3_Tunnel::laserScanMsgCallBack,this);
    odom_sub_        = nh_.subscribe("/odom",10,&Turtlebot3_Tunnel::odomMsgCallBack,this);
    imu_sub_         = nh_.subscribe("/imu",10,&Turtlebot3_Tunnel::imuMsgCallBack,this);
  }
}

/*******************************************************************************
* Path finding function
*******************************************************************************/

/* operator overloading for priority queue */
bool operator<(const Turtlebot3_Tunnel::Move& a, const Turtlebot3_Tunnel::Move& b)
{
  return a.f_cost > b.f_cost;
}

/* printing map and path to goal position */
void Turtlebot3_Tunnel::printing_path()
{
    int x = XFIN, y = YFIN;

    stack<pair<int, int>> st;

    // create path
    while(!(x == grid_pos_robot_x && y == grid_pos_robot_y))
    {
      cv_grid_clone.at<uchar>(y,x) = PATH;
      pair<int, int>& p = path[x][y];
      st.push({x, y});
      x = p.first;
      y = p.second;
    }

    // print grid map
    print_count ++;
    if(print_count > 400)
    {
      for(int i = 79 ; i >= 0 ; i --)
      {
        for(int j = 79 ; j >= 0 ; j --)
        {
          if(cv_grid_clone.at<uchar>(i,j) == NONE)
            printf("%s0 ",KYEL);
          else if(cv_grid_clone.at<uchar>(i,j) == OBSTACLE)
            printf("%s1 ",KBLU);
          else if(cv_grid_clone.at<uchar>(i,j) == POS_ROBOT)
            printf("%s2 ",KRED);
          else if(cv_grid_clone.at<uchar>(i,j) == PATH)
            printf("%s3 ",KMAG);
        }
        std::cout<<std::endl;
      }
      std::cout<<std::endl<<std::endl;

      robot_pos_in_grid = cv::Mat::zeros(80,80,CV_8UC1);
      robot_pos_in_grid.at<uchar>(grid_pos_robot_y , grid_pos_robot_x) = POS_ROBOT;

      print_count = 0;
    }

    first_top_count = 0;
    while (!st.empty())
    {
      auto& p = st.top();
      first_top_count ++;
      if(first_top_count == 1)
      {
        top_data_x = p.first;
        top_data_y = p.second;
      }
      cv_grid_clone.at<uchar>(p.second,p.first) = 0;
      st.pop();
    }
}

/* estimate huristic value  */
int Turtlebot3_Tunnel::estimate_H(int xDep,int xGoal, int yDep, int yGoal)
{
    int mht_distance = abs(xGoal-xDep) + abs(yGoal-yDep); // Manhattan distance

    return mht_distance ;
}

/* A star */
void Turtlebot3_Tunnel::pathFinding()
{
    for (int i = 0; i <80; i ++)
      for (int j = 0; j <80; j ++)
       path[i][j] = {0,0};

     memset(open_nodes_, 0, sizeof open_nodes_);
     memset(closed_nodes_, 0, sizeof closed_nodes_);

     priority_queue <Move> pq;
     int f_cost;
     int g_cost;

     for(int i = 0; i < 80; i++)
       for(int j =0; j < 80; j++)
         cost[i][j] = INF;

     cost[grid_pos_robot_x][grid_pos_robot_y] = 0;

     pq.push({grid_pos_robot_x,grid_pos_robot_y,0,0});

     while(!pq.empty())
     {
       Move cur = pq.top(); // get smallest cost node

       pq.pop();

       if (closed_nodes_[cur.x][cur.y]) continue;

       if(cur.x == XFIN && cur.y == YFIN) break;

       open_nodes_[cur.x][cur.y] = false;
       closed_nodes_[cur.x][cur.y] = true;

       // Let's move
       for(int i = 0; i < DIR; i++)
       {
         // next node position
         int nx = cur.x + dx[i];
         int ny = cur.y + dy[i];

         // ignore if next node is closed or have obstacle
         if(cv_grid_clone.at<uchar>(ny,nx) ==  OBSTACLE || closed_nodes_[nx][ny] == true) continue;

         // ignore if next node exceed the map
         if(nx<0 || ny<0 || nx>=72 || ny>=72) continue;

         open_nodes_[nx][ny] = true;
         closed_nodes_[nx][ny] = false;

         if(i % 2 == 0) // if next node is vertical or horizontal
         {
           g_cost = cur.g_cost + 10;
           f_cost = g_cost + 10 * estimate_H(nx,XFIN,ny,YFIN);
         }
         else if(i % 2 == 1) // if next node is diagonal
         {
           g_cost = cur.g_cost + 14;
           f_cost = g_cost + 10 * estimate_H(nx,XFIN,ny,YFIN);
         }

         if(g_cost < cost[nx][ny]) // cost update
         {
           path[nx][ny] = pair<int, int>{cur.x, cur.y}; // path record (parent)
           cost[nx][ny] = g_cost;
           pq.push(Move{nx,ny,g_cost,f_cost});
         }
       }
     }
     printing_path();
}

/* get direction from the path */
void Turtlebot3_Tunnel::find_direction(bool find_path)
{
    int direction_mode;

    if(find_path == false)
    {
      int direction_x = top_data_x - grid_pos_robot_x;
      int direction_y = top_data_y - grid_pos_robot_y;

      for(direction_mode = 0 ; direction_mode < DIR ; direction_mode ++)
      {
        if(direction_x == robot_navigation_x[direction_mode] && direction_y == robot_navigation_y[direction_mode]) break;
      }
    }

    // turn left when turtlebot get reached to the finish position
    else
      direction_mode = WEST;

    double target_angle = 0.0;

    // get direction from the imu data
    // publish driving information
    if(direction_mode == WEST)
    {
      target_angle = 180.0;
      if(target_angle - direction_angle > 2.0)
        updatecommandVelocity(0, 0.25);
      else if(target_angle - direction_angle < -2.0)
        updatecommandVelocity(0, -0.25);
      else if(target_angle - direction_angle >= -2.0 && target_angle - direction_angle <= 2.0 && find_path == false)
        updatecommandVelocity(0.07, 0.0);
      else
      {
        updatecommandVelocity(0.0, 0.0);
        exit_tunnel_pub_.publish(driving_msg);
        driving_msg.is_in_tunnel = 1;
      }
    }
    else if(direction_mode == NORTH_WEST)
    {
      target_angle = 135.0;
      if(target_angle - direction_angle > 2.0)
        updatecommandVelocity(0, 0.25);
      else if(target_angle - direction_angle < -2.0)
        updatecommandVelocity(0, -0.25);
      else
        updatecommandVelocity(0.07, 0.0);
    }
    else if(direction_mode == NORTH)
    {
      target_angle = 90.0;
      if(target_angle - direction_angle > 2.0)
        updatecommandVelocity(0, 0.25);
      else if(target_angle - direction_angle < -2.0)
        updatecommandVelocity(0, -0.25);
      else
        updatecommandVelocity(0.07, 0.0);
    }
    else if(direction_mode == NORTH_EAST)
    {
      target_angle = 45.0;
      if(target_angle - direction_angle > 2.0)
        updatecommandVelocity(0, 0.25);
      else if(target_angle - direction_angle < -2.0)
        updatecommandVelocity(0, -0.25);
      else
        updatecommandVelocity(0.07, 0.0);
    }
    else if(direction_mode == EAST)
    {

      if(direction_angle > 180 && direction_angle <= 358.0 )
        updatecommandVelocity(0, 0.25);
      else if(direction_angle <= 180 && direction_angle > 2.0)
        updatecommandVelocity(0, -0.25);
      else
        updatecommandVelocity(0.07, 0.0);
    }
    else if(direction_mode == SOUTH_EAST)
    {
      target_angle = 315.0;
      if(target_angle - direction_angle > 2.0)
        updatecommandVelocity(0, 0.25);
      else if(target_angle - direction_angle < -2.0)
        updatecommandVelocity(0, -0.25);
      else
        updatecommandVelocity(0.07, 0.0);
    }
    else if(direction_mode == SOUTH)
    {
      target_angle = 270.0;
      if(target_angle - direction_angle > 2.0)
        updatecommandVelocity(0, 0.25);
      else if(target_angle - direction_angle < -2.0)
        updatecommandVelocity(0, -0.25);
      else
        updatecommandVelocity(0.07, 0.0);
    }
    else if(direction_mode == SOUTH_WEST)
    {
      target_angle = 225.0;
      if(target_angle - direction_angle > 2.0)
        updatecommandVelocity(0, 0.25);
      else if(target_angle - direction_angle < -2.0)
        updatecommandVelocity(0, -0.25);
      else
        updatecommandVelocity(0.07, 0.0);
    }
}

/* update velocity */
void Turtlebot3_Tunnel::updatecommandVelocity(double linear, double angular)
{
  if(driving_msg.is_in_tunnel != 1)
  {
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x  = linear;
    cmd_vel.angular.z = angular;

    cmd_vel_pub_.publish(cmd_vel);
  }
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_tunnel");
  Turtlebot3_Tunnel turtlebot3tunnel;

  turtlebot3tunnel.init();

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

    // shut down this node when turtlebot get out of the tunnel
    if(turtlebot3tunnel.driving_msg.is_in_tunnel == 1)
      ros::shutdown();
  }

  return 0;
}
