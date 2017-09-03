#ifndef ROBITAUTODV_H
#define ROBITAUTODV_H

#include "ros/ros.h"
#include "../include/robit_master/turtlevision_msg.h"

#include    <iostream>

#define GAIN 0.010

class RobitAutoDV
{
private:

    enum{NOTHING, GATE_BAR_MARK , TRAFFIC_MARK_RED, TRAFFIC_MARK_YELLOW, PARKING_MARK , TUNNEL_MARK};
    enum{PARKING_MODE = 1 , TUNNEL_MODE};
    enum{RED = 1, YELLOW};

    // messages

    double degree_l;
    double degree_r;
    int in_line_mpt_x;
    int out_line_mpt_x;
    int notice_information;
    int is_traffic_mark;
    bool notice_parking;
    bool parking_flag;
    bool notice_tunnel;
    bool is_traffic_success;
    bool parking_mark_labeling;
    bool tunnel_mark_labeling;

    int driving_mode;

public:
    RobitAutoDV();

    enum{PARKING_FRONT_MODE , PARKING_REAR_MODE};

    static int number;
    static int parking_count;
    static int tunnel_count;
    static int is_in_tunnel;

    int parking_mode;

    double linear_x;
    double angular_z;

    void setMsgData(const turtlevision::turtlevision_msg::ConstPtr& msg);
    void setData(const double &data_l, const double &data_r);

    void lineTracing();

    void timer_parking_callback(ros::TimerEvent&);
    void turtlebot_parking(int number);
    inline void setVel(const double &target_vel, const double &target_ang)
    {
        linear_x = target_vel;
        angular_z = target_ang;
    }

    inline void setVel(const double &target_vel)
    {
        linear_x = target_vel;
    }


};

#endif // ROBITAUTODV_H
