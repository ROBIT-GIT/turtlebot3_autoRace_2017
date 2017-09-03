# Team ROBIT
  ## Introduction
  The Team ROBIT is a robot team of Kwangwoon University in Seoul, Republic of Korea. Team ROBIT has been established in November 2006. We have studied hardware and software of robot system. We also has participated in several domestic and international robot competitions. 
  
# turtlebot3_autoRace

# Overview
    This document describes the team ROBIT hardware and software setting for 2017 R-Biz challenge Turtlebot3 autonomous race.


# User's Guide

  ## Setting the permission
  You should set the permission of some devices
  To set commands are:
    ```
    $ sudo chmod 777 /dev/video0
    $ sudo chmod 777 /dev/ttyUSB0
    $ sudo chmod 777 /dev/ttyACM0
    ``` 

  ## Run robit master node
  To execute master node commands are:

    ```
    $ rosrun robit_master robit_master_node
    ``` 

  ## Run vision node


  To execute master node commands are:
    ```
    $ roslaunch usb_cam usb_cam-test.launch 
    ```
