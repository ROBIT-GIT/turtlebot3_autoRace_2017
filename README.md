# Team ROBIT
  ## Introduction
  The Team ROBIT is a robot team of Kwangwoon University in Seoul, Republic of Korea. Team ROBIT has been established in November 2006. We have studied hardware and software of robot system. We also has participated in several domestic and international robot competitions. 

  ## Overview
   This document describes the team ROBIT's hardware and software for 2017 R-Biz challenge Turtlebot3 autonomous race.
  ## Hardware platform
  <img src="https://raw.githubusercontent.com/ROBOTIS-GIT/ROBOTIS-Documents/master/wiki-images/Turtlebot3/Turtlebot3_logo.jpg" width="300">
  Turtlebot3: [turtlebot3 official wiki](http://turtlebot3.readthedocs.io/en/latest/)
  
   <img src="http://buyings.co.kr/shop/data/goods/1501522796_549003.m.jpg" width="300">
   Usb camera: Logitech Webcam HD Pro C920
   
   <img src="http://shop1.phinf.naver.net/20170511_41/nmp_1494490738679tqETF_JPEG/11494418960286917_722889021.jpg" width="300">
   PC : NUC7i5BNK (RAM 4GB)
   
# User's Guide
  ## Setting the default setting about turtlebot3
  You should set the default setting about the turtlebot3.
  Refer the turtlebot3 official wiki and follow the official manual.
  Turtlebot3 Wiki: [turtlebot3 official wiki](http://turtlebot3.readthedocs.io/en/latest/)
  
  ## Setting the permission
  You should set the permission of some devices
  To set commands are:
    
      $ sudo chmod 777 /dev/video0
      $ sudo chmod 777 /dev/ttyUSB0
      $ sudo chmod 777 /dev/ttyACM0

  ## Run robit master node
  To execute master node commands are:
    
      $ rosrun robit_master robit_master_node
    
  ## Run turtlevision node
  We use ROS usb_cam package. Install the kinetic usb_cam package. 
  Reference link: http://wiki.ros.org/usb_cam
       
       $ sudo apt-get install ros-kinetic-usb-cam
  
  You should edit launch file. 
      
      $ cd /opt/ros/kinetic/share/usb_cam/launch/
      $ sudo gedit usb_cam-test.launch 
    
  Fill in this contents:
      
      <launch>
        <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
          <param name="video_device" value="/dev/video0" />
          <param name="image_width" value="320" />
          <param name="image_height" value="240" />
          <param name="framerate" value="30" />
          <param name="pixel_format" value="mjpeg" />
          <param name="camera_frame_id" value="usb_cam" />
          <param name="autoexposure" value="false" />
          <param name="exposure" value="130" />
          <param name="focus" value="1" />
          <param name="brightness" value="130" />
          <param name="contrast" value="100" />
          <param name="saturation" value="100" />
          <param name="auto_white_balance" value="false" />
          <param name="white_balance" value="4800" />
          <param name="io_method" value="mmap"/>
        </node>
      </launch>
  
  Execute usb_cam node
   
      $ roslaunch usb_cam usb_cam-test.launch 
    
  Execute turtlevision node  
  
      $ rosrun turtlevision turtlevision
  
  ## Run turtlebot3 tunnel node
        
