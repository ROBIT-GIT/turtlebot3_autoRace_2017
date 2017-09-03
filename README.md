# Team ROBIT
  ## Introduction
  The Team ROBIT is a robot team of Kwangwoon University in Seoul, Republic of Korea. Team ROBIT has been established in November 2006. We have studied hardware and software of robot system. We also has participated in several domestic and international robot competitions. 

  ## Overview
   This document describes the team ROBIT's hardware and software for 2017 R-Biz challenge Turtlebot3 autonomous race.
   
   - robit_master: Manage the entire used packages and nodes.
   - turtlevision: Process the vision data.
   - turtlebot3_tunnel: Contain the mapping and path planning by LiDAR sensor in the tunnel mission.
   - turtlebot3_core: Modified firmware for turtlebot3 autonomous race.
   - capture: turtlevision UI capture.
   - sample_video: Sample video.
   
  ## Hardware platform
  <img src="https://raw.githubusercontent.com/ROBOTIS-GIT/ROBOTIS-Documents/master/wiki-images/Turtlebot3/Turtlebot3_logo.jpg" width="300">
  -Turtlebot3: [turtlebot3 official wiki](http://turtlebot3.readthedocs.io/en/latest/)
  
  -Usb camera: Logitech Webcam HD Pro C920
  
  -PC : Intel NUC7i5BNK (RAM 4GB)
   
# User's Guide
  ## Setting the default setting about turtlebot3
  You should set the default setting about the turtlebot3.
  Refer the turtlebot3 official wiki and follow the official manual.
  Turtlebot3 Wiki: [turtlebot3 official wiki](http://turtlebot3.readthedocs.io/en/latest/)
  
  ## Uploading the modifed openCR firmware
  This 'turtlebot3_core' package has been modified for the autonomous race. So you should upload this arduino sketch.
  
  ## Setting the permission
  You should set the permission of some devices.
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
  
  Execute usb_cam node after "catkin_make".
   
      $ roslaunch usb_cam usb_cam-test.launch 
    
  The turtlevision node use ROS qt UI. You should install ros-kinetic-qt. To install command is:
  
      $ sudo apt-get install ros-kinetic-qt-*
    
  Execute turtlevision node.  
  
      $ rosrun turtlevision turtlevision     
  
  You can threshold the range of several color in turtlevision node UI and press the "save parameter" to save the range of color. If you press the "run" button, robot will start.
  
  ## Run turtlebot3 tunnel node
   Bring up basic packages to start TurtleBot3 applications.
    
        $ roslaunch turtlebot3_bringup turtlebot3_robot.launch

   If you want to launch Lidar sensor and core separately, please use below commands.
        
        $ roslaunch turtlebot3_bringup turtlebot3_lidar.launch
        $ roslaunch turtlebot3_bringup turtlebot3_core.launch
        
   Execute turtlebot3 tunnel node.
    
        $ roslaunch turtlebot3_tunnel turtlebot3_tunnel.launch
