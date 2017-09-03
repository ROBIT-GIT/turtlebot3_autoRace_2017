/*******************************************************************************
* Copyright ROBIT, Kwangwoon Univ.
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

#include "../include/turtlevision/qnode.hpp"


namespace enc = sensor_msgs::image_encodings;

namespace turtlevision {

QNode::QNode(int argc, char** argv ):
    init_argc(argc),
    init_argv(argv)
{

}

QNode::~QNode() {}


void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg_img)
{
    cv::Mat img_bgr;

    try
    {
        img_bgr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Q_EMIT recvImage(img_bgr.clone(), FORMAT_BGR);
}


bool QNode::init()
{
    ros::init(init_argc,init_argv,"turtlevision");
    if ( ! ros::master::check() )
    {
        return false;
    }
    ros::start();

    ros::NodeHandle n;
    ros::NodeHandle nh;
    ros::Rate loop_rate(200);

    turtle_line_pub = nh.advertise<turtlevision::turtlevision_msg>("turtle_bot",1);

    image_transport::ImageTransport it(n);
    image_sub = it.subscribe("/usb_cam/image_raw",1,&QNode::imageCallback,this);
    loop_rate.sleep();
    start();
}

void QNode::run()
{
    ros::spin();
}

}
