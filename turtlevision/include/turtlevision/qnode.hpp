/**
 * @file /include/turtlevision/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef turtlevision_QNODE_HPP_
#define turtlevision_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN

#include <ros/ros.h>

#include <QThread>
#include <QStringListModel>
#include <QMutex>


#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <fstream>


#include "../../include/turtlevision/turtlevision_msg.h"
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace turtlevision {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
     enum{FORMAT_BGR};

        QNode(int argc, char** argv );
        virtual ~QNode();
        ros::Publisher turtle_line_pub;

        bool init();

        void run();

Q_SIGNALS:
        void rosShutdown();
        void recvImage(cv::Mat img, int num);

private:
        int init_argc;
        char** init_argv;
        image_transport::Subscriber image_sub;
        void imageCallback(const sensor_msgs::ImageConstPtr& img_cam);
};

}  // namespace turtlevision

#endif /* turtlevision_QNODE_HPP_ */
