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

#include "../include/turtlevision/main_window.hpp"

#define PI 3.141592

#define RIGHT   0
#define LEFT    1

// tunnel_mark parameter
#define TRIANGLE_AREA_RATE_MIN      0.35
#define TRIANGLE_AREA_RATE_MAX      0.57
#define TRIANGLE_AREA_MIN           280
#define TRIANGLE_AREA_MAX           450
#define TRIANGLE_FIRST_ANGLE_MIN    50.0
#define TRIANGLE_FIRST_ANGLE_MAX    65.0
#define TRIANGLE_SECOND_ANGLE_MIN   110.0
#define TRIANGLE_SECOND_ANGLE_MAX   130.0
#define TUNNEL_MATCHES_SIZE_MIN     10
#define TUNNEL_MATCHES_SIZE_MAX     50

// parking_mark parameter
#define REC_AREA_RATE_MIN           0.88
#define REC_AREA_RATE_MAX           0.0
#define REC_AREA_MIN                750
#define REC_AREA_MAX                1200
#define REC_CHECK                   7
#define PARKING_MATCHES_SIZE_MIN    30
#define PARKING_MATCHES_SIZE_MAX    100

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

namespace enc = sensor_msgs::image_encodings;

namespace turtlevision {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
    , is_in_line(false)
    , is_out_line(false)
    , is_gate_bar(false)
    , is_traffic_yellow_light(false)
    , is_traffic_red_light(false)
    , is_parking_mark(false)
    , is_tunnel_mark(false)
    , detection_flag(0)
    , check_data_count(0)
    , mode(0)
{
    ui.setupUi(this);
    qnode.init();


    get_parameter();


    initImageProcessing();
}

inline void MainWindow::initImageProcessing()
{
    ui.labelRGB->setScaledContents(true);
    ui.labelLINE->setScaledContents(true);
    ui.labelGATE_BAR->setScaledContents(true);
    ui.labelTRAFFIC_LIGHT->setScaledContents(true);
    ui.labelPARKING_MARK->setScaledContents(true);
    ui.labelTUNNEL_MARK->setScaledContents(true);
    ui.label_image->setScaledContents(true);

    qRegisterMetaType<cv::Mat>("cv::Mat");

    // image update
    QObject::connect(&qnode, SIGNAL(recvImage(cv::Mat,int)), this, SLOT(updateImg(cv::Mat,int)));
}

void MainWindow::updateImg(cv::Mat img, int num)
{
    turtlevision::turtlevision_msg turtle_vision_msg;
    {
      // make matrix
      Mat img_bgr = img.clone();

      Mat img_hsv;

      Mat img_in_line_binary;
      Mat img_out_line_binary;
      Mat img_line_binary;

      Mat gate_bar_row_binary;
      Mat gate_bar_high_binary;
      Mat gate_bar_binary;

      Mat traffic_light_row_binary;
      Mat traffic_light_high_binary;
      Mat traffic_light_yellow_binary;
      Mat traffic_light_binary;

      Mat parking_mark_binary;

      Mat tunnel_mark_row_binary;
      Mat tunnel_mark_high_binary;
      Mat tunnel_mark_binary;

      Mat mask_erode     = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1)),
          mask_dilate    = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));

      // noise_termination (gaussianBlur)
      GaussianBlur(img_bgr, img_bgr, Size(5,5), 1.5);

      // convert_color (bgr to hsv)
      cvtColor(img_bgr, img_hsv, CV_BGR2HSV);

      // threshold_field_binary
      Mat field_binary(img_bgr.rows/2, img_bgr.cols, CV_8UC1);

      inRange(img_hsv(Rect(0, img_bgr.rows/2, img_bgr.cols, img_bgr.rows/2)) ,
              Scalar(0,0,0),
              Scalar(255,255,57),
              field_binary);

      erode(field_binary,
      field_binary,
      mask_erode,
      Point(-1, -1),
      2);

      dilate(field_binary,
      field_binary,
      mask_dilate,
      Point(-1, -1),
      2);

      // threshold_in_line_image
      inRange(img_hsv(Rect(0, img_bgr.rows/2, img_bgr.cols, img_bgr.rows/2)) ,
              Scalar(in_line_hue_min,in_line_sat_min,in_line_val_min),
              Scalar(in_line_hue_max,in_line_sat_max,in_line_val_max),
              img_in_line_binary);

      erode(img_in_line_binary,
      img_in_line_binary,
      mask_erode,
      Point(-1, -1),
      3);

      dilate(img_in_line_binary,
      img_in_line_binary,
      mask_dilate,
      Point(-1, -1),
      3);

      // threshold_out_line_image
      inRange(img_hsv(Rect(0, img_bgr.rows/2, img_bgr.cols, img_bgr.rows/2)) ,
              Scalar(out_line_hue_min,out_line_sat_min,out_line_val_min),
              Scalar(out_line_hue_max,out_line_sat_max,out_line_val_max),
              img_out_line_binary);

      erode(img_out_line_binary,
      img_out_line_binary,
      mask_erode,
      Point(-1, -1),
      3);

      dilate(img_out_line_binary,
      img_out_line_binary,
      mask_dilate,
      Point(-1, -1),
      3);

      // threshold_gate_bar_row_red_color_image
      inRange(img_hsv(Rect(0, 0, img_bgr.cols, img_bgr.rows/2)) ,
              Scalar(gate_bar_row_hue_min,gate_bar_sat_min,gate_bar_val_min),
              Scalar(gate_bar_row_hue_max,gate_bar_sat_max,gate_bar_val_max),
              gate_bar_row_binary);

      erode(gate_bar_row_binary,
      gate_bar_row_binary,
      mask_erode,
      Point(-1, -1),
      1);

      dilate(gate_bar_row_binary,
      gate_bar_row_binary,
      mask_dilate,
      Point(-1, -1),
      1);

      // threshold_gate_bar_hige_red_color_image
      inRange(img_hsv(Rect(0, 0, img_bgr.cols, img_bgr.rows/2)) ,
              Scalar(gate_bar_high_hue_min,gate_bar_sat_min,gate_bar_val_min),
              Scalar(gate_bar_high_hue_max,gate_bar_sat_max,gate_bar_val_max),
              gate_bar_high_binary);

      erode(gate_bar_high_binary,
      gate_bar_high_binary,
      mask_erode,
      Point(-1, -1),
      1);

      dilate(gate_bar_high_binary,
      gate_bar_high_binary,
      mask_dilate,
      Point(-1, -1),
      1);

      // threshold_traffic_light_row_red_color_image
      inRange(img_hsv(Rect(img_bgr.cols*3/4 , 0, img_bgr.cols/4, img_bgr.rows)) ,
              Scalar(traffic_light_row_red_hue_min,traffic_light_red_sat_min,traffic_light_red_val_min),
              Scalar(traffic_light_row_red_hue_max,traffic_light_red_sat_max,traffic_light_red_val_max),
              traffic_light_row_binary);

      erode(traffic_light_row_binary,
      traffic_light_row_binary,
      mask_erode,
      Point(-1, -1),
      1);

      dilate(traffic_light_row_binary,
      traffic_light_row_binary,
      mask_dilate,
      Point(-1, -1),
      1);

      // threshold_traffic_light_high_red_color_image
      inRange(img_hsv(Rect(img_bgr.cols*3/4 , 0, img_bgr.cols/4, img_bgr.rows)) ,
              Scalar(traffic_light_high_red_hue_min,traffic_light_red_sat_min,traffic_light_red_val_min),
              Scalar(traffic_light_high_red_hue_max,traffic_light_red_sat_max,traffic_light_red_val_max),
              traffic_light_high_binary);

      erode(traffic_light_high_binary,
      traffic_light_high_binary,
      mask_erode,
      Point(-1, -1),
      1);

      dilate(traffic_light_high_binary,
      traffic_light_high_binary,
      mask_dilate,
      Point(-1, -1),
      1);

      // threshold_traffic_light_yellow_color_image
      inRange(img_hsv(Rect(img_bgr.cols*3/4 , 0, img_bgr.cols/4, img_bgr.rows)) ,
              Scalar(traffic_light_yellow_hue_min,traffic_light_yellow_sat_min,traffic_light_yellow_val_min),
              Scalar(traffic_light_yellow_hue_max,traffic_light_yellow_sat_max,traffic_light_yellow_val_max),
              traffic_light_yellow_binary);

      erode(traffic_light_yellow_binary,
      traffic_light_yellow_binary,
      mask_erode,
      Point(-1, -1),
      1);

      dilate(traffic_light_yellow_binary,
      traffic_light_yellow_binary,
      mask_dilate,
      Point(-1, -1),
      1);

      // threshold_parkimg_mark_image
      inRange(img_hsv(Rect(img_bgr.cols / 2, 0, img_bgr.cols / 2, img_bgr.rows*3/4)) ,
              Scalar(parking_mark_hue_min,parking_mark_sat_min,parking_mark_val_min),
              Scalar(parking_mark_hue_max,parking_mark_sat_max,parking_mark_val_max),
              parking_mark_binary);

      erode(parking_mark_binary,
      parking_mark_binary,
      mask_erode,
      Point(-1, -1),
      3);

      dilate(parking_mark_binary,
      parking_mark_binary,
      mask_dilate,
      Point(-1, -1),
      3);

      // threshold_tunnel_mark_image
      inRange(img_hsv(Rect(0, 0, img_bgr.cols/2, img_bgr.rows*3/4)) ,
              Scalar(tunnel_mark_row_hue_min,tunnel_mark_sat_min,tunnel_mark_val_min),
              Scalar(tunnel_mark_row_hue_max,tunnel_mark_sat_max,tunnel_mark_val_max),
              tunnel_mark_row_binary);

      erode(tunnel_mark_row_binary,
      tunnel_mark_row_binary,
      mask_erode,
      Point(-1, -1),
      2);

      dilate(tunnel_mark_row_binary,
      tunnel_mark_row_binary,
      mask_dilate,
      Point(-1, -1),
      2);

      inRange(img_hsv(Rect(0, 0, img_bgr.cols/2, img_bgr.rows*3/4)) ,
              Scalar(tunnel_mark_high_hue_min,tunnel_mark_sat_min,tunnel_mark_val_min),
              Scalar(tunnel_mark_high_hue_max,tunnel_mark_sat_max,tunnel_mark_val_max),
              tunnel_mark_high_binary);

      erode(tunnel_mark_high_binary,
      tunnel_mark_high_binary,
      mask_erode,
      Point(-1, -1),
      2);

      dilate(tunnel_mark_high_binary,
      tunnel_mark_high_binary,
      mask_dilate,
      Point(-1, -1),
      2);

      // add image
      add(img_in_line_binary,img_out_line_binary,img_line_binary);
      add(gate_bar_row_binary, gate_bar_high_binary, gate_bar_binary);
      add(traffic_light_row_binary, traffic_light_high_binary, traffic_light_binary);
      add(traffic_light_yellow_binary, traffic_light_binary, traffic_light_binary);
      add(tunnel_mark_row_binary, tunnel_mark_high_binary, tunnel_mark_binary);

      // run labeling
      CBlobLabelingRobit  labeling_in_line(img_in_line_binary,400),
                          labeling_out_line(img_out_line_binary,400),
                          labeling_gate_bar(gate_bar_binary,250),
                          labeling_traffic_light(traffic_light_binary,100),
                          labeling_parking_mark(parking_mark_binary,400),
                          labeling_tunnel_mark(tunnel_mark_binary,150),
                          labeling_field(field_binary,1000);

      labeling_in_line.doLabeling();
      labeling_out_line.doLabeling();
      labeling_gate_bar.doLabeling();
      labeling_traffic_light.doLabeling();
      labeling_parking_mark.doLabeling();
      labeling_tunnel_mark.doLabeling();
      labeling_field.doLabeling();

      // set field (terminate external noise)
      Point pastCen(0,0);

      for(int i = 0 ; i < labeling_field.m_nBlobs ; i++)
      {
        const int sX=labeling_field.m_recBlobs[i].x;
        const int sY=labeling_field.m_recBlobs[i].y;

        const int eX=labeling_field.m_recBlobs[i].x+labeling_field.m_recBlobs[i].width;
        const int eY=labeling_field.m_recBlobs[i].y+labeling_field.m_recBlobs[i].height;

        int CenX=0,CenY=0,nCount=0;
        Point exceptionPt;
        for(int j_field=sY,posY_field=sY*field_binary.cols;j_field<=eY;j_field++,posY_field+=field_binary.cols)
        {
          for(int k_field=sX;k_field<=eX;k_field++)
          {
            if(labeling_field.m_ImgData[posY_field+k_field]==i+1)
            {
                CenX+=k_field;
                CenY+=j_field;
                nCount++;
                exceptionPt.x=k_field;
                exceptionPt.y=j_field;
            }
          }
        }

        CenX/=nCount;
        CenY/=nCount;

        if(labeling_field.m_ImgData[CenX+CenY*field_binary.cols]!=i+1)
        {
            CenX= exceptionPt.x;
            CenY= exceptionPt.y;
        }

        if(pastCen.x!=0&&pastCen.y!=0)
        {
            if(norm(Point(CenX,CenY)-exceptionPt)<200)
            {
                cv::line(field_binary,
                         pastCen,
                         Point(CenX,CenY),
                         Scalar(200),
                         4
                        );
            }
        }
        pastCen.x=CenX;
        pastCen.y=CenY;
      }

      // contour
      vector< vector<Point> >contours;
      vector<Vec4i>hierarchy;

      cv::Mat img_contour=field_binary.clone();

      cv::findContours(field_binary,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
      vector< vector<Point> >hull(contours.size());

      for(size_t s=0;s<contours.size();s++)convexHull(Mat(contours[s]),hull[s],false);

      Mat drawing = Mat::zeros(img_contour.size(),CV_8UC1);

      for(size_t s=0;s<contours.size();s++) cv::drawContours(drawing,hull,s,Scalar(69),2,8,hierarchy,0,Point());

      for(int j=0,idxY=0;j<drawing.rows;j++,idxY+=drawing.cols)
      {
        for(int k=0;k<drawing.cols;k++)
        {
          img_in_line_binary.data[idxY+k]=0;
          img_out_line_binary.data[idxY+k]=0;

          if(drawing.data[idxY+k]==69)break;
        }

        for(int k=drawing.cols-1;k>=0;k--)
        {
          img_in_line_binary.data[idxY+k]=0;
          img_out_line_binary.data[idxY+k]=0;

          if(drawing.data[idxY+k]==69)break;
        }
      }

      for(int i = 0 ; i < img_line_binary.cols * img_line_binary.rows ; i++)
      {
        if(field_binary.data[i]==255)
        {
          img_in_line_binary.data[i]=0;
          img_out_line_binary.data[i]=0;
        }
      }

      for(int j=0;j<drawing.cols;j++)
      {
        for(int k=0;k<drawing.rows;k++)
        {
          img_in_line_binary.data[k*drawing.cols+j]=0;
          img_out_line_binary.data[k*drawing.cols+j]=0;

          if(drawing.data[k*drawing.cols+j]==69)break;
        }

        for(int k=drawing.rows-1;k>=0;k--)
        {
          img_in_line_binary.data[k*drawing.cols+j]=0;
          img_out_line_binary.data[k*drawing.cols+j]=0;

          if(drawing.data[k*drawing.cols+j]==69)break;
        }
      }

      //find traffic_light
      if(labeling_traffic_light.m_nBlobs != 0)
      {
        int idx_max_area = 0;

        for(int i=0;i<labeling_traffic_light.m_nBlobs;i++)
        {
          if(labeling_traffic_light.m_recBlobs[idx_max_area].size().area() < labeling_traffic_light.m_recBlobs[i].size().area())
          {
            idx_max_area = i;
          }
        }

        Point center;
        Point left;
        Point right;

        int circle_area = 0;
        double circle_area_rate = 0;
        int check_circle = 0;
        center.x = (img_bgr.cols*3/4) + labeling_traffic_light.m_recBlobs[idx_max_area].x + (labeling_traffic_light.m_recBlobs[idx_max_area].width/2);
        center.y = labeling_traffic_light.m_recBlobs[idx_max_area].y + (labeling_traffic_light.m_recBlobs[idx_max_area].height/2);

        labeling_traffic_light.m_recBlobs[idx_max_area].x += img_bgr.cols*3/4;

        // find center point(x,y)
        for(int i = 0 ; i < traffic_light_binary.cols ; i ++)
        {
          left.x = i;
          left.y = center.y;
          if((int)traffic_light_binary.at<uchar>(center.y,i)==255)
          {
              break;
          }
        }

        for(int i = center.x - 240 ; i <= traffic_light_binary.cols ; i ++)
        {
          right.x = i;
          right.y = center.y;
          if((int)traffic_light_binary.at<uchar>(center.y,i)==0)
          {
              break;
          }
        }

        // get circle area
        for(int i = labeling_traffic_light.m_recBlobs[idx_max_area].y ; i < labeling_traffic_light.m_recBlobs[idx_max_area].y +labeling_traffic_light.m_recBlobs[idx_max_area].height ; i ++)
        {
          for(int j = labeling_traffic_light.m_recBlobs[idx_max_area].x - 240 ; j < labeling_traffic_light.m_recBlobs[idx_max_area].x - 240 + labeling_traffic_light.m_recBlobs[idx_max_area].width ; j++)
          {
            if((int)traffic_light_binary.at<uchar>(i,j) == 255)
            {
                circle_area++;
            }
          }
        }

        // get circle area rate
        circle_area_rate = (double)circle_area / (double)(labeling_traffic_light.m_recBlobs[idx_max_area].area());

        // check circle (difference between width and height)
        check_circle = abs( (right.x - left.x) - (labeling_traffic_light.m_recBlobs[idx_max_area].height) );

        // circle condition
        if(check_circle < 4 && circle_area_rate > 0.83 && circle_area_rate < 0.95 && center.y < 70)
        {
          //red
          turtle_vision_msg.notice_information = TRAFFIC_MARK_RED;
          circle(img_bgr, center, (right.x - left.x)/2 ,Scalar(0,255,0),2);
          detection_flag = 1;
        }

        else if(check_circle < 4 && circle_area_rate > 0.83 && circle_area_rate < 0.95 && center.y > 70)
        {
          //yellow
          turtle_vision_msg.notice_information = TRAFFIC_MARK_YELLOW;
          circle(img_bgr, center, (right.x - left.x)/2 ,Scalar(0,255,0),2);
          detection_flag = 1;
        }
      }

      //find gate_bar
      if(labeling_gate_bar.m_nBlobs >= 2)
      {
        int * area = new int[labeling_gate_bar.m_nBlobs];
        int gate_bar_area = 0;
        for(int i=0;i<labeling_gate_bar.m_nBlobs;i++)
        {
            area[i] = labeling_gate_bar.m_recBlobs[i].size().area();
        }

        // sorting for finding the_biggest two images
        for(int i = 0 ; i < labeling_gate_bar.m_nBlobs ; i ++)
        {
            for(int j = i+1; j < labeling_gate_bar.m_nBlobs ; j ++)
            {
                if(area[i]<area[j])
                {
                    int temp = area[i];
                    area[i] = area[j];
                    area[j] = temp;
                }
            }
        }

        // find gate_bar center point(x,y)
        Point gate_bar_detection[2];
        gate_bar_detection[0].x = labeling_gate_bar.m_recBlobs[0].x + (labeling_gate_bar.m_recBlobs[0].width/2);
        gate_bar_detection[0].y = labeling_gate_bar.m_recBlobs[0].y + (labeling_gate_bar.m_recBlobs[0].height/2);

        gate_bar_detection[1].x = labeling_gate_bar.m_recBlobs[1].x + (labeling_gate_bar.m_recBlobs[1].width/2);
        gate_bar_detection[1].y = labeling_gate_bar.m_recBlobs[1].y + (labeling_gate_bar.m_recBlobs[1].height/2);

        line(img_bgr, gate_bar_detection[0], gate_bar_detection[1], Scalar(255,0,0),3);

        // get gate_bar area
        for(int i = labeling_gate_bar.m_recBlobs[0].y ; i < labeling_gate_bar.m_recBlobs[0].y +labeling_gate_bar.m_recBlobs[0].height ; i ++)
        {
          for(int j = labeling_gate_bar.m_recBlobs[0].x - 240 ; j < labeling_gate_bar.m_recBlobs[0].x - 240 + labeling_gate_bar.m_recBlobs[0].width ; j++)
          {
            if((int)gate_bar_binary.at<uchar>(i,j) == 255)
            {
              gate_bar_area++;
            }
          }
        }

        // set gate_bar angle
        double gate_bar_angle = abs(atan2(gate_bar_detection[0].y - gate_bar_detection[1].y, gate_bar_detection[0].x - gate_bar_detection[1].x)*180/PI);

        // gate_bar condition
        if(gate_bar_angle>170 || gate_bar_angle<10)
        {
          turtle_vision_msg.notice_information = GATE_BAR_MARK;
          detection_flag = 1;
        }
      }

      // find tunnel mark
      if(labeling_tunnel_mark.m_nBlobs != 0 )
      {
        int idx_max_area = 0;

        for(int i = 0 ; i < labeling_tunnel_mark.m_nBlobs ; i ++)
        {
          idx_max_area = i;

          rectangle(img_bgr, labeling_tunnel_mark.m_recBlobs[idx_max_area], Scalar(0,255,255),2);

          Mat tunnel_mark_copy_roi(labeling_tunnel_mark.m_recBlobs[idx_max_area].height, labeling_tunnel_mark.m_recBlobs[idx_max_area].width,CV_8UC3);

          // extending tunnel mark range
          int tunnel_mark_x = labeling_tunnel_mark.m_recBlobs[idx_max_area].x - 15;
          int tunnel_mark_y = labeling_tunnel_mark.m_recBlobs[idx_max_area].y - 15;
          int tunnel_mark_width = labeling_tunnel_mark.m_recBlobs[idx_max_area].width + 30;
          int tunnel_mark_height = labeling_tunnel_mark.m_recBlobs[idx_max_area].height + 30;

          if(tunnel_mark_x < 0) tunnel_mark_x = 0;
          if(tunnel_mark_y < 0) tunnel_mark_y = 0;
          if(tunnel_mark_width + labeling_tunnel_mark.m_recBlobs[idx_max_area].x > 320)
          {
            tunnel_mark_width -= 15;
          }

          if(tunnel_mark_height + labeling_tunnel_mark.m_recBlobs[idx_max_area].y > 240)
          {
            tunnel_mark_height -= 15;
          }

          int top_area = 0;
          int bottom_area = 0;
          double triangle_area_rate = 0;

          int left_check = 0;
          int right_check = 0;

          Point angle[3];

          // set tunnel mark vertex
          for(int i = labeling_tunnel_mark.m_recBlobs[idx_max_area].x ; i < labeling_tunnel_mark.m_recBlobs[idx_max_area].x + labeling_tunnel_mark.m_recBlobs[idx_max_area].width ; i++)
          {
            if((int)tunnel_mark_binary.at<uchar>(labeling_tunnel_mark.m_recBlobs[idx_max_area].y,i) == 255)
            {
              angle[0].x = i;
              angle[0].y = (labeling_tunnel_mark.m_recBlobs[idx_max_area].y);
            }
          }

          angle[1].x = (labeling_tunnel_mark.m_recBlobs[idx_max_area].x);
          angle[1].y = (labeling_tunnel_mark.m_recBlobs[idx_max_area].y) + labeling_tunnel_mark.m_recBlobs[idx_max_area].height;


          angle[2].x = (labeling_tunnel_mark.m_recBlobs[idx_max_area].x) + labeling_tunnel_mark.m_recBlobs[idx_max_area].width;
          angle[2].y = (labeling_tunnel_mark.m_recBlobs[idx_max_area].y) + labeling_tunnel_mark.m_recBlobs[idx_max_area].height;

          double tunnel_angle1 = abs(atan2(angle[0].y - angle[1].y, angle[0].x - angle[1].x)*180/PI);
          double tunnel_angle2 = abs(atan2(angle[0].y - angle[2].y, angle[0].x - angle[2].x)*180/PI);

          // make tunnel to triangle
          for(int i = labeling_tunnel_mark.m_recBlobs[idx_max_area].y ; i < ( labeling_tunnel_mark.m_recBlobs[idx_max_area].y) + (labeling_tunnel_mark.m_recBlobs[idx_max_area].height) ; i++)
          {
            for(left_check = labeling_tunnel_mark.m_recBlobs[idx_max_area].x, right_check = labeling_tunnel_mark.m_recBlobs[idx_max_area].x + labeling_tunnel_mark.m_recBlobs[idx_max_area].width ;
                left_check < labeling_tunnel_mark.m_recBlobs[idx_max_area].x + labeling_tunnel_mark.m_recBlobs[idx_max_area].width,
                right_check > labeling_tunnel_mark.m_recBlobs[idx_max_area].x;
                left_check ++ , right_check --)
             {
                if((int)tunnel_mark_binary.at<uchar>(i,left_check) == 255 && (int)tunnel_mark_binary.at<uchar>(i,right_check) == 255)
                {
                    for(int k = left_check ; k < right_check ; k ++)
                    {
                        tunnel_mark_binary.at<uchar>(i,k) = 255;
                    }
                }

                else
                {
                  tunnel_mark_binary.at<uchar>(i,left_check) = 0;
                  tunnel_mark_binary.at<uchar>(i,right_check) = 0;
                }
             }
          }

          // get tunnel mark top area
          for(int i = labeling_tunnel_mark.m_recBlobs[idx_max_area].y ; i < labeling_tunnel_mark.m_recBlobs[idx_max_area].y + (labeling_tunnel_mark.m_recBlobs[idx_max_area].height)/2 ; i ++)
          {
            for(int j = labeling_tunnel_mark.m_recBlobs[idx_max_area].x ; j < labeling_tunnel_mark.m_recBlobs[idx_max_area].x + labeling_tunnel_mark.m_recBlobs[idx_max_area].width ; j++)
            {
              if((int)tunnel_mark_binary.at<uchar>(i,j) == 255)
              {
                top_area ++;
              }
            }
          }

          // get tunnel mark bottom area
          for(int i = labeling_tunnel_mark.m_recBlobs[idx_max_area].y + (labeling_tunnel_mark.m_recBlobs[idx_max_area].height)/2 ; i < labeling_tunnel_mark.m_recBlobs[idx_max_area].y + (labeling_tunnel_mark.m_recBlobs[idx_max_area].height) ; i ++)
          {
            for(int j = labeling_tunnel_mark.m_recBlobs[idx_max_area].x ; j < labeling_tunnel_mark.m_recBlobs[idx_max_area].x + labeling_tunnel_mark.m_recBlobs[idx_max_area].width ; j++)
            {
              if((int)tunnel_mark_binary.at<uchar>(i,j) == 255)
              {
                  bottom_area ++;
              }
            }
          }

          // sum area = tunnel mark area
          double triangle_area = bottom_area + top_area;

          // get tunnel mark area rate
          triangle_area_rate = (double)top_area / (double)bottom_area;
          turtle_vision_msg.tunnel_mark_labeling = false;

          // tunnel mark condition
          if(triangle_area_rate > TRIANGLE_AREA_RATE_MIN &&
             triangle_area_rate < TRIANGLE_AREA_RATE_MAX &&
             triangle_area > TRIANGLE_AREA_MIN &&
             triangle_area < TRIANGLE_AREA_MAX &&
             tunnel_angle1 > TRIANGLE_FIRST_ANGLE_MIN &&
             tunnel_angle1 < TRIANGLE_FIRST_ANGLE_MAX &&
             tunnel_angle2 > TRIANGLE_SECOND_ANGLE_MIN &&
             tunnel_angle1 < TRIANGLE_SECOND_ANGLE_MAX &&
             (int)tunnel_mark_binary.at<uchar>(labeling_tunnel_mark.m_recBlobs[idx_max_area].y,labeling_tunnel_mark.m_recBlobs[idx_max_area].x) == 0 &&
             (int)tunnel_mark_binary.at<uchar>(labeling_tunnel_mark.m_recBlobs[idx_max_area].y,labeling_tunnel_mark.m_recBlobs[idx_max_area].x + labeling_tunnel_mark.m_recBlobs[idx_max_area].width) == 0 &&
             (int)tunnel_mark_binary.at<uchar>(labeling_tunnel_mark.m_recBlobs[idx_max_area].y + (labeling_tunnel_mark.m_recBlobs[idx_max_area].height/2),labeling_tunnel_mark.m_recBlobs[idx_max_area].x + labeling_tunnel_mark.m_recBlobs[idx_max_area].width/2) == 255)
          {
             turtle_vision_msg.tunnel_mark_labeling = true;

             // draw line
             line(img_bgr, angle[0], angle[1], Scalar(255,255,0),2);
             line(img_bgr, angle[1], angle[2], Scalar(255,255,0),2);
             line(img_bgr, angle[0], angle[2], Scalar(255,255,0),2);

             tunnel_mark_copy_roi = img(Rect(tunnel_mark_x, tunnel_mark_y ,tunnel_mark_width,tunnel_mark_height)).clone();

             check_data_count ++;

             // run SRUF for detecting tunnel mark
             if(check_data_count > 5)
             {
                Ptr<SURF> detector = SURF::create(200);
                Ptr<SURF> extractor = SurfDescriptorExtractor::create();
                FlannBasedMatcher matcher;

                Mat des_object;
                Mat des_image;
                Mat img_matches;

                vector <KeyPoint> kp_object;
                vector <KeyPoint> kp_image;
                vector <DMatch> matches;
                vector <DMatch> good_matches;

                Mat object2 = imread("/home/turtle1/Desktop/mark_sample/tunnel_mark_robotis.png");
                Mat object1;

                cv::resize(tunnel_mark_copy_roi, tunnel_mark_copy_roi, cv::Size(tunnel_mark_copy_roi.cols*6, tunnel_mark_copy_roi.rows*6), 0, 0, CV_INTER_NN);

                cvtColor(tunnel_mark_copy_roi, object1, CV_BGR2RGB);
                detector->detect(object1, kp_object);
                extractor->compute(object1, kp_object, des_object);

                cvtColor(object2, object2, CV_BGR2RGB);
                detector->detect(object2, kp_image);
                extractor->compute(object2, kp_image, des_image);

                matcher.match(des_object, des_image, matches);

                double max_dist = 0; double min_dist = 100;

                for( int i = 0; i < des_object.rows; i++ )
                {
                  double dist = matches[i].distance;
                  if( dist < min_dist )
                    min_dist = dist;

                  if( dist > max_dist )
                    max_dist = dist;
                }

                for( int i = 0; i < des_object.rows; i++ )
                {
                  if( matches[i].distance <= max(4* min_dist, 0.05 ) )
                  {
                    good_matches.push_back( matches[i]);
                  }
                }

                drawMatches( object1, kp_object, object2, kp_image,
                             good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                             vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

                // show SURF image in ui
                QImage Qimg_tunnel_copy_mark((const unsigned char*)(img_matches.data), img_matches.cols, img_matches.rows, img_matches.step, QImage::Format_RGB888);
                ui.label_image->setPixmap(QPixmap::fromImage(Qimg_tunnel_copy_mark));

                img_matches.release();
                object1.release();
                object2.release();
                tunnel_mark_copy_roi.release();
                des_image.release();
                des_object.release();

                check_data_count = 0;

                if(good_matches.size() > TUNNEL_MATCHES_SIZE_MIN && good_matches.size() < TUNNEL_MATCHES_SIZE_MAX)
                {
                  turtle_vision_msg.notice_information = TUNNEL_MARK;
                  if(mode == RUN)
                  {
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                  }
                  else if(mode == STOP)
                  {
                      turtle_vision_msg.degree_left = 0.0;
                      turtle_vision_msg.degree_right = 0.0;
                      turtle_vision_msg.notice_information = GATE_BAR_MARK;
                      qnode.turtle_line_pub.publish(turtle_vision_msg);
                  }
                }
              }
            }
          }
        }

        // find parking mark
        if(labeling_parking_mark.m_nBlobs != 0)
        {
          int idx_max_area = 0;

          for(int i=0;i<labeling_parking_mark.m_nBlobs;i++)
          {
            idx_max_area = i;

            Mat parking_mark_copy_roi(labeling_parking_mark.m_recBlobs[idx_max_area].height, labeling_parking_mark.m_recBlobs[idx_max_area].width,CV_8UC3);

            int rec_area = 0;
            double rec_area_rate = 0;
            int check_rec = 0;

            // get rectangular area
            for(int i = labeling_parking_mark.m_recBlobs[idx_max_area].y ; i < labeling_parking_mark.m_recBlobs[idx_max_area].y +labeling_parking_mark.m_recBlobs[idx_max_area].height ; i ++)
            {
              for(int j = labeling_parking_mark.m_recBlobs[idx_max_area].x ; j < labeling_parking_mark.m_recBlobs[idx_max_area].x + labeling_parking_mark.m_recBlobs[idx_max_area].width ; j++)
              {
                if((int)parking_mark_binary.at<uchar>(i,j) == 255)
                {
                  rec_area++;
                }
              }
            }

            // copy parking mark partition
            parking_mark_copy_roi = img(Rect(labeling_parking_mark.m_recBlobs[idx_max_area].x  + img_bgr.cols/2, labeling_parking_mark.m_recBlobs[idx_max_area].y,(labeling_parking_mark.m_recBlobs[idx_max_area].width),(labeling_parking_mark.m_recBlobs[idx_max_area].height))).clone();

            // get parking mark area rate
            rec_area_rate = (double)rec_area / (double)(labeling_parking_mark.m_recBlobs[idx_max_area].area());

            // diffrence between width and height
            check_rec = abs( (labeling_parking_mark.m_recBlobs[idx_max_area].width)- (labeling_parking_mark.m_recBlobs[idx_max_area].height) );

            turtle_vision_msg.parking_mark_labeling = false;

            // parking mark condition
            if(check_rec < REC_CHECK && rec_area_rate > REC_AREA_RATE_MIN && rec_area > REC_AREA_MIN && rec_area < REC_AREA_MAX )
            {
              turtle_vision_msg.parking_mark_labeling = true;
              rectangle(img_bgr , Rect(labeling_parking_mark.m_recBlobs[idx_max_area].x  + img_bgr.cols/2, labeling_parking_mark.m_recBlobs[idx_max_area].y,(labeling_parking_mark.m_recBlobs[idx_max_area].width),(labeling_parking_mark.m_recBlobs[idx_max_area].height)), Scalar(255,0,255),2);

              check_data_count ++;

              // run SURF for detectiong parking mark
              if(check_data_count > 5)
              {
                Ptr<SURF> detector = SURF::create(250);
                Ptr<SURF> extractor = SurfDescriptorExtractor::create();
                FlannBasedMatcher matcher;

                Mat des_object;
                Mat des_image;
                Mat img_matches;

                vector <KeyPoint> kp_object;
                vector <KeyPoint> kp_image;
                vector <DMatch> matches;
                vector <DMatch> good_matches;

                Mat object2 = imread("/home/turtle1/Desktop/mark_sample/parking_mark_1.png");
                Mat object1;

                cv::resize(parking_mark_copy_roi, parking_mark_copy_roi, cv::Size(parking_mark_copy_roi.cols*7, parking_mark_copy_roi.rows*7), 0, 0, CV_INTER_NN);

                cvtColor(parking_mark_copy_roi, object1, CV_BGR2RGB);
                detector->detect(object1, kp_object);
                extractor->compute(object1, kp_object, des_object);

                cvtColor(object2, object2, CV_BGR2RGB);
                detector->detect(object2, kp_image);
                extractor->compute(object2, kp_image, des_image);

                matcher.match(des_object, des_image, matches);

                double max_dist = 0; double min_dist = 100;

                for( int i = 0; i < des_object.rows; i++ )
                {
                  double dist = matches[i].distance;
                  if( dist < min_dist )
                    min_dist = dist;

                  if( dist > max_dist )
                    max_dist = dist;
                }

                for( int i = 0; i < des_object.rows; i++ )
                {
                  if( matches[i].distance <= max(5* min_dist, 0.02 ) )
                  {
                    good_matches.push_back( matches[i]);
                  }
                }

                drawMatches( object1, kp_object, object2, kp_image,
                             good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                             vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );



                if(good_matches.size() > PARKING_MATCHES_SIZE_MIN && good_matches.size() < PARKING_MATCHES_SIZE_MAX)
                {
                  turtle_vision_msg.notice_information = PARKING_MARK;
                  if(mode == RUN)
                  {
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                  }
                  else if(mode == STOP)
                  {
                    turtle_vision_msg.degree_left = 0.0;
                    turtle_vision_msg.degree_right = 0.0;
                    turtle_vision_msg.notice_information = GATE_BAR_MARK;
                    qnode.turtle_line_pub.publish(turtle_vision_msg);
                  }

                }

                QImage Qimg_parking_copy_mark((const unsigned char*)(img_matches.data), img_matches.cols, img_matches.rows, img_matches.step, QImage::Format_RGB888);
                ui.label_image->setPixmap(QPixmap::fromImage(Qimg_parking_copy_mark));

                img_matches.release();
                object1.release();
                object2.release();
                parking_mark_copy_roi.release();
                des_image.release();
                des_object.release();

                check_data_count = 0;

              }
            }
         }
      }


      // find line
      if(labeling_in_line.m_nBlobs != 0 && detection_flag==0)
      {
        int idx_max_area = 0;

        for(int i=0;i<labeling_in_line.m_nBlobs;i++)
        {
          if(labeling_in_line.m_recBlobs[idx_max_area].size().area() < labeling_in_line.m_recBlobs[i].size().area())
          {
            idx_max_area = i;
          }
        }

        Mat img_in_line_edge;

        // get edge
        Canny(img_in_line_binary(labeling_in_line.m_recBlobs[idx_max_area]), img_in_line_edge, 0, 255);

        // run ransac for detecting in line (yellow line)
        RobitRansacLine ransac_in_line(img_in_line_edge);
        ransac_in_line.runRansac();

        Point in_line_a = ransac_in_line.getPointLineA();
        Point in_line_b = ransac_in_line.getPointLineB();

        // get in line degree
        double in_line_degree = ransac_in_line.getDegree();

        in_line_a.x += labeling_in_line.m_recBlobs[idx_max_area].x;
        in_line_b.x += labeling_in_line.m_recBlobs[idx_max_area].x;
        in_line_a.y += img_bgr.rows/2 + labeling_in_line.m_recBlobs[idx_max_area].y;
        in_line_b.y += img_bgr.rows/2 + labeling_in_line.m_recBlobs[idx_max_area].y;
        labeling_in_line.m_recBlobs[idx_max_area].y += img_bgr.rows/2;

        line(img_bgr, in_line_a, in_line_b, Scalar(0,0,255),3);
        rectangle(img_bgr, labeling_in_line.m_recBlobs[idx_max_area], Scalar(0,255,255),2);

        turtle_vision_msg.degree_left = in_line_degree;
        turtle_vision_msg.in_line_mpt_x = labeling_in_line.m_recBlobs[idx_max_area].x ;
      }

      if(labeling_out_line.m_nBlobs != 0 && detection_flag==0)
      {
          int idx_max_area = 0;

          for(int i=0;i<labeling_out_line.m_nBlobs;i++)
          {
              if(labeling_out_line.m_recBlobs[idx_max_area].size().area() < labeling_out_line.m_recBlobs[i].size().area())
              {
                  idx_max_area = i;
              }
          }

          Mat img_out_line_edge;

          // get edge
          Canny(img_out_line_binary(labeling_out_line.m_recBlobs[idx_max_area]), img_out_line_edge, 0, 255);

          // run ransac for detecting outline (white line)
          RobitRansacLine ransac_out_line(img_out_line_edge);
          ransac_out_line.runRansac();

          Point out_line_a = ransac_out_line.getPointLineA();
          Point out_line_b = ransac_out_line.getPointLineB();

          // get in line degree
          double out_line_degree = ransac_out_line.getDegree();

          out_line_a.x += labeling_out_line.m_recBlobs[idx_max_area].x;
          out_line_b.x += labeling_out_line.m_recBlobs[idx_max_area].x;
          out_line_a.y += img_bgr.rows/2 + labeling_out_line.m_recBlobs[idx_max_area].y;
          out_line_b.y += img_bgr.rows/2 + labeling_out_line.m_recBlobs[idx_max_area].y;
          labeling_out_line.m_recBlobs[idx_max_area].y += img_bgr.rows/2;

          line(img_bgr, out_line_a, out_line_b, Scalar(0,0,255),3);
          rectangle(img_bgr, labeling_out_line.m_recBlobs[idx_max_area], Scalar(255,255,0),2);

          turtle_vision_msg.degree_right = out_line_degree;
          turtle_vision_msg.out_line_mpt_x = labeling_out_line.m_recBlobs[idx_max_area].x +(labeling_out_line.m_recBlobs[idx_max_area].width);
      }

      // detect interference
      if(detection_flag ==1)
      {
        detection_flag = 0;
      }

      // publish messages
      if(mode == RUN)
      qnode.turtle_line_pub.publish(turtle_vision_msg);


      else if(mode == STOP)
      {
         turtle_vision_msg.degree_left = 0.0;
         turtle_vision_msg.degree_right = 0.0;
         turtle_vision_msg.notice_information = GATE_BAR_MARK;
         qnode.turtle_line_pub.publish(turtle_vision_msg);
      }

      // send image to ui
      QImage Qimg_bgr((const unsigned char*)(img_bgr.data), img_bgr.cols, img_bgr.rows, QImage::Format_RGB888);
      ui.labelRGB->setPixmap(QPixmap::fromImage(Qimg_bgr.rgbSwapped()));

      QImage Qimg_line((const unsigned char*)(img_line_binary.data), img_line_binary.cols, img_line_binary.rows, QImage::Format_Indexed8);
      ui.labelLINE->setPixmap(QPixmap::fromImage(Qimg_line));

      QImage Qimg_gate_bar((const unsigned char*)(gate_bar_binary.data), gate_bar_binary.cols, gate_bar_binary.rows, QImage::Format_Indexed8);
      ui.labelGATE_BAR->setPixmap(QPixmap::fromImage(Qimg_gate_bar));

      QImage Qimg_traffic_light((const unsigned char*)(traffic_light_binary.data), traffic_light_binary.cols, traffic_light_binary.rows, QImage::Format_Indexed8);
      ui.labelTRAFFIC_LIGHT->setPixmap(QPixmap::fromImage(Qimg_traffic_light));

      QImage Qimg_parking_mark((const unsigned char*)(parking_mark_binary.data), parking_mark_binary.cols, parking_mark_binary.rows, QImage::Format_Indexed8);
      ui.labelPARKING_MARK->setPixmap(QPixmap::fromImage(Qimg_parking_mark));

      QImage Qimg_tunnel_mark((const unsigned char*)(tunnel_mark_binary.data), tunnel_mark_binary.cols, tunnel_mark_binary.rows, QImage::Format_Indexed8);
      ui.labelTUNNEL_MARK->setPixmap(QPixmap::fromImage(Qimg_tunnel_mark));
   }
}

// ui setting
void MainWindow::on_horizontalSlider_row_hue_min_valueChanged(int value)
{
  if(is_in_line == true)
  {
    in_line_hue_min = value;
  }
  else if(is_out_line == true)
  {
    out_line_hue_min = value;
  }
  else if(is_gate_bar == true)
  {
    gate_bar_row_hue_min = value;
  }
  else if(is_traffic_red_light == true)
  {
    traffic_light_row_red_hue_min = value;
  }
  else if(is_traffic_yellow_light == true)
  {
    traffic_light_yellow_hue_min = value;
  }
  else if(is_parking_mark == true)
  {
    parking_mark_hue_min = value;
  }
  else if(is_tunnel_mark == true)
  {
    tunnel_mark_row_hue_min = value;
  }
  else
  {

  }
  ui.label_row_hue_min->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_row_hue_max_valueChanged(int value)
{
  if(is_in_line == true)
  {
    in_line_hue_max = value;
  }
  else if(is_out_line == true)
  {
    out_line_hue_max = value;
  }
  else if(is_gate_bar == true)
  {
    gate_bar_row_hue_max = value;
  }
  else if(is_traffic_red_light == true)
  {
    traffic_light_row_red_hue_max = value;
  }
  else if(is_traffic_yellow_light == true)
  {
    traffic_light_yellow_hue_max = value;
  }
  else if(is_parking_mark == true)
  {
    parking_mark_hue_max = value;
  }
  else if(is_tunnel_mark == true)
  {
    tunnel_mark_row_hue_max = value;
  }
  else
  {

  }
  ui.label_row_hue_max->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_high_hue_min_valueChanged(int value)
{
  if(is_in_line == true)
  {
    in_line_hue_min = value;
  }
  else if(is_out_line == true)
  {
    out_line_hue_min = value;
  }
  else if(is_gate_bar == true)
  {
    gate_bar_high_hue_min = value;
  }
  else if(is_traffic_red_light == true)
  {
    traffic_light_high_red_hue_min = value;
  }
  else if(is_traffic_yellow_light == true)
  {
    traffic_light_yellow_hue_min = value;
  }
  else if(is_parking_mark == true)
  {
    parking_mark_hue_min = value;
  }
  else if(is_tunnel_mark == true)
  {
    tunnel_mark_high_hue_min = value;
  }
  else
  {

  }
  ui.label_high_hue_min->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_high_hue_max_valueChanged(int value)
{
  if(is_in_line == true)
  {
    in_line_hue_max = value;
  }
  else if(is_out_line == true)
  {
    out_line_hue_max = value;
  }
  else if(is_gate_bar == true)
  {
    gate_bar_high_hue_max = value;
  }
  else if(is_traffic_red_light == true)
  {
    traffic_light_high_red_hue_max = value;
  }
  else if(is_traffic_yellow_light == true)
  {
    traffic_light_yellow_hue_max = value;
  }
  else if(is_parking_mark == true)
  {
    parking_mark_hue_max = value;
  }
  else if(is_tunnel_mark == true)
  {
    tunnel_mark_high_hue_max = value;
  }
  else
  {

  }
  ui.label_high_hue_max->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_sat_min_valueChanged(int value)
{
  if(is_in_line == true )
  {
    in_line_sat_min = value;
  }
  else if(is_out_line == true)
  {
    out_line_sat_min = value;
  }
  else if(is_gate_bar == true)
  {
    gate_bar_sat_min = value;
  }
  else if(is_traffic_red_light == true)
  {
    traffic_light_red_sat_min = value;
  }
  else if(is_traffic_yellow_light == true)
  {
    traffic_light_yellow_sat_min = value;
  }
  else if(is_parking_mark == true)
  {
    parking_mark_sat_min = value;
  }
  else if(is_tunnel_mark == true)
  {
    tunnel_mark_sat_min = value;
  }
  else
  {

  }
  ui.label_sat_min->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_sat_max_valueChanged(int value)
{
  if(is_in_line == true)
  {
    in_line_sat_max = value;
  }
  else if(is_out_line == true)
  {
    out_line_sat_max = value;
  }
  else if(is_gate_bar == true)
  {
    gate_bar_sat_max = value;
  }
  else if(is_traffic_red_light == true)
  {
    traffic_light_red_sat_max = value;
  }
  else if(is_traffic_yellow_light == true)
  {
    traffic_light_yellow_sat_max = value;
  }
  else if(is_parking_mark == true)
  {
    parking_mark_sat_max = value;
  }
  else if(is_tunnel_mark == true)
  {
    tunnel_mark_sat_max = value;
  }
  else
  {

  }
  ui.label_sat_max->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_val_min_valueChanged(int value)
{
  if(is_in_line == true )
  {
    in_line_val_min = value;
  }
  else if(is_out_line == true)
  {
    out_line_val_min = value;
  }
  else if(is_gate_bar == true)
  {
    gate_bar_val_min = value;
  }
  else if(is_traffic_red_light == true)
  {
    traffic_light_red_val_min = value;
  }
  else if(is_traffic_yellow_light == true)
  {
    traffic_light_yellow_val_min = value;
  }
  else if(is_parking_mark == true)
  {
    parking_mark_val_min = value;
  }
  else if(is_tunnel_mark == true)
  {
    tunnel_mark_val_min = value;
  }
  else
  {

  }
  ui.label_val_min->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_val_max_valueChanged(int value)
{
  if(is_in_line == true)
  {
    in_line_val_max = value;
  }
  else if(is_out_line == true)
  {
    out_line_val_max = value;
  }
  else if(is_gate_bar == true)
  {
    gate_bar_val_max = value;
  }
  else if(is_traffic_red_light == true)
  {
    traffic_light_red_val_max = value;
  }
  else if(is_traffic_yellow_light == true)
  {
    traffic_light_yellow_val_max = value;
  }
  else if(is_parking_mark == true)
  {
    parking_mark_val_max = value;
  }
  else if(is_tunnel_mark == true)
  {
    tunnel_mark_val_max = value;
  }
  else
  {

  }
  ui.label_val_max->setText(QString::number(value));
}

void MainWindow::on_pushButton_save_parameter_clicked()
{
  ofstream fout_default_color("/home/turtle1/catkin_ws/src/turtlevision/parameter_file/default_color.txt");
  if(fout_default_color.is_open())
  {
    fout_default_color      <<      in_line_hue_min             << endl
                            <<      in_line_sat_min             << endl
                            <<      in_line_hue_max             << endl
                            <<      in_line_sat_max             << endl
                            <<      in_line_val_min             << endl
                            <<      in_line_val_max             << endl

                            <<      out_line_hue_min            << endl
                            <<      out_line_sat_min            << endl
                            <<      out_line_hue_max            << endl
                            <<      out_line_sat_max            << endl
                            <<      out_line_val_min            << endl
                            <<      out_line_val_max            << endl

                            <<      gate_bar_row_hue_min           << endl
                            <<      gate_bar_sat_min               << endl
                            <<      gate_bar_row_hue_max           << endl
                            <<      gate_bar_sat_max               << endl
                            <<      gate_bar_high_hue_min          << endl
                            <<      gate_bar_high_hue_max          << endl
                            <<      gate_bar_val_min               << endl
                            <<      gate_bar_val_max               << endl

                            <<      traffic_light_row_red_hue_min   << endl
                            <<      traffic_light_red_sat_min       << endl
                            <<      traffic_light_row_red_hue_max   << endl
                            <<      traffic_light_red_sat_max       << endl
                            <<      traffic_light_high_red_hue_min  << endl
                            <<      traffic_light_high_red_hue_max  << endl
                            <<      traffic_light_red_val_min       << endl
                            <<      traffic_light_red_val_max       << endl

                            <<      parking_mark_hue_min        << endl
                            <<      parking_mark_sat_min        << endl
                            <<      parking_mark_hue_max        << endl
                            <<      parking_mark_sat_max        << endl
                            <<      parking_mark_val_min        << endl
                            <<      parking_mark_val_max        << endl

                            <<      tunnel_mark_row_hue_min     << endl
                            <<      tunnel_mark_sat_min         << endl
                            <<      tunnel_mark_row_hue_max     << endl
                            <<      tunnel_mark_sat_max         << endl
                            <<      tunnel_mark_high_hue_min    << endl
                            <<      tunnel_mark_high_hue_max    << endl
                            <<      tunnel_mark_val_min         << endl
                            <<      tunnel_mark_val_max         << endl


                            <<      traffic_light_yellow_hue_min   << endl
                            <<      traffic_light_yellow_sat_min   << endl
                            <<      traffic_light_yellow_hue_max   << endl
                            <<      traffic_light_yellow_sat_max   << endl
                            <<      traffic_light_yellow_val_min   << endl
                            <<      traffic_light_yellow_val_max   << endl;

    fout_default_color.close();
  }
}

void MainWindow::on_checkBox_in_line_toggled(bool checked)
{
  get_parameter();

  is_in_line = checked;
  if(is_in_line == true)
  {
    is_out_line = false;
    is_gate_bar = false;
    is_traffic_red_light = false;
    is_traffic_yellow_light = false;
    is_parking_mark = false;
    is_tunnel_mark = false;

    on_horizontalSlider_row_hue_min_valueChanged(in_line_hue_min);
    on_horizontalSlider_row_hue_max_valueChanged(in_line_hue_max);
    on_horizontalSlider_high_hue_min_valueChanged(in_line_hue_min);
    on_horizontalSlider_high_hue_max_valueChanged(in_line_hue_max);
    on_horizontalSlider_sat_min_valueChanged(in_line_sat_min);
    on_horizontalSlider_sat_max_valueChanged(in_line_sat_max);
    on_horizontalSlider_val_min_valueChanged(in_line_val_min);
    on_horizontalSlider_val_max_valueChanged(in_line_val_max);

    ui.horizontalSlider_row_hue_min->setValue(in_line_hue_min);
    ui.horizontalSlider_row_hue_max->setValue(in_line_hue_max);
    ui.horizontalSlider_high_hue_min->setValue(in_line_hue_min);
    ui.horizontalSlider_high_hue_max->setValue(in_line_hue_max);
    ui.horizontalSlider_sat_min->setValue(in_line_sat_min);
    ui.horizontalSlider_sat_max->setValue(in_line_sat_max);
    ui.horizontalSlider_val_min->setValue(in_line_val_min);
    ui.horizontalSlider_val_max->setValue(in_line_val_max);

  }
}

void MainWindow::on_checkBox_out_line_toggled(bool checked)
{
  get_parameter();

  is_out_line = checked;
  if(is_out_line == true)
  {
    is_in_line = false;
    is_gate_bar = false;
    is_traffic_red_light = false;
    is_traffic_yellow_light = false;
    is_parking_mark = false;
    is_tunnel_mark = false;

    on_horizontalSlider_row_hue_min_valueChanged(out_line_hue_min);
    on_horizontalSlider_row_hue_max_valueChanged(out_line_hue_max);
    on_horizontalSlider_high_hue_min_valueChanged(out_line_hue_min);
    on_horizontalSlider_high_hue_max_valueChanged(out_line_hue_max);
    on_horizontalSlider_sat_min_valueChanged(out_line_sat_min);
    on_horizontalSlider_sat_max_valueChanged(out_line_sat_max);
    on_horizontalSlider_val_min_valueChanged(out_line_val_min);
    on_horizontalSlider_val_max_valueChanged(out_line_val_max);


    ui.horizontalSlider_row_hue_min->setValue(out_line_hue_min);
    ui.horizontalSlider_row_hue_max->setValue(out_line_hue_max);
    ui.horizontalSlider_high_hue_min->setValue(out_line_hue_min);
    ui.horizontalSlider_high_hue_max->setValue(out_line_hue_max);
    ui.horizontalSlider_sat_min->setValue(out_line_sat_min);
    ui.horizontalSlider_sat_max->setValue(out_line_sat_max);
    ui.horizontalSlider_val_min->setValue(out_line_val_min);
    ui.horizontalSlider_val_max->setValue(out_line_val_max);
  }
}

void MainWindow::on_checkBox_gate_bar_toggled(bool checked)
{
  get_parameter();

  is_gate_bar = checked;
  if(is_gate_bar == true)
  {
    is_in_line = false;
    is_out_line = false;
    is_traffic_red_light = false;
    is_traffic_yellow_light = false;
    is_parking_mark = false;
    is_tunnel_mark = false;

    on_horizontalSlider_row_hue_min_valueChanged(gate_bar_row_hue_min);
    on_horizontalSlider_row_hue_max_valueChanged(gate_bar_row_hue_max);
    on_horizontalSlider_high_hue_min_valueChanged(gate_bar_high_hue_min);
    on_horizontalSlider_high_hue_max_valueChanged(gate_bar_high_hue_max);
    on_horizontalSlider_sat_min_valueChanged(gate_bar_sat_min);
    on_horizontalSlider_sat_max_valueChanged(gate_bar_sat_max);
    on_horizontalSlider_val_min_valueChanged(gate_bar_val_min);
    on_horizontalSlider_val_max_valueChanged(gate_bar_val_max);

    ui.horizontalSlider_row_hue_min->setValue(gate_bar_row_hue_min);
    ui.horizontalSlider_row_hue_max->setValue(gate_bar_row_hue_max);
    ui.horizontalSlider_high_hue_min->setValue(gate_bar_high_hue_min);
    ui.horizontalSlider_high_hue_max->setValue(gate_bar_high_hue_max);
    ui.horizontalSlider_sat_min->setValue(gate_bar_sat_min);
    ui.horizontalSlider_sat_max->setValue(gate_bar_sat_max);
    ui.horizontalSlider_val_min->setValue(gate_bar_val_min);
    ui.horizontalSlider_val_max->setValue(gate_bar_val_max);
  }
}

void MainWindow::on_checkBox_traffic_light_red_toggled(bool checked)
{
  get_parameter();

  is_traffic_red_light = checked;
  if(is_traffic_red_light == true)
  {
    is_in_line = false;
    is_out_line = false;
    is_traffic_yellow_light = false;
    is_gate_bar = false;
    is_parking_mark = false;
    is_tunnel_mark = false;

    on_horizontalSlider_row_hue_min_valueChanged(traffic_light_row_red_hue_min);
    on_horizontalSlider_row_hue_max_valueChanged(traffic_light_row_red_hue_max);
    on_horizontalSlider_high_hue_min_valueChanged(traffic_light_high_red_hue_min);
    on_horizontalSlider_high_hue_max_valueChanged(traffic_light_high_red_hue_max);
    on_horizontalSlider_sat_min_valueChanged(traffic_light_red_sat_min);
    on_horizontalSlider_sat_max_valueChanged(traffic_light_red_sat_max);
    on_horizontalSlider_val_min_valueChanged(traffic_light_red_val_min);
    on_horizontalSlider_val_max_valueChanged(traffic_light_red_val_max);

    ui.horizontalSlider_row_hue_min->setValue(traffic_light_row_red_hue_min);
    ui.horizontalSlider_row_hue_max->setValue(traffic_light_row_red_hue_max);
    ui.horizontalSlider_high_hue_min->setValue(traffic_light_high_red_hue_min);
    ui.horizontalSlider_high_hue_max->setValue(traffic_light_high_red_hue_max);
    ui.horizontalSlider_sat_min->setValue(traffic_light_red_sat_min);
    ui.horizontalSlider_sat_max->setValue(traffic_light_red_sat_max);
    ui.horizontalSlider_val_min->setValue(traffic_light_red_val_min);
    ui.horizontalSlider_val_max->setValue(traffic_light_red_val_max);
  }
}

void MainWindow::on_checkBox_traffic_light_yellow_toggled(bool checked)
{
  get_parameter();

  is_traffic_yellow_light = checked;
  if(is_traffic_yellow_light == true)
  {
    is_in_line = false;
    is_out_line = false;
    is_traffic_red_light = false;
    is_gate_bar = false;
    is_parking_mark = false;
    is_tunnel_mark = false;

    on_horizontalSlider_row_hue_min_valueChanged(traffic_light_yellow_hue_min);
    on_horizontalSlider_row_hue_max_valueChanged(traffic_light_yellow_hue_max);
    on_horizontalSlider_high_hue_min_valueChanged(traffic_light_yellow_hue_min);
    on_horizontalSlider_high_hue_max_valueChanged(traffic_light_yellow_hue_max);
    on_horizontalSlider_sat_min_valueChanged(traffic_light_yellow_sat_min);
    on_horizontalSlider_sat_max_valueChanged(traffic_light_yellow_sat_max);
    on_horizontalSlider_val_min_valueChanged(traffic_light_yellow_val_min);
    on_horizontalSlider_val_max_valueChanged(traffic_light_yellow_val_max);

    ui.horizontalSlider_row_hue_min->setValue(traffic_light_yellow_hue_min);
    ui.horizontalSlider_row_hue_max->setValue(traffic_light_yellow_hue_max);
    ui.horizontalSlider_high_hue_min->setValue(traffic_light_yellow_hue_min);
    ui.horizontalSlider_high_hue_max->setValue(traffic_light_yellow_hue_max);
    ui.horizontalSlider_sat_min->setValue(traffic_light_yellow_sat_min);
    ui.horizontalSlider_sat_max->setValue(traffic_light_yellow_sat_max);
    ui.horizontalSlider_val_min->setValue(traffic_light_yellow_val_min);
    ui.horizontalSlider_val_max->setValue(traffic_light_yellow_val_max);
  }
}

void MainWindow::on_checkBox_parking_mark_toggled(bool checked)
{
  get_parameter();

  is_parking_mark = checked;
  if(is_parking_mark == true)
  {
    is_in_line = false;
    is_out_line = false;
    is_gate_bar = false;
    is_traffic_red_light = false;
    is_traffic_yellow_light = false;
    is_tunnel_mark = false;

    on_horizontalSlider_row_hue_min_valueChanged(parking_mark_hue_min);
    on_horizontalSlider_row_hue_max_valueChanged(parking_mark_hue_max);
    on_horizontalSlider_high_hue_min_valueChanged(parking_mark_hue_min);
    on_horizontalSlider_high_hue_max_valueChanged(parking_mark_hue_max);
    on_horizontalSlider_sat_min_valueChanged(parking_mark_sat_min);
    on_horizontalSlider_sat_max_valueChanged(parking_mark_sat_max);
    on_horizontalSlider_val_min_valueChanged(parking_mark_val_min);
    on_horizontalSlider_val_max_valueChanged(parking_mark_val_max);

    ui.horizontalSlider_row_hue_min->setValue(parking_mark_hue_min);
    ui.horizontalSlider_row_hue_max->setValue(parking_mark_hue_max);
    ui.horizontalSlider_high_hue_min->setValue(parking_mark_hue_min);
    ui.horizontalSlider_high_hue_max->setValue(parking_mark_hue_max);
    ui.horizontalSlider_sat_min->setValue(parking_mark_sat_min);
    ui.horizontalSlider_sat_max->setValue(parking_mark_sat_max);
    ui.horizontalSlider_val_min->setValue(parking_mark_val_min);
    ui.horizontalSlider_val_max->setValue(parking_mark_val_max);
  }
}

void MainWindow::on_checkBox_tunnel_mark_toggled(bool checked)
{
  get_parameter();

  is_tunnel_mark = checked;
  if(is_tunnel_mark == true)
  {
    is_in_line = false;
    is_out_line = false;
    is_gate_bar = false;
    is_traffic_red_light = false;
    is_traffic_yellow_light = false;
    is_parking_mark = false;

    on_horizontalSlider_row_hue_min_valueChanged(tunnel_mark_row_hue_min);
    on_horizontalSlider_row_hue_max_valueChanged(tunnel_mark_row_hue_max);
    on_horizontalSlider_high_hue_min_valueChanged(tunnel_mark_high_hue_min);
    on_horizontalSlider_high_hue_max_valueChanged(tunnel_mark_high_hue_max);
    on_horizontalSlider_sat_min_valueChanged(tunnel_mark_sat_min);
    on_horizontalSlider_sat_max_valueChanged(tunnel_mark_sat_max);
    on_horizontalSlider_val_min_valueChanged(tunnel_mark_val_min);
    on_horizontalSlider_val_max_valueChanged(tunnel_mark_val_max);

    ui.horizontalSlider_row_hue_min->setValue(tunnel_mark_row_hue_min);
    ui.horizontalSlider_row_hue_max->setValue(tunnel_mark_row_hue_max);
    ui.horizontalSlider_high_hue_min->setValue(tunnel_mark_high_hue_min);
    ui.horizontalSlider_high_hue_max->setValue(tunnel_mark_high_hue_max);
    ui.horizontalSlider_sat_min->setValue(tunnel_mark_sat_min);
    ui.horizontalSlider_sat_max->setValue(tunnel_mark_sat_max);
    ui.horizontalSlider_val_min->setValue(tunnel_mark_val_min);
    ui.horizontalSlider_val_max->setValue(tunnel_mark_val_max);
  }
}

void MainWindow::get_parameter()
{
  ifstream fin_default_color("/home/turtle1/catkin_ws/src/turtlevision/parameter_file/default_color.txt");

  if(fin_default_color.is_open())
  {
    fin_default_color >> in_line_hue_min
                      >> in_line_sat_min
                      >> in_line_hue_max
                      >> in_line_sat_max
                      >> in_line_val_min
                      >> in_line_val_max

                      >> out_line_hue_min
                      >> out_line_sat_min
                      >> out_line_hue_max
                      >> out_line_sat_max
                      >> out_line_val_min
                      >> out_line_val_max

                      >> gate_bar_row_hue_min
                      >> gate_bar_sat_min
                      >> gate_bar_row_hue_max
                      >> gate_bar_sat_max
                      >> gate_bar_high_hue_min
                      >> gate_bar_high_hue_max
                      >> gate_bar_val_min
                      >> gate_bar_val_max

                      >> traffic_light_row_red_hue_min
                      >> traffic_light_red_sat_min
                      >> traffic_light_row_red_hue_max
                      >> traffic_light_red_sat_max
                      >> traffic_light_high_red_hue_min
                      >> traffic_light_high_red_hue_max
                      >> traffic_light_red_val_min
                      >> traffic_light_red_val_max

                      >> parking_mark_hue_min
                      >> parking_mark_sat_min
                      >> parking_mark_hue_max
                      >> parking_mark_sat_max
                      >> parking_mark_val_min
                      >> parking_mark_val_max

                      >> tunnel_mark_row_hue_min
                      >> tunnel_mark_sat_min
                      >> tunnel_mark_row_hue_max
                      >> tunnel_mark_sat_max
                      >> tunnel_mark_high_hue_min
                      >> tunnel_mark_high_hue_max
                      >> tunnel_mark_val_min
                      >> tunnel_mark_val_max


                      >> traffic_light_yellow_hue_min
                      >> traffic_light_yellow_sat_min
                      >> traffic_light_yellow_hue_max
                      >> traffic_light_yellow_sat_max
                      >> traffic_light_yellow_val_min
                      >> traffic_light_yellow_val_max;

      fin_default_color.close();
  }

  else
  {
      cout << " default_color.txt is close " << endl;
  }
}

void MainWindow::on_pushButton_run_clicked()
{
  if(mode == RUN) mode = STOP;
  else if(mode == STOP) mode = RUN;
}

MainWindow::~MainWindow()
{
  qnode.~QNode();
  delete &ui;
}

}
