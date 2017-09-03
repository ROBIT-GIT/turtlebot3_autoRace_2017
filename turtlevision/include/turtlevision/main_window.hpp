/**
 * @file /include/turtlevision/main_window.hpp
 *
 * @brief Qt based gui for turtlevision.
 *
 * @date November 2010
 **/
#ifndef turtlevision_MAIN_WINDOW_H
#define turtlevision_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN

#include <QtGui>
#include <QtGui/QMainWindow>
#include <iostream>
#include <fstream>
#include <QMainWindow>
#include <QMessageBox>
#include <math.h>
#include <stdio.h>


#include "ui_main_window.h"
#include "qnode.hpp"
#include "robitvision.hpp"
#include "cbloblabelingrobit.h"

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/core.hpp"


#endif
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace turtlevision {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:


         MainWindow(int argc, char** argv, QWidget *parent = 0);
        ~MainWindow();


public Q_SLOTS:


        // update image slot
        void updateImg(cv::Mat img, int num);

        // color parameter ui slot
        void on_horizontalSlider_row_hue_min_valueChanged(int value);
        void on_horizontalSlider_row_hue_max_valueChanged(int value);
        void on_horizontalSlider_high_hue_min_valueChanged(int value);
        void on_horizontalSlider_high_hue_max_valueChanged(int value);
        void on_horizontalSlider_sat_min_valueChanged(int value);
        void on_horizontalSlider_sat_max_valueChanged(int value);
        void on_horizontalSlider_val_min_valueChanged(int value);
        void on_horizontalSlider_val_max_valueChanged(int value);

        void on_checkBox_in_line_toggled(bool checked);
        void on_checkBox_out_line_toggled(bool checked);
        void on_checkBox_gate_bar_toggled(bool checked);
        void on_checkBox_traffic_light_red_toggled(bool checked);
        void on_checkBox_traffic_light_yellow_toggled(bool checked);
        void on_checkBox_parking_mark_toggled(bool checked);
        void on_checkBox_tunnel_mark_toggled(bool checked);

        void on_pushButton_save_parameter_clicked();
        void on_pushButton_run_clicked();

private:
        inline void initImageProcessing();
        void get_parameter();

private:
        Ui::MainWindowDesign ui;
        QNode qnode;

        enum{GATE_BAR_MARK = 1, TRAFFIC_MARK_RED, TRAFFIC_MARK_YELLOW, PARKING_MARK , TUNNEL_MARK};
        enum{STOP, RUN};

        int mode;


        // in line parameter
        int in_line_hue_min;
        int in_line_sat_min;

        int in_line_hue_max;
        int in_line_sat_max;

        int in_line_val_min;
        int in_line_val_max;


        // out line parameter
        int out_line_hue_min;
        int out_line_sat_min;

        int out_line_hue_max;
        int out_line_sat_max;

        int out_line_val_min;
        int out_line_val_max;

        // gate_bar parameter
        int gate_bar_row_hue_min;
        int gate_bar_row_hue_max;

        int gate_bar_high_hue_min;
        int gate_bar_high_hue_max;

        int gate_bar_sat_min;
        int gate_bar_sat_max;

        int gate_bar_val_min;
        int gate_bar_val_max;


        // traffic light parameter
        int traffic_light_row_red_hue_min;
        int traffic_light_row_red_hue_max;

        int traffic_light_high_red_hue_min;
        int traffic_light_high_red_hue_max;

        int traffic_light_red_sat_max;
        int traffic_light_red_sat_min;

        int traffic_light_red_val_min;
        int traffic_light_red_val_max;

        int traffic_light_yellow_hue_min;
        int traffic_light_yellow_hue_max;

        int traffic_light_yellow_sat_max;
        int traffic_light_yellow_sat_min;

        int traffic_light_yellow_val_min;
        int traffic_light_yellow_val_max;

        // parking mark parameter
        int parking_mark_hue_min;
        int parking_mark_sat_min;

        int parking_mark_hue_max;
        int parking_mark_sat_max;

        int parking_mark_val_min;
        int parking_mark_val_max;


        // tunnel mark parameter
        int tunnel_mark_row_hue_min;
        int tunnel_mark_row_hue_max;

        int tunnel_mark_high_hue_min;
        int tunnel_mark_high_hue_max;

        int tunnel_mark_sat_max;
        int tunnel_mark_sat_min;

        int tunnel_mark_val_max;
        int tunnel_mark_val_min;


        int line_position[2];


        int detection_flag;


        bool is_in_line;
        bool is_out_line;
        bool is_gate_bar;
        bool is_traffic_red_light;
        bool is_traffic_yellow_light;
        bool is_parking_mark;
        bool is_tunnel_mark;


        int check_data_count;


        struct timeval start_time, end_time;

};

}  // namespace turtlevision

#endif // turtlevision_MAIN_WINDOW_H
