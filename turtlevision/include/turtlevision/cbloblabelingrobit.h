#ifndef CBLOBLABELINGROBIT_H
#define CBLOBLABELINGROBIT_H

/*==============================================================================
 *    	Date           	: 2016.09.07
 *		Modified       	: 2016.09.08 By Kwon, Yonghye
 *		Author         	: Kwon, Yonghye  (ROBIT 10th, Kwangwoon University 2014)
 *      E-mail          : robotmanyh@naver.com
 *		NOTE : 8-Neighbor labeling
==============================================================================*/

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

class CBlobLabelingRobit
{
public:

        unsigned int            m_nBlobs;
        unsigned int            m_nCount;
        unsigned char*          m_ImgData;

        vector<Rect>            m_recBlobs;

        CBlobLabelingRobit();
        CBlobLabelingRobit(const Mat& Img,const unsigned int nThreshold);

        ~CBlobLabelingRobit();

        void            setParam(const Mat &Img, const unsigned int nThreshold);
        void            doLabeling();

private:
        Mat            m_Img;
        bool*          m_isChecked;
        Point *        Pt_visited;

        unsigned int m_num;
        unsigned int m_nThreshold;

        int m_width;
        int m_height;

        void _labeling();
        const unsigned int _check_Four_Neighbor(unsigned int &StartX, unsigned int &StartY, unsigned int &EndX, unsigned int &EndY);
};
#endif // CBLOBLABELINGROBIT_H
