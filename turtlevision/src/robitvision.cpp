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

#include "../include/turtlevision/robitvision.hpp"

inline double SQR(double num)
{
    return (num*num);
}

void RV::rangeFilter(const Mat& input,const Scalar& min,const Scalar& max,Mat& output)
{
    register const unsigned int IMGSIZE=input.size().area();
    if(output.empty()==true)output=Mat::zeros(input.size(),CV_8UC1);

    register const unsigned char A=min[0],B=min[1],C=min[2]
                                ,D=max[0],E=max[1],F=max[2];
    register const unsigned char *data=input.data;
    register unsigned char *outputdata=output.data;
    register unsigned int index=0;

    for(register unsigned int i=0; i<IMGSIZE; i++)
    {
        index=i*3;
        if((data[index]>=A)&&(data[index]<=D))
            if((data[++index]>=B)&&(data[index]<=E))
                if((data[++index]>=C)&&(data[index]<=F))
                {
                    outputdata[i]=255;
                    continue;
                }
        outputdata[i]=0;
    }
}

void RV::rgb2hsv(const Mat& data_rgb,Mat& data_hsv)
{
    if(data_hsv.empty()==true)data_hsv=Mat::zeros(data_rgb.size(),CV_8UC3);

    unsigned char* data=       (unsigned char*)data_rgb.data;
    unsigned char* outputdata= (unsigned char*)data_hsv.data;

    int IMGSIZE=data_rgb.size().area(),
        h=0,s=0,v=0,
         bgrbg[5]={0,}   ,
         theta[3]={120,60,0}   ,
            delta=0;
    do
    {
        bgrbg[0]=*data++;
        bgrbg[1]=*data++;
        bgrbg[2]=*data++;
        bgrbg[3]=bgrbg[0];
        bgrbg[4]=bgrbg[1];

        v=(bgrbg[0]>bgrbg[1])?0:1;
        v=(bgrbg[v]>bgrbg[2])?v:2;
        delta= bgrbg[v]-((bgrbg[v+1]<bgrbg[v+2])?bgrbg[v+1]:bgrbg[v+2]);

        h=s=0;
        if(delta)
        {
            h=(((bgrbg[v+2]-bgrbg[v+1])*30.0)/delta)+theta[v];
            h += (-180)*(h>>31);
            s =((delta*255)/bgrbg[v]);
        }
        *outputdata++= h;
        *outputdata++= s;
        *outputdata++= bgrbg[v];
    }
    while(--IMGSIZE);
}

void RV::DrawLine(Mat& img, const Point& PtA, const Point& PtB , const Scalar &color)
{
    unsigned char ucColor=color[0];
    unsigned char* imgData=(unsigned char *)img.data;

    const int dX = PtB.x-PtA.x,
              nX = abs(dX),
              dY = PtB.y-PtA.y,
              nY = abs(dY);

    const int sign_x = dX > 0? 1 : -1,
              sign_y = dY > 0? 1 : -1;


    int x=PtA.x,
        y=PtA.y;

    for (int ix = 0, iy = 0; (ix < nX || iy < nY);)
    {
        if ((0.5+ix) / nX == (0.5+iy) / nY)
        {
            // next step is diagonal
            x += sign_x;
            y += sign_y;
            ix++;
            iy++;
        }
        else if ((0.5+ix) / nX < (0.5+iy) / nY)
        {
            // next step is horizontal
            x += sign_x;
            ix++;
        }
        else
        {
            // next step is vertical
            y += sign_y;
            iy++;
        }

        if(y>=0&&y<img.rows)
        if(x>=0&&x<img.cols)
        if(imgData[y*img.cols+x]!=255)
        imgData[y*img.cols+x]=ucColor;
    }
}


void RobitKalmanFilter::measurementUpdate(){
    K = (P + Q) / (P + Q + R);
    P = ((R * (P + Q)) / (R + P + Q));
}

const double RobitKalmanFilter::update(double measurement)
{
   measurementUpdate();
   X = X + ((measurement - X) * K);
   return X;
}


RobitRansacLine::RobitRansacLine(const Mat& img)
{
    m_ImgData=(unsigned char *)img.data;
    m_width  =img.cols;
    m_height =img.rows;
    m_lineThreshold = 10;
    m_pos.clear();

    m_noSamples=true;
    m_degree=0.0;
}

void RobitRansacLine::getSamples()
{
    const unsigned int threshold_nSamples=(m_width+m_height)/4;
    Point Pt_temp;

    for(int i=0,idxY=0; i<m_height;i++,idxY+=m_width)
    {
        for(int j=0; j<m_width;j++)
        {
            if(m_ImgData[idxY+j]!=0)
            {
                Pt_temp.x = j;
                Pt_temp.y = i;

                m_pos.push_back(Pt_temp);
            }
        }
    }

    if(m_pos.size()>=threshold_nSamples)m_noSamples=false;
}

void RobitRansacLine::compute_model_parameter(const vector<Point>& vInlierPoint, sLine &model)
{
// PCA ¹æ½ÄÀ¸·Î Á÷¼± ¸ðµ¨ÀÇ ÆÄ¶ó¸ÞÅÍ¸¦ ¿¹ÃøÇÑ´Ù.
    double sx  = 0, sy  = 0;
    double sxx = 0, syy = 0;
    double sxy = 0;

    for(size_t i = 0; i<vInlierPoint.size();++i)
    {
        const double x = (double)vInlierPoint.at(i).x;
        const double y = (double)vInlierPoint.at(i).y;

        sx  += x;
        sy  += y;
        sxx += x*x;
        sxy += x*y;
        syy += y*y;
    }
    const double sw= vInlierPoint.size();

//variance;
    const double vxx = (sxx - sx*sx/sw)/sw;
    const double vxy = (sxy - sx*sy/sw)/sw;
    const double vyy = (syy - sy*sy/sw)/sw;

//principal axis
    const double theta = atan2(2.0*vxy, vxx - vyy)/2.0;

    model.mx = cos(theta);
    model.my = sin(theta);

//center of mass(xc, yc)
    model.sx = sx/sw;
    model.sy = sy/sw;
}

double RobitRansacLine::compute_distance(sLine &line, Point &x)const
{
    return fabs((double)((double)(x.x - line.sx)*line.my - ((double)x.y - line.sy)*line.mx))/sqrt(line.mx*line.mx + line.my*line.my);
}

void RobitRansacLine::runRansac()
{
    getSamples();
    if(m_noSamples==true) return;
    srand(time(NULL));

    unsigned int nSamples=0;

    nSamples=(m_pos.size()*4)/10;

    const double distance_threshold=1.0;
    double max_cost=0.0;

    sLine estimated_model;

    for(int i=0;i<nSamples;i++)
    {
        const int  n=rand()%m_pos.size();
        const int  m=rand()%m_pos.size();

        if(n==m)continue;
        if(norm(m_pos[n]-m_pos[m])<m_lineThreshold)continue;

        vector<Point> SamplePt;
        SamplePt.push_back(m_pos[n]);
        SamplePt.push_back(m_pos[m]);

        compute_model_parameter(SamplePt,estimated_model);

        double cost = 0.;

        const double sqrtmmyy=sqrt(estimated_model.mx*estimated_model.mx + estimated_model.my*estimated_model.my);
        const double mysx=estimated_model.sx*estimated_model.my;
        const double mxsy=estimated_model.sy*estimated_model.mx;
        const double sub_ms=mxsy-mysx;

        for(size_t i=0; i<m_pos.size(); i++)
        {
            const double distance = fabs((((double)m_pos[i].x*estimated_model.my)-((double)m_pos[i].y*estimated_model.mx))+sub_ms)/sqrtmmyy;
            if (distance < distance_threshold)
            {
                cost += 1.0;
            }
        }
        const double CostThresHold=norm(SamplePt[0]-SamplePt[1])/3.0;

        if(cost<CostThresHold)continue;

        if(cost>max_cost)
        {
            m_line_A= m_pos[n];
            m_line_B= m_pos[m];
            max_cost=cost;
        }
    }

    if(m_line_A.x<m_line_B.x)
    {
        Point tempP=m_line_A;
        m_line_A=m_line_B;
        m_line_B=tempP;
    }

    m_degree=-(atan2(m_line_A.y-m_line_B.y,m_line_A.x-m_line_B.x)*180.0)/CV_PI;
    if(m_degree<0.0)m_degree+=180.0;
}



RobitRansacCircle::RobitRansacCircle(const Mat& img)
{
    m_ImgData=(unsigned char *)img.data;
    m_width  =img.cols;
    m_height =img.rows;

    m_pos.clear();

    m_noSamples=true;

    m_center.x=0;
    m_center.y=0;

    m_radius=0.0;
}


void RobitRansacCircle::getSamples()
{
    Point Pt_temp;
    for(int i=0,idxY=0; i<m_height;i++,idxY+=m_width)
    {
        for(int j=0; j<m_width;j++)
        {
            if(m_ImgData[idxY+j]!=0)
            {
                Pt_temp.x = j;
                Pt_temp.y = i;

                m_pos.push_back(Pt_temp);
            }
        }
    }
    if(m_pos.size()>m_minRadius*CV_PI)m_noSamples=false;
}

bool RobitRansacCircle::getCircle(Point& p1,Point& p2,Point& p3, Point& center, double& radius)const
{
    const double A = p1.x - p2.x; //R :P3  P: P2  Q: P1
    const double B = p1.y - p2.y;
    const double C = p3.x - p2.x;
    const double D = p3.y - p2.y;
    const double E = A * ((double)(p2.x + p1.x)) + B * ((double)(p2.y + p1.y));
    const double F = C * ((double)(p2.x + p3.x)) + D * ((double)(p2.y + p3.y));

    const double G = 2.0 * ( (A * D )- (B * C));

    if (G != 0.) {
        center.x = (D * E - B * F) / G;
        center.y = (A * F - C * E) / G;

        radius = sqrt((double)((center.x-p1.x)*(center.x-p1.x) + (center.y-p1.y)*(center.y-p1.y)));
    }

    if(radius>m_maxRadius||radius<m_minRadius) return false;
    return true;
}

double RobitRansacCircle::verifyCircle(const Point& center, const double radius)const
{
    const double threshold_dist=2.0;

    int inlier = 0;

    for(size_t i=0; i<m_pos.size(); i++)
    {
        const double distance=norm(m_pos[i]-center);

        if (fabs(distance-radius) <= threshold_dist)inlier += 1;
    }

    const double cost= ((double)inlier)/((2.0*CV_PI)*radius);
    return cost;
}

void RobitRansacCircle::runRansac()
{
    getSamples();
    if(m_noSamples==true) return;
    srand(time(NULL));

    const int nSamples=m_pos.size()*1.0;
    const double minCirclePercentage = 0.55;
    const double min_point_separation = 1.0;
    double max_cPerc=0.0;

    Point centerP;

    for(int i=0;i<nSamples;i++)
    {
        const int  n=rand()%m_pos.size();
        const int  m=rand()%m_pos.size();
        const int  k=rand()%m_pos.size();

        if(n==m)continue;
        if(n==k)continue;
        if(k==m)continue;

        const double distm=norm(m_pos[n]-m_pos[m]);
        const double distn=norm(m_pos[m]-m_pos[k]);
        const double distk=norm(m_pos[k]-m_pos[n]);

        if(distm < min_point_separation ||
           distn < min_point_separation ||
           distk < min_point_separation)continue;

        double radius=0.0;

        if(getCircle(m_pos[n],m_pos[m],m_pos[k],centerP,radius)==false)continue;

        const double cPerc = verifyCircle(centerP,radius);

        if(cPerc >= minCirclePercentage)
        {
            if(cPerc>max_cPerc)
            {
                max_cPerc=cPerc;
                m_center=centerP;
                m_radius=radius;
            }
        }
    }
    m_cost=max_cPerc;
}






RansacLine::RansacLine(Mat & edgeMat)

    :m_edgeMat(edgeMat)
{
    srand((unsigned int)time(NULL));

}



void RansacLine::run()
{

    getEdgePoints();



    const unsigned int POINTS_NUMBERS = m_vecEdgePoints.size();

    const unsigned int ITER_NUMBERS = (POINTS_NUMBERS * 3) / 10;

    const unsigned int THRESHOLD = 14;



    const double MAX_SCORE = norm(Point(0,0)-Point(m_edgeMat.cols - 1, m_edgeMat.rows - 1));



    Point point[2];

    double max_line_score = 0.5;



    for (unsigned int i = 0; i < ITER_NUMBERS; i++)
    {

        unsigned int rand_idxA = rand() % POINTS_NUMBERS;
        unsigned int rand_idxB = rand() % POINTS_NUMBERS;



        point[0] = m_vecEdgePoints[rand_idxA];
        point[1] = m_vecEdgePoints[rand_idxB];


        if (abs(point[0].x - point[1].x) + abs(point[0].y - point[1].y) < THRESHOLD)
        {
            continue;
        }

        double maxScore = norm(point[0]-point[1]);
        if (maxScore <= THRESHOLD)continue;

        double score = getScore(point);// + (maxScore/ MAX_SCORE)*0.25;

        score += (maxScore/ MAX_SCORE)*0.25;
        if(score < maxScore/2.0)continue;




        if (score >= max_line_score)
        {
            max_line_score = score;


            m_point[0] = point[0];
            m_point[1] = point[1];
        }

    }

}



void RansacLine::getEdgePoints()
{

    unsigned char* val = m_edgeMat.data;
    Point point;



    for (int i = 1; i < m_edgeMat.rows - 1; i++)
        for (int j = 1; j < m_edgeMat.cols - 1; j++)
        {

            if (val[(i * m_edgeMat.cols) + j] != 0)
            {

                point.x = j;
                point.y = i;



                m_vecEdgePoints.push_back(point);
            }
        }
}



int RansacLine::getScore(Point point[2])
{

    const int DX = point[0].x - point[1].x,

        NX = abs(DX),

        DY = point[0].y - point[1].y,

        NY = abs(DY);



    const int SIGN_X = DX > 0 ? 1 : -1,

        SIGN_Y = DY > 0 ? 1 : -1;



    const int MASK_SIZE = 1;



    unsigned char* val = m_edgeMat.data;



    int x = point[0].x,

        y = point[0].y;



    int score = 0;



    Point point_old;

    Point point_cur;



    for (int ix = 0, iy = 0; (ix < NX || iy < NY);)
    {

        if ((0.5 + ix) / NX == (0.5 + iy) / NY)
        {

            // next step is diagonal

            x += SIGN_X;

            y += SIGN_Y;

            ix++;

            iy++;

        }

        else if ((0.5 + ix) / NX < (0.5 + iy) / NY)
        {

            // next step is horizontal

            x += SIGN_X;

            ix++;

        }

        else
        {

            // next step is vertical

            y += SIGN_Y;

            iy++;

        }



        bool b_pt_exist = false;



        for (int my = -MASK_SIZE; my <= MASK_SIZE && !b_pt_exist; my++)
        {

            point_cur.y = y - my;

            if (point_cur.y < 0 || point_cur.y >= m_edgeMat.rows)continue;



            for (int mx = -MASK_SIZE; mx <= MASK_SIZE && !b_pt_exist; mx++)
            {

                point_cur.x = x - mx;
                if (point_cur.x < 0 || point_cur.x >= m_edgeMat.cols)continue;





                if (val[(point_cur.y * m_edgeMat.cols) + point_cur.x] == 255)
                {
                    if (point_cur != point_old)
                    {
                        point_old = point_cur;


                        b_pt_exist = true;

                        score++;
                    }
                }
            }
        }

    }


    return score;
}
