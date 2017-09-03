#include "../include/turtlevision/cbloblabelingrobit.h"

enum{UP,DOWN,LEFT,RIGHT};

CBlobLabelingRobit::CBlobLabelingRobit()
    :m_nThreshold(0),
     m_width(0),
     m_height(0),
     m_nBlobs(0),
     m_num(0)
{
}

CBlobLabelingRobit::CBlobLabelingRobit(const Mat& Img, const unsigned int nThreshold)
    :m_nThreshold(nThreshold),
     m_width(Img.cols),
     m_height(Img.rows),
     m_nBlobs(0),
     m_num(0)
{
    m_nCount=0;
    m_Img			= Img.clone();
    m_ImgData       = (unsigned char *)m_Img.data;
}

CBlobLabelingRobit::~CBlobLabelingRobit()
{
    m_Img.release();
}

void CBlobLabelingRobit::setParam(const cv::Mat& Img, const unsigned int nThreshold)
{
    m_Img			= Img.clone();
    m_ImgData       = (unsigned char *)m_Img.data;

    m_width         = Img.cols;
    m_height        = Img.rows;

    m_nThreshold	= nThreshold;
    m_nBlobs        = 0;
    m_num           = 0;
}


void CBlobLabelingRobit::doLabeling()
{
    if( m_Img.channels() != 1 ) 	return ;
    _labeling();
}

void CBlobLabelingRobit::_labeling()
{
    m_isChecked = new bool [m_Img.size().area()];
    memset(m_isChecked, 0, (m_Img.size().area())*sizeof(bool));

    Pt_visited = new Point [m_Img.size().area()];
    memset(Pt_visited, 0, (m_Img.size().area())*sizeof(Point));


    for(int nY = 0,posY=0; nY < m_height; nY++,posY+=m_width)
    {
        for(int nX = 0; nX < m_width; nX++)
        {
            if(m_isChecked[posY+nX]==true)continue;
            if(m_ImgData[posY + nX]==0)continue;

            m_ImgData[posY + nX] = ++m_num;
            m_isChecked[posY + nX] = true;

            unsigned int StartX = nX, StartY = nY, EndX = nX, EndY= nY;

            if(_check_Four_Neighbor( StartX, StartY, EndX, EndY) >m_nThreshold)
            {
                m_recBlobs.push_back(Rect(StartX,StartY,EndX-StartX,EndY-StartY));
            }
            else
            {
                for(int y=StartY,idxY=StartY*m_width;y<=EndY;y++,idxY+=m_width)
                {
                    for(int x=StartX;x<=EndX;x++)
                    {
                        if(m_ImgData[x+idxY]==m_num)m_ImgData[x+idxY]=0;
                    }
                }
                m_ImgData[posY + nX]=0;
                m_num--;
            }
        }
    }
    m_nBlobs=m_num;
    delete[] m_isChecked; m_isChecked=NULL;
    delete[] Pt_visited; m_isChecked=NULL;
}


const unsigned int CBlobLabelingRobit::_check_Four_Neighbor(unsigned int &StartX, unsigned int &StartY, unsigned int &EndX, unsigned int &EndY)
{
    Point Pt_present(StartX,StartY);

    unsigned int nArea=1;

    const int ngbMove[4]={-m_width,m_width,-1,1};

    for(;;)
    {
        unsigned int Idx_present=(Pt_present.y * m_width) +  Pt_present.x;

        if(Pt_present.x!=0&&m_isChecked[Idx_present+ngbMove[LEFT]]==false)
        {
            m_isChecked[Idx_present+ngbMove[LEFT]]=true;

            if(m_ImgData[Idx_present+ngbMove[LEFT]]!=0)
            {
                m_ImgData[Idx_present+ngbMove[LEFT]]= m_ImgData[Idx_present];
                Pt_visited[Idx_present+ngbMove[LEFT]]= Pt_present;
                Pt_present.x-=1;
                if((int)StartX > Pt_present.x)       StartX = Pt_present.x;

                nArea++;
                continue;
            }
        }

        if(Pt_present.x!=(int)m_width-1&&m_isChecked[Idx_present+ngbMove[RIGHT]]==false)
        {
            m_isChecked[Idx_present+ngbMove[RIGHT]]=true;

            if(m_ImgData[Idx_present+ngbMove[RIGHT]]!=0)
            {
                m_ImgData[Idx_present+ngbMove[RIGHT]]= m_ImgData[Idx_present];
                Pt_visited[Idx_present+ngbMove[RIGHT]]= Pt_present;
                Pt_present.x+=1;
                if((int)EndX < Pt_present.x)    EndX   = Pt_present.x;

                nArea++;
                continue;
            }
        }

        if(Pt_present.y!=0&&m_isChecked[Idx_present+ngbMove[UP]]==false)
        {
            m_isChecked[Idx_present+ngbMove[UP]]=true;

            if(m_ImgData[Idx_present+ngbMove[UP]]!=0)
            {
                m_ImgData[Idx_present+ngbMove[UP]]= m_ImgData[Idx_present];
                Pt_visited[Idx_present+ngbMove[UP]]= Pt_present;
                Pt_present.y-=1;
                if((int)StartY > Pt_present.y)       StartY = Pt_present.y;

                nArea++;
                continue;
            }
        }

        if(Pt_present.y!=(int)m_height-1&&m_isChecked[Idx_present+ngbMove[DOWN]]==false)
        {
            m_isChecked[Idx_present+ngbMove[DOWN]]=true;

            if(m_ImgData[Idx_present+ngbMove[DOWN]]!=0)
            {
                m_ImgData[Idx_present+ngbMove[DOWN]]= m_ImgData[Idx_present];
                Pt_visited[Idx_present+ngbMove[DOWN]]= Pt_present;
                Pt_present.y+=1;
                if((int)EndY < Pt_present.y)    EndY   = Pt_present.y;

                nArea++;
                continue;
            }
        }

        if(Pt_visited[Idx_present]==Pt_present)break;
        Pt_present=Pt_visited[Idx_present];
    }
    return nArea;
}
