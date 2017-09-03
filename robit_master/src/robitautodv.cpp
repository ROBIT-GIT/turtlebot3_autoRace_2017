#include "../include/robit_master/robitautodv.h"

using namespace std;


RobitAutoDV::RobitAutoDV()
{
  driving_mode        = 0;
  parking_mode        = 0;
  notice_parking      = false;
  notice_tunnel       = false;
  parking_flag        = false;
  is_traffic_success  = false;
}

void RobitAutoDV::setMsgData(const turtlevision::turtlevision_msg::ConstPtr& msg)
{
  // set message data
  degree_l               = msg->degree_left;
  degree_r               = msg->degree_right;
  in_line_mpt_x          = msg->in_line_mpt_x;
  out_line_mpt_x         = msg->out_line_mpt_x;
  notice_information     = msg->notice_information;
  parking_mark_labeling  = msg->parking_mark_labeling;

  if(in_line_mpt_x > 160 || degree_l > 90.0)
    degree_l = 0.0;
  if(out_line_mpt_x < 160 || degree_r < 90.0)
    degree_r = 0.0;
  if(notice_information == PARKING_MARK && number > 210)    parking_flag = true;
}

void RobitAutoDV::lineTracing()
{
  // line tracing
  if(notice_information != NOTHING)
  {
    if(notice_information == GATE_BAR_MARK)
      setVel(0.0, 0.0);

    else if(notice_information == TRAFFIC_MARK_YELLOW)
    {
      is_traffic_mark = YELLOW;
      setVel(0.02, 0.0);
    }
    else if(notice_information == TRAFFIC_MARK_RED)
    {
      is_traffic_mark = RED;
      setVel(0.0, 0.0);
    }
    else if(notice_information == PARKING_MARK)
    {
      notice_parking = true;
        if(parking_flag == true)
          number = 0;
    }

    else if(notice_information == TUNNEL_MARK)
    {

    }
  }

  else if(notice_information == NOTHING && notice_parking     == false)
  {
    double out_line_nowX = out_line_mpt_x;
    double out_line_pos = 320 - out_line_nowX;
    double in_line_nowX = in_line_mpt_x;
    double in_line_pos = in_line_nowX;

    if(is_traffic_mark == YELLOW)
    {
      setVel(0.02, 0.0);
    }
    else if(degree_l !=0 && degree_r != 0)
    {
      if(out_line_pos > 10)
        setVel(0.2, 0.3);
      else if(in_line_pos > 10)
        setVel(0.2, -0.3);
      else if(degree_l < 40.0 && degree_r < 40.0)
        setVel(0.18, 0.6);
      else
        setVel(1.0, 0.0);
    }
    else if (degree_l == 0 && degree_r != 0 )
    {
      if(0 <= out_line_pos && out_line_pos < 18 && degree_r < 140.0)
       setVel(1.0, 0.0);
      else if(0 <= out_line_pos && out_line_pos < 18 && degree_r >= 140.0 )
       setVel(0.2, (out_line_pos * GAIN));
      else if(18 <= out_line_pos && out_line_pos < 40)
       setVel(0.19, (out_line_pos * GAIN ));
      else if(40 <= out_line_pos && out_line_pos < 70)
       setVel(0.19, (out_line_pos * GAIN ) * (1.1));
      else
       setVel(0.18, (out_line_pos * GAIN * (1.2)));
    }

    else if (degree_l != 0 && degree_r == 0)
    {
      if(0 <= in_line_pos && in_line_pos < 18 && degree_l < 60.0)
       setVel(1.0, 0.0);
      else if(0 <= in_line_pos && in_line_pos < 18 && degree_l >= 60.0)
       setVel(0.2, (in_line_pos * GAIN * (-1)));
      else if(18 <= in_line_pos && in_line_pos < 40)
       setVel(0.19, (in_line_pos * GAIN * (-1)));
      else if(40 <= in_line_pos && in_line_pos < 70)
       setVel(0.19, (in_line_pos * GAIN * (-1) * (1.1)));

      else
       setVel(0.18, (in_line_pos * GAIN * (-1) * (1.2)));
    }

    else
    {
       setVel(0.1) ;
    }
  }

  else if(notice_parking == true)
  {
    if(parking_count < 90)
    {
      double out_line_nowX = out_line_mpt_x;
      double out_line_pos = 320 - out_line_nowX;
      double in_line_nowX = in_line_mpt_x;
      double in_line_pos = in_line_nowX;
      if(degree_l !=0 && degree_r != 0)
      {
        if(out_line_pos > 10)
          setVel(0.2, 0.3);
        else if(in_line_pos > 10)
          setVel(0.2, -0.3);
        else if(degree_l < 40.0 && degree_r < 40.0)
          setVel(0.18, 0.6);
        else
          setVel(1.0, 0.0);
      }
      else if (degree_l == 0 && degree_r != 0 )
      {
        if(0 <= out_line_pos && out_line_pos < 18 && degree_r < 140.0)
         setVel(1.0, 0.0);
        else if(0 <= out_line_pos && out_line_pos < 18 && degree_r >= 140.0 )
         setVel(0.2, (out_line_pos * GAIN));
        else if(18 <= out_line_pos && out_line_pos < 40)
         setVel(0.19, (out_line_pos * GAIN ));
        else if(40 <= out_line_pos && out_line_pos < 70)
         setVel(0.19, (out_line_pos * GAIN ) * (1.1));
        else
         setVel(0.18, (out_line_pos * GAIN * (1.2)));
      }

      else if (degree_l != 0 && degree_r == 0)
      {
        if(0 <= in_line_pos && in_line_pos < 18 && degree_l < 60.0)
         setVel(1.0, 0.0);
        else if(0 <= in_line_pos && in_line_pos < 18 && degree_l >= 60.0)
         setVel(0.2, (in_line_pos * GAIN * (-1)));
        else if(18 <= in_line_pos && in_line_pos < 40)
         setVel(0.19, (in_line_pos * GAIN * (-1)));
        else if(40 <= in_line_pos && in_line_pos < 70)
         setVel(0.19, (in_line_pos * GAIN * (-1) * (1.1)));
        else
         setVel(0.18, (in_line_pos * GAIN * (-1) * (1.2)));
      }

      else
      {
        setVel(0.1) ;
      }
    }

    else if(parking_count >= 75 && parking_count <= 356 && number == 0)
    {
      if(degree_l !=0)
      {
         setVel(0.15, -0.3);
      }

      else if (degree_l == 0)
      {
        setVel(0.15, 0.4);
      }
    }

    parking_flag = false;

     if(parking_count > 150 && parking_mode == PARKING_FRONT_MODE)
     {
       turtlebot_parking(number);
     }

     else if(parking_count > 210 && parking_mode == PARKING_REAR_MODE)
     {
       turtlebot_parking(number);
     }
   }
}


void RobitAutoDV::setData(const double &data_l, const double &data_r)
{
  // set degree data
  degree_l = data_l;
  degree_r = data_r;
}

void RobitAutoDV::turtlebot_parking(int number)
{
  // parking mode
  if(number < 30)
  {
    setVel(0.0, -1.0);
  }
  else if(number >= 30 && number<60)
  {
    setVel(1.0, 0.0);
  }
  else if(number >= 60 && number < 90)
  {
    setVel(0.0, 1.0);
  }
  else if(number >= 90 && number < 130)
  {
    setVel(0.0, 0.0);
  }
  else if(number >= 130 && number < 160)
  {
    setVel(0.0, 1.0);
  }
  else if(number >= 160 && number < 190)
  {
    setVel(1.0, 0.0);
  }
  else if(number >= 190 && number < 210)
  {
    setVel(0.0, -1.0);
  }
  else if(number >= 210)
  {
    number = 0;
    notice_parking = false;
    parking_flag = true;
    parking_count = 0;
  }
}
