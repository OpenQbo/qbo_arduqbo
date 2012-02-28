
#include <servos.h>
#include <cmath>
#include <algorithm>
#include <XmlRpcValue.h>
#include <ros/console.h>

CServo::CServo(std::string name, CQboduinoDriver *device_p, bool single)
{
    name_ = name;
    device_p_ = device_p;

    id_ = -1;
    neutral_ = 1500;
    ticks_ = 1800;
    max_angle_ = M_PI/2;               // limit angle, radians
    min_angle_ = -M_PI/2;
    range_=M_PI;
    rad_per_tick_ = range_/ticks_;
    max_speed_ = M_PI;                 // radians per second
    max_ticks_speed_ = (int)(max_speed_/rad_per_tick_); // ticks per second
    invert_ = false;

    angle_ = 0.0;                            // current position
}

float CServo::getAngle()
{
    return angle_;
}

float CServo::getAngleStored()
{
    return angle_;
}

std::string CServo::getName()
{
    return name_;
}

void CServo::setAngle(float ang, float velocity)
{
    if (ang > max_angle_ || ang < min_angle_)
    {
        ROS_WARN_STREAM("Servo " << name_ << ": angle out of range (" << ang << ").Limits are: " << min_angle_ << "," << max_angle_);
        //return;
        ang=std::min(ang,max_angle_);
        ang=std::max(ang,min_angle_);
    }
    if (velocity > max_speed_)
    {
        ROS_WARN_STREAM("Servo " << name_ << ": velocity out of range (" << velocity << ").Limit is: " << max_speed_);
    }
    angle_ = ang;    // store it for joint state updates
    if (invert_)
        ang = ang * -1.0;
    int ticks = (int)(round( ang / rad_per_tick_ ));
    int speed = (int)(round(velocity / rad_per_tick_));
    ticks += neutral_;
    int code=device_p_->setServo(id_, ticks, speed);
    if (code<0)
        ROS_ERROR_STREAM("Unable to send angle for the servo " << name_ << " to the head control board " << code);
    else
        ROS_DEBUG_STREAM("Sent angle " << angle_ << " for the servo " << name_ << " to the head control board");
}

void CServo::setParams(XmlRpc::XmlRpcValue params)
{
    if(params.hasMember("id"))
    {
      id_=params["id"];
    }
    if(params.hasMember("max_angle_degrees"))
    {
      max_angle_=(double)radians(params["max_angle_degrees"]);
    }
    if(params.hasMember("min_angle_degrees"))
    {
      min_angle_=(double)radians(params["min_angle_degrees"]);
    }
    if(params.hasMember("max_angle_radians"))
    {
      max_angle_=(double)(params["max_angle_radians"]);
    }
    if(params.hasMember("min_angle_radians"))
    {
      min_angle_=(double)(params["min_angle_radians"]);
    }
    if(params.hasMember("max_speed"))
    {
      max_speed_=(double)params["max_speed"];
    }
    if(params.hasMember("range"))
    {
      range_=(double)radians(params["range"]);
    }
    if(params.hasMember("ticks"))
    {
      ticks_=params["ticks"];
    }
    if(params.hasMember("neutral"))
    {
      neutral_=params["neutral"];
    }
    if(params.hasMember("invert"))
    {
      invert_=params["invert"];
    }
    recalculateAngleParams();
}

void CServo::recalculateAngleParams()
{
    max_angle_=std::min(max_angle_,range_/2);
    min_angle_=std::max(min_angle_,-range_/2);
    rad_per_tick_ = range_/ticks_;
    max_ticks_speed_ = (int)(max_speed_/rad_per_tick_); // ticks per second
}

float ControledServo::getAngle()
{
    unsigned short pos;
    int code=device_p_->getServoPosition(id_,pos);
    if(code>=0)
    {
        float angle = ((int)pos - neutral_) * rad_per_tick_;
        if (invert_)
            angle = angle * -1.0;
        angle_ = angle;
        ROS_DEBUG_STREAM("Recibed angle " << angle_ << " for servo " << name_ << " from the head board");
        return angle_;
    }
    ROS_ERROR_STREAM("Unable to get angle for servo " << name_ << " from head board " << code);
    return -1000;
}
