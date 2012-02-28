
#include <controllers/battery_controller.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "qbo_arduqbo/BatteryLevel.h"

CBatteryController::CBatteryController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    level_=0;
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("battery_state"));
    nh.param("controllers/"+name+"/rate", rate_, 15.0);
    battery_pub_ = nh.advertise<qbo_arduqbo::BatteryLevel>(topic, 1);
    timer_=nh.createTimer(ros::Duration(1/rate_),&CBatteryController::timerCallback,this);
}

void CBatteryController::timerCallback(const ros::TimerEvent& e)
{
    int code=device_p_->getBattery(level_);
    if (code<0)
        ROS_ERROR("Unable to get battery level from the base control board");
    else
    {
        ROS_DEBUG_STREAM("Obtained battery level " << level_ << " from the base control board ");
        qbo_arduqbo::BatteryLevel msg;
        msg.level=level_;
        msg.header.stamp = ros::Time::now();
        //publish
        battery_pub_.publish(msg);
    }
}
