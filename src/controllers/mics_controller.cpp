
#include <controllers/mics_controller.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "qbo_arduqbo/NoiseLevels.h"
#include "qbo_arduqbo/Mic.h"

CMicsController::CMicsController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    mic_=0;
    m0_=0;
    m1_=0;
    m2_=0;
    std::string topic,mics_topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_mics"));
    nh.param("controllers/"+name+"/mics_topic", mics_topic, std::string("mics_state"));
    nh.param("controllers/"+name+"/rate", rate_, 1.0);
    mics_sub_ = nh.subscribe<qbo_arduqbo::Mic>(topic, 1, &CMicsController::micsCallback, this);
    mics_pub_ = nh.advertise<qbo_arduqbo::NoiseLevels>(mics_topic, 1);
    timer_=nh.createTimer(ros::Duration(1/rate_),&CMicsController::timerCallback,this);
}

void CMicsController::micsCallback(const qbo_arduqbo::Mic::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Set mic comand arrived: " << (int)msg->mic);
    int code=device_p_->setMic(msg->mic);
    if (code<0)
        ROS_ERROR("Unable to set mic input at the head control board");
    else
    {
        ROS_DEBUG_STREAM("Mic set to the input " << (int)msg->mic << " at the head control board");
        mic_=msg->mic;
    }
}

void CMicsController::timerCallback(const ros::TimerEvent& e)
{
    int code=device_p_->getMics(m0_,m1_,m2_);
    if (code<0)
        ROS_ERROR("Unable to get mics levels from the head control board");
    else
    {
        ROS_DEBUG_STREAM("Obtained mics levels (" << (int)m0_ << "," << (int)m1_ << "," << (int)m2_ << ") from the head control board ");
        qbo_arduqbo::NoiseLevels msg;
        msg.m0=m0_;
        msg.m1=m1_;
        msg.m2=m2_;
        msg.header.stamp = ros::Time::now();
        //publish
        mics_pub_.publish(msg);
    }
}
