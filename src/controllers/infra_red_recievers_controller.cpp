
#include <controllers/infra_red_recievers_controller.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "qbo_arduqbo/Irs.h"

CInfraRedsController::CInfraRedsController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    ir0_=0;
    ir1_=0;
    ir2_=0;
    std::string topic,irs_topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_mics"));
    nh.param("controllers/"+name+"/irs_topic", irs_topic, std::string("irs_state"));
    nh.param("controllers/"+name+"/rate", rate_, 1.0);
    irs_pub_ = nh.advertise<qbo_arduqbo::Irs>(irs_topic, 1);
    timer_=nh.createTimer(ros::Duration(1/rate_),&CInfraRedsController::timerCallback,this);
}

void CInfraRedsController::timerCallback(const ros::TimerEvent& e)
{
    int code=device_p_->getIRs(ir0_,ir1_,ir2_);
    if (code<0)
        ROS_ERROR("Unable to get irs values from the base control board");
    else
    {
        ROS_DEBUG_STREAM("Obtained irs values (" << (int)ir0_ << "," << (int)ir1_ << "," << (int)ir2_ << ") from the base control board ");
        qbo_arduqbo::Irs msg;
        msg.ir0=ir0_;
        msg.ir1=ir1_;
        msg.ir2=ir2_;
        msg.header.stamp = ros::Time::now();
        //publish
        irs_pub_.publish(msg);
    }
}
