
#include <controllers/nose_controller.h>
#include "ros/ros.h"
#include "qbo_arduqbo/Nose.h"
#include <ros/console.h>

CNoseController::CNoseController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_nose"));
    nose_sub_ = nh.subscribe<qbo_arduqbo::Nose>(topic, 1, &CNoseController::setNose,this);
}

void CNoseController::setNose(const qbo_arduqbo::Nose::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Nose comand arrived: " << msg->color );
    int code=device_p_->setNose((uint8_t)(msg->color));
    //int code=device_p_->setNose(0);
    if (code<0)
        ROS_ERROR("Unable to send nose to the head control board");
    else
        ROS_DEBUG_STREAM("Sent nose " << msg->color << " to the head board ");
}
