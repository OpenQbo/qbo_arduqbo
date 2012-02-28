
#include <controllers/mouth_controller.h>
#include "ros/ros.h"
#include "qbo_arduqbo/Mouth.h"
#include <ros/console.h>

CMouthController::CMouthController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_mouth"));
    mouth_sub_ = nh.subscribe<qbo_arduqbo::Mouth>(topic, 1, &CMouthController::setMouth,this);
}

void CMouthController::setMouth(const qbo_arduqbo::Mouth::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Mouth comand arrived: " << msg->mouthImage[0] << "," << msg->mouthImage[1] << "," << msg->mouthImage[2] << "," << msg->mouthColor);
    int code=device_p_->setMouth((uint8_t)(msg->mouthImage[0]), (uint8_t)(msg->mouthImage[1]), (uint8_t)(msg->mouthImage[2]),(uint8_t)(msg->mouthColor));
    if (code<0)
        ROS_ERROR("Unable to send mouth to the head control board");
    else
        ROS_DEBUG_STREAM("Sent mouth " << msg->mouthImage[0] << "," << msg->mouthImage[1] << "," << msg->mouthImage[2] << "," << msg->mouthColor << " to the head board ");
}
