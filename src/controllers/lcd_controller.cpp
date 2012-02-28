
#include <controllers/lcd_controller.h>
#include "ros/ros.h"
#include "qbo_arduqbo/LCD.h"
#include <ros/console.h>

CLCDController::CLCDController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_lcd"));
    lcd_sub_ = nh.subscribe<qbo_arduqbo::LCD>(topic, 1, &CLCDController::setLCD,this);
    //qbo_arduqbo::LCD osloaded_msg;
    //osloaded_msg.msg="QBO ready to accept orders";
    //setLCD(&osloaded_msg);
    std::string hello;
    hello.push_back(12);
    hello+="QBO ready to accept orders";
    //device_p_->setLCD("QBO ready to accept orders");
    device_p_->setLCD(hello);
}

void CLCDController::setLCD(const qbo_arduqbo::LCD::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("LCD comand arrived: " << msg->msg);
    int code=device_p_->setLCD(msg->msg);
    if (code<0)
        ROS_ERROR_STREAM("Unable to send string \"" << msg->msg << "\" to the base control board");
    else
        ROS_DEBUG_STREAM("Sent string " << msg->msg << " to the base control board ");
}
