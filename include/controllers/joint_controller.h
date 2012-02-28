#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <map>
#include <ros/console.h>
#include <servos.h>

class CJointController : public CController
{
    public:
        
        CJointController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh, std::map<std::string,CServo *>& servos);
        //sensor_msgs::JointState joint_state;
    
    protected:
        
        ros::Subscriber joint_sub_;
        sensor_msgs::JointState joint_msg_;
        std::map<std::string,CServo *> *servos_p_;
        
        
        // internal data
        bool joints_dirty_;
        
        void timerCallback(const ros::TimerEvent& e);
        void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
};
        
#endif
