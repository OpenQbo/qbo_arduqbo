#ifndef NOSE_CONTROLLER_H
#define NOSE_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include "ros/ros.h"
#include "qbo_arduqbo/Nose.h"
#include <ros/console.h>

class CNoseController : public CController
{
    public:
        CNoseController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh);
        
    protected:
        ros::Subscriber nose_sub_;
        void setNose(const qbo_arduqbo::Nose::ConstPtr& msg);
};

#endif
