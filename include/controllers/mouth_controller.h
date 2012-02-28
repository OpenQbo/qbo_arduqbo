#ifndef MOUTH_CONTROLLER_H
#define MOUTH_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include "ros/ros.h"
#include "qbo_arduqbo/Mouth.h"
#include <ros/console.h>

class CMouthController : public CController
{
    public:
        CMouthController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh);
        
    protected:
        ros::Subscriber mouth_sub_;
        void setMouth(const qbo_arduqbo::Mouth::ConstPtr& msg);
};

#endif
