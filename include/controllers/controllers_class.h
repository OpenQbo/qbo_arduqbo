#ifndef CONTROLLERS_CLASS_H
#define CONTROLLERS_CLASS_H

#include <driver/qboduino_driver.h>
#include "ros/ros.h"

class CController
{
    public:
        CController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : rate_(15), name_(name), device_p_(device_p)
        {
        }
        std::string getName()
        {
            return name_;
        }
    protected:
        double rate_;
        ros::Timer timer_;
        std::string name_;
        CQboduinoDriver *device_p_;
};

#endif
