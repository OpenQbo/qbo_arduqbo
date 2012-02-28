#ifndef BATTERY_CONTROLLER_H
#define BATTERY_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "qbo_arduqbo/BatteryLevel.h"

class CBatteryController : public CController
{
    public:
        CBatteryController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh);
        
    protected:
	ros::Publisher battery_pub_;
        float level_;
        void timerCallback(const ros::TimerEvent& e);
};

#endif
