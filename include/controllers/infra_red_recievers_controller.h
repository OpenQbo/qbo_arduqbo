#ifndef INFRA_RED_RECIEVERS_CONTROLLER_H
#define INFRA_RED_RECIEVERS_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "qbo_arduqbo/Irs.h"

class CInfraRedsController : public CController
{
    public:
        CInfraRedsController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh);
        
    protected:
	ros::Publisher irs_pub_;
        uint8_t ir0_;
        uint8_t ir1_;
        uint8_t ir2_;
        void timerCallback(const ros::TimerEvent& e);
};

#endif
