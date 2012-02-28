#ifndef MICS_CONTROLLER_H
#define MICS_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "qbo_arduqbo/NoiseLevels.h"
#include "qbo_arduqbo/Mic.h"

class CMicsController : public CController
{
    public:
        CMicsController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh);
        
    protected:
	ros::Publisher mics_pub_;
        ros::Subscriber mics_sub_;
        uint16_t m0_;
        uint16_t m1_;
        uint16_t m2_;
        uint8_t mic_;
	void micsCallback(const qbo_arduqbo::Mic::ConstPtr& msg);
        void timerCallback(const ros::TimerEvent& e);
};

#endif
