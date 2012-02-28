#ifndef LCD_CONTROLLER_H
#define LCD_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include "ros/ros.h"
#include "qbo_arduqbo/LCD.h"
#include <ros/console.h>

class CLCDController : public CController
{
    public:
        CLCDController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh);
        
    protected:
        ros::Subscriber lcd_sub_;
        void setLCD(const qbo_arduqbo::LCD::ConstPtr& msg);
};

#endif
