#ifndef SERVOS_H
#define SERVOS_H

#include <driver/qboduino_driver.h>
#include <cmath>
#include <algorithm>
#include <XmlRpcValue.h>
#include <ros/console.h>

double radians(double angle)
{
    static double ratio =M_PI/180.0;
    return angle*ratio;
};

class CServo
{
    public:
        CServo(std::string name, CQboduinoDriver *device_p, bool single=false);
        void setParams(XmlRpc::XmlRpcValue params);
        void setAngle(float ang, float velocity=1);
        virtual float getAngle();
        float getAngleStored();
        std::string getName();
    
    protected:
        std::string name_;
        CQboduinoDriver *device_p_;
        int id_;
        int neutral_;
        int ticks_;
        float max_angle_;
        float min_angle_;
        float rad_per_tick_;
        float max_speed_;
        int max_ticks_speed_;
        bool invert_;
        float angle_;
        float range_;
  
        void recalculateAngleParams();
};

class ControledServo : public CServo
{
    public:
        
        ControledServo(std::string name, CQboduinoDriver *device_p, bool single=false)
            : CServo(name,device_p,single)
        {
        }
        virtual float getAngle();
        
};

#endif
