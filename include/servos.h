/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2012 OpenQbo, Inc.
 *
 * This program is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 * Authors: Miguel Angel Julian <miguel.a.j@openqbo.org>;
 * 
 */

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
