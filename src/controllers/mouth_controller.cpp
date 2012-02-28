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

#include <controllers/mouth_controller.h>
#include "ros/ros.h"
#include "qbo_arduqbo/Mouth.h"
#include <ros/console.h>

CMouthController::CMouthController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_mouth"));
    mouth_sub_ = nh.subscribe<qbo_arduqbo::Mouth>(topic, 1, &CMouthController::setMouth,this);
}

void CMouthController::setMouth(const qbo_arduqbo::Mouth::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Mouth comand arrived: " << msg->mouthImage[0] << "," << msg->mouthImage[1] << "," << msg->mouthImage[2] << "," << msg->mouthColor);
    int code=device_p_->setMouth((uint8_t)(msg->mouthImage[0]), (uint8_t)(msg->mouthImage[1]), (uint8_t)(msg->mouthImage[2]),(uint8_t)(msg->mouthColor));
    if (code<0)
        ROS_ERROR("Unable to send mouth to the head control board");
    else
        ROS_DEBUG_STREAM("Sent mouth " << msg->mouthImage[0] << "," << msg->mouthImage[1] << "," << msg->mouthImage[2] << "," << msg->mouthColor << " to the head board ");
}
