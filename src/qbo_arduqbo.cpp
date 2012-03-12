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

#include <qbo_arduqbo.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <XmlRpcValue.h>
#include <map>
#include <vector>
#include <ros/console.h>
#include <servos.h>
#include <sensor_msgs/JointState.h>
#include <myXmlRpc.h>
#include <controllers/controllers_class.h>
#include <controllers/base_controller.h>
#include <controllers/joint_controller.h>
#include <controllers/mouth_controller.h>
#include <controllers/nose_controller.h>
#include <controllers/battery_controller.h>
#include <controllers/mics_controller.h>
#include <controllers/srf10_controller.h>
#include <controllers/lcd_controller.h>
#include <controllers/imu_controller.h>
#include <controllers/infra_red_recievers_controller.h>


CSerialController::CSerialController(std::string port1, int baud1, std::string port2, int baud2, float timeout1, float timeout2, double rate, ros::NodeHandle nh) :
  CQboduinoDriver(port1, baud1, port2, baud2, timeout1, timeout2), rate_(rate), nh_(nh)
{
  //Check parameters in ROS_PARAM and start controllers
  //Advertise the test service
  qboTestService_=nh_.advertiseService("/Qbo/test_service", &CSerialController::qboTestService, this);
  //Check for controllers that are governed by the head board
  if(boards_.count("head")==1)
  {
    //Controlled servos
    if(nh_.hasParam("controlledservos"))
    {
      std::map< std::string, XmlRpc::XmlRpcValue >::iterator it;
      std::map< std::string, XmlRpc::XmlRpcValue > value;
      XmlRpc::MyXmlRpcValue servos;
      nh_.getParam("controlledservos", servos);
      ROS_ASSERT(servos.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      value=servos;
      for(it=value.begin();it!=value.end();it++)
      {
	ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	servosList_[(*it).first]=new ControledServo((*it).first,this);
	servosNamesList_.push_back((*it).first);
	//Set servos parameters
	servosList_[(*it).first]->setParams((*it).second);
	servosList_[(*it).first]->setAngle(0);
	ROS_INFO_STREAM("Servo pseudocontrolado " << (*it).first << " started");
      }
    }
    //Un-controlled servos
    if(nh_.hasParam("uncontrolledservos"))
    {
      std::map< std::string, XmlRpc::XmlRpcValue >::iterator it;
      std::map< std::string, XmlRpc::XmlRpcValue > value;
      XmlRpc::MyXmlRpcValue servos;
      nh_.getParam("uncontrolledservos", servos);
      ROS_ASSERT(servos.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      value=servos;
      for(it=value.begin();it!=value.end();it++)
      {
	ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	servosList_[(*it).first]=new CServo((*it).first,this);
	servosNamesList_.push_back((*it).first);
        //Set servos parameters
	servosList_[(*it).first]->setParams((*it).second);
	servosList_[(*it).first]->setAngle(0);
	ROS_INFO_STREAM("Servo " << (*it).first << " started");
      }
    }
    //Check for controllers
    if(nh_.hasParam("controllers"))
    {
      std::map< std::string, XmlRpc::XmlRpcValue >::iterator it;
      std::map< std::string, XmlRpc::XmlRpcValue > value;
      XmlRpc::MyXmlRpcValue controllers;
      nh_.getParam("controllers", controllers);
      ROS_ASSERT(controllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      value=controllers;
      for(it=value.begin();it!=value.end();it++)
      {
	ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	XmlRpc::XmlRpcValue controller_params=(*it).second;
	if(controller_params.hasMember("type"))
	{
	  std::string type=controller_params["type"];
	  //Joint controller
	  if(type.compare("joint_controller")==0)
	  {
	    ROS_INFO("joint_controller started");
	    controllersList_.push_back(new CJointController((*it).first, this, nh, servosList_));
	  }
	  //Mouth controller
	  else if(type.compare("mouth_controller")==0)
	  {
	    ROS_INFO("mouth_controller started");
	    controllersList_.push_back(new CMouthController((*it).first, this, nh));
	  }
          else if(type.compare("nose_controller")==0)
          {
            ROS_INFO("nose_controller started");
            //std::cout << "mouth_controller encontrado" << std::endl;
            controllersList_.push_back(new CNoseController((*it).first, this, nh));
          }
	  else if(type.compare("mics_controller")==0)
	  {
	    ROS_INFO("mics_controller started");
	    //std::cout << "mouth_controller encontrado" << std::endl;
	    controllersList_.push_back(new CMicsController((*it).first, this, nh));
	  }
	}
      }
    }
    //The following lines setup the joint state publisher at the given rate
    std::string topic;
    nh_.param("joint_states_topic", topic, std::string("joint_states"));
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>(topic, 1);
    timer_=nh_.createTimer(ros::Duration(1/rate_),&CSerialController::timerCallback,this);
  }
  //Check for controllers that are governed by the base board
  if(boards_.count("base")==1)
  {
    if(nh_.hasParam("controllers"))
    {
      std::map< std::string, XmlRpc::XmlRpcValue >::iterator it;
      std::map< std::string, XmlRpc::XmlRpcValue > value;
      XmlRpc::MyXmlRpcValue controllers;
      nh_.getParam("controllers", controllers);
      ROS_ASSERT(controllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      value=controllers;
      for(it=value.begin();it!=value.end();it++)
      {
	ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	XmlRpc::XmlRpcValue controller_params=(*it).second;
	if(controller_params.hasMember("type"))
	{
	  std::string type=controller_params["type"];
	  //Base controller (speed and odometry)
	  if(type.compare("base_controller")==0)
	  {
	    ROS_INFO("base_controller started");
	    controllersList_.push_back(new CBaseController((*it).first, this, nh));
	  }
	  //Battery controller
	  else if(type.compare("battery_controller")==0)
	  {
	    ROS_INFO("battery_controller started");
	    controllersList_.push_back(new CBatteryController((*it).first, this, nh));
	  }
	  //Sensors controller (I2C sonar sensors and ADC readings)
	  else if(type.compare("sensors_controller")==0)
	  {
	    ROS_INFO("sensors_controller started");
	    controllersList_.push_back(new CSrf10Controller((*it).first, this, nh));
	  }
	  //Back LCD controller
          else if(type.compare("lcd_controller")==0)
          {
            ROS_INFO("lcd_controller started");
            controllersList_.push_back(new CLCDController((*it).first, this, nh));
          }
          else if(type.compare("imu_controller")==0)
          {
            ROS_INFO("imu_controller started");
            //std::cout << "imu_controller encontrado" << std::endl;
            controllersList_.push_back(new CImuController((*it).first, this, nh));
          }
          else if(type.compare("irs_controller")==0)
          {
            ROS_INFO("irs_controller started");
            //std::cout << "imu_controller encontrado" << std::endl;
            controllersList_.push_back(new CInfraRedsController((*it).first, this, nh));
          }
	}
      }
    }
  } 
}

CSerialController::~CSerialController()
{
  std::string goodbye;
  goodbye.push_back(12);        //Code that cleans the LCD screen
  goodbye+="QBO is shutting down...";
  this->setLCD(goodbye);
  
  std::map< std::string, CServo * >::iterator it;
  for (it=servosList_.begin();it!=servosList_.end();it++)
  {
    delete (*it).second;
  }
  servosList_.clear();
  while(controllersList_.size()>0)
  {
    delete controllersList_.back();
    controllersList_.pop_back();
  }
}


void CSerialController::timerCallback(const ros::TimerEvent& e) {
  sensor_msgs::JointState joint_state;
  int servos_count=servosList_.size();
  //joint_state.name.resize(servos_count);
  //joint_state.position.resize(servos_count);
  //joint_state.velocity.resize(servos_count);
  for(int i=0;i<servos_count;i++)
  {
    float angle=servosList_[servosNamesList_[i]]->getAngle();
    if(angle!=-1000)
    {
        joint_state.name.push_back(servosNamesList_[i]);
        joint_state.position.push_back(angle);
        joint_state.velocity.push_back(0);
    }
  }
  joint_state.header.stamp = ros::Time::now();
  //publish
  joint_pub_.publish(joint_state);
}

bool CSerialController::qboTestService(qbo_arduqbo::Test::Request  &req, qbo_arduqbo::Test::Response &res)
{
  //estado de qboard1
  if(boards_.count("base")==1)
  {
    std::map<uint8_t,unsigned short> sensorsDistances;
    uint8_t I2cDevicesState=0;
    res.Qboard1=true;
    getDistanceSensors(sensorsDistances);
    std::map<uint8_t,unsigned short>::iterator p;
    uint8_t i=0;
    res.SRFAddress.clear();
    for(p = sensorsDistances.begin(); p != sensorsDistances.end(); p++)
    {
      res.SRFAddress.push_back(p->first);
      i++;
    }
    res.SRFcount=i;
    //res.SRFAddress=0;
    getI2cDevicesState(I2cDevicesState);
    res.Gyroscope=I2cDevicesState&0x01;
    res.Accelerometer=I2cDevicesState&0x02;
    res.LCD=I2cDevicesState&0x04;
    res.Qboard3=I2cDevicesState&0x08;
    //Obtener SRFCount
    //sus direcciones
    //gyro, acel y lcd
  }
  else
  {
    res.Qboard1=true;
    //Obtener SRFCount
    res.SRFcount=0;
    res.Gyroscope=false;
    res.Accelerometer=false;
    res.LCD=false;
    res.Qboard3=false;
  }
  //estado de qboard2
  if(boards_.count("head")==1)
    res.Qboard2=true;
  else
    res.Qboard2=false;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "serial_comm_node");
  ros::NodeHandle n("~");
  std::string port1,port2;
  int baud1,baud2;
  double timeout1,timeout2,rate;
  //Read serial port configuration
  n.param("port1", port1, std::string("/dev/ttyUSB0"));
  std::cout << port1 << std::endl;
  n.param("port2", port2, std::string("/dev/ttyUSB1"));
  std::cout << port2 << std::endl;
  n.param("baud1", baud1, 115200);
  n.param("baud2", baud2, 115200);
  n.param("timeout1", timeout1, 0.01);
  n.param("timeout2", timeout2, 0.01);
  //Read joint state publishing rate
  n.param("rate", rate, 15.0);
  CSerialController serial_controller(port1,baud1,port2,baud2,timeout1,timeout2,rate,n);
  ROS_INFO("Ready to send messages to the Arduino.");
  ros::spin();

  return 0;
}

