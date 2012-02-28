
#include <controllers/srf10_controller.h>
#include "ros/ros.h"
#include <ros/console.h>
#include <map>
#include <vector>
#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/ros/conversions.h>
#include <myXmlRpc.h>

CDistanceSensor::CDistanceSensor(std::string name, uint8_t address, std::string topic, ros::NodeHandle& nh, std::string type, std::string frame_id) :
    name_(name), address_(address), type_(type)
{
    //cloud_.points.resize(1);
    cloud_.set_points_size(1);
    cloud_.set_channels_size(0);
    cloud_.header.frame_id=frame_id;
    cloud_.points[0].x=0;
    cloud_.points[0].y=0;
    cloud_.points[0].z=0;
    //sensor_pub_ = nh.advertise<pcl::PointXYZ>(topic, 1);
    //sensor_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);
    sensor_pub_ = nh.advertise<sensor_msgs::PointCloud>(topic, 1);
}

void CDistanceSensor::publish(unsigned int readedValue, ros::Time time)
{
    float distance=0;
    if(type_.compare("srf10")==0)
    {
	distance=((float)readedValue)/100;
  //std::cout << "srf10 " << distance << std::endl;
    }
    else if(type_.compare("gp2d120")==0)
    {
	distance=(2914 / ((float)readedValue + 5)) - 1;
  //std::cout << "gp2d120 " << distance << std::endl;
    }
    else if(type_.compare("gp2d12")==0)
    {
	if (readedValue<3)
	    distance=-1;
	else
	    distance=(6787.0 /((float)readedValue - 3.0)) - 4.0;
      //std::cout << "gp2d12 " << distance << std::endl;
    }
    cloud_.points[0].x=distance;
    cloud_.header.stamp=time;
    //pcl::toROSMsg(cloud_,msg_)
    sensor_pub_.publish(cloud_);
    //sensor_pub_.publish(msg_);
}

std::string CDistanceSensor::getName()
{
    return name_;
}

CSrf10Controller::CSrf10Controller(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) :
    CController(name,device_p,nh)
{
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("srf10_state"));
    nh.param("controllers/"+name+"/rate", rate_, 15.0);
    if(nh.hasParam("controllers/"+name+"/sensors/front"))
    {
      std::map< std::string, XmlRpc::XmlRpcValue >::iterator it;
      std::map< std::string, XmlRpc::XmlRpcValue > value;
      XmlRpc::MyXmlRpcValue sensors;
      nh.getParam("controllers/"+name+"/sensors/front", sensors);
      ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      value=sensors;
      for(it=value.begin();it!=value.end();it++)
      {
	ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if(!nh.hasParam("controllers/"+name+"/sensors/front/"+(*it).first+"/address"))
        {
            ROS_WARN_STREAM("You need to set the address atribute for the sensor " << (*it).first);
            continue;
        }
        if(!nh.hasParam("controllers/"+name+"/sensors/front/"+(*it).first+"/type"))
        {
            ROS_WARN_STREAM("You need to set the type of the sensor " << (*it).first);
            continue;
        }
        int address;
	std::string type;
        std::string frame_id;
        std::string sensor_topic;
        nh.getParam("controllers/"+name+"/sensors/front/"+(*it).first+"/address", address);
        nh.getParam("controllers/"+name+"/sensors/front/"+(*it).first+"/type", type);
        nh.param("controllers/"+name+"/sensors/front/"+(*it).first+"/frame_id", frame_id, std::string(""));
        nh.param("controllers/"+name+"/sensors/front/"+(*it).first+"/topic", sensor_topic, topic+"/"+(*it).first);
	if (type.compare("srf10")==0)
	{
	    srf10Sensors_[(uint8_t)address]=new CDistanceSensor((*it).first, (uint8_t)address, sensor_topic, nh, type, frame_id);
	    srf10SensorsUpdateGroup_[(uint8_t)address]=1;
	}
	else if (type.compare("gp2d12")==0 || type.compare("gp2d120")==0)
	{
	    adcSensors_[(uint8_t)address]=new CDistanceSensor((*it).first, (uint8_t)address, sensor_topic, nh, type, frame_id);
	    adcSensorsAddresses_.push_back((uint8_t)address);
	}
	ROS_INFO_STREAM("Sensor " << (*it).first << " of type " << type << " inicializado");
      }
    }
    if(nh.hasParam("controllers/"+name+"/sensors/back"))
    {
      std::map< std::string, XmlRpc::XmlRpcValue >::iterator it;
      std::map< std::string, XmlRpc::XmlRpcValue > value;
      XmlRpc::MyXmlRpcValue sensors;
      nh.getParam("controllers/"+name+"/sensors/back", sensors);
      ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      value=sensors;
      for(it=value.begin();it!=value.end();it++)
      {
	ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if(!nh.hasParam("controllers/"+name+"/sensors/back/"+(*it).first+"/address"))
        {
            ROS_WARN_STREAM("You need to set the address atribute for the sensor " << (*it).first);
            continue;
        }
        if(!nh.hasParam("controllers/"+name+"/sensors/back/"+(*it).first+"/type"))
        {
            ROS_WARN_STREAM("You need to set the type of the sensor " << (*it).first);
            continue;
        }
        int address;
	std::string type;
        std::string frame_id;
        std::string sensor_topic;
        nh.getParam("controllers/"+name+"/sensors/back/"+(*it).first+"/address", address);
        nh.getParam("controllers/"+name+"/sensors/back/"+(*it).first+"/type", type);
        nh.param("controllers/"+name+"/sensors/back/"+(*it).first+"/frame_id", frame_id, std::string(""));
        nh.param("controllers/"+name+"/sensors/back/"+(*it).first+"/topic", sensor_topic, topic+"/"+(*it).first);
	if (type.compare("srf10")==0)
	{
	    srf10Sensors_[(uint8_t)address]=new CDistanceSensor((*it).first, (uint8_t)address, sensor_topic, nh, type, frame_id);
	    srf10SensorsUpdateGroup_[(uint8_t)address]=1;
	}
	else if (type.compare("gp2d12")==0 || type.compare("gp2d120")==0)
	{
	    adcSensors_[(uint8_t)address]=new CDistanceSensor((*it).first, (uint8_t)address, sensor_topic, nh, type, frame_id);
	    adcSensorsAddresses_.push_back((uint8_t)address);
	}
	ROS_INFO_STREAM("Sensor " << (*it).first << " of type " << type << " inicializado");
      }
    }
    if(nh.hasParam("controllers/"+name+"/sensors/floor"))
    {
      std::map< std::string, XmlRpc::XmlRpcValue >::iterator it;
      std::map< std::string, XmlRpc::XmlRpcValue > value;
      XmlRpc::MyXmlRpcValue sensors;
      nh.getParam("controllers/"+name+"/sensors/floor", sensors);
      ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      value=sensors;
      for(it=value.begin();it!=value.end();it++)
      {
	ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if(!nh.hasParam("controllers/"+name+"/sensors/floor/"+(*it).first+"/address"))
        {
            ROS_WARN_STREAM("You need to set the address atribute for the sensor " << (*it).first);
            continue;
        }
        if(!nh.hasParam("controllers/"+name+"/sensors/floor/"+(*it).first+"/type"))
        {
            ROS_WARN_STREAM("You need to set the type of the sensor " << (*it).first);
            continue;
        }
        int address;
	std::string type;
        std::string frame_id;
        std::string sensor_topic;
        nh.getParam("controllers/"+name+"/sensors/floor/"+(*it).first+"/address", address);
        nh.getParam("controllers/"+name+"/sensors/floor/"+(*it).first+"/type", type);
        nh.param("controllers/"+name+"/sensors/floor/"+(*it).first+"/frame_id", frame_id, std::string(""));
        nh.param("controllers/"+name+"/sensors/floor/"+(*it).first+"/topic", sensor_topic, topic+"/"+(*it).first);
	if (type.compare("srf10")==0)
	{
	    ROS_WARN("srf10 sensors can only be declared at positions front or back");
	    continue;
	}
	else if (type.compare("gp2d12")==0 || type.compare("gp2d120")==0)
	{
	    adcSensors_[(uint8_t)address]=new CDistanceSensor((*it).first, (uint8_t)address, sensor_topic, nh, type, frame_id);
	    adcSensorsAddresses_.push_back((uint8_t)address);
	}
	ROS_INFO_STREAM("Sensor " << (*it).first << " of type " << type << " inicializado");
      }
    }
    //mando los sensores a qbo
    int code=device_p_->setAutoupdateSensors(srf10SensorsUpdateGroup_);
    if (code<0)
        ROS_WARN("Unable to activate all srf10 sensors at the base control board");
    else
        ROS_INFO("All srf10 sensors updated correctly at the base control board");
    timer_=nh.createTimer(ros::Duration(1/rate_),&CSrf10Controller::timerCallback,this);
}

CSrf10Controller::~CSrf10Controller()
{
    std::map< uint8_t,CDistanceSensor * >::iterator it;
    for (it=srf10Sensors_.begin();it!=srf10Sensors_.end();it++)
    {
      delete (*it).second;
    }
    srf10Sensors_.clear();
}

void CSrf10Controller::timerCallback(const ros::TimerEvent& e)
{
    ros::Time now=ros::Time::now();
    std::map<uint8_t,unsigned short> updatedDistances;
    int code=device_p_->getDistanceSensors(updatedDistances);
    if (code<0)
        ROS_ERROR("Unable to get srf10 sensor distances from the base control board");
    else
    {
        std::map< uint8_t,CDistanceSensor * >::iterator it;
        for (it=srf10Sensors_.begin();it!=srf10Sensors_.end();it++)
        {
            if (updatedDistances.count((*it).first)>0)
            {
                //float distance=((float)updatedDistances[(*it).first])/100;  //distancia en metros
                ROS_DEBUG_STREAM("Obtained distance " << updatedDistances[(*it).first] << " for srf10 sensor " << (*it).second->getName() << " from the base control board");
                if(updatedDistances[(*it).first]>0)
                    (*it).second->publish((float)updatedDistances[(*it).first],now);
            }
            else
                ROS_WARN_STREAM("Could not obtain distance of srf10 sensor " << (int)((*it).first) << " from base control board");
        }
    }
    std::vector<unsigned int> adcReads;
    code=device_p_->getAdcReads(adcSensorsAddresses_,adcReads);
    if (code<0)
        ROS_ERROR("Unable to get adc sensor reads from the base control board");
    else
    {
	if(adcReads.size()!=adcSensorsAddresses_.size())
	    ROS_ERROR("The asked addreses and the returned reads for the adc sensors do not match");
	else
	{
	    for (int i=0;i<adcSensorsAddresses_.size();i++)
	    {
		ROS_DEBUG_STREAM("Obtained distance " << adcReads[i] << " for adc sensor " << adcSensors_[adcSensorsAddresses_[i]]->getName() << " from the base control board");
		adcSensors_[adcSensorsAddresses_[i]]->publish(adcReads[i],now);
	    }
	}
    }
}
