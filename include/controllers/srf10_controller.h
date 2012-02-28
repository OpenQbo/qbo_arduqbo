#ifndef SRF10_CONTROLLER_H
#define SRF10_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
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


class CDistanceSensor
{
    public:
        CDistanceSensor(std::string name, uint8_t address, std::string topic, ros::NodeHandle& nh, std::string type, std::string frame_id="");
        void publish(unsigned int readedValue, ros::Time time);
        std::string getName();
    protected:
        std::string name_;
        uint8_t address_;
	std::string type_;
	ros::Publisher sensor_pub_;
        //pcl::PointCloud<pcl::PointXYZ> cloud_;
        //sensor_msgs::PointCloud2 msg_;
        sensor_msgs::PointCloud cloud_;
};

class CSrf10Controller : public CController
{
    public:
        CSrf10Controller(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh);
        ~CSrf10Controller();

    protected:
        void timerCallback(const ros::TimerEvent& e);
        std::map<uint8_t,CDistanceSensor *> srf10Sensors_;
        std::map<uint8_t,uint8_t> srf10SensorsUpdateGroup_;
        std::map<uint8_t,CDistanceSensor *> adcSensors_;
        std::vector<uint8_t> adcSensorsAddresses_;
};

#endif
