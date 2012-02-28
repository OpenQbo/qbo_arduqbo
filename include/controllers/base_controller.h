#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include <cmath>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>

class CBaseController : public CController
{
    public:
        CBaseController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh);

        /*
        void commandCallback(const turtlesim::Velocity::ConstPtr& vel)
        {
            device_p_->setSpeed((float)vel->linear,(float)vel->angular);
        }
        */
        
    protected:
        
        //ros::Subscriber vel_sub_;
        ros::Subscriber twist_sub_;
	ros::Publisher odom_pub_;
	ros::ServiceServer stall_unlock_service_;
        tf::TransformBroadcaster odom_broadcaster_;
        //void commandCallback(const turtlesim::Velocity::ConstPtr& vel);
        //void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
        //bool sendSpeedCommand(float left_vel, float right_vel);
        //bool stopped_;
        //float vl_,va_;
        //void getSpeed();
        
        // internal data            
        float v_linear_;             // current setpoint velocity
        float v_angular_;
        bool v_dirty_;
        float x_;                  // position in xy plane
        float y_;
        float th_;
        nav_msgs::Odometry odom_;
	ros::Time then_;
	bool is_odom_broadcast_enabled_;
        
        void timerCallback(const ros::TimerEvent& e);
        void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
        bool unlockStall(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res );
};

#endif
