/*
 * qbo_arduqbo.h
 *
 *  Created on: 04/08/2011
 *      Author: maps
 */

#ifndef QBO_ARDUQBO_H_
#define QBO_ARDUQBO_H_

#include <ros/ros.h>
#include <driver/qboduino_driver.h>
#include <driver/arduqbo_instructions.h>
#include <controllers/controllers_class.h>
#include <servos.h>
#include <std_msgs/String.h>
#include <map>
#include <vector>


//TODO: Floor Distance, infraRed sensors

class CSerialController : public CQboduinoDriver
{
public:
  CSerialController(std::string port1="/dev/ttyUSB0", int baud1=115200, std::string port2="/dev/ttyUSB1", int baud2=115200, float timeout1=0.05, float timeout2=0.05, double rate=15, ros::NodeHandle nh=ros::NodeHandle("~"));
  ~CSerialController();

private:
  double rate_;
  ros::NodeHandle nh_;

  ros::Timer timer_;

  std::map<std::string, CServo *> servosList_;
  std::vector<std::string> servosNamesList_;
  std::vector<CController *> controllersList_;

  ros::Publisher joint_pub_;

  void timerCallback(const ros::TimerEvent& e);
};

#endif /* QBO_ARDUQBO_H_ */
