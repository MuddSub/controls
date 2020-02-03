#ifndef AXIS
#define AXIS

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include "controls/pid/PidUtils.hpp"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

class Axis{

public:
  Axis() = default;

  //note: intentionally not by const referance; want a copy
  Axis(std::string axisName, const ros::NodeHandle& nh);

  //should call the service
  void updateController(const PidUtils::UpdateParams& param, const double& val);

  //just call updateController accordingly
  void setPidEnabled(const bool& enabled = true);
  void setPlantState(const double& val);
  void setSetpoint(const double& val, const size_t& stabilityBand = 100);
  void setInputType(const PidUtils::Inputs&);
  inline bool isEnabled(){return enabled_;}


  //for manually setting thrust
  void setPercentThrust(const double& val);

  // void setInputType(const PidUtils&::Inputs);

  inline bool isStable(const double& stableMargin = .1){return true;}


private:

  // short axisInt_;

  std::string axisName_;
  //used to keep track of steps to track stability
  double setpointBuffer_[2] = {0,0};

  //sub to control effort
  std::string controlEffortTopic_;

  //published topics
  std::string setpointTopic_, enabledTopic_, inputTopic_, percentThrustTopic_,
              plantStateTopic_;

  //if we want to manually set percent thrust
  double percentThrust_;


  //state of controller
  double plantState_;
  double controlEffort_;
  bool enabled_;

  double setpoint_;
  double stabilityBand_;

  PidUtils::Inputs inputType_;


  ros::NodeHandle nh_;
  ros::Publisher plantPub_, setpointPub_, enabledPub_, inputPub_, percentThrustPub_;
  ros::Subscriber controlEffortSub_;


  void controlEffortCallback(const std_msgs::Float64& msg);

};



#endif
