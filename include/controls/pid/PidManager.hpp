//stamps out six axes
//provides mux for calling all the functions
//additional functionality such as go-to-depth which only applied to a specific axis

#ifndef PID_MANAGER
#define PID_MANAGER

#include "controls/Axis.hpp"
#include "controls/EnabledService.h"
#include "controls/InputService.h"
#include "controls/SetpointService.h"
#include "controls/ThrustOverrideService.h"

class PidManager{


public:

  PidManager() = default;
  PidManager(ros::NodeHandle);
  void setPidEnabled(const PidUtils::Axes& axis, const bool& enabled = true);
  void setPlantState(const PidUtils::Axes& axis, const double& val);
  void setSetpoint(const PidUtils::Axes& axis, const double& val);
  void setInputType(const PidUtils::Axes& axis, const PidUtils::Inputs&);

private:

  //get corresponding axis from enum or string
  Axis& selectAxis(const PidUtils::Axes& axis);
  Axis& selectAxis(const std::string& axis);

  void setPercentThrust(const PidUtils::Axes& axis, const double& val);

  //to handle yaw wrapping
  double yawSetpoint_;
  ros::Subscriber yawSub_;


  void CheckStability();

  Axis axisSurge_, axisSway_, axisHeave_, axisRoll_, axisPitch_, axisYaw_;

  ros::NodeHandle nh_;
  ros::ServiceServer enabledService_, inputService_, setpointService_, thrustOverrideService_;


  bool enabledServiceCB(controls::EnabledService::Request &req,
                        controls::EnabledService::Response &res);

  bool inputServiceCB(controls::InputService::Request &req,
                      controls::InputService::Response &res);

  bool setpointServiceCB(controls::SetpointService::Request &req,
                         controls::SetpointService::Response &res);


  bool thrustOverrideCB(controls::ThrustOverrideService::Request &req,
                        controls::ThrustOverrideService::Response &res);

};


#endif
