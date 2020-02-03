#include <controls/pid/PidManager.hpp>


PidManager::PidManager(ros::NodeHandle nh)
  : nh_{nh}{


  axisSurge_ = Axis("surge", nh);
  axisSway_ = Axis("sway", nh);
  axisHeave_ = Axis("heave", nh);
  axisRoll_ = Axis("roll", nh);
  axisPitch_ = Axis("pitch", nh);
  axisYaw_ = Axis("yaw", nh);

  enabledService_ = nh_.advertiseService("EnabledService", &PidManager::enabledServiceCB, this);
  inputService_ = nh_.advertiseService("InputTypeService", &PidManager::inputServiceCB, this);
  setpointService_ = nh_.advertiseService("SetpointService", &PidManager::setpointServiceCB, this);
  thrustOverrideService_ = nh_.advertiseService("ThrustOverrideService", &PidManager::thrustOverrideCB, this);

  //pause for a bit to let the messages catch up
  ros::Duration(2).sleep();

  ROS_INFO("Initialized PID Manager");

}


void PidManager::setPidEnabled(const PidUtils::Axes& axis, const bool& enabled){
  selectAxis(axis).setPidEnabled(enabled);
}


void PidManager::setPlantState(const PidUtils::Axes& axis, const double& val){
  selectAxis(axis).setPlantState(val);
}

void PidManager::setSetpoint(const PidUtils::Axes& axis, const double& val){
  selectAxis(axis).setSetpoint(val);
}

void PidManager::setInputType(const PidUtils::Axes& axis, const PidUtils::Inputs& input){
  selectAxis(axis).setInputType(input);
}

void PidManager::setPercentThrust(const PidUtils::Axes& axis, const double& val){
  selectAxis(axis).setPercentThrust(val);
}

Axis& PidManager::selectAxis(const PidUtils::Axes& axis){
  switch(axis){
    case(PidUtils::SURGE):
      return axisSurge_;
    case(PidUtils::SWAY):
      return axisSway_;
    case(PidUtils::HEAVE):
      return axisHeave_;
    case(PidUtils::ROLL):
      return axisRoll_;
    case(PidUtils::PITCH):
      return axisPitch_;
    case(PidUtils::YAW):
      return axisYaw_;
    default:
      ROS_FATAL("Invalid axis enum provided with integer value %d", (int)axis);
  }
}

Axis& PidManager::selectAxis(const std::string& axis){
    if(axis == "surge")
      return axisSurge_;
    else if(axis == "sway")
      return axisSway_;
    else if(axis == "heave")
      return axisHeave_;
    else if(axis == "roll")
      return axisRoll_;
    else if(axis == "pitch")
      return axisPitch_;
    else if(axis == "yaw")
      return axisYaw_;

    else{
      ROS_FATAL("Axis string provided (%s) is invalid.", axis.c_str());
      return axisSurge_;
    }
}


//Service callbacks
bool PidManager::enabledServiceCB(controls::EnabledService::Request &req,
                      controls::EnabledService::Response &res){

  res.success = true;
  std::string axis = req.axis;
  std::transform(axis.begin(), axis.end(),axis.begin(), ::tolower);

  if(std::find(PidUtils::AxisStrings.begin(), PidUtils::AxisStrings.end(), axis) != PidUtils::AxisStrings.end()) {
    selectAxis(axis).setPidEnabled(req.en);
  }
  else {
      ROS_FATAL("Axis string provided (%s) is invalid.", axis.c_str());
      res.success = false;
  }
  return res.success;
}


bool PidManager::inputServiceCB(controls::InputService::Request &req,
                    controls::InputService::Response &res){

  res.success = true;
  std::string axis = req.axis;
  PidUtils::Inputs inputType = (PidUtils::Inputs)req.input;
  std::transform(axis.begin(), axis.end(),axis.begin(), ::tolower);

  if(std::find(PidUtils::AxisStrings.begin(), PidUtils::AxisStrings.end(), axis) != PidUtils::AxisStrings.end()) {
    selectAxis(axis).setInputType(inputType);
  }
  else {
      ROS_FATAL("Axis string provided (%s) is invalid.", axis.c_str());
      res.success = false;
  }
  return res.success;
}

bool PidManager::setpointServiceCB(controls::SetpointService::Request &req,
                       controls::SetpointService::Response &res){


  res.success = true;
  std::string axis = req.axis;
  double setpoint = req.value;
  std::transform(axis.begin(), axis.end(),axis.begin(), ::tolower);


  if(std::find(PidUtils::AxisStrings.begin(), PidUtils::AxisStrings.end(), axis) != PidUtils::AxisStrings.end()) {
    selectAxis(axis).setSetpoint(setpoint);
    Axis& currentAxis = selectAxis(axis);

    selectAxis(axis).setSetpoint(setpoint);

    if(!currentAxis.isEnabled()){
      ROS_INFO("Implicitly enabling %s pid loop in order to set the setpoint.", axis.c_str());
      currentAxis.setPidEnabled(true);
    }

  }
  else {
      ROS_FATAL("Axis string provided (%s) is invalid.", axis.c_str());
      res.success = false;
  }
  return res.success;
}

bool PidManager::thrustOverrideCB(controls::ThrustOverrideService::Request &req,
                      controls::ThrustOverrideService::Response &res){

  res.success = true;
  std::string axis = req.axis;
  double percentThrust = req.value;
  std::transform(axis.begin(), axis.end(),axis.begin(), ::tolower);

  if(std::find(PidUtils::AxisStrings.begin(), PidUtils::AxisStrings.end(), axis) != PidUtils::AxisStrings.end()) {
    Axis& currentAxis = selectAxis(axis);
    if(currentAxis.isEnabled()){
      ROS_INFO("Implicitly disabling %s pid loop in order to set the thrust.", axis.c_str());
      currentAxis.setPidEnabled(false);
    }

    selectAxis(axis).setPercentThrust(percentThrust);
  }
  else {
      ROS_FATAL("Axis string provided (%s) is invalid.", axis.c_str());
      res.success = false;
  }
  return res.success;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "PidManager");
  ros::NodeHandle nh_;
  // PidManager a(nh_);

  PidManager b(nh_);
  b.setPidEnabled(PidUtils::HEAVE);
  b.setPidEnabled(PidUtils::YAW);

  ros::Rate r(100);
  while(ros::ok){
    ros::spinOnce();
    r.sleep();
  }

}
