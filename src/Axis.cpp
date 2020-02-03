#include <controls/Axis.hpp>

Axis::Axis(std::string axisName, const ros::NodeHandle& nh)
    : axisName_(axisName), plantState_(0), setpoint_(0), controlEffort_(0), nh_(nh)
{

  std::transform(axisName.begin(), axisName.end(),axisName.begin(), ::tolower);


  //For bypassing PID Control
  percentThrustTopic_ = axisName + "ControlEffort";


  //PID Interface
  plantStateTopic_ = axisName + "PlantState";
  setpointTopic_ = axisName + "Setpoint";
  enabledTopic_ = axisName + "Enabled";
  inputTopic_ = axisName + "InputType";
  controlEffortTopic_ = axisName + "ControlEffort";

  //PID Publishers
  plantPub_ = nh_.advertise<std_msgs::Float64>(plantStateTopic_, 1);
  setpointPub_ = nh_.advertise<std_msgs::Float64>(setpointTopic_, 1);
  enabledPub_ = nh_.advertise<std_msgs::Bool>(enabledTopic_, 1);
  inputPub_ = nh_.advertise<std_msgs::Int32>(inputTopic_, 1);

  percentThrustPub_ = nh_.advertise<std_msgs::Float64>(percentThrustTopic_, 1);


}

//publishers for PID interface


void Axis::setPidEnabled(const bool& enabled){
  enabled_ = enabled;
  std_msgs::Bool msg;
  msg.data = enabled_;
  enabledPub_.publish(msg);
}

void Axis::setSetpoint(const double& val, const size_t& stabilityBand){
  setpoint_ = val;
  stabilityBand_ = stabilityBand;
  std_msgs::Float64 msg;
  msg.data = val;
  setpointPub_.publish(msg);
}

void Axis::setInputType(const PidUtils::Inputs& input){
  inputType_ = input;
  std_msgs::Int32 msg;
  msg.data = (int)input;
  inputPub_.publish(msg);
}


void Axis::setPlantState(const double& val){
  if(axisName_ == "yaw") return;
  std_msgs::Float64 msg;
  msg.data = val;
  plantPub_.publish(msg);
}

void Axis::controlEffortCallback(const std_msgs::Float64& msg){
  controlEffort_ = msg.data;
}

void Axis::setPercentThrust(const double& val){
    std_msgs::Float64 msg;
    msg.data = val;
    percentThrustPub_.publish(msg);
}
