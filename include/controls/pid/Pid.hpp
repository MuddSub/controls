/**
\file Pid.hpp
\author Seth Isaacson
\brief Implements the Axis class, which contains a PID controller which
       governs a specific axis. Also includes a handy interface for updating
       relevant parameters.
\remark Despite what some of my peers my claim, at least in the context of this
        package, it is not only correct but REQUIRED to refer to a PID
        controller not as a "P. I. D. controller", but as a "pid controller",
        where pid is a single word rhyming with "kid" (@omari).
\remark Work largely based on the ROS PID package, written by Andy Zelenak and
        Paul Bouchier. Package documented at http://wiki.ros.org/pid. This is
        really just a major refactor of that code with some features added,
        and much of the code copy-pastad over.
*/

#ifndef PIDAXIS_H
#define PIDAXIS_H


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <controls/PidConfig.h>
#include <dynamic_reconfigure/server.h>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <ros/package.h>
#include "controls/pid/PidUtils.hpp"
#include <deque>

class Pid{

private:
  std::vector<std::string> axes_ = {"SURGE", "SWAY", "HEAVE", "ROLL", "PITCH", "YAW"};
  std::vector<std::string> inputs_ = {"IMU_POS", "IMU_ACCEL", "DEPTH", "CAM_FRONT", "CAM_BOTTOM", "LOCALIZE"};


public:

  Pid(int axis = -1, int input = -1);
  //no copy constructor, shouldn't be used
  Pid(const Pid&) = delete;
  //no assignment operator either
  Pid operator=(const Pid&) const = delete;

  //Saves parameters to file. Requires clean exit.
  ~Pid();


private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhPriv_;

  //central PID parameters
  double kP_, kD_, kI_;

  bool enabled_;

  std::ofstream derivFile;

  std::string inputType_ = "OTHER_INPUT";
  std::string prevInputType_ = "OTHER_INPUT";
  int inputTypeInt_ = -1; //for rosparam usage, immediately casts as enum

  std::string axis_ = "OTHER_AXIS";
  int axisInt_ = -1;

  double controlEffort_ = 0;
  double prevControlEffort_ = 0;

  //Important stuff to a PID controller
  double setpoint_ = 0;
  double plantState_ = 0;
  std::vector<double> error_ = {0,0,0};
  std::vector<double> errorDeriv_ = {0,0,0};

  std::vector<double> filteredError_ = {0,0,0};
  double errorIntegral_ = 0;
  std::vector<double> filteredErrorDeriv_ = {0,0,0};
  double filteredDeriv_ = 0;
  double doubleFilteredDeriv_ = 0;
  std::deque<double> derivQueue_;
  double FIRDeriv_;

  double derivTime_;
  double prevDerivErr_;

  //has one of parameters recently changed?
  bool setpointChanged_; //goes false once ramp is complete
  bool plantStateChanged_;

  double setpointChangeTime_ = 0;
  double setpointStep_[2]; //[oldVal, newVal]
  double setpointRampDuration_ = 1.;
  ros::Time prevTime_;

  //max value for integral term to reach
  double windupLimit_;

  //filter stuff - swiped from ROS PID

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoffFrequency_ = -1;


  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency
  // at
  // 1/4 of the sample rate.
  double c_ = 1.;

  //pub/sub topics
  std::string controlEffortTopic_, plantStateTopic_, setpointTopic_, enabledTopic_,
              inputTopic_;
  std_msgs::Float64 controlEffortMsg_;
  ros::Publisher controlEffortPub_;
  ros::Subscriber plantStateSub_, setpointSub_, enabledSub_, inputSub_, flushSub_;


  ///Get new kP, kI, kD values and topic names from config.
  void loadParamsFromFile();
  std::unordered_map<std::string, std::vector<double>> paramMap_;
  std::unordered_map<std::string, std::string> topicMap_;


  void getParamsFromMap();
  void updatePlantSub();


  bool isFirstCallBack_;

  //private methods
  inline double getError(){return setpoint_ - plantState_;}

  ///do all the PID stuff
  void executeController(double timePassed);

  ///updates buffers and filters. The filters were taken DIRECTLY from the ROS
  ///PID package, which in turn cites Julius O. Smith III, Intro. to
  /// Digital Filters With Audio Applications.
  void updateErrors(double);

  void reconfigureCallback(controls::PidConfig& config, uint32_t level);

  void saveParams();

  std::string getConfigPath();

  void writeToFile();


  //pack these all into a service.
  void updateInputType(std::string input);
  void updatePlantState(const double&);
  void updateSetpoint(const double&);


  //callbacks
  void plantStateCallback(const std_msgs::Float64& msg);
  void setpointCallback(const std_msgs::Float64& msg);
  void enabledCallback(const std_msgs::Bool& msg);
  void inputCallback(const std_msgs::Int32& msg);
  void flushPidCallback(const std_msgs::Bool& msg);



};



#endif
