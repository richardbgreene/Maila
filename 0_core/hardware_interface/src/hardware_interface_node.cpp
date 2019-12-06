/*--------------------------------------------------------------------------*\
 |                                                                          |
 |  Copyright (C) 2019                                                      |
 |                                   .__  __                                |
 |                        __ __  ____ |__|/  |_  ____                       |
 |                       |  |  \/    \|  \   __\/    \                      |
 |                       |  |  /   |  \  ||  | |   |  \                     |
 |                       |____/|___|  /__||__| |___|  /                     |
 |                       .__  __         .__                                |
 |                       |__|/  |______  |  | ___.__.                       |
 |                       |  \   __\__  \ |  |<   |  |                       |
 |                       |  ||  |  / __ \|  |_\___  |                       |
 |                       |__||__| (____  /____/ ____|                       |
 |                                     \/     \/                            |
 |                                                                          |
 |      Main of trajectory planning for formula student driverless.         |
 |                                                                          |  
 |      Magnago Valerio                                                     |
 |      Dipartimento di Ingegneria Industriale                              |
 |      Universita` degli Studi di Trento                                   |
 |      email: valerio.magnago@unitn.it                                     |
 |                                                                          |
\*--------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <atomic>

#ifdef IS_PC
#include "pca9685/fake_pca9685.h"
#else
#include "pca9685/pca9685.h"
#include <wiringPi.h>
#endif



#include "custom_msgs/ActuatorControl.h"
#include "custom_msgs/ActuatorControlInfo.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Int32.h>

#define PCA9685_ADDRESS 0x40
#define STEER_CH 0
#define MOTOR_CH 1
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50


ros::Subscriber manualCmdSubscriber;
ros::Subscriber autonomousCmdSubscriber;
ros::Subscriber emergencySubscriber;
ros::Subscriber drivingModeSubscriber;

ros::Publisher  cmdInfoPublisher;
ros::Publisher  simCmdPublisher;

ros::Timer freq_timer;

// Topic name list
std::string manual_cmd_topic_name; // = "/hw/manual_cmd";
std::string auto_cmd_topic_name; //   = "/hw/autonomous_cmd";
std::string cmd_info_topic_name; //   = "/hw/info";
std::string emergency_topic_name; //  = "/hw/emergency_brake";
std::string autonomous_mode_topic_name; // = "/autonomous_driver";
std::string cmd_sim_topic_name; //      = "/sim/cmd";


double max_steering_delta;
float  steer_range, steer_center;

float  motor_range, motor_center;



int fd0;
bool emergency_brake = false;
bool autonomous_mode = false;
bool sim_publisher = false;


// Subscriber msg period check
double min_subs_period;
std::atomic_uint n_manual_clbk;
std::atomic_uint n_autonom_clbk;
std::atomic_uint n_drive_type_clbk;

bool freq_auto_ok = false;
bool freq_manual_ok = false;
bool freq_drive_type_ok = false;


double throttle_manual_cmd = 0;
template<class Type>
Type getParam(const ros::NodeHandle &nh, const std::string &name) {
    Type       val;
    const bool success = nh.getParam(name, val);
    if(success)
        ROS_DEBUG("Got param: %s", name.c_str());
    else
        ROS_ERROR("mms_interface_node: Failed to get param '%s'", name.c_str()); 
    assert(success && "PARAMETER DOES NOT EXIST");
    return val;
}

// Load params
void load_params(const ros::NodeHandle& nh){
    const double kPi = M_PI;

    ROS_INFO_STREAM("Loading params");    

    manual_cmd_topic_name = getParam<std::string>(nh, "ros_topic/manual_cmd");
    auto_cmd_topic_name   = getParam<std::string>(nh, "ros_topic/auto_cmd");
    cmd_info_topic_name   = getParam<std::string>(nh, "ros_topic/cmd_info");
    emergency_topic_name  = getParam<std::string>(nh, "ros_topic/emergency");
    autonomous_mode_topic_name = getParam<std::string>(nh, "ros_topic/autonomous_mode");
    cmd_sim_topic_name = getParam<std::string>(nh, "ros_topic/cmd_sim");


    sim_publisher = getParam<bool>(nh, "sim_publisher");
    

    // Initialize Transformation from car frame to camera
    const double max_degree = getParam<double>(nh, "max_steering");
    max_steering_delta = max_degree*M_PI/180.;

    steer_range  = getParam<double>(nh, "steer_pwm/range");
    steer_center = getParam<double>(nh, "steer_pwm/center");
    motor_range  = getParam<double>(nh, "motor_pwm/range");
    motor_center = getParam<double>(nh, "motor_pwm/center");

    min_subs_period = getParam<double>(nh, "min_msg_period");
}


/**
 * input is [0..1]
 * output is [min..max]
 */
float map(float input, float min, float max){
	input = input/2. + 0.5;
	return (input * max) + (1 - input) * min;
}

double trimValue(double value){
	if(value >  1) value =  1;
	if(value < -1) value = -1;
	return value;	
}

/**
 * Calculate the number of ticks the signal should be high for the required amount of time
 */
int calcTicks(float impulseMs, int hertz)
{
	float cycleMs = 1000.0f / hertz;
	return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

void setSteeringAngle(double value){

	value = -trimValue(value/max_steering_delta);
	float millis = map(value, steer_center - steer_range, 
							steer_center + steer_range);
	int tick = calcTicks(millis, HERTZ);

	pwmWrite(PIN_BASE + STEER_CH, tick);
	return;
}

void setThrottle(double value){
	value = trimValue(value);
	float millis = map(value, motor_center - motor_range, 
							motor_center + motor_range);
	int tick = calcTicks(millis, HERTZ);

	pwmWrite(PIN_BASE + MOTOR_CH, tick);
	return;
}


void cmdCallback(const custom_msgs::ActuatorControl &ctrl){
  auto tmp = ctrl;

	if(emergency_brake){
	  tmp.dc = 0;
  }

  setThrottle(tmp.dc);
  setSteeringAngle(tmp.delta);

  // Publish node information
  custom_msgs::ActuatorControlInfo info;
  info.cmd_recived_stamp = ros::Time::now();
  info.ctrl_set_point = ctrl;
  info.emergency_brake = emergency_brake;
  cmdInfoPublisher.publish(info);

  if(sim_publisher){
    simCmdPublisher.publish(tmp);
  }
}

void neutralControl(){
  custom_msgs::ActuatorControl ctrl;
  ctrl.stamp = ros::Time::now();
  ctrl.dc    = 0;
  ctrl.delta = 0;
  cmdCallback(ctrl);
}

void manualCmdCallback(const custom_msgs::ActuatorControl &ctrl){
  n_manual_clbk++;  
  throttle_manual_cmd = ctrl.dc;
  if(!freq_drive_type_ok) return;
  
  if(!autonomous_mode && freq_manual_ok){        
    cmdCallback(ctrl);    
  }
}

void autonomousCmdCallback(const custom_msgs::ActuatorControl &ctrl){
  n_autonom_clbk++;

  if(!freq_drive_type_ok) return;

  if(autonomous_mode && freq_auto_ok){
    custom_msgs::ActuatorControl ctrl_new = ctrl;
    if(freq_manual_ok){
      ctrl_new.dc += throttle_manual_cmd;
    }
    cmdCallback(ctrl_new);
  }
}

void emergencyCallback(const std_msgs::Bool& msg){
	emergency_brake = msg.data;
}

void driveModeCallback(const std_msgs::Bool& msg){
  n_drive_type_clbk++;  
	autonomous_mode = msg.data;
}


void timerCallback(const ros::TimerEvent& event){  
  if(n_manual_clbk == 0){
    freq_manual_ok = false;
  }else{
    freq_manual_ok = true;
  }

  if(n_autonom_clbk == 0){
    freq_auto_ok = false;
  }else{
    freq_auto_ok = true;
  }

  if(n_drive_type_clbk == 0){
    freq_drive_type_ok = false;
  }else{
    freq_drive_type_ok = true;
  }    
  //Safety operation
  if(!freq_drive_type_ok){
    neutralControl();
    double dt = (event.current_real  - event.last_real).toSec();
    ROS_WARN_STREAM_THROTTLE(1,"Period of topic " << autonomous_mode_topic_name << " more than "<< dt <<" sec, forcing system to neutral position");
  }else{
    if(autonomous_mode){
      if(!freq_auto_ok){
        neutralControl();
        ROS_WARN_STREAM_THROTTLE(1,"Mode is autonomous but period of topic " << auto_cmd_topic_name << " more than 0.3 sec, forcing system to neutral position");
      }
    }else{
      if(!freq_manual_ok){
        neutralControl();
        ROS_WARN_STREAM_THROTTLE(1,"Mode is manual but period of topic " << manual_cmd_topic_name << " more than 0.3 hz, forcing system to neutral position");
      }
    }
  }

  // Reinit counter
  n_manual_clbk  = 0;
  n_autonom_clbk = 0;
  n_drive_type_clbk = 0;
}

void subscribeToTopics(ros::NodeHandle& nh)
{
    ROS_DEBUG("HwIface Initializing subscribers");
    manualCmdSubscriber     = nh.subscribe(manual_cmd_topic_name, 1, manualCmdCallback, ros::TransportHints().tcpNoDelay());
    autonomousCmdSubscriber = nh.subscribe(auto_cmd_topic_name,   1, autonomousCmdCallback, ros::TransportHints().tcpNoDelay());
    emergencySubscriber     = nh.subscribe(emergency_topic_name,  1, emergencyCallback, ros::TransportHints().tcpNoDelay());
    drivingModeSubscriber   = nh.subscribe(autonomous_mode_topic_name,  1, driveModeCallback, ros::TransportHints().tcpNoDelay());
    
    freq_timer = nh.createTimer(ros::Duration(min_subs_period), timerCallback);
}

void publishToTopics(ros::NodeHandle& nh)
{
    ROS_DEBUG("TPH Publishing");
    cmdInfoPublisher = 
    		nh.advertise<custom_msgs::ActuatorControlInfo>
    							(cmd_info_topic_name, 1, false);

    if(sim_publisher){
    simCmdPublisher = 
    		nh.advertise<custom_msgs::ActuatorControl>
    							(cmd_sim_topic_name, 1, false);      
    }    
}

void resetPWM(){
	// Reset all output
	pca9685PWMReset(fd0);
}

bool setUp(){
	// Calling wiringPi setup first.
	wiringPiSetup();

	// Setup with pinbase 300 and i2c location 0x40
	fd0 = pca9685Setup(PIN_BASE + STEER_CH , PCA9685_ADDRESS, HERTZ);
	if (fd0 < 0)
	{
		ROS_WARN_STREAM("Error in setup steering\n");
		return false;
	}


	resetPWM();

	// Set neutral position
	setSteeringAngle(0);
	setThrottle(0);

	return true;
}

int main(int argc, char **argv) {
  n_manual_clbk = 0;
  n_autonom_clbk = 0;
  n_drive_type_clbk = 0;
  // Init Ros nodle handle
  ros::init(argc, argv, "hw_interface");
  ROS_INFO("Hardware interface node started");
  ros::NodeHandle nodeHandle("~");
  load_params(nodeHandle);
  publishToTopics(nodeHandle);
  if(!setUp()){
  	return -1;
  }
  subscribeToTopics(nodeHandle);
  
  ros::spin();
  
  return 0;
}
