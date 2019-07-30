#ifndef ENCODER_H
#define ENCODER_H

#include <PID_v1.h>

#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

class Arduino_Encoder {
public:
	
	int temp; // for temporal datatype conversion
	double vCurrent, pwm, vTarget;

	unsigned volatile short pulse;

	PID myPID(&double, &double, &double, double, double, double, DIRECT);

	volatile int encoderPos = 0;
	int encoder; //encoderPos before for testing CCW OR CW
	int encoder_b; //mark the encoderPos when pulses change

	//fetch target velocity from ROS
	void updateVTarget(const std_msgs::Int32&);
	std_msgs::String str_msg;


	ros::Subscriber<std_msgs::Int32> pc_vTarget(__cpp_raw_strings, &std_msgs::String);
	ros::Publisher pub_vTarget(__cpp_raw_strings, &std_msgs::String);

	//int motor;
	int encoderPinA;
	int encoderPinB;
	int encoderPinZ;


	class Arduino_Encoder(int, int, int);
	
	void doEncoder();
	void updateSpeed();
	int getPulse();
};

#endif