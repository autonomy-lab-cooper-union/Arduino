#include <encoder.h>
#include <PID_v1.h>
//input:velocity from
//output:pwm to motor controller
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

//ros::NodeHandle  nh;

/*
#define encoderPinA  2
#define encoderPinB  4
#define encoderPinZ  5
*/
#define TOTAL_TICK 6800
//#define MOTORL 9

#define KP 10
#define KI 0
#define KD 0

//output conencts to potentiometer of the motor controller's arduino?
class Arduino_Encoder
{
public:
	//ros::NodeHandle  nh;
	int temp; // for temporal datatype conversion
	double vCurrent, pwm, vTarget;

	unsigned volatile short pulse;

	PID myPID(&vCurrent, &pwm, &vTarget, KP, KI, KD, DIRECT);

	volatile int encoderPos = 0;
	int encoder; //encoderPos before for testing CCW OR CW
	int encoder_b; //mark the encoderPos when pulses change

	

	//fetch target velocity from ROS
	void updateVTarget(const std_msgs::Int32& val) {
		temp = val.data;
		vTarget = (double)temp;
	}

	std_msgs::String str_msg;

	//"" is the topic name, pc...--name of subscriber
	ros::Subscriber<std_msgs::Int32> pc_vTarget("motor_vTarget", &updateVTarget);
	ros::Publisher pub_vTarget("pub_vTarget", &str_msg);
	
	//int motor;
	int encoderPinA;
	int encoderPinB;
	int encoderPinZ;
	
	void doEncoder();
	
	void updateSpeed();
	void updateVTarget(const std_msgs::Int32 &);
	int getPulse();

	class Arduino_Encoder(int motor_n, int pinA, int pinB, int pinZ)//:motor(motor_n)
	{

		encoderPinA = pinA;
		encoderPinB = pinB;
		encoderPinZ = pinZ;
		
		/*
		nh.initNode();
		nh.subscribe(pc_vTarget);
		nh.advertise(pub_vTarget);
		*/

		pinMode(encoderPinA, INPUT);
		digitalWrite(encoderPinA, HIGH);       // turn on pull-up resistor
		pinMode(encoderPinB, INPUT);
		digitalWrite(encoderPinB, HIGH);       // turn on pull-up resistor
		pinMode(encoderPinZ, INPUT);
		digitalWrite(encoderPinZ, HIGH);       // turn on pull-up resistor
		pinMode(motor, OUTPUT);

		attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
		attachInterrupt(digitalPinToInterrupt(encoderPinZ), doEncoder, CHANGE);
		//attachInterrupt(digitalPinToInterrupt(13), rpm, CHANGE); //for pid output                

		
		myPID.SetMode(AUTOMATIC);


	}
	
	int getPulse()
	{
		return pulse;
	}



	//fetch target velocity from ROS
	void updateVTarget(const std_msgs::Int32& val) {
		temp = val.data;
		vTarget = (double)temp;
	}

	void doEncoder() {
		pulse++;
		encoder = encoderPos;
		if (digitalRead(encoderPinZ) == 1) {
			encoderPos = 0;
		}
		else if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
			encoderPos++;
		}
		else {
			encoderPos--;
		}

		if (abs(encoder) > abs(encoderPos)) {
			pulse = 0;

		}
		Serial.println(encoderPos, DEC);
		
	}

	void updateSpeed() {

		vCurrent = ((encoderPos - encoder_b) / TOTAL_TICK) * 1000; //rps
		myPID.Compute();
		//Serial.println(pwm, DEC);
		analogWrite(MOTORL, pwm);
		String str_pwm = String(pwm);
		int str_pwm_length = str_pwm.length() + 1;
		char pwm_str_array[str_pwm_length];
		str_pwm.toCharArray(pwm_str_array, str_pwm_length);
		str_msg.data = pwm_str_array;
		pub_vTarget.publish(&str_msg);
	}
	
};

/*
  You also need to move the other encoder wire over to pin 3 (interrupt 1).
*/