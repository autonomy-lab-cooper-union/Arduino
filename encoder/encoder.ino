#include <PID_v1.h>
//input:velocity from
//output:pwm to motor controller
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

#define encoder0PinA  2
#define encoder0PinB  4
#define encoder0PinZ  5
#define TOTAL_TICK 6800
#define MOTORL 9

#define KP 10
#define KI 0
#define KD 0

//output conencts to potentiometer of the motor controller's arduino?

int temp; // for temporal datatype conversion
double vCurrent, pwm, vTarget;

unsigned volatile short pulse;

PID myPID (&vCurrent, &pwm, &vTarget, KP, KI, KD, DIRECT);

volatile int encoder0Pos = 0;
int encoder0; //encoder0Pos before for testing CCW OR CW
int encoder_b; //mark the encoder0Pos when pulses change

//fetch target velocity from ROS
void updateVTarget(const std_msgs::Int32 &val) {
  temp = val.data;
  vTarget = (double)temp;  
} 

std_msgs::String str_msg;

//"" is the topic name, pc...--name of subscriber
ros::Subscriber<std_msgs::Int32> pc_vTarget("motor_vTarget", &updateVTarget);
ros::Publisher pub_vTarget("pub_vTarget", &str_msg);

void setup() {
  nh.initNode();  
  nh.subscribe(pc_vTarget);
  nh.advertise(pub_vTarget);
  
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinZ, INPUT);
  digitalWrite(encoder0PinZ, HIGH);       // turn on pull-up resistor
  pinMode(MOTORL, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(2), doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(digitalPinToInterrupt(5), doEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(13), rpm, CHANGE); //for pid output                
  
  //accept setPoint value from commands of PC
  //will it get update again if put in the setup() function?
  myPID.SetMode(AUTOMATIC);

  
}

void loop() {
  pulse++;
  Serial.println (encoder0Pos, DEC);
  
  if (pulse % 1000 == 0)
  {
    nh.spinOnce();
    updateSpeed();
    encoder_b=encoder0Pos;
  }
}

void doEncoder() {
  
  encoder0 = encoder0Pos;
  if (digitalRead(encoder0PinZ)==1) {
    encoder0Pos=0;
  }else if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)){
    encoder0Pos++;
  }else {
    encoder0Pos--;
  } 
  
  if (abs(encoder0)>abs(encoder0Pos)){
    pulse=0;
    
  }
}

void updateSpeed(){
  
    vCurrent = ((encoder0Pos-encoder_b)/TOTAL_TICK)*1000; //rps
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



/*
  You also need to move the other encoder wire over to pin 3 (interrupt 1).
*/
