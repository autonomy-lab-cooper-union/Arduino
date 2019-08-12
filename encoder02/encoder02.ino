
#include <PID_v1.h>
//input:velocity from computer sent through ros
//output:pwm to motor controller
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

//*** Below are the pin configurations for both encoders. A is Green wire, B is White wire, Z is Yellow wire, Red and Black are 5V VCC and GND.
#define encoder0PinA 2
#define encoder0PinB 3
#define encoder0PinZ 18
#define encoder1PinA 19
#define encoder1PinB 20
#define encoder1PinZ 21
#define TOTAL_TICK 6800
#define MOTORL 9

#define KP 3
#define KI 0
#define KD 0

//output conencts to potentiometer of the motor controller's arduino?

int temp; // for temporal datatype conversion
double vCurrent, pwm, vTarget;

unsigned volatile short pulse0;
unsigned volatile short pulse1;

PID myPID (&vCurrent, &pwm, &vTarget, KP, KI, KD, DIRECT);

volatile int encoder0Pos = 0;
int encoder0; //encoder0Pos before for testing CCW OR CW
int encoder0_b; //mark the encoder0Pos when pulses change

volatile int encoder1Pos = 0;
int encoder1; //encoder1Pos before for testing CCW OR CW
int encoder1_b; //mark the encoder0Pos when pulse changes (this means that a revolution for the rear wheel has passed)

//fetch target velocity from ROS
void updateVTarget(const std_msgs::Int32 &val) {
  temp = val.data;
  vTarget = (double)temp;  
} 

std_msgs::String str_msg;
int left_encoder;
int right_encoder;

//"" is the topic name, pc...--name of subscriber
ros::Subscriber<std_msgs::Int32> pc_vTarget("motor_vTarget", &updateVTarget);
ros::Publisher lwheel("lwheel", left_encoder);
ros::Publisher rwheel("rwheel", right_encoder);

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
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);       // turn on pull-up resistor
  pinMode(encoder1PinZ, INPUT);
  digitalWrite(encoder1PinZ, HIGH);       // turn on pull-up resistor
  pinMode(MOTORL, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA(0), CHANGE);  // Output channel A from encoder 0 -> interrupt pin 2
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB(0), CHANGE);  // Output channel B from encoder 0 -> interrupt pin 3
  attachInterrupt(digitalPinToInterrupt(encoder0PinZ), doEncoderC(0), RISING); // Output channel C from encoder 0 -> interrupt pin 18       
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderA(1), CHANGE);  // Output channel A from encoder 1 -> interrupt pin 2
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoderB(1), CHANGE);  // Output channel B from encoder 1 -> interrupt pin 3
  attachInterrupt(digitalPinToInterrupt(encoder1PinZ), doEncoderC(1), RISING); // Output channel C from encoder 1 -> interrupt pin 18          
  
  //accept setPoint value from commands of PC
  //will it get update again if put in the setup() function?
  myPID.SetMode(AUTOMATIC);

  
}

void loop() {
  Serial.print("Encoder 0: ");
  Serial.println (encoder0Pos, DEC);
  Serial.print("Encoder 1: ");
  Serial.println (encoder1Pos, DEC);
  left_encoder = encoder0Pos;
  right_encoder = encoder1Pos;
  lwheel.publish(&left_encoder);
  rwheel.publish(&right_encoder);
  if (pulse0 % 1000 == 0)
  {
    nh.spinOnce();
    updateSpeed();
    encoder0_b = encoder0Pos;
    encoder1_b = encoder1Pos;
  }

  //I commented this part out because it's redudant--two updateSpeed function calls at almost the same time. Instead just use the pulse for encoder0
  /*if (pulse1 % 1000 == 0)
  {
    nh.spinOnce();
    updateSpeed();
    encoder1_b = encoder1Pos;
  }*/
}

void doEncoderA(bool encoderNum) {
  if (encoderNum == 0)
  { 
    encoder0 = encoder0Pos;
    if (digitalRead(encoder0PinA) == HIGH){
      pulse0++;
    }
    if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)){
      encoder0Pos--;
    }
    else { 
      encoder0Pos++;
    }
    if (abs(encoder0) > abs(encoder0Pos)){
      pulse0 = 0;
    }
  }
  else
  {
    encoder1 = encoder1Pos;
    if (digitalRead(encoder1PinA) == HIGH){
      pulse1++;
    }
    if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)){
      encoder1Pos--;
    }
    else { 
      encoder1Pos++;
    }
    if (abs(encoder1) > abs(encoder1Pos)){
      pulse1 = 0;
    }
  }
}

void doEncoderB(bool encoderNum) {
  if (encoderNum == 0){
    encoder0 = encoder0Pos;
    if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)){
      encoderPos++;
    }
    else {
      encoder0Pos--;
    }
    if (abs(encoder0) > abs(encoder0Pos)){
      pulse0 = 0;
    }  
  }
  else{
    encoder1 = encoder1Pos;
    if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)){
      encoderPos++;
    }
    else {
      encoder1Pos--;
    }
    if (abs(encoder1) > abs(encoder1Pos)){
      pulse1 = 0;
    }  
  }

}

void doEncoderC(bool encoderNum) {
  if (encoderNum == 0){
    encoder0Pos = 0;
  }
  else{
    encoder1Pos = 0;
  }
}

/*void doEncoder() {
  
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
}*/

void updateSpeed(){
  
    vCurrent = ((encoder0Pos-encoder0_b)+(encoder1Pos - encoder1_b)/2*TOTAL_TICK)*1000; //rps, add the two velocities and divide them by 2 (average)
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
