#include <PID_v1.h>
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>

#define PWM 3
#define DIR 4

//Define Variables w
//e'll be connecting to
double Setpoint, Input, Output;
int temp //temporary storage used for datatype conversion

const int PIN_CS = 8;  //chip  select
const int PIN_CLOCK = 6;  //clock
const int PIN_DATA = 7;  //digital output from encoder

//Specify the links and initial tuning parameters
double Kp=3, Ki=0.3, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//update target setpoint
void updateTarget(std_msgs::Int32 &val)
{
  temp = val.data
  Setpoint = double(temp);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> theta_target("motor_vTarget", &updateTarget);
ros::Publisher encoder_pub("fwheel", &Input);


void setup() {
  nh.initNode();
  nh.subscribe(theta_target);

  Setpoint = 500;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, INPUT);

  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);

  digitalWrite(PIN_CLOCK, HIGH);
  digitalWrite(PIN_CS, LOW);
}

void loop() {
  Input = updateEncoder();
  encoder_pub.publish(&Input);

  //limit the threshold of rotation
  if(Setpoint > 540)
      Setpoint = 540;
  else if(Setpoint <410)
      Setpoint = 410;
  myPID.Compute();
  updateMotor(Output);
  nh.spinOnce();
}

int updateEncoder() {
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_CS, LOW);
  int pos = 0;
  for (int i=0; i<10; i++) {
    digitalWrite(PIN_CLOCK, LOW);
    delay(1);
    digitalWrite(PIN_CLOCK, HIGH);
    delay(1);

    pos= pos | digitalRead(PIN_DATA);

    if(i<9) pos = pos << 1;
  }
  for (int i=0; i<6; i++) {
    digitalWrite(PIN_CLOCK, LOW);
    delay(1);
    digitalWrite(PIN_CLOCK, HIGH);
    delay(1);
  }
  digitalWrite(PIN_CLOCK, LOW);
  delay(1);
  digitalWrite(PIN_CLOCK, HIGH);
  delay(1);
  return pos;
}

void updateMotor(int pwm) {
  if(pwm <= 0) {
    digitalWrite(DIR, HIGH);
  }
  else {
    digitalWrite(DIR, LOW);
  }
  analogWrite(PWM, abs(pwm));
}
