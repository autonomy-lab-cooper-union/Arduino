#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>


#define PIN_CS 8  //chip  select
#define PIN_CLOCK 6  //clock
#define PIN_DATA 7  //digital output from encoder

#define MID_POINT 461

ros::NodeHandle nh;

std_msgs::Int32 tick;
int absencoderPos; //Numbers of ticks from absolute encoder

ros::Publisher fwheel_tick("fwheel_tick", &tick);


void setup() {
  nh.initNode();
  nh.advertise(fwheel_tick);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, INPUT);
  digitalWrite(PIN_CLOCK, HIGH);
  digitalWrite(PIN_CS, LOW);
}

void loop() {
  absencoderPos = 0;
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_CS, LOW);
  for (int i=0; i<10; i++) {
    digitalWrite(PIN_CLOCK, LOW);
    delay(1);
    digitalWrite(PIN_CLOCK, HIGH);
    delay(1);

    absencoderPos = absencoderPos | digitalRead(PIN_DATA);

    if(i < 9) absencoderPos = absencoderPos << 1;
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

  absencoderPos -= MID_POINT;
  absencoderPos = -absencoderPos;
  tick.data = absencoderPos;
  fwheel_tick.publish(&tick);
  nh.spinOnce();
}
