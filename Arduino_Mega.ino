#include <encoder.h>

//#include <ros.h>
#include <sensor_msgs/Imu.h>
#include "SparkFunLSM6DS3.h"
//#include <Arduino.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Int8.h>
//#include <std_msgs/String.h>
#include <tf/tf.h>


//#define PI 3.1415
#define MOTORL 4
#define MOTORR 6
#define PINA_L 2
#define PINB_L 3
#define PINZ_L 18
#define PINA_R 19
#define PINB_R 20
#define PINZ_R 21

ros::NodeHandle nhIMU;
ros::NodeHandle nhEncoder;

sensor_msgs::Imu imu_raw;
ros::Publisher imu_pub("IMU", &imu_raw);

LSM6DS3 myIMU(SPI_MODE, 10);

void IMU_Setup();
void IMU_loop();
Arduino_Encoder EncoderL(MOTORL, PINA_L, PINB_L, PINZ_L);
Arduino_Encoder EncoderR(MOTORR, PINA_R, PINB_R, PINZ_R);

void doEncoderL() {
  EncoderL.pulse++;
  EncoderL.encoder = EncoderL.encoderPos;
  if (digitalRead(EncoderL.encoderPinZ) == 1) {
    EncoderL.encoderPos = 0;
  }
  else if (digitalRead(EncoderL.encoderPinA) == digitalRead(EncoderL.encoderPinB)) {
    EncoderL.encoderPos++;
  }
  else {
    EncoderL.encoderPos--;
  }

  if (abs(EncoderL.encoder) > abs(EncoderL.encoderPos)) {
    EncoderL.pulse = 0;
  }
  Serial.println(EncoderL.encoderPos, DEC);
}

void doEncoderR() {
  EncoderR.pulse++;
  EncoderR.encoder = EncoderR.encoderPos;
  if (digitalRead(EncoderR.encoderPinZ) == 1) {
    EncoderR.encoderPos = 0;
  }
  else if (digitalRead(EncoderR.encoderPinA) == digitalRead(EncoderR.encoderPinB)) {
    EncoderR.encoderPos++;
  }
  else {
    EncoderR.encoderPos--;
  }

  if (abs(EncoderR.encoder) > abs(EncoderR.encoderPos)) {
    EncoderR.pulse = 0;
  }
  Serial.println(EncoderR.encoderPos, DEC);
}

void setup()
{
  IMU_Setup();
  
  nhIMU.initNode();
  nhEncoder.initNode();

  attachInterrupt(digitalPinToInterrupt(EncoderL.encoderPinA), doEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderL.encoderPinZ), doEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderR.encoderPinA), doEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderR.encoderPinZ), doEncoderR, CHANGE);
}

void loop() 
{
  IMU_loop();
  if (EncoderL.getPulse() % 1000 == 0)
  {
    nhEncoder.spinOnce();
    EncoderL.updateSpeed();
    EncoderL.encoder_b = EncoderL.encoderPos;

  }
  if (EncoderR.getPulse() % 1000 == 0)
  {
    nhEncoder.spinOnce();
    EncoderR.updateSpeed();
    EncoderR.encoder_b = EncoderR.encoderPos;

  }  
}





void IMU_Setup()
{
  nhIMU.initNode();
  nhIMU.advertise(imu_pub);
  myIMU.begin();

}

void IMU_loop()
{
  imu_raw.header.frame_id = "imu";
  imu_raw.header.seq = 0;

  imu_raw.linear_acceleration.x = float(myIMU.readFloatAccelX());
  imu_raw.linear_acceleration.y = float(myIMU.readFloatAccelY());
  imu_raw.linear_acceleration.z = float(myIMU.readFloatAccelZ());


  imu_raw.angular_velocity.x = float(myIMU.readFloatGyroX()) - 3.451572;
  imu_raw.angular_velocity.y = float(myIMU.readFloatGyroY()) + 9.933638;
  imu_raw.angular_velocity.z = float(myIMU.readFloatGyroZ()) + 3.680171;

  imu_raw.angular_velocity.x = imu_raw.angular_velocity.x * PI / 180;
  imu_raw.angular_velocity.y = imu_raw.angular_velocity.y * PI / 180;
  imu_raw.angular_velocity.z = imu_raw.angular_velocity.z * PI / 180;

  imu_pub.publish(&imu_raw);

  if (millis() % 10 == 0)
  {
    nhIMU.spinOnce();
  }
}
