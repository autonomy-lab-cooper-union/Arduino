#include <ros.h>
#include <sensor_msgs/Imu.h>
#include "SparkFunLSM6DS3.h"
#include <Arduino.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf/tf.h>


ros::NodeHandle n;
//ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("IMU", 50);
sensor_msgs::Imu imu_raw;
ros::Publisher imu_pub("IMU", &imu_raw);

LSM6DS3 myIMU(SPI_MODE, 10);

void setup()
{
  n.initNode();
  n.advertise(imu_pub);
  myIMU.begin();
  
}
  

void loop()
{
  imu_raw.header.stamp = n.now();
  imu_raw.header.frame_id = "base_link";
  imu_raw.header.seq = 0;

  imu_raw.linear_acceleration.x = float(myIMU.readFloatAccelX()); // acc_fact
  imu_raw.linear_acceleration.y = float(myIMU.readFloatAccelY()); // acc_fact
  imu_raw.linear_acceleration.z = float(myIMU.readFloatAccelZ()); // acc_fact
  
  
  imu_raw.angular_velocity.x = float(myIMU.readFloatGyroX()); // gyr_fact
  imu_raw.angular_velocity.y = float(myIMU.readFloatGyroY()); // gyr_fact
  imu_raw.angular_velocity.z = float(myIMU.readFloatGyroZ());// gyr_fact

  imu_pub.publish(&imu_raw);
  
  n.spinOnce();
  delay(10);

}
