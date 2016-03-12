/*
 GeduinoV3.ino
 http://geduino.blogspot.it/

 Copyright (C) 2014 Alessandro Francescon

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros.h>
#include <sensor_msgs/Range.h>
#include <md25_msgs/StampedEncodersWithSpeeds.h>
#include <md25_msgs/Speeds.h>
#include <md25_msgs/StampedStatus.h>
#include <mpu9150_msgs/StampedMotion9.h>
#include <mpu9150_msgs/StampedStatus.h>
#include <geduino_diagnostic_msgs/StampedStatus.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "PING.h"
#include "Rate.h"
#include "Counter.h"
#include "Loop.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// Ros parameter
#define ROS_SERIAL_BAUD_RATE 115200

// Ping sensor specification
#define PING_FIELD_OF_VIEW 0.1745
#define PING_MIN_RANGE 0.02
#define PING_MAX_RANGE 3.00

// Geduino connection parameter
#define LEFT_PING_PIN 26
#define CENTRAL_PING_PIN 24
#define RIGHT_PING_PIN 28
#define BATTERY_VOLT_PIN A11
#define MPU6050_INTERRUPT_PIN 22

// The PING))) sensors
PING leftPing(LEFT_PING_PIN);
PING centralPing(CENTRAL_PING_PIN);
PING rightPing(RIGHT_PING_PIN);

// The MPU6050
MPU6050 mpu6050;

// The MPU6050 expected DMP packet size
uint16_t mpu6050dmpPacketSize;

// The MPU6050 DMP FIFO buffer
uint8_t mpu6050dmpFifoBuffer[64];

// The MPU6050 DMP initializiation result
bool mpu6050dmpInitialized = true;

// The MPU9150 FIFO overflow counter
Counter mpu9150FifoOverflowCounter;

// The motion9 publisher rate
Rate motion9PublisherRate(10);

// The range publisher rate
Rate rangePublisherRate(10);

// The diagnostic publisher rate
Rate diagnosticPublisherRate(0.1);

// The main loop statistic
Loop mainLoop;

// The ROS node handle
ros::NodeHandle nodeHandle;

// The motion9 message and its publisher
mpu9150_msgs::StampedMotion9 motion9Message;
ros::Publisher motion9MessagePublisher("mpu9150/motion9", &motion9Message);

// The range messages and their publishers
sensor_msgs::Range leftRangeMessage;
ros::Publisher leftRangeMessagePublisher("left_range", &leftRangeMessage);
sensor_msgs::Range centerRangeMessage;
ros::Publisher centerRangeMessagePublisher("center_range", &centerRangeMessage);
sensor_msgs::Range rightRangeMessage;
ros::Publisher rightRangeMessagePublisher("right_range", &rightRangeMessage);

// The mpu9050 status message and its publisher
mpu9150_msgs::StampedStatus mpu9150StatusMessage;
ros::Publisher mpu9150StatusMessagePublisher("mpu9150/status", &mpu9150StatusMessage);

// The geduino status message and its publisher
geduino_diagnostic_msgs::StampedStatus geduinoStatusMessage;
ros::Publisher geduinoStatusMessagePublisher("geduino/status", &geduinoStatusMessage);

/****************************************************************************************
 * Setup
 */

void setup() {
  
  // Set pin 38 to input since GPIO 54 is handled by iMX.6
  pinMode(38, INPUT);

  // Set pin 13 to input since GPIO 40 is handled by iMX.6
  pinMode(13, INPUT);
  
  // Set pins 47 and 53 to input since UART3 is handled by iMX.6
  pinMode(47, INPUT);
  pinMode(53, INPUT);
  
  // Set pins 48 and 49 to input since UART5 is handled by iMX.6
  pinMode(48, INPUT);
  pinMode(49, INPUT);
  
  // Begin serial communication for debugging
  Serial.begin(115200);
  
  // Debug
  Serial.println("geduino started");
  Serial.println("ending debug serial comminication");
  
  // End serial
  Serial.end();

  // Set up ROS
  setupROS();

  // Setup MPU6050
  setupMPU6050();

  // Debug
  nodeHandle.loginfo("geduino is ready!");

}

void setupROS() {

  // Set baud rate
  nodeHandle.getHardware()->setBaud(ROS_SERIAL_BAUD_RATE);

  // Init node handle
  nodeHandle.initNode();

  // Debug
  nodeHandle.logdebug("intialize messages...");

  // Init motion9 message
  motion9Message.header.frame_id = "0";
  motion9Message.motion9.full_scale_accel_range = motion9Message.motion9.ACCEL_FS_4; // Should be ACCEL_FS_2 but using the current offset result in ACCEL_FS_4 used instead
  motion9Message.motion9.full_scale_gyro_range = motion9Message.motion9.GYRO_FS_2000;

  // Init range messages
  leftRangeMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
  leftRangeMessage.header.frame_id =  "base_left_range";
  leftRangeMessage.field_of_view = PING_FIELD_OF_VIEW;
  leftRangeMessage.min_range = PING_MIN_RANGE;
  leftRangeMessage.max_range = PING_MAX_RANGE;
  centerRangeMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
  centerRangeMessage.header.frame_id =  "base_center_range";
  centerRangeMessage.field_of_view = PING_FIELD_OF_VIEW;
  centerRangeMessage.min_range = PING_MIN_RANGE;
  centerRangeMessage.max_range = PING_MAX_RANGE;
  rightRangeMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
  rightRangeMessage.header.frame_id =  "base_right_range";
  rightRangeMessage.field_of_view = PING_FIELD_OF_VIEW;
  rightRangeMessage.min_range = PING_MIN_RANGE;
  rightRangeMessage.max_range = PING_MAX_RANGE;

  // Init stamped diagnostic message
  geduinoStatusMessage.status.power.type = geduinoStatusMessage.status.power.TYPE_LIPO_4S;

  // Debug
  nodeHandle.logdebug("advertise node handle for publishers...");

  // Advertise node handle for publishers
  nodeHandle.advertise(motion9MessagePublisher);
  nodeHandle.advertise(leftRangeMessagePublisher);
  nodeHandle.advertise(centerRangeMessagePublisher);
  nodeHandle.advertise(rightRangeMessagePublisher);
  nodeHandle.advertise(mpu9150StatusMessagePublisher);
  nodeHandle.advertise(geduinoStatusMessagePublisher);

}

void setupMPU6050() {

  // Join I2C bus
  Wire.begin();

  // Debug
  nodeHandle.logdebug("initializing MPU6050...");

  // Initialize MPU6050
  mpu6050.initialize();

  // Test connection
  mpu6050dmpInitialized = mpu6050dmpInitialized && mpu6050.testConnection();

  if (mpu6050dmpInitialized) {

    // Initialize DMP
    mpu6050dmpInitialized = mpu6050dmpInitialized && (mpu6050.dmpInitialize() == 0);

    if (mpu6050dmpInitialized) {

      // Set accel and gyro offset
      mpu6050.setXAccelOffset(1014);
      mpu6050.setYAccelOffset(-454);
      mpu6050.setZAccelOffset(269);
      mpu6050.setXGyroOffset(91);
      mpu6050.setYGyroOffset(64);
      mpu6050.setZGyroOffset(-1);

      // Enable temperature sensor
      mpu6050.setTempSensorEnabled(true);

      // Enabling DMP
      mpu6050.setDMPEnabled(true);

      // Get DMP packet size
      mpu6050dmpPacketSize = mpu6050.dmpGetFIFOPacketSize();

      // Debug
      nodeHandle.logdebug("MPU6050 and DMP initialized");

    } else {

      // Debug
      nodeHandle.logerror("DMP initialization failed");

    }

  } else {

    // Debug
    nodeHandle.logerror("MPU6050 connection failed");

  }

}

/****************************************************************************************
 * Loop
 */

void loop() {
  
  if (motion9PublisherRate.ellapsed()) {

    // Start duration
    motion9PublisherRate.start();
    
    // Publish motion9
    publishMotion9();
    
    // End duration
    motion9PublisherRate.end();

    // Used cycle
    mainLoop.cycleUsed();
    
  }

  if (rangePublisherRate.ellapsed()) {

     // Start duration
    rangePublisherRate.start();
    
    // Publish range data
    publishRange();
    
    // End duration
    rangePublisherRate.end();
    
    // Used cycle
    mainLoop.cycleUsed();

  }

  if (diagnosticPublisherRate.ellapsed()) {

     // Start duration
    diagnosticPublisherRate.start();
    
    // Publish diagnostic
    publishDiagnostic();
    
    // End duration
    diagnosticPublisherRate.end();
    
    // Used cycle
    mainLoop.cycleUsed();

  }

  if (mpu6050dmpInitialized) {

    // Read MPU6050 DMP FIFO
    readMPU6050dmpFifo();

  }
  
  // Cycle performed
  mainLoop.cyclePerformed();

}

/****************************************************************************************
 * Publishing methods
 */

void publishMotion9() {

  // Get ROS current time
  ros::Time now = nodeHandle.now();

  if (mpu6050dmpInitialized) {

    // Set header stamp
    motion9Message.header.stamp = now;

    Quaternion orientation;
    VectorInt16 acceleration;
    VectorInt16 angularVelocity;

    // Get orientation, acceleration and angular velocity
    mpu6050.dmpGetQuaternion(&orientation, mpu6050dmpFifoBuffer);
    mpu6050.dmpGetAccel(&acceleration, mpu6050dmpFifoBuffer);
    mpu6050.dmpGetGyro(&angularVelocity, mpu6050dmpFifoBuffer);

    // Set orientation, acceleration and angular velocity
    motion9Message.motion9.orientation.x = orientation.x;
    motion9Message.motion9.orientation.y = orientation.y;
    motion9Message.motion9.orientation.z = orientation.z;
    motion9Message.motion9.orientation.w = orientation.w;
    motion9Message.motion9.accel.x = acceleration.x;
    motion9Message.motion9.accel.y = acceleration.y;
    motion9Message.motion9.accel.z = acceleration.z;
    motion9Message.motion9.gyro.x = angularVelocity.x;
    motion9Message.motion9.gyro.y = angularVelocity.y;
    motion9Message.motion9.gyro.z = angularVelocity.z;

    // Publish motion9 message
    motion9MessagePublisher.publish(&motion9Message);

  }

  // Spin once
  nodeHandle.spinOnce();

}

void publishRange() {

  float leftMeasure, centralMeasure, rightMeasure, temperature;

  // Get ROS current time
  ros::Time now = nodeHandle.now();

  // Read temperature (needed for more accuracy on ultrasonic range measure)
  readTemperature(&temperature);

  // Read ping sensor
  readPingSensor(temperature, &leftMeasure, &centralMeasure, &rightMeasure);

  // Update range messages
  leftRangeMessage.range = leftMeasure;
  leftRangeMessage.header.stamp = now;
  centerRangeMessage.range = centralMeasure;
  centerRangeMessage.header.stamp = now;
  rightRangeMessage.range = rightMeasure;
  rightRangeMessage.header.stamp = now;

  // Publish range messages
  leftRangeMessagePublisher.publish(&leftRangeMessage);
  centerRangeMessagePublisher.publish(&centerRangeMessage);
  rightRangeMessagePublisher.publish(&rightRangeMessage);

  // Spin once
  nodeHandle.spinOnce();

}

void publishDiagnostic() {

  // Get ROS current time
  ros::Time now = nodeHandle.now();

  // Read temperature, mpu9150 initialized and fifo overflow counters
  mpu9150StatusMessage.status.initialized = mpu6050dmpInitialized;
  mpu9150FifoOverflowCounter.getCounters(&mpu9150StatusMessage.status.fifo_overflows.sum, &mpu9150StatusMessage.status.fifo_overflows.counter);
  readTemperature(&mpu9150StatusMessage.status.temperature);
  
  // Set stamp
  mpu9150StatusMessage.header.stamp = now;

  // Publish mpu9150 status message
  mpu9150StatusMessagePublisher.publish(&mpu9150StatusMessage);
  
  // Read battery volt
  readBatteryVolt(&geduinoStatusMessage.status.power.raw_voltage);

  // Get main loop usage
  mainLoop.getUsedCounter().getCounters(&geduinoStatusMessage.status.proc_stat.used_cycles, &geduinoStatusMessage.status.proc_stat.idle_cycles);
  
  // Get rate's delay
  motion9PublisherRate.getDelayCounter().getCounters(&geduinoStatusMessage.status.proc_stat.encoders_motion9_delay.sum, &geduinoStatusMessage.status.proc_stat.encoders_motion9_delay.counter);
  rangePublisherRate.getDelayCounter().getCounters(&geduinoStatusMessage.status.proc_stat.range_delay.sum, &geduinoStatusMessage.status.proc_stat.range_delay.counter);
  diagnosticPublisherRate.getDelayCounter().getCounters(&geduinoStatusMessage.status.proc_stat.diagnostic_delay.sum, &geduinoStatusMessage.status.proc_stat.diagnostic_delay.counter);
  
  // Get rate's duration
  motion9PublisherRate.getDurationCounter().getCounters(&geduinoStatusMessage.status.proc_stat.encoders_motion9_duration.sum, &geduinoStatusMessage.status.proc_stat.encoders_motion9_duration.counter);
  rangePublisherRate.getDurationCounter().getCounters(&geduinoStatusMessage.status.proc_stat.range_duration.sum, &geduinoStatusMessage.status.proc_stat.range_duration.counter);
  diagnosticPublisherRate.getDurationCounter().getCounters(&geduinoStatusMessage.status.proc_stat.diagnostic_duration.sum, &geduinoStatusMessage.status.proc_stat.diagnostic_duration.counter);
  
  // Get ping sensors failure
  leftPing.getFailureCounter().getCounters(&geduinoStatusMessage.status.sensor_stat.left_ping_failures.sum, &geduinoStatusMessage.status.sensor_stat.left_ping_failures.counter);
  centralPing.getFailureCounter().getCounters(&geduinoStatusMessage.status.sensor_stat.center_ping_failures.sum, &geduinoStatusMessage.status.sensor_stat.center_ping_failures.counter);
  rightPing.getFailureCounter().getCounters(&geduinoStatusMessage.status.sensor_stat.right_ping_failures.sum, &geduinoStatusMessage.status.sensor_stat.right_ping_failures.counter);
  
  // Set stamp
  geduinoStatusMessage.header.stamp = now;

  // Publish stamped diagnostic message
  geduinoStatusMessagePublisher.publish(&geduinoStatusMessage);

  // Spin once
  nodeHandle.spinOnce();

}

/****************************************************************************************
 * Other methods
 */

void readMPU6050dmpFifo() {

  // Get MPU6050 DMP fifo count
  uint16_t mpu6050dmpFifoCount = mpu6050.getFIFOCount();

  if (mpu6050dmpFifoCount >= mpu6050dmpPacketSize) {

    // Get MPU6050 DMP interrupt status
    uint8_t mpu6050dmpInterruptStatus = mpu6050.getIntStatus();

    // Check for overflow
    if (mpu6050dmpInterruptStatus & 0x10 || mpu6050dmpFifoCount == 1024) {

      // Debug
      nodeHandle.logwarn("MPU6050 DMP FIFO overflow!");

      // Reset DMP fifo
      mpu6050.resetFIFO();
      
      // Increase fifo overflow counter by one
      mpu9150FifoOverflowCounter.increase(1);

    } else if (mpu6050dmpInterruptStatus & 0x02) {

      // Wait for correct available data length, should be a VERY short wait
      while (mpu6050.getFIFOCount() < mpu6050dmpPacketSize);

      // Read a packet from MPU6050 DMP FIFO
      mpu6050.getFIFOBytes(mpu6050dmpFifoBuffer, mpu6050dmpPacketSize);
      
      // Increase fifo overflow counter by zero, i.e. no overflow
      mpu9150FifoOverflowCounter.increase(0);

    }

  }

}

void readTemperature(float * temperature) {

  // Read temperature value
  float temperatureValue = mpu6050.getTemperature() / 340.0 + 36.53;

  // Set pointers
  *temperature = temperatureValue;

}

void readPingSensor(float temperature, float * leftMeasure, float * centralMeasure, float * rightMeasure) {

  // Make ping sensors measure and convert cm to m
  float leftMeasureValue = leftPing.measure(temperature);
  float centralMeasureValue = centralPing.measure(temperature);
  float rightMeasureValue = rightPing.measure(temperature);

  // Set pointers
  *leftMeasure = leftMeasureValue;
  *centralMeasure = centralMeasureValue;
  *rightMeasure = rightMeasureValue;

}

void readBatteryVolt(uint16_t * batteryVolt) {

  int rawBatteryVolt[3];
  
  for (int index = 0; index < 3; index++) {
    
    // Read raw battery volt from analog pin
    rawBatteryVolt[index] = analogRead(BATTERY_VOLT_PIN);
  
    // Short delay
    delay(50);
  
  }
 
  // Get average raw battery volt
  *batteryVolt = (rawBatteryVolt[0] + rawBatteryVolt[1] + rawBatteryVolt[2]) / 3;

}

































