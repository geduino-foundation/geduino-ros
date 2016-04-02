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
#include <geduino_diagnostic_msgs/StampedStatus.h>
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

// The PING))) sensors
PING leftPing(LEFT_PING_PIN);
PING centralPing(CENTRAL_PING_PIN);
PING rightPing(RIGHT_PING_PIN);

// The range publisher rate
Rate rangePublisherRate(10);

// The diagnostic publisher rate
Rate diagnosticPublisherRate(0.1);

// The main loop statistic
Loop mainLoop;

// The ROS node handle
ros::NodeHandle nodeHandle;

// The range messages and their publishers
sensor_msgs::Range leftRangeMessage;
ros::Publisher leftRangeMessagePublisher("left_range", &leftRangeMessage);
sensor_msgs::Range centerRangeMessage;
ros::Publisher centerRangeMessagePublisher("center_range", &centerRangeMessage);
sensor_msgs::Range rightRangeMessage;
ros::Publisher rightRangeMessagePublisher("right_range", &rightRangeMessage);

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
  nodeHandle.advertise(leftRangeMessagePublisher);
  nodeHandle.advertise(centerRangeMessagePublisher);
  nodeHandle.advertise(rightRangeMessagePublisher);
  nodeHandle.advertise(geduinoStatusMessagePublisher);

}

/****************************************************************************************
 * Loop
 */

void loop() {

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
  
  // Cycle performed
  mainLoop.cyclePerformed();

}

/****************************************************************************************
 * Publishing methods
 */

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
  
  // Read battery volt
  readBatteryVolt(&geduinoStatusMessage.status.power.raw_voltage);

  // Get main loop usage
  mainLoop.getUsedCounter().getCounters(&geduinoStatusMessage.status.proc_stat.used_cycles, &geduinoStatusMessage.status.proc_stat.idle_cycles);
  
  // Get rate's delay
  rangePublisherRate.getDelayCounter().getCounters(&geduinoStatusMessage.status.proc_stat.range_delay.sum, &geduinoStatusMessage.status.proc_stat.range_delay.counter);
  diagnosticPublisherRate.getDelayCounter().getCounters(&geduinoStatusMessage.status.proc_stat.diagnostic_delay.sum, &geduinoStatusMessage.status.proc_stat.diagnostic_delay.counter);
  
  // Get rate's duration
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

void readTemperature(float * temperature) {

  // Read temperature value
  *temperature = 30;

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

































