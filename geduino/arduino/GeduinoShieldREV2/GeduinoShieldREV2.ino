/*
 GeduinoShieldREV2.ino
 http://www.geduino.org
 
 Copyright (C) 2016 Alessandro Francescon

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
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "PString.h"
#include "PING.h"
#include "TMP36.h"
#include "Battery.h"
#include "Rate.h"
#include "Counter.h"
#include "Loop.h"

/****************************************************************************************
 * Begin configuration
 */

// The ROS serial baud rare
#define ROS_SERIAL_BAUD_RATE                  115200

// Connected PINs
#define LEFT_PING_PIN                         26
#define CENTRAL_PING_PIN                      24
#define RIGHT_PING_PIN                        28
#define BATTERY_VOLT_PIN                      A11
#define TMP36_PIN                             A10

// Ping sensor specification
#define PING_FIELD_OF_VIEW                    0.1745      // [rad]
#define PING_MIN_RANGE                        0.02        // [m]
#define PING_MAX_RANGE                        3.00        // [m]
#define PING_MOUNTING_GAP                     0.02        // [m]

// The battery parameters
#define BATTERY_PARAM_A                       2.805
#define BATTERY_PARAM_B                       10.232

// The battery threshold levels
#define BATTERY_WARNING_VOLTS                 13.6        // [V]
#define BATTERY_CRITICAL_VOLTS                12.4        // [V]

// The SAMx8 threshold levels
#define SAMX8_WARNING_LOAD                    10          // [%]
#define SAMX8_CRITICAL_LOAD                   50          // [%]

// The range publisher frequency
#define RANGE_PUBLISHER_FREQUENCY             2          // [Hz]

// The battery state publisher frequency
#define BATTERY_STATE_PUBLISHER_FREQUENCY     0.1           // [Hz]

// The diagnostics publisher frequency
#define DIAGNOSTICS_PUBLISHER_FREQUENCY       0.1           // [Hz]

/****************************************************************************************
 * End configuration
 */

// The PING))) sensors
PING leftPing(LEFT_PING_PIN, PING_MOUNTING_GAP);
PING centralPing(CENTRAL_PING_PIN, PING_MOUNTING_GAP);
PING rightPing(RIGHT_PING_PIN, PING_MOUNTING_GAP);

// The battery voltage sensor
Battery battery(BATTERY_VOLT_PIN, BATTERY_PARAM_A, BATTERY_PARAM_B);

// The TMP36 sensor
TMP36 tmp36(TMP36_PIN);

// The range publisher rate
Rate rangePublisherRate(RANGE_PUBLISHER_FREQUENCY);

// The battery state publisher rate
Rate batteryStatePublisherRate(BATTERY_STATE_PUBLISHER_FREQUENCY);

// The diagnostics publisher rate
Rate diagnosticsPublisherRate(DIAGNOSTICS_PUBLISHER_FREQUENCY);

// The main loop statistic
Loop mainLoop;

// The ROS node handle
ros::NodeHandle nodeHandle;

// The range messages and their publishers
sensor_msgs::Range leftRangeMessage;
ros::Publisher leftRangeMessagePublisher("left_range", & leftRangeMessage);
sensor_msgs::Range centerRangeMessage;
ros::Publisher centerRangeMessagePublisher("center_range", & centerRangeMessage);
sensor_msgs::Range rightRangeMessage;
ros::Publisher rightRangeMessagePublisher("right_range", & rightRangeMessage);

// The battery state message and its publisher
std_msgs::Float32 batteryStateMessage;
ros::Publisher batteryStateMessagePublisher("battery_state", & batteryStateMessage);

// The diagnostics message and its publisher
diagnostic_msgs::DiagnosticArray diagnosticsMessage;
diagnostic_msgs::DiagnosticStatus diagnosticStatusArray[5];
diagnostic_msgs::KeyValue leftPingValues[2];
diagnostic_msgs::KeyValue centerPingValues[2];
diagnostic_msgs::KeyValue rightPingValues[2];
diagnostic_msgs::KeyValue batteryValues[1];
diagnostic_msgs::KeyValue samx8Values[7];
ros::Publisher diagnosticsMessagePublisher("diagnostics", & diagnosticsMessage);

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

  // Set baud rate
  nodeHandle.getHardware()->setBaud(ROS_SERIAL_BAUD_RATE);
  
  // Init node handle
  nodeHandle.initNode();
  
  // Debug
  nodeHandle.logdebug("intializing messages...");
  
  // Init range messages
  leftRangeMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
  leftRangeMessage.header.frame_id = "base_left_range";
  leftRangeMessage.field_of_view = PING_FIELD_OF_VIEW;
  leftRangeMessage.min_range = PING_MIN_RANGE;
  leftRangeMessage.max_range = PING_MAX_RANGE;
  centerRangeMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
  centerRangeMessage.header.frame_id = "base_center_range";
  centerRangeMessage.field_of_view = PING_FIELD_OF_VIEW;
  centerRangeMessage.min_range = PING_MIN_RANGE;
  centerRangeMessage.max_range = PING_MAX_RANGE;
  rightRangeMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
  rightRangeMessage.header.frame_id = "base_right_range";
  rightRangeMessage.field_of_view = PING_FIELD_OF_VIEW;
  rightRangeMessage.min_range = PING_MIN_RANGE;
  rightRangeMessage.max_range = PING_MAX_RANGE;
  
  // Init diagnostics message
  diagnosticsMessage.header.frame_id = "";
  diagnosticsMessage.status_length = 5;
  diagnosticsMessage.status = diagnosticStatusArray;
  
  // The left ping diagnostic status
  diagnosticsMessage.status[0].name = "Left PING)))";
  diagnosticsMessage.status[0].hardware_id = "left-ping";
  diagnosticsMessage.status[0].values_length = 2;
  diagnosticsMessage.status[0].values = leftPingValues;
  diagnosticsMessage.status[0].values[0].key = "Failures";
  diagnosticsMessage.status[0].values[0].value = (char *) malloc(16);
  diagnosticsMessage.status[0].values[1].key = "Measurement";
  diagnosticsMessage.status[0].values[1].value = (char *) malloc(16);
  
  // The center ping diagnostic status
  diagnosticsMessage.status[1].name = "Center PING)))";
  diagnosticsMessage.status[1].hardware_id = "center-ping";
  diagnosticsMessage.status[1].values_length = 2;
  diagnosticsMessage.status[1].values = centerPingValues;
  diagnosticsMessage.status[1].values[0].key = "Failures";
  diagnosticsMessage.status[1].values[0].value = (char *) malloc(16);
  diagnosticsMessage.status[1].values[1].key = "Measurement";
  diagnosticsMessage.status[1].values[1].value = (char *) malloc(16);
  
  // The right ping diagnostic status
  diagnosticsMessage.status[2].name = "Right PING)))";
  diagnosticsMessage.status[2].hardware_id = "right-ping";
  diagnosticsMessage.status[2].values_length = 2;
  diagnosticsMessage.status[2].values = rightPingValues;
  diagnosticsMessage.status[2].values[0].key = "Failures";
  diagnosticsMessage.status[2].values[0].value = (char *) malloc(16);
  diagnosticsMessage.status[2].values[1].key = "Measurement";
  diagnosticsMessage.status[2].values[1].value = (char *) malloc(16);
 
  // The battery diagnostic status
  diagnosticsMessage.status[3].name = "Battery";
  diagnosticsMessage.status[3].hardware_id = "battery";
  diagnosticsMessage.status[3].values_length = 1;
  diagnosticsMessage.status[3].values = batteryValues;
  diagnosticsMessage.status[3].values[0].key = "Volts";
  diagnosticsMessage.status[3].values[0].value = (char *) malloc(16);

  // The SAMx8 diagnostic status
  diagnosticsMessage.status[4].name = "SAMx8";
  diagnosticsMessage.status[4].hardware_id = "samx8";
  diagnosticsMessage.status[4].values_length = 7;
  diagnosticsMessage.status[4].values = samx8Values;
  diagnosticsMessage.status[4].values[0].key = "Load";
  diagnosticsMessage.status[4].values[0].value = (char *) malloc(16);
  diagnosticsMessage.status[4].values[1].key = "Range average delay";
  diagnosticsMessage.status[4].values[1].value = (char *) malloc(16);
  diagnosticsMessage.status[4].values[2].key = "Range main duration";
  diagnosticsMessage.status[4].values[2].value = (char *) malloc(16);
  diagnosticsMessage.status[4].values[3].key = "Battery state average delay";
  diagnosticsMessage.status[4].values[3].value = (char *) malloc(16);
  diagnosticsMessage.status[4].values[4].key = "Battery state main duration";
  diagnosticsMessage.status[4].values[4].value = (char *) malloc(16);
  diagnosticsMessage.status[4].values[5].key = "Diagnostics average delay";
  diagnosticsMessage.status[4].values[5].value = (char *) malloc(16);
  diagnosticsMessage.status[4].values[6].key = "Diagnostics main duration";
  diagnosticsMessage.status[4].values[6].value = (char *) malloc(16);

  // Debug
  nodeHandle.logdebug("advertise node handle for publishers...");
  
  // Advertise node handle for publishers
  nodeHandle.advertise(leftRangeMessagePublisher);
  nodeHandle.advertise(centerRangeMessagePublisher);
  nodeHandle.advertise(rightRangeMessagePublisher);
  nodeHandle.advertise(batteryStateMessagePublisher);
  nodeHandle.advertise(diagnosticsMessagePublisher);
  
  // Debug
  nodeHandle.loginfo("initialization complete");

}

/****************************************************************************************
 * Loop
 */

void loop() {
  
  if (rangePublisherRate.ellapsed()) {
       
    // Start duration
    rangePublisherRate.start();
    
    // Publish range
    publishRange();
    
    // End duration
    rangePublisherRate.end();
    
    // Used cycle
    mainLoop.cycleUsed();
  
  }
  
  if (batteryStatePublisherRate.ellapsed()) {
       
    // Start duration
    batteryStatePublisherRate.start();
    
    // Publish battery state
    publishBatteryState();
    
    // End duration
    batteryStatePublisherRate.end();
    
    // Used cycle
    mainLoop.cycleUsed();
  
  }
  
  if (diagnosticsPublisherRate.ellapsed()) {
       
    // Start duration
    diagnosticsPublisherRate.start();
    
    // Publish dagnostics
    publishDiagnostics();
    
    // End duration
    diagnosticsPublisherRate.end();
    
    // Used cycle
    mainLoop.cycleUsed();
  
  }
  
  // Cycle performed
  mainLoop.cyclePerformed();

}

void publishRange() {
  
  // Get temperature
  float temperature;
  tmp36.getTemperature(& temperature);

  // Get ROS current time
  ros::Time now = nodeHandle.now();

  // Get measurements
  float leftPingMeasurement, centralPingMeasurement, rightPingMeasurement;
  leftPing.measure(temperature, & leftPingMeasurement);
  centralPing.measure(temperature, & centralPingMeasurement);
  rightPing.measure(temperature, & rightPingMeasurement);

  // Update range messages
  leftRangeMessage.range = leftPingMeasurement;
  leftRangeMessage.header.stamp = now;
  centerRangeMessage.range = centralPingMeasurement;
  centerRangeMessage.header.stamp = now;
  rightRangeMessage.range = rightPingMeasurement;
  rightRangeMessage.header.stamp = now;
  
  // Publish range messages
  leftRangeMessagePublisher.publish(& leftRangeMessage);
  centerRangeMessagePublisher.publish(& centerRangeMessage);
  rightRangeMessagePublisher.publish(& rightRangeMessage);
  
  // Spin once
  nodeHandle.spinOnce();

}

void publishBatteryState() {
  
  // Get battery volts
  float volts;
  battery.getVolts(& batteryStateMessage.data);
  
  // Publish battery state message
  batteryStateMessagePublisher.publish(& batteryStateMessage);
  
  // Spin once
  nodeHandle.spinOnce();

}

void getPingDiagnosticStatus(PING * ping, diagnostic_msgs::DiagnosticStatus * diagnosticStatus) {
  
   // Get ping diagnostic status
  unsigned long failures, measurement;
  ping->getFailureCounter().getCounters(& failures, & measurement);
  
  // Set ping diagnostic values
  String(failures, DEC).toCharArray(diagnosticStatus->values[0].value, 16);
  String(measurement, DEC).toCharArray(diagnosticStatus->values[1].value, 16);

  // Set ping diagnostic level and message
  if (failures == 0) {
    
    diagnosticStatus->level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnosticStatus->message = "OK";
    
  } else if (failures < measurement) {
    
    diagnosticStatus->level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnosticStatus->message = "Some measurement failed";
  
  } else {
    
    diagnosticStatus->level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnosticStatus->message = "All measurement failed";
  
  }
  
}

void publishDiagnostics() {
  
  // Get ROS current time
  ros::Time now = nodeHandle.now();
  
  // Set time on diagnstics message header
  diagnosticsMessage.header.stamp = now;

  // Get pings diagnostic status
  getPingDiagnosticStatus(& leftPing, & diagnosticsMessage.status[0]);
  getPingDiagnosticStatus(& centralPing, & diagnosticsMessage.status[1]);
  getPingDiagnosticStatus(& rightPing, & diagnosticsMessage.status[2]);

  // Get battery volts
  float volts;
  battery.getVolts(& volts);

  // Set SAMx8 diagnostic values
  PString voltsString(diagnosticsMessage.status[3].values[0].value, 16);
  voltsString.print(volts, 1);
  voltsString.print(" V");

  // Set battery diagnostic level and message
  if (volts > BATTERY_WARNING_VOLTS) {
    
    diagnosticsMessage.status[3].level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnosticsMessage.status[3].message = "OK";
    
  } else if (volts > BATTERY_CRITICAL_VOLTS) {
    
    diagnosticsMessage.status[3].level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnosticsMessage.status[3].message = "Voltage under warning level, charge battery";
  
  } else {
    
    diagnosticsMessage.status[3].level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnosticsMessage.status[3].message = "Voltage under critical level, power off immediatly to avoid battery damage";
  
  }

  // Get SAMx8 load, delays and duration
  float load, rangeDelay, rangeDuration, batteryStateDelay, batteryStateDuration, diagnosticsDelay, diagnosticsDuration;
  mainLoop.getUsedCounter().getAverage(& load);
  rangePublisherRate.getDelayCounter().getAverage(& rangeDelay);
  rangePublisherRate.getDurationCounter().getAverage(& rangeDuration);
  batteryStatePublisherRate.getDelayCounter().getAverage(& batteryStateDelay);
  batteryStatePublisherRate.getDurationCounter().getAverage(& batteryStateDuration);
  diagnosticsPublisherRate.getDelayCounter().getAverage(& diagnosticsDelay);
  diagnosticsPublisherRate.getDurationCounter().getAverage(& diagnosticsDuration);
  
  // Set SAMx8 diagnostic values
  char loadChars[16], rangeDelayChars[16], rangeDurationChars[16], batteryStateDelayChars[16], batteryStateDurationChars[16], diagnosticsDelayChars[16], diagnosticsDurationChars[16];
  PString loadString(diagnosticsMessage.status[4].values[0].value, 16);
  loadString.print(load, 2);
  loadString.print(" %");
  PString rangeDelayString(diagnosticsMessage.status[4].values[1].value, 16);
  rangeDelayString.print(rangeDelay, 2);
  rangeDelayString.print(" millis");
  PString rangeDurationString(diagnosticsMessage.status[4].values[2].value, 16);
  rangeDurationString.print(rangeDuration, 2);
  rangeDurationString.print(" millis");
  PString batteryStateDelayString(diagnosticsMessage.status[4].values[3].value, 16);
  batteryStateDelayString.print(batteryStateDelay, 2);
  batteryStateDelayString.print(" millis");
  PString batteryStateDurationString(diagnosticsMessage.status[4].values[4].value, 16);
  batteryStateDurationString.print(batteryStateDuration, 2);
  batteryStateDurationString.print(" millis");
  PString diagnosticsDelayString(diagnosticsMessage.status[4].values[5].value, 16);
  diagnosticsDelayString.print(diagnosticsDelay, 2);
  diagnosticsDelayString.print(" millis");
  PString diagnosticsDurationString(diagnosticsMessage.status[4].values[6].value, 16);
  diagnosticsDurationString.print(diagnosticsDuration, 2);
  diagnosticsDurationString.print(" millis");
  
  // Set SAMx8 diagnostic level and message
  if (load < SAMX8_WARNING_LOAD) {
    
    diagnosticsMessage.status[4].level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnosticsMessage.status[4].message = "OK";
    
  } else if (load < SAMX8_CRITICAL_LOAD) {
    
    diagnosticsMessage.status[4].level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnosticsMessage.status[4].message = "Load over warning level";
  
  } else {
    
    diagnosticsMessage.status[4].level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnosticsMessage.status[4].message = "Load over critical level";
  
  }
  
  // Publish diagnostics message
  diagnosticsMessagePublisher.publish(& diagnosticsMessage);
  
  // Spin once
  nodeHandle.spinOnce();
  
}


