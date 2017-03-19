/*
 GeduinoShieldREV4.ino
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
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "PING.h"
#include "TMP36.h"
#include "Battery.h"
#include "Rate.h"
#include "Counter.h"
#include "Loop.h"
#include "Led.h"

/****************************************************************************************
 * Begin configuration
 */
 
// Enable/disable serial debug
//#define SERIAL_DEBUG

// The ROS serial baud rare
#define ROS_SERIAL_BAUD_RATE                          115200

// Connected PINs
#define LEFT_PING_PIN                                 8
#define CENTRAL_PING_PIN                              12
#define RIGHT_PING_PIN                                13
#define BATTERY_VOLT_PIN                              A1
#define TMP36_PIN                                     A0
#define RED_LED_PIN                                   7
#define GREEN_LED_PIN                                 4

// Ping sensor specification
#define PING_FIELD_OF_VIEW                            0.1745      // [rad]
#define PING_MIN_RANGE                                0.05        // [m]
#define PING_MAX_RANGE                                3.00        // [m]
#define PING_MOUNTING_GAP                             0.02        // [m]

// The battery parameters
#define BATTERY_PARAM_A_DEFAULT                       2.805
#define BATTERY_PARAM_B_DEFAULT                       10.232

// The battery threshold default levels
#define BATTERY_WARNING_VOLTS_DEFAULT                 13.6        // [V]
#define BATTERY_CRITICAL_VOLTS_DEFAULT                12.4        // [V]

// The MCU threshold default levels
#define MCU_WARNING_LOAD_DEFAULT                      10          // [%]
#define MCU_CRITICAL_LOAD_DEFAULT                     50          // [%]

// The range publisher default frequency
#define RANGE_PUBLISHER_FREQUENCY_DEFAULT             10          // [Hz]

// The battery state publisher default frequency
#define BATTERY_STATE_PUBLISHER_FREQUENCY_DEFAULT     1           // [Hz]

// The diagnostics publisher default frequency
#define DIAGNOSTICS_PUBLISHER_FREQUENCY_DEFAULT       1          // [Hz]

/****************************************************************************************
 * End configuration
 */

// The PING))) sensors
PING leftPing(LEFT_PING_PIN, PING_MOUNTING_GAP);
PING centralPing(CENTRAL_PING_PIN, PING_MOUNTING_GAP);
PING rightPing(RIGHT_PING_PIN, PING_MOUNTING_GAP);

// The battery voltage sensor
Battery battery(BATTERY_VOLT_PIN, BATTERY_PARAM_A_DEFAULT, BATTERY_PARAM_B_DEFAULT);

// The TMP36 sensor
TMP36 tmp36(TMP36_PIN);

// The leds
Led redLed(RED_LED_PIN);
Led greenLed(GREEN_LED_PIN);

// The range publisher rate
Rate rangePublisherRate(RANGE_PUBLISHER_FREQUENCY_DEFAULT);

// The battery state publisher rate
Rate batteryStatePublisherRate(BATTERY_STATE_PUBLISHER_FREQUENCY_DEFAULT);

// The diagnostics publisher rate
Rate diagnosticsPublisherRate(DIAGNOSTICS_PUBLISHER_FREQUENCY_DEFAULT);

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

// The temperature message and its publisher
sensor_msgs::Temperature temperatureMessage;
ros::Publisher temperatureMessagePublisher("temperature", & temperatureMessage);

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

// The battery threshold levels
float batteryWarningVolts = BATTERY_WARNING_VOLTS_DEFAULT;
float batteryCriticalVolts = BATTERY_CRITICAL_VOLTS_DEFAULT;

// The MCU threshold levels
int mcuWarningLoad = MCU_WARNING_LOAD_DEFAULT;
int mcuCriticalLoad = MCU_CRITICAL_LOAD_DEFAULT;

/****************************************************************************************
 * Setup
 */

void setup() {

  // Blink leds to inform start 
  greenLed.ledOn();
  redLed.ledBlinkFastFor(2000);
  redLed.ledOn();
  greenLed.ledOff();

#ifndef SERIAL_DEBUG

  // Set baud rate
  nodeHandle.getHardware()->setBaud(ROS_SERIAL_BAUD_RATE);
  
  // Init node handle
  nodeHandle.initNode();
  
  // Debug
  nodeHandle.logdebug("intializing messages...");
  
#else

  // Begin debug serial
  Serial.begin(ROS_SERIAL_BAUD_RATE);
  
  // Debug
  Serial.println("setup completed: starting loop...");
  
#endif
  
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
  
  // Init temperature message
  temperatureMessage.header.frame_id = "base_temperature";
  temperatureMessage.variance = 0.0016;
  
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
  diagnosticsMessage.status[0].values[0].value = "N/A";
  diagnosticsMessage.status[0].values[1].key = "Measurement";
  diagnosticsMessage.status[0].values[1].value = "N/A";
  
  // The center ping diagnostic status
  diagnosticsMessage.status[1].name = "Center PING)))";
  diagnosticsMessage.status[1].hardware_id = "center-ping";
  diagnosticsMessage.status[1].values_length = 2;
  diagnosticsMessage.status[1].values = centerPingValues;
  diagnosticsMessage.status[1].values[0].key = "Failures";
  diagnosticsMessage.status[1].values[0].value = "N/A";
  diagnosticsMessage.status[1].values[1].key = "Measurement";
  diagnosticsMessage.status[1].values[1].value = "N/A";
  
  // The right ping diagnostic status
  diagnosticsMessage.status[2].name = "Right PING)))";
  diagnosticsMessage.status[2].hardware_id = "right-ping";
  diagnosticsMessage.status[2].values_length = 2;
  diagnosticsMessage.status[2].values = rightPingValues;
  diagnosticsMessage.status[2].values[0].key = "Failures";
  diagnosticsMessage.status[2].values[0].value = "N/A";
  diagnosticsMessage.status[2].values[1].key = "Measurement";
  diagnosticsMessage.status[2].values[1].value = "N/A";
 
  // The battery diagnostic status
  diagnosticsMessage.status[3].name = "Battery";
  diagnosticsMessage.status[3].hardware_id = "battery";
  diagnosticsMessage.status[3].values_length = 1;
  diagnosticsMessage.status[3].values = batteryValues;
  diagnosticsMessage.status[3].values[0].key = "Volts";
  diagnosticsMessage.status[3].values[0].value = "N/A";

  // The MCU diagnostic status
  diagnosticsMessage.status[4].name = "MCU";
  diagnosticsMessage.status[4].hardware_id = "mcu";
  diagnosticsMessage.status[4].values_length = 7;
  diagnosticsMessage.status[4].values = samx8Values;
  diagnosticsMessage.status[4].values[0].key = "Load";
  diagnosticsMessage.status[4].values[0].value = "N/A";
  diagnosticsMessage.status[4].values[1].key = "Range average delay";
  diagnosticsMessage.status[4].values[1].value = "N/A";
  diagnosticsMessage.status[4].values[2].key = "Range main duration";
  diagnosticsMessage.status[4].values[2].value = "N/A";
  diagnosticsMessage.status[4].values[3].key = "Battery state average delay";
  diagnosticsMessage.status[4].values[3].value = "N/A";
  diagnosticsMessage.status[4].values[4].key = "Battery state main duration";
  diagnosticsMessage.status[4].values[4].value = "N/A";
  diagnosticsMessage.status[4].values[5].key = "Diagnostics average delay";
  diagnosticsMessage.status[4].values[5].value = "N/A";
  diagnosticsMessage.status[4].values[6].key = "Diagnostics main duration";
  diagnosticsMessage.status[4].values[6].value = "N/A";
  
#ifndef SERIAL_DEBUG
  
  // Advertise node handle for publishers
  nodeHandle.advertise(leftRangeMessagePublisher);
  nodeHandle.advertise(centerRangeMessagePublisher);
  nodeHandle.advertise(rightRangeMessagePublisher);
  nodeHandle.advertise(temperatureMessagePublisher);
  nodeHandle.advertise(batteryStateMessagePublisher);
  nodeHandle.advertise(diagnosticsMessagePublisher);
  
#endif

}

/****************************************************************************************
 * Loop
 */

void loop() {

  while (!nodeHandle.connected()) {

    // Blink green led to inform wating for connection
    greenLed.ledBlinkSlow();
    redLed.ledOff();

    // Spin once
    nodeHandle.spinOnce();

    // Check battery state
    checkBatteryState();

    if (nodeHandle.connected()) {

      // Update params
      updateParams();

      // Log
      nodeHandle.loginfo("Starting loop...");
      
    }
    
  }

  // Turn off red led and turn on green led
  redLed.ledOff();
  greenLed.ledOn();
  
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

#ifdef SERIAL_DEBUG

  // Debug
  Serial.println("Publish range...");
  
#endif

  // Get temperature
  float temperature;
  tmp36.getTemperature(& temperature);

  // Get ROS current time
  ros::Time now = nodeHandle.now();

  // Get measurements
  float leftPingMeasurement, centralPingMeasurement, rightPingMeasurement;
  leftPing.measure(temperature, & leftPingMeasurement, PING_MIN_RANGE, PING_MAX_RANGE);
  centralPing.measure(temperature, & centralPingMeasurement, PING_MIN_RANGE, PING_MAX_RANGE);
  rightPing.measure(temperature, & rightPingMeasurement, PING_MIN_RANGE, PING_MAX_RANGE);
  
  // Update range messages
  leftRangeMessage.range = leftPingMeasurement;
  leftRangeMessage.header.stamp = now;
  centerRangeMessage.range = centralPingMeasurement;
  centerRangeMessage.header.stamp = now;
  rightRangeMessage.range = rightPingMeasurement;
  rightRangeMessage.header.stamp = now;
  
  // Update temperature messages
  temperatureMessage.header.stamp = now;
  temperatureMessage.temperature = temperature;

#ifndef SERIAL_DEBUG

  // Publish range messages
  leftRangeMessagePublisher.publish(& leftRangeMessage);
  centerRangeMessagePublisher.publish(& centerRangeMessage);
  rightRangeMessagePublisher.publish(& rightRangeMessage);
  
  // Publish temperature message
  temperatureMessagePublisher.publish(& temperatureMessage);
  
  // Spin once
  nodeHandle.spinOnce();

#else

  // Debug
  Serial.print("Temperature [C]: ");
  Serial.print(temperature);
  Serial.print(" Left [m]: ");
  Serial.print(leftPingMeasurement);
  Serial.print(" Central [m]: ");
  Serial.print(centralPingMeasurement);
  Serial.print(" Right [m]: ");
  Serial.println(rightPingMeasurement);

#endif

}

void publishBatteryState() {

#ifdef SERIAL_DEBUG

  // Debug
  Serial.println("Publish battery state...");
  
#endif

  // Get battery volts
  float volts;
  battery.getVolts(& volts);
  
#ifndef SERIAL_DEBUG

  // Update battery state message
  batteryStateMessage.data = volts;
  
  // Publish battery state message
  batteryStateMessagePublisher.publish(& batteryStateMessage);
  
  // Spin once
  nodeHandle.spinOnce();
  
#else

  // Debug
  Serial.print("Battery Voltage [V]: ");
  Serial.println(volts);
  
#endif

}

void getPingDiagnosticStatus(PING * ping, diagnostic_msgs::DiagnosticStatus * diagnosticStatus) {
  
   // Get ping diagnostic status
  unsigned long failures, measurement;
  ping->getFailureCounter().getCounters(& failures, & measurement);
  
  // Set ping diagnostic values
  diagnosticStatus->values[0].value = String(failures, DEC).c_str();
  diagnosticStatus->values[1].value = String(measurement, DEC).c_str();

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
  
#ifdef SERIAL_DEBUG

  // Debug
  Serial.println("Publish diagnostics...");
  
#endif

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

  // Set battery diagnostic values
  diagnosticsMessage.status[3].values[0].value = String(String(volts, 1) + " V").c_str();

  // Set battery diagnostic level and message
  if (volts > batteryWarningVolts) {
    
    diagnosticsMessage.status[3].level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnosticsMessage.status[3].message = "OK";

    // Turn off red led
    redLed.ledOff();

  } else if (volts > batteryCriticalVolts) {
    
    diagnosticsMessage.status[3].level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnosticsMessage.status[3].message = "Voltage under warning level, charge battery";

    // Turn on red led
    redLed.ledOn();

#ifndef SERIAL_DEBUG

    // Log
    nodeHandle.logwarn("Voltage under warning level, charge battery");

#endif
  
  } else {
    
    diagnosticsMessage.status[3].level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnosticsMessage.status[3].message = "Voltage under critical level, power off immediatly to avoid battery damage";

    // Turn on red led
    redLed.ledOn();

#ifndef SERIAL_DEBUG

    // Log
    nodeHandle.logwarn("Voltage under critical level, power off immediatly to avoid battery damage");

#endif
    
  
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
  
  // Set Arduino diagnostic values
  diagnosticsMessage.status[4].values[0].value = String(String(load, 2) + " %").c_str();
  diagnosticsMessage.status[4].values[1].value = String(String(rangeDelay, 2) + " millis").c_str();
  diagnosticsMessage.status[4].values[2].value = String(String(rangeDuration, 2) + " millis").c_str();
  diagnosticsMessage.status[4].values[3].value = String(String(batteryStateDelay, 2) + " millis").c_str();
  diagnosticsMessage.status[4].values[4].value = String(String(batteryStateDuration, 2) + " millis").c_str();
  diagnosticsMessage.status[4].values[5].value = String(String(diagnosticsDelay, 2) + " millis").c_str();
  diagnosticsMessage.status[4].values[6].value = String(String(diagnosticsDuration, 2) + " millis").c_str();
  
  // Set MCU diagnostic level and message
  if (load < mcuWarningLoad) {
    
    diagnosticsMessage.status[4].level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnosticsMessage.status[4].message = "OK";
    
  } else if (load < mcuCriticalLoad) {
    
    diagnosticsMessage.status[4].level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnosticsMessage.status[4].message = "Load over warning level";
  
  } else {
    
    diagnosticsMessage.status[4].level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnosticsMessage.status[4].message = "Load over critical level";
  
  }
  
#ifndef SERIAL_DEBUG
  
  // Publish diagnostics message
  diagnosticsMessagePublisher.publish(& diagnosticsMessage);
  
  // Spin once
  nodeHandle.spinOnce();
  
#else

  // Debug
  Serial.println("diagnostics published");

#endif
  
}

void checkBatteryState() {

  // Get battery volts
  float volts;
  battery.getVolts(& volts);

  if (volts > batteryWarningVolts) {
    
    // Blink red led
    redLed.ledBlinkFast();

#ifdef SERIAL_DEBUG

    // Debug
    Serial.println("Voltage under warning level, charge battery");
    
#endif
    
  } else if (volts > batteryCriticalVolts) {
    
    // Turn on red led
    redLed.ledOn();

#ifdef SERIAL_DEBUG

    // Debug
    Serial.println("Voltage under critical level, power off immediatly to avoid battery damage");

#endif

  } else {
    
    // Turn off red led
    redLed.ledOn();
  
  }
  
}

void updateParams() {

  nodeHandle.getParam("~batteryWarningVolts", & batteryWarningVolts);
  nodeHandle.getParam("~batteryCriticalVolts", & batteryCriticalVolts);
  
  nodeHandle.getParam("~mcuWarningLoad", & mcuWarningLoad);
  nodeHandle.getParam("~mcuCriticalLoad", & mcuCriticalLoad);

  float rangePublisherFrequency;
  if (nodeHandle.getParam("~rangePublisherFrequency", & rangePublisherFrequency)) {
    rangePublisherRate.setFrequency(rangePublisherFrequency);
  }

  float batteryStatePublisherFrequency;
  if (nodeHandle.getParam("~batteryStatePublisherFrequency", & batteryStatePublisherFrequency)) {
    batteryStatePublisherRate.setFrequency(batteryStatePublisherFrequency);
  }

  float diagnosticsPublisherFrequency;
  if (nodeHandle.getParam("~diagnosticsPublisherFrequency", & diagnosticsPublisherFrequency)) {
    diagnosticsPublisherRate.setFrequency(diagnosticsPublisherFrequency);
  }

  float batteryParamA, batteryParamB;
  if (nodeHandle.getParam("~batteryParamA", & batteryParamA) &&
      nodeHandle.getParam("~batteryParamB", & batteryParamB)) {
    battery.setParams(batteryParamA, batteryParamB);
  }
  
  nodeHandle.loginfo("Parameter updated");
  
}

