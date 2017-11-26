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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>
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

// The ROS specification
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

// The IMU specification
#define IMU_GYRO_RANGE                                125         // [degree/s]
#define IMU_ACCELEROMETER_RANGE                       2           // [g]
#define IMU_RATE                                      50          // [Hz]
#define IMU_GYRO_ANGULAR_VELOCITY_COVARIANCE          0.0000009
#define IMU_ACCELEROMETER_LINEAR_ACCELERATION_COVARIANCE 0.000038

// The battery parameters
#define BATTERY_PARAM_A_DEFAULT                       2.29
#define BATTERY_PARAM_B_DEFAULT                       12.30

// The battery threshold default levels
#define BATTERY_WARNING_VOLTS_DEFAULT                 14.8        // [V]
#define BATTERY_CRITICAL_VOLTS_DEFAULT                14.0        // [V]

// The MCU threshold default levels
#define MCU_WARNING_LOAD_DEFAULT                      10          // [%]
#define MCU_CRITICAL_LOAD_DEFAULT                     50          // [%]

// The range publisher default frequency
#define RANGE_PUBLISHER_FREQUENCY_DEFAULT             10          // [Hz]

// The imu publisher default frequency
#define IMU_PUBLISHER_FREQUENCY                       30          // [Hz]

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

// The IMU filter
Madgwick imuFilter;

// The battery voltage sensor
Battery battery(BATTERY_VOLT_PIN, BATTERY_PARAM_A_DEFAULT, BATTERY_PARAM_B_DEFAULT);

// The TMP36 sensor
TMP36 tmp36(TMP36_PIN);

// The leds
Led redLed(RED_LED_PIN);
Led greenLed(GREEN_LED_PIN);

// The range publisher rate
Rate rangePublisherRate(RANGE_PUBLISHER_FREQUENCY_DEFAULT);

// The IMU publisher rate
Rate imuPublisherRate(IMU_PUBLISHER_FREQUENCY);

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

// The IMU message and its publisher
sensor_msgs::Imu imuMessage;
ros::Publisher imuMessagePublisher("imu", & imuMessage);

// The temperature message and its publisher
sensor_msgs::Temperature temperatureMessage;
ros::Publisher temperatureMessagePublisher("temperature", & temperatureMessage);

// The battery state message and its publisher
std_msgs::Float32 batteryStateMessage;
ros::Publisher batteryStateMessagePublisher("battery_state", & batteryStateMessage);

// The diagnostics message and its publisher
diagnostic_msgs::DiagnosticArray diagnosticsMessage;
diagnostic_msgs::DiagnosticStatus diagnosticStatusArray[6];
diagnostic_msgs::KeyValue leftPingValues[2];
diagnostic_msgs::KeyValue centerPingValues[2];
diagnostic_msgs::KeyValue rightPingValues[2];
diagnostic_msgs::KeyValue imuValues[1];
diagnostic_msgs::KeyValue batteryValues[1];
diagnostic_msgs::KeyValue mcuValues[9];
ros::Publisher diagnosticsMessagePublisher("diagnostics", & diagnosticsMessage);

// The battery threshold levels
float batteryWarningVolts = BATTERY_WARNING_VOLTS_DEFAULT;
float batteryCriticalVolts = BATTERY_CRITICAL_VOLTS_DEFAULT;

// The MCU threshold levels
int mcuWarningLoad = MCU_WARNING_LOAD_DEFAULT;
int mcuCriticalLoad = MCU_CRITICAL_LOAD_DEFAULT;

// Get ROS connected status
bool connected = false;

/****************************************************************************************
 * Setup
 */

void setup() {

  // Init leds
  greenLed.init();
  redLed.init();

  // Turn on leds
  greenLed.ledOn();
  redLed.ledOn();

  // Just delay
  delay(1000);

  // Turn off leds 
  greenLed.ledOff();
  redLed.ledOff();

  // Start IMU
  CurieIMU.begin();

  // Setup IMU
  CurieIMU.setGyroRange(IMU_GYRO_RANGE);
  CurieIMU.setAccelerometerRange(IMU_ACCELEROMETER_RANGE);
  CurieIMU.setGyroRate(IMU_RATE);
  CurieIMU.setAccelerometerRate(IMU_RATE);
  
  // Calibrate IMU
  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  
  // Set baud rate
  nodeHandle.getHardware()->setBaud(ROS_SERIAL_BAUD_RATE);
  
  // Init node handle
  nodeHandle.initNode();
  
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

  // Init IMU message
  imuMessage.header.frame_id = "imu_link";
  imuMessage.angular_velocity_covariance[0] = IMU_GYRO_ANGULAR_VELOCITY_COVARIANCE;
  imuMessage.angular_velocity_covariance[4] = IMU_GYRO_ANGULAR_VELOCITY_COVARIANCE;
  imuMessage.angular_velocity_covariance[8] = IMU_GYRO_ANGULAR_VELOCITY_COVARIANCE;
  imuMessage.linear_acceleration_covariance[0] = IMU_ACCELEROMETER_LINEAR_ACCELERATION_COVARIANCE;
  imuMessage.linear_acceleration_covariance[4] = IMU_ACCELEROMETER_LINEAR_ACCELERATION_COVARIANCE;
  imuMessage.linear_acceleration_covariance[8] = IMU_ACCELEROMETER_LINEAR_ACCELERATION_COVARIANCE;
  
  // Init temperature message
  temperatureMessage.header.frame_id = "base_temperature";
  temperatureMessage.variance = 0.0016;
  
  // Init diagnostics message
  diagnosticsMessage.header.frame_id = "";
  diagnosticsMessage.status_length = 6;
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

  // The IMU diagnostic status
  diagnosticsMessage.status[3].name = "IMU";
  diagnosticsMessage.status[3].hardware_id = "imu";
  diagnosticsMessage.status[3].values_length = 1;
  diagnosticsMessage.status[3].values = imuValues;
  diagnosticsMessage.status[3].values[0].key = "Temperature";
  diagnosticsMessage.status[3].values[0].value = "N/A";
 
  // The battery diagnostic status
  diagnosticsMessage.status[4].name = "Battery";
  diagnosticsMessage.status[4].hardware_id = "battery";
  diagnosticsMessage.status[4].values_length = 1;
  diagnosticsMessage.status[4].values = batteryValues;
  diagnosticsMessage.status[4].values[0].key = "Volts";
  diagnosticsMessage.status[4].values[0].value = "N/A";

  // The MCU diagnostic status
  diagnosticsMessage.status[5].name = "MCU";
  diagnosticsMessage.status[5].hardware_id = "mcu";
  diagnosticsMessage.status[5].values_length = 9;
  diagnosticsMessage.status[5].values = mcuValues;
  diagnosticsMessage.status[5].values[0].key = "Load";
  diagnosticsMessage.status[5].values[0].value = "N/A";
  diagnosticsMessage.status[5].values[1].key = "Range average delay";
  diagnosticsMessage.status[5].values[1].value = "N/A";
  diagnosticsMessage.status[5].values[2].key = "Range main duration";
  diagnosticsMessage.status[5].values[2].value = "N/A";
  diagnosticsMessage.status[5].values[3].key = "IMU average delay";
  diagnosticsMessage.status[5].values[3].value = "N/A";
  diagnosticsMessage.status[5].values[4].key = "IMU main duration";
  diagnosticsMessage.status[5].values[4].value = "N/A";
  diagnosticsMessage.status[5].values[5].key = "Battery state average delay";
  diagnosticsMessage.status[5].values[5].value = "N/A";
  diagnosticsMessage.status[5].values[6].key = "Battery state main duration";
  diagnosticsMessage.status[5].values[6].value = "N/A";
  diagnosticsMessage.status[5].values[7].key = "Diagnostics average delay";
  diagnosticsMessage.status[5].values[7].value = "N/A";
  diagnosticsMessage.status[5].values[8].key = "Diagnostics main duration";
  diagnosticsMessage.status[5].values[8].value = "N/A";
  
  // Advertise node handle for publishers
  nodeHandle.advertise(leftRangeMessagePublisher);
  nodeHandle.advertise(centerRangeMessagePublisher);
  nodeHandle.advertise(rightRangeMessagePublisher);
  nodeHandle.advertise(imuMessagePublisher);
  nodeHandle.advertise(temperatureMessagePublisher);
  nodeHandle.advertise(batteryStateMessagePublisher);
  nodeHandle.advertise(diagnosticsMessagePublisher);
  
}

/****************************************************************************************
 * Loop
 */

void loop() {

  // Get if actually connected
  bool currentConnected = nodeHandle.connected();

  if (!connected && currentConnected) {

    // Update params
    boolean result = updateParams();

    if (result) {

      // Turn on green led
      greenLed.ledOn();
      
    }

    // Set as connected if paramers uodate success
    connected = result;
    
  } else if (!connected && !currentConnected) {

    // Blink green led to inform wating for connection
    greenLed.ledBlinkSlow();

    // Just delay
    delay(50);
    
  } else if (connected && !currentConnected) {

    // Set connected to false
    connected = false;
    
  }
  
  if (connected && rangePublisherRate.ellapsed()) {
       
    // Start duration
    rangePublisherRate.start();
    
    // Publish range
    publishRange();
    
    // End duration
    rangePublisherRate.end();
    
    // Used cycle
    mainLoop.cycleUsed();
  
  }

  if (connected && imuPublisherRate.ellapsed()) {
       
    // Start duration
    imuPublisherRate.start();
    
    // Publish IMU
    publishIMU();
    
    // End duration
    imuPublisherRate.end();
    
    // Used cycle
    mainLoop.cycleUsed();
  
  }
  
  if (connected && batteryStatePublisherRate.ellapsed()) {
       
    // Start duration
    batteryStatePublisherRate.start();
    
    // Publish battery state
    publishBatteryState();
    
    // End duration
    batteryStatePublisherRate.end();
    
    // Used cycle
    mainLoop.cycleUsed();
  
  }
  
  if (connected && diagnosticsPublisherRate.ellapsed()) {
       
    // Start duration
    diagnosticsPublisherRate.start();
    
    // Publish dagnostics
    publishDiagnostics();
    
    // End duration
    diagnosticsPublisherRate.end();
    
    // Used cycle
    mainLoop.cycleUsed();
  
  }

  // Check battery state
  checkBatteryState();
  
  // Cycle performed
  mainLoop.cyclePerformed();

  if (!connected) {

    // Spin once
    nodeHandle.spinOnce();
  
  }
  
}

void publishRange() {

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

  // Publish range messages
  leftRangeMessagePublisher.publish(& leftRangeMessage);
  centerRangeMessagePublisher.publish(& centerRangeMessage);
  rightRangeMessagePublisher.publish(& rightRangeMessage);
  
  // Publish temperature message
  temperatureMessagePublisher.publish(& temperatureMessage);
  
  // Spin once
  nodeHandle.spinOnce();

}

void publishIMU() {

  int ax, ay, az;
  int gx, gy, gz;

  // Get ROS current time
  ros::Time now = nodeHandle.now();

  // Read IMU raw data
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);

  // Convert raw data to g and degree/s
  float linear_acceleration_x = (ax / 32768.0) * IMU_ACCELEROMETER_RANGE;
  float linear_acceleration_y = (ay / 32768.0) * IMU_ACCELEROMETER_RANGE;
  float linear_acceleration_z = (az / 32768.0) * IMU_ACCELEROMETER_RANGE;
  float angular_velocity_x = (gx / 32768.0) * IMU_GYRO_RANGE;
  float angular_velocity_y = (gy / 32768.0) * IMU_GYRO_RANGE;
  float angular_velocity_z = (gz / 32768.0) * IMU_GYRO_RANGE;

  // Update IMU filter
  imuFilter.updateIMU(angular_velocity_x, angular_velocity_y, angular_velocity_z,
                      linear_acceleration_x, linear_acceleration_y, linear_acceleration_z, millis());

  // Update IMU message
  imuMessage.header.stamp = now;
  imuFilter.getQuaternion(& imuMessage.orientation.w, & imuMessage.orientation.x, & imuMessage.orientation.y, & imuMessage.orientation.z);
  imuMessage.linear_acceleration.x = linear_acceleration_x * 9.81;
  imuMessage.linear_acceleration.y = linear_acceleration_y * 9.81;
  imuMessage.linear_acceleration.z = linear_acceleration_z * 9.81;
  imuMessage.angular_velocity.x = angular_velocity_x / 57.3;
  imuMessage.angular_velocity.y = angular_velocity_y / 57.3;
  imuMessage.angular_velocity.z = angular_velocity_z / 57.3;

  // Publish IMU message
  imuMessagePublisher.publish(& imuMessage);
  
  // Spin once
  nodeHandle.spinOnce();
  
}

void publishBatteryState() {

  // Get battery volts
  float volts;
  battery.getVolts(& volts);

  // Update battery state message
  batteryStateMessage.data = volts;
  
  // Publish battery state message
  batteryStateMessagePublisher.publish(& batteryStateMessage);
  
  // Spin once
  nodeHandle.spinOnce();

}

void publishDiagnostics() {

  // Get ROS current time
  ros::Time now = nodeHandle.now();
  
  // Set time on diagnstics message header
  diagnosticsMessage.header.stamp = now;

  // Get left ping failures and measurements
  unsigned long leftPingFailures, leftPingMeasurement;
  leftPing.getFailureCounter().getCounters(& leftPingFailures, & leftPingMeasurement);

  // Set left ping diagnostic values
  String leftPingFailuresString = String(leftPingFailures, DEC);
  diagnosticsMessage.status[0].values[0].value = leftPingFailuresString.c_str();
  String leftPingMeasurementString = String(leftPingMeasurement, DEC);
  diagnosticsMessage.status[0].values[1].value = leftPingMeasurementString.c_str();

  // Set ping diagnostic level and message
  if (leftPingFailures == 0) {
    
    diagnosticsMessage.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnosticsMessage.status[0].message = "OK";
    
  } else if (leftPingFailures < leftPingMeasurement) {
    
    diagnosticsMessage.status[0].level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnosticsMessage.status[0].message = "Some measurement failed";
  
  } else {
    
    diagnosticsMessage.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnosticsMessage.status[0].message = "All measurement failed";
  

  }

  // Get central ping failures and measurements
  unsigned long centralPingFailures, centralPingMeasurement;
  centralPing.getFailureCounter().getCounters(& centralPingFailures, & centralPingMeasurement);

  // Set central ping diagnostic values
  String centralPingFailuresString = String(centralPingFailures, DEC);
  diagnosticsMessage.status[1].values[0].value = centralPingFailuresString.c_str();
  String centralPingMeasurementString = String(centralPingMeasurement, DEC);
  diagnosticsMessage.status[1].values[1].value = centralPingMeasurementString.c_str();

  // Set ping diagnostic level and message
  if (centralPingFailures == 0) {
    
    diagnosticsMessage.status[1].level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnosticsMessage.status[1].message = "OK";
    
  } else if (centralPingFailures < centralPingMeasurement) {
    
    diagnosticsMessage.status[1].level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnosticsMessage.status[1].message = "Some measurement failed";
  
  } else {
    
    diagnosticsMessage.status[1].level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnosticsMessage.status[1].message = "All measurement failed";
  
  }

  // Get right ping failures and measurements
  unsigned long rightPingFailures, rightPingMeasurement;
  rightPing.getFailureCounter().getCounters(& rightPingFailures, & rightPingMeasurement);

  // Set right ping diagnostic values
  String rightPingFailuresString = String(rightPingFailures, DEC);
  diagnosticsMessage.status[2].values[0].value = rightPingFailuresString.c_str();
  String rightPingMeasurementString = String(rightPingMeasurement, DEC);
  diagnosticsMessage.status[2].values[1].value = rightPingMeasurementString.c_str();

  // Set ping diagnostic level and message
  if (rightPingFailures == 0) {
    
    diagnosticsMessage.status[2].level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnosticsMessage.status[2].message = "OK";
    
  } else if (rightPingFailures < rightPingMeasurement) {
    
    diagnosticsMessage.status[2].level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnosticsMessage.status[2].message = "Some measurement failed";
  
  } else {
    
    diagnosticsMessage.status[2].level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnosticsMessage.status[2].message = "All measurement failed";
  
  }

  // Get IMU temperature
  int imuTemperature = CurieIMU.readTemperature();
  String imuTemperatureString = String(String(imuTemperature / 512.0 + 23, 1) + " C");

  // Set IMU diagnostic
  diagnosticsMessage.status[3].level = diagnostic_msgs::DiagnosticStatus::OK;
  diagnosticsMessage.status[3].message = "OK";
  diagnosticsMessage.status[3].values[0].value = imuTemperatureString.c_str();

  // Get battery volts
  float volts;
  battery.getVolts(& volts);

  // Set battery diagnostic values
  String voltsString = String(String(volts, 1) + " V");
  diagnosticsMessage.status[4].values[0].value = voltsString.c_str();

  // Set battery diagnostic level and message
  if (volts > batteryWarningVolts) {
    
    diagnosticsMessage.status[4].level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnosticsMessage.status[4].message = "OK";

    // Turn off red led
    redLed.ledOff();

  } else if (volts > batteryCriticalVolts) {
    
    diagnosticsMessage.status[4].level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnosticsMessage.status[4].message = "Voltage under warning level, charge battery";

    // Turn on red led
    redLed.ledOn();

    // Log
    nodeHandle.logwarn("Voltage under warning level, charge battery");

  } else {
    
    diagnosticsMessage.status[4].level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnosticsMessage.status[4].message = "Voltage under critical level, power off immediatly to avoid battery damage";

    // Turn on red led
    redLed.ledOn();

    // Log
    nodeHandle.logwarn("Voltage under critical level, power off immediatly to avoid battery damage");
  
  }

  // Get MCU load, delays and duration
  float load, rangeDelay, rangeDuration, imuDelay, imuDuration, batteryStateDelay, batteryStateDuration, diagnosticsDelay, diagnosticsDuration;
  mainLoop.getUsedCounter().getAverage(& load);
  rangePublisherRate.getDelayCounter().getAverage(& rangeDelay);
  rangePublisherRate.getDurationCounter().getAverage(& rangeDuration);
  imuPublisherRate.getDelayCounter().getAverage(& imuDelay);
  imuPublisherRate.getDurationCounter().getAverage(& imuDuration);
  batteryStatePublisherRate.getDelayCounter().getAverage(& batteryStateDelay);
  batteryStatePublisherRate.getDurationCounter().getAverage(& batteryStateDuration);
  diagnosticsPublisherRate.getDelayCounter().getAverage(& diagnosticsDelay);
  diagnosticsPublisherRate.getDurationCounter().getAverage(& diagnosticsDuration);

  // Normalize load from 0 to 100
  load = load * 100;

  // Set Arduino diagnostic values
  String loadString = String(String(load, 2) + " %").c_str();
  diagnosticsMessage.status[5].values[0].value = loadString.c_str();
  String rangeDelayString = String(String(rangeDelay, 2) + " millis");
  diagnosticsMessage.status[5].values[1].value = rangeDelayString.c_str();
  String rangeDurationString = String(String(rangeDuration, 2) + " millis");
  diagnosticsMessage.status[5].values[2].value = rangeDurationString.c_str();
  String imuDelayString = String(String(imuDelay, 2) + " millis");
  diagnosticsMessage.status[5].values[3].value = imuDelayString.c_str();
  String imuDurationString = String(String(imuDuration, 2) + " millis");
  diagnosticsMessage.status[5].values[4].value = imuDurationString.c_str();
  String batteryStateDelayString = String(String(batteryStateDelay, 2) + " millis");
  diagnosticsMessage.status[5].values[5].value = batteryStateDelayString.c_str();
  String batteryStateDurationString = String(String(batteryStateDuration, 2) + " millis");
  diagnosticsMessage.status[5].values[6].value = batteryStateDurationString.c_str();
  String diagnosticsDelayString = String(String(diagnosticsDelay, 2) + " millis");
  diagnosticsMessage.status[5].values[7].value = diagnosticsDelayString.c_str();
  String diagnosticsDurationString = String(String(diagnosticsDuration, 2) + " millis");
  diagnosticsMessage.status[5].values[8].value = diagnosticsDurationString.c_str();
  
  // Set MCU diagnostic level and message
  if (load < mcuWarningLoad) {
    
    diagnosticsMessage.status[5].level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnosticsMessage.status[5].message = "OK";
    
  } else if (load < mcuCriticalLoad) {
    
    diagnosticsMessage.status[5].level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnosticsMessage.status[5].message = "Load over warning level";
  
  } else {
    
    diagnosticsMessage.status[5].level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnosticsMessage.status[5].message = "Load over critical level";
  
  }
  
  // Publish diagnostics message
  diagnosticsMessagePublisher.publish(& diagnosticsMessage);
  
  // Spin once
  nodeHandle.spinOnce();
  
}

void checkBatteryState() {

  // Get battery volts
  float volts;
  battery.getVolts(& volts);

  if (volts > batteryWarningVolts) {
    
    // Turn off red led
    redLed.ledOff();
    
  } else if (volts > batteryCriticalVolts) {
    
    // Turn on red led
    redLed.ledOn();

  } else {
    
    // Blink red led
    redLed.ledBlinkFast();
      
  }
  
}

bool updateParams() {

  float rangePublisherFrequency;
  float batteryStatePublisherFrequency;
  float diagnosticsPublisherFrequency;
  float imuOrientationCovarianceDiagonal[3];
  float batteryParamA, batteryParamB;
  
  // Update params
  bool result = getFloatParam("~batteryWarningVolts", & batteryWarningVolts, 1) && 
                getFloatParam("~batteryCriticalVolts", & batteryCriticalVolts, 1) &&
                getIntParam("~mcuWarningLoad", & mcuWarningLoad, 1) &&
                getIntParam("~mcuCriticalLoad", & mcuCriticalLoad, 1) && 
                getFloatParam("~rangePublisherFrequency", & rangePublisherFrequency, 1) &&
                getFloatParam("~batteryStatePublisherFrequency", & batteryStatePublisherFrequency, 1) &&
                getFloatParam("~diagnosticsPublisherFrequency", & diagnosticsPublisherFrequency, 1) &&
                getFloatParam("~imuOrientationCovarianceDiagonal", imuOrientationCovarianceDiagonal, 3) &&
                getFloatParam("~batteryParamA", & batteryParamA, 1) &&
                getFloatParam("~batteryParamB", & batteryParamB, 1);

  if (result) {

    // Set params
    rangePublisherRate.setFrequency(rangePublisherFrequency);
    batteryStatePublisherRate.setFrequency(batteryStatePublisherFrequency);
    diagnosticsPublisherRate.setFrequency(diagnosticsPublisherFrequency);
    imuMessage.orientation_covariance[0] = imuOrientationCovarianceDiagonal[0];
    imuMessage.orientation_covariance[4] = imuOrientationCovarianceDiagonal[1];
    imuMessage.orientation_covariance[8] = imuOrientationCovarianceDiagonal[2];
    battery.setParams(batteryParamA, batteryParamB);

  }

  return result;

}

bool getIntParam(const char * key, int * buffer, int length) {

  bool result = false;
  
  // Get param
  result = nodeHandle.getParam(key, buffer, length);

  // Spin once
  nodeHandle.spinOnce();

  // Just wait
  delay(10);

  return result;
  
}

bool getFloatParam(const char * key, float * buffer, int length) {

  bool result = false;

  // Get param
  result = nodeHandle.getParam(key, buffer, length);

  // Spin once
  nodeHandle.spinOnce();

  // Just wait
  delay(10);

  return result;

}


