#include <ros.h>
#include <std_msgs/Int16MultiArray.h> 
#include <std_msgs/Float32MultiArray.h>
#include <NewPing.h>
#include "BKD_Spio.h"
#include "BKD_Motor.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
unsigned long lastBatteryTime = 0;
const unsigned long batteryInterval = 2000;

unsigned long timeLimit1 = 1500;
unsigned long timeLimit2 = 1500;

unsigned long startTime1 = 0;
unsigned long startTime2 = 0;

int last_cmd1 = 2; 
int last_cmd2 = 2;

Spio SpioInstance(40, 41, 38, 39);
Motor cylinder0(Motor::type::H_STD, 30, 7, 0.0, false);
Motor cylinder1(Motor::type::H_STD, 32, 8, 0.0, false);     
Motor cylinder2(Motor::type::H_STD, 36, 10, 0.0, false);
Motor cylinder3(Motor::type::H_STD, 34, 9, 0.0, false);

#define TRIGGER_PIN_1  31 
#define ECHO_PIN_1     33 
#define TRIGGER_PIN_2  23 
#define ECHO_PIN_2     25 
#define TRIGGER_PIN_3  27 
#define ECHO_PIN_3     29  
#define TRIGGER_PIN_4  35 
#define ECHO_PIN_4     37
#define MAX_DISTANCE 300 
#define SONAR_NUM 4      

NewPing sonars[SONAR_NUM] = {
  NewPing(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE),
  NewPing(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE),
  NewPing(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE),
  NewPing(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE)
};

#define LED_NUM 4
const uint16_t LED_BIT_MAPPING[LED_NUM] = {1, 2, 3, 4}; 

std_msgs::Float32MultiArray battery_msg;
ros::Publisher pub_battery("batterySensor", &battery_msg);
float battery_data[2]; // [0]: Voltage, [1]: Percent

int cmd_pair1 = 2; 
int cmd_pair2 = 2; 

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ros::NodeHandle nh;
std_msgs::Int16MultiArray sonar_msg;
ros::Publisher pub_sonar("sonarSensor", &sonar_msg);
int16_t range_values[SONAR_NUM];


void ledControl(const std_msgs::Int16MultiArray& msg) {  
  if (msg.data_length >= LED_NUM) {
    for (int i = 0; i < LED_NUM; i++) {
        SpioInstance.writeBit(LED_BIT_MAPPING[i], (uint8_t)msg.data[i]);
    }
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> sub_led("ledControl", &ledControl);

void cylinderControl(const std_msgs::Int16MultiArray& msg) {
  if (msg.data_length >= 2) {
    cmd_pair1 = msg.data[0]; 
    cmd_pair2 = msg.data[1]; 

    if (cmd_pair1 == 0 && last_cmd1 != 0) startTime1 = millis();
    if (cmd_pair2 == 0 && last_cmd2 != 0) startTime2 = millis();

    last_cmd1 = cmd_pair1;
    last_cmd2 = cmd_pair2;
  }                                                 
}
ros::Subscriber<std_msgs::Int16MultiArray> sub_cylinder("cylinderControl", &cylinderControl);

void handleCylinders() {
  unsigned long now = millis();

  if (cmd_pair1 == 1) { // Open
    cylinder0.setPower(-0.5);
    cylinder1.setPower(-0.5);
    SpioInstance.writeBit(7, 0); 
  } 
  else if (cmd_pair1 == 0) { // Close
    bool isTimeout1 = (now - startTime1 >= timeLimit1);

    if (SpioInstance.readBit(SpioInstance.bufferInput, 0) && !isTimeout1) {
      cylinder0.setPower(0.5);
    } else if (!SpioInstance.readBit(SpioInstance.bufferInput, 0) || isTimeout1){
      cylinder0.setPower(0.0);
    }

    if (SpioInstance.readBit(SpioInstance.bufferInput, 1) && !isTimeout1) {
      cylinder1.setPower(0.5);
    } else if (!SpioInstance.readBit(SpioInstance.bufferInput, 1) || isTimeout1) {
      cylinder1.setPower(0.0); 
    }
    SpioInstance.writeBit(7, 1);
  } 
  else { // Stop
    cylinder0.setPower(0.0);
    cylinder1.setPower(0.0);
    SpioInstance.writeBit(7, 0);
  }

  if (cmd_pair2 == 1) { // Open
    cylinder2.setPower(-0.5);
    cylinder3.setPower(-0.5);
    SpioInstance.writeBit(5, 0);
  } 
  else if (cmd_pair2 == 0) { // Close
    bool isTimeout2 = (now - startTime2 >= timeLimit2);

    if (SpioInstance.readBit(SpioInstance.bufferInput, 3) && !isTimeout2) {
      cylinder3.setPower(0.5);
    } else if (!SpioInstance.readBit(SpioInstance.bufferInput, 3) || isTimeout2) {
      cylinder3.setPower(0.0);
    }

    if (SpioInstance.readBit(SpioInstance.bufferInput, 2) && !isTimeout2) {
      cylinder2.setPower(0.5);
    } else if (!SpioInstance.readBit(SpioInstance.bufferInput, 2) || isTimeout2) {
      cylinder2.setPower(0.0);
    }
    SpioInstance.writeBit(5, 1);
  } 
  else { // Stop
    cylinder2.setPower(0.0);
    cylinder3.setPower(0.0);
    SpioInstance.writeBit(5, 0);
  }
}

void handleBattery() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastBatteryTime >= batteryInterval) {
    lastBatteryTime = currentMillis;

    float busVoltage = ina219.getBusVoltage_V();
  
    float batteryPercent = mapFloat(busVoltage, 20.0, 29.2, 0, 100); 

    if (batteryPercent > 100) batteryPercent = 100;
    if (batteryPercent < 0) batteryPercent = 0;

    battery_data[0] = busVoltage;
    battery_data[1] = batteryPercent;
    
    pub_battery.publish(&battery_msg);
  }
}  
void setup() {
  SpioInstance.init();
  
  nh.initNode();
  nh.advertise(pub_sonar);
  nh.advertise(pub_battery); 
  nh.subscribe(sub_led);
  nh.subscribe(sub_cylinder);

  sonar_msg.data_length = SONAR_NUM;
  sonar_msg.data = range_values;
  
  battery_msg.data_length = 2;
  battery_msg.data = battery_data;

  if (!ina219.begin()) {

  }
}

void loop() {
  SpioInstance.onLoop();
  handleCylinders();     
  handleBattery();

  for (int i = 0; i < SONAR_NUM; i++) {
    range_values[i] = (int16_t)sonars[i].ping_cm();
  }
  pub_sonar.publish(&sonar_msg);

  nh.spinOnce();
}