#include <ros.h>
#include <std_msgs/Int16MultiArray.h> 
#include <NewPing.h>
#include "BKD_Spio.h"
#include "BKD_Motor.h"

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

int cmd_pair1 = 2; 
int cmd_pair2 = 2; 

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

void setup() {
  SpioInstance.init();
  nh.initNode();
  nh.advertise(pub_sonar);
  nh.subscribe(sub_led);
  nh.subscribe(sub_cylinder);

  sonar_msg.data_length = SONAR_NUM;
  sonar_msg.data = range_values;
}

void loop() {
  SpioInstance.onLoop();
  handleCylinders();     

  for (int i = 0; i < SONAR_NUM; i++) {
    range_values[i] = (int16_t)sonars[i].ping_cm();
  }
  pub_sonar.publish(&sonar_msg);

  nh.spinOnce();
}