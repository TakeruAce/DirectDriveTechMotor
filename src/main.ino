#include "DDT_Motor_M15M06.h"
#include <Arduino.h>
#include <utils.hpp>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

// SerialHandler mySerial(Serial);

int16_t Speed = 0;   // Speed of motor
uint8_t Acce = 0;    // Acceleration of motor
uint8_t Brake_P = 0; // Brake position of motor
uint8_t IDs[] = {1,3};      // ID of Motor (default:1)
double angle_offset[] = {-0.536,1.0853};
double prev_angle[] = {0,0};
double motor_direction[] = {-1,1};

Receiver Receiv[2];
Receiver prev_Receiv[2];
// ,TXのピン番号が違うためM5製品とRS485モジュールに対応させてくださいM5Stackのモジュールによって対応するRX
auto motor_handler = MotorHandler(1, 0); // RX,TX
const int16_t SPEED_MAX = 40;
const int16_t SPEED_MIN = -40;
float start_t = 0;

float cmd_effort[] = {0,0};

// for ROS
using namespace ros;
NodeHandle nh;
sensor_msgs::JointState msg_joint;
Publisher pub_joint("joint_state_arm", &msg_joint);

void joint_torque_cb(const std_msgs::Float64MultiArray& msg_sub) {
  for(int i=0;i<sizeof(cmd_effort)/sizeof(cmd_effort[0]);i++) {
    cmd_effort[i] = msg_sub.data[i];
  }
}
Subscriber<std_msgs::Float64MultiArray> sub_joint_torque("joint_torque", &joint_torque_cb);

void setup()
{
  // while (!Serial);
  Serial.begin(115200);
  Serial.println("DDT-Motor RS485");
  delay(100);
  uint8_t Mode = 0x01;                   
  // torque mode: 0x01
  // velocity mode: 0x02
  // position mode: 0x03
  for (int i=0;i<sizeof(IDs)/sizeof(IDs[0]);i++) {
    motor_handler.Control_Motor(0, IDs[i], Acce, Brake_P, &Receiv[i]); //モータ停止
    delay(100);
    motor_handler.Set_MotorMode(Mode, IDs[i]);
    delay(100);
  }

  // msg_joint.name = (String*) malloc(sizeof(String) * 2);
  // msg_joint.name_length = 2;
  msg_joint.position = (float*) malloc(sizeof(float) * 2);
  msg_joint.position_length = 2;
  msg_joint.velocity = (float*) malloc(sizeof(float) * 2);
  msg_joint.velocity_length = 2;
  msg_joint.effort = (float*) malloc(sizeof(float) * 2);
  msg_joint.effort_length = 2;

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_joint_torque);
  nh.advertise(pub_joint);

  // for motor id reset
  // motor_handler.Set_MotorID(1);
  // delay(10);
  // motor_handler.Set_MotorID(1);
  // delay(10);
  // motor_handler.Set_MotorID(1);
  // delay(10);
  // motor_handler.Set_MotorID(1);
  // delay(10);
  // motor_handler.Set_MotorID(1);
  // delay(10);
  // motor_handler.Check_Motor(&Receiv);
  // delay(10);
  // mySerial.Printf("motor ID is %d",Receiv.ID);
  start_t = millis();
}

int prev_micro = 0;
void loop()
{
  float t = millis();
  for (int i=0;i<sizeof(IDs)/sizeof(IDs[0]);i++) {
    spin_and_get(-motor_direction[i] * cmd_effort[i] / 0.37 / 8.0 * 32767.0, IDs[i], Receiv[i], prev_Receiv[i]);
    msg_joint.position[i] = motor_direction[i] * Receiv[i].Position / 32767.0 * 2 * PI - angle_offset[i]; // rad
    msg_joint.velocity[i]=  motor_direction[i] * Receiv[i].BSpeed / 60.0 * 2 * PI; // rad/s
    msg_joint.effort[i] = motor_direction[i] * Receiv[i].ECurru / 32767.0 * 8.0 * 0.37; // Nm

    // convert angle to (-pi,pi)
    if (msg_joint.position[i] < -PI) msg_joint.position[i] += 2* PI;
    if (msg_joint.position[i] >  PI) msg_joint.position[i] -= 2* PI;
    prev_angle[i] = msg_joint.position[i];
    prev_Receiv[i] = Receiv[i];
  }
  // delay(10);
  pub_joint.publish(&msg_joint);
  nh.spinOnce();

  while(micros()-prev_micro < 5000);
  // Serial.println(micros()-prev_micro);
  prev_micro = micros();
}

void spin_and_get(int16_t _Speed, uint8_t id, Receiver& Receiv, Receiver& prev_Receiv)
{
  if(motor_handler.Control_Motor(_Speed, id, Acce, Brake_P, &Receiv)) {

  } else {
    Serial.print("receive failed.");
    Receiv = prev_Receiv;
  }
  //温度を取得したい場合はGet_Motor関数を呼び出す
  // Serial.print("ID:");
  // Serial.print(Receiv.ID);
  // Serial.print(" Mode:");
  // Serial.print(Receiv.BMode);
  // // Serial.print(" Current:");
  // // Serial.print(Receiv.ECurru);
  // // Serial.print(" Speed:");
  // // Serial.print(Receiv.BSpeed);
  // Serial.print(" Position:");
  // Serial.print(Receiv.Position / 32767.0 * 360.0);
  // Serial.print(" Tmp:");
  // Serial.print(Receiv.Temp);
  // Serial.print(" ErrCode:");
  // Serial.println(Receiv.ErrCode);
}
