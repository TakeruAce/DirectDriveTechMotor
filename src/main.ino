#include "DDT_Motor_M15M06.h"
#include <Arduino.h>
#include <utils.hpp>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

int16_t Speed = 0;   // Speed of motor
uint8_t Acce = 0;    // Acceleration of motor
uint8_t Brake_P = 0; // Brake position of motor
uint8_t IDs[] = {1,3};      // ID of Motor (default:1)
double angle_offset[] = {0, 0}; // for caliblation
double angle_coeff[] = {0,0};
// for calibration
double value_1[] = {0,0};
double value_2[] = {0,0};
const double initial_angle[] = {PI/2,PI/2};
const double initial_angle_1[] = {PI/2,PI/2};
const double initial_angle_2[] = {PI/3,2*PI/3};
double prev_angle[] = {0,0};
double motor_direction[] = {1,1}; // define the relationship between motor direction and coordinate direction.

Receiver Receiv[2];
Receiver prev_Receiv[2];
auto motor_handler = MotorHandler(1, 0); // RX,TX
const int16_t SPEED_MAX = 40;
const int16_t SPEED_MIN = -40;
float start_t = 0;

float cmd_effort[] = {0,0};

// for calibration
bool start_calib_flag = false;
bool start_calib_flag_1 = false;
bool start_calib_flag_2 = false;
float calib_count = 0;

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

void calib_cb(std_msgs::Bool& msg) {
  if (msg.data) {
		if (!start_calib_flag) {
			nh.loginfo("start force sensor calibration...");
			start_calib_flag = true;
			for(int i=0;i<2;i++) angle_offset[i]=0;
			calib_count = 0;
		} else {
			for(int i=0;i<2;i++) {
				angle_offset[i] = Receiv[i].Position / (calib_count+1) + calib_count*angle_offset[i] / (calib_count+1);
			}
			calib_count++;
		}
	} else {
		if (start_calib_flag) {
			nh.loginfo("calibration finished.");
			start_calib_flag = false;
		}
	}
}

void calib_pi_2_cb(std_msgs::Bool& msg) {
  if (msg.data) {
		if (!start_calib_flag_1) {
			nh.loginfo("Start force sensor calibration. Using Calib Prop 1.");
			start_calib_flag_1 = true;
			for(int i=0;i<2;i++) value_1[i]=0;
			calib_count = 0;
		} else {
			for(int i=0;i<2;i++) {
				value_1[i] = Receiv[i].Position / (calib_count+1) + calib_count*value_1[i] / (calib_count+1);
			}
			calib_count++;
		}
	} else {
		if (start_calib_flag_1) {
			nh.loginfo("Calibration finished.");
      for(int i=0;i<2;i++) {
        angle_coeff[i] = (initial_angle_1[i] - initial_angle_2[i])/((value_1[i]-value_2[i]));
        angle_offset[i] = initial_angle_1[i] - angle_coeff[i]*value_1[i];
      }
			start_calib_flag_1 = false;
		}
	}
}
void calib_pi_3_cb(std_msgs::Bool& msg) {
  if (msg.data) {
		if (!start_calib_flag_2) {
			nh.loginfo("Start force sensor calibration. Using Calib Prop 2.");
			start_calib_flag_2 = true;
			for(int i=0;i<2;i++) value_2[i]=0;
			calib_count = 0;
		} else {
			for(int i=0;i<2;i++) {
				value_2[i] = Receiv[i].Position / (calib_count+1) + calib_count*value_2[i] / (calib_count+1);
			}
			calib_count++;
		}
	} else {
		if (start_calib_flag_2) {
			nh.loginfo("Calibration finished.");
      for(int i=0;i<2;i++) {
        angle_coeff[i] = (initial_angle_1[i] - initial_angle_2[i])/((value_1[i]-value_2[i]));
        angle_offset[i] = initial_angle_1[i] - angle_coeff[i]*value_1[i];
      }
			start_calib_flag_2 = false;
		}
	}
}




Subscriber<std_msgs::Float64MultiArray> sub_joint_torque("joint_torque", &joint_torque_cb);
Subscriber<std_msgs::Bool> sub_calib("calib", &calib_cb);
Subscriber<std_msgs::Bool> sub_calib_1("calib_pi_2", &calib_pi_2_cb);
Subscriber<std_msgs::Bool> sub_calib_2("calib_pi_3", &calib_pi_3_cb);

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
  nh.subscribe(sub_calib);
  nh.subscribe(sub_calib_1);
  nh.subscribe(sub_calib_2);
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

bool is_print_time = true;
void print_time(int mil) {
  if (is_print_time) Serial.println(millis() - mil);
}
int prev_micro = 0;
void loop()
{
  float t = millis();
  for (int i=0;i<sizeof(IDs)/sizeof(IDs[0]);i++) {
    spin_and_get(-motor_direction[i] * cmd_effort[i] / 0.37 / 8.0 * 32767.0, IDs[i], Receiv[i], prev_Receiv[i]);
    msg_joint.position[i] = motor_direction[i] * (Receiv[i].Position - angle_offset[i])/ 32767.0 * 2 * PI + initial_angle[i]; // rad
    // msg_joint.position[i] = motor_direction[i] * (angle_coeff[i] * Receiv[i].Position + angle_offset[i]); // rad
    msg_joint.velocity[i]=  motor_direction[i] * Receiv[i].BSpeed / 60.0 * 2 * PI; // rad/s
    msg_joint.effort[i] = motor_direction[i] * Receiv[i].ECurru / 32767.0 * 8.0 * 0.37; // Nm
    // convert angle to (-pi,pi)
    if (msg_joint.position[i] < -PI) msg_joint.position[i] += 2* PI;
    if (msg_joint.position[i] >  PI) msg_joint.position[i] -= 2* PI;
    prev_angle[i] = msg_joint.position[i];
    prev_Receiv[i] = Receiv[i];
  }
  msg_joint.header.stamp = nh.now();
  // delay(10);
  pub_joint.publish(&msg_joint);
  nh.spinOnce();

  while(micros()-prev_micro < 5500);
  Serial.println(micros()-prev_micro);
  prev_micro = micros();
}

void spin_and_get(int16_t _Speed, uint8_t id, Receiver& Receiv, Receiver& prev_Receiv)
{
  if(motor_handler.Control_Motor(_Speed, id, Acce, Brake_P, &Receiv)) {
    // Serial.println("received.");
  } else {
    Serial.print("receive failed.");
    Serial.println(id);
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
