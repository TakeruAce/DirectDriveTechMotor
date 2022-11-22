#include "DDT_Motor_M15M06.h"
#include <Arduino.h>
#include <utils.hpp>

SerialHandler mySerial(Serial);

int16_t Speed = 0;   // Speed of motor
uint8_t Acce = 0;    // Acceleration of motor
uint8_t Brake_P = 0; // Brake position of motor
uint8_t IDs[] = {1,3};      // ID of Motor (default:1)

Receiver Receiv;
// M5Stackのモジュールによって対応するRX,TXのピン番号が違うためM5製品とRS485モジュールに対応させてください
auto motor_handler = MotorHandler(1, 0); // RX,TX
const int16_t SPEED_MAX = 40;
const int16_t SPEED_MIN = -40;
float start_t = 0;
void setup()
{
  while (!Serial);
  Serial.begin(115200);
  Serial.println("DDT-Motor RS485");
  delay(100);
  uint8_t Mode = 0x01;                   
  // torque mode: 0x01
  // velocity mode: 0x02
  // position mode: 0x03
  for (int i=0;i<sizeof(IDs)/sizeof(IDs[0]);i++) {
    motor_handler.Control_Motor(0, IDs[i], Acce, Brake_P, &Receiv); //モータ停止
    delay(100);
    motor_handler.Set_MotorMode(Mode, IDs[i]);
    delay(100);
  }
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
    spin_and_get(0.2 * sin(2*3.14*(t-start_t)/1000.0) / 0.37 / 8.0 * 32767.0, IDs[i]);
  }
  // delay(10);

  while(micros()-prev_micro < 1000);
  Serial.println(micros()-prev_micro);
  prev_micro = micros();
  // spin_and_get(0.2 / 0.37 / 8.0 * 32767.0); // torque
  // delay(1500);
  // spin_and_get(0.0 / 0.37 / 8.0 * 32767.0); // torque
  // delay(1500);
  // spin_and_get(-0.2 / 0.37 / 8.0 * 32767.0); // torque
  // delay(1500);
  // for (int i=0;i<sizeof(IDs)/sizeof(IDs[0]);i++) {
  //   spin_and_get(90.0 / 360.0 * 32767.0, IDs[i]);
  //   delay(10);
  // }
  // delay(1500);
  // for (int i=0;i<sizeof(IDs)/sizeof(IDs[0]);i++) {
  //   spin_and_get(0.0 / 360.0 * 32767.0, IDs[i]);
  //   delay(10);
  // }
  // delay(1500);

  // delay(1000);
  // spin_and_get(Speed);  
  // while (true)
  // {
  //   Speed++;
  //   spin_and_get(Speed);
  //   if (Speed > int16_t(SPEED_MAX))
  //   {
  //     break;
  //   }
  // }

  // while (true)
  // {
  //   Speed--;
  //   spin_and_get(Speed);
  //   if (Speed < SPEED_MIN)
  //   {
  //     break;
  //   }
  // }
}
void spin_and_get(int16_t _Speed, uint8_t id)
{
  if(motor_handler.Control_Motor(_Speed, id, Acce, Brake_P, &Receiv)) {

  } else {
    Serial.println("receive failed.");
  }
  //温度を取得したい場合はGet_Motor関数を呼び出す
  motor_handler.Get_Motor(id, &Receiv);
  Serial.print("ID:");
  Serial.print(Receiv.ID);
  Serial.print(" Mode:");
  Serial.print(Receiv.BMode);
  Serial.print(" Current:");
  Serial.print(Receiv.ECurru);
  Serial.print(" Speed:");
  Serial.print(Receiv.BSpeed);
  Serial.print(" Position:");
  Serial.print(Receiv.Position);
  Serial.print(" Tmp:");
  Serial.print(Receiv.Temp);
  Serial.print(" ErrCode:");
  Serial.println(Receiv.ErrCode);
}
