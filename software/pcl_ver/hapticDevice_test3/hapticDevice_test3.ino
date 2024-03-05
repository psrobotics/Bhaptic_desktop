#include <SoftwareSerial.h>
#include <Wire.h>
#include "TCA9548A.h"
#include "AS5600.h"

TCA9548A device1;
AS5600 sensor;

int motorTorque[3]={0,0,0};
int sensorRaw[3]={0,0,0};
int motorArr[6] = {3, 5, 6, 9, 10, 11};

void setup() {
  Serial.begin(115200);
  for (int s = 0; s < 6; s++)
    pinMode(motorArr[s], OUTPUT);
  device1.init();
  device1.disable();
}

void serialRead(int *data)
{
  if (Serial.available()){
    char ch = Serial.read();
    if (ch == 'h'){
      for (int i = 0; i < 3; i++)
        *(data + i) = Serial.parseInt();
    }
  }
}

void serialWrite(int *data)
{
  if (Serial.availableForWrite())
  {
    char buffertemp[4];

    Serial.print('a');
    itoa(*data, buffertemp, 10);
    Serial.print(buffertemp);
    Serial.print(' ');
    itoa(*(data + 1), buffertemp, 10);
    Serial.print(buffertemp);
    Serial.print(' ');
    itoa(*(data + 2), buffertemp, 10);
    Serial.print(buffertemp);
    Serial.print('e');
  }
}

void sensorRead(int *data)
{
  for (int n = 0; n < 3; n++)
  {
    device1.set_port(n); //选定复用器IIC端口
    *(data + n) = sensor.getAngle();

  }
}

void motor_run(int p1, int p2, int pwm)
{
  if (pwm > 0) {
    analogWrite(p1, abs(pwm));
    analogWrite(p2, 0);
  }
  else if (pwm == 0) {
    analogWrite(p1, 0);
    analogWrite(p2, 0);
  }
  else if(pwm<0){
    analogWrite(p2, abs(pwm));
    analogWrite(p1, 0);
  }
}

void loop() {
  //sensorRead(sensorRaw);//读取传感器数值
  //serialWrite(sensorRaw);
  serialRead(motorTorque);
  motor_run(3,5,motorTorque[0]);
  motor_run(9,6,motorTorque[1]);
  motor_run(11,10,motorTorque[2]);
  //delay(10);
}
