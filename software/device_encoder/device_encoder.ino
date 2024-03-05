#include <Wire.h>
#include "TCA9548A.h"
#include "AS5600.h"
#include <SoftwareSerial.h>

SoftwareSerial serial1(7, 8); // RX, TX

TCA9548A device1;
AS5600 sensor;

const int receive_pack_len=5;

class encoder
{
  public:
    int EncoderId;
    int ReadRaw;
    double AngleRaw;
    double AnglePro;
    int dir;
    int offset;

    encoder(int id);
    int GetRaw();
    double GetAngleraw();
    double GetAnglepro();
    void SetOffset(double in);
    void SetDir(int in);
    friend void PrintData();
};
encoder::encoder(int id)
{
  EncoderId = id;
  dir = 1;
  offset = 0;
}
int encoder::GetRaw()
{
  device1.set_port(EncoderId);
  ReadRaw = sensor.getAngle();
  return ReadRaw;
}
double encoder::GetAngleraw()
{
  device1.set_port(EncoderId);
  ReadRaw = sensor.getAngle();
  AngleRaw = (ReadRaw / 4096.0) * 360.0;
  return AngleRaw;
}
double encoder::GetAnglepro()
{
  device1.set_port(EncoderId);
  ReadRaw = sensor.getAngle();
  AngleRaw = (ReadRaw / 4096.0) * 360.0;
  AnglePro = dir * AngleRaw + offset;
  return AnglePro;
}
void encoder::SetOffset(double in)
{
  offset = in;
}
void encoder::SetDir(int in)
{
  dir = in;
}

encoder encoder1(0), encoder2(1),encoder3(2);


void PrintData()
{
  Serial.print("e1:  ");
  Serial.print(encoder1.AngleRaw);
  Serial.print("  ");
  Serial.print("e2:  ");
  Serial.print(encoder2.AngleRaw);
  Serial.print("  ");
  Serial.print("e3:  ");
  Serial.println(encoder3.AngleRaw);
}

void setup()
{
  Serial.begin(9600);
  serial1.begin(9600);
  //Serial.println("testori");
  device1.init();

  /*encoder1.SetOffset(0);
  encoder1.SetDir(1);
  encoder2.SetOffset(0);
  encoder2.SetDir(1);
  encoder3.SetOffset(0);
  encoder3.SetDir(1);*/
}

void data_sender()//sent encoder value to master device
{
  char buf[receive_pack_len];

  if (Serial.availableForWrite())
  {
    /*Serial.write('a');
    dtostrf(encoder1.AngleRaw, 3, 1, buf);
    Serial.write(buf, receive_pack_len);
    Serial.write('b');
    dtostrf(encoder2.AngleRaw, 3, 1, buf);
    Serial.write(buf, receive_pack_len);
    Serial.write('c');
    dtostrf(encoder3.AngleRaw, 3, 1, buf);
    Serial.write(buf, receive_pack_len);*/
    Serial.print("a");
    float sent_temp;
    if(encoder1.AngleRaw<100){
      sent_temp=encoder1.AngleRaw+500;
    }
    else{
      sent_temp=encoder1.AngleRaw;
    }
    Serial.print(int(sent_temp));
    Serial.print("b");
    Serial.print(int(encoder2.AngleRaw));
    Serial.print("c");
    Serial.print(int(encoder2.AngleRaw)+int(encoder3.AngleRaw)-289);
    int checksum=int((int(encoder1.AngleRaw)+int(encoder2.AngleRaw)+int(encoder2.AngleRaw)+int(encoder3.AngleRaw)-289)/10)+100;
    Serial.print("d");
    Serial.println(checksum);
  }
}

void data_receiver()
{
  if(Serial.available())
  {
    serial1.println(Serial.read());
  }
}
/*void data_receiver()

  char pack_temp[receive_pack_len];
  double value_temp;
  if (Serial.available())
  {
    Serial.find('h');
    Serial.readBytes(pack_temp, receive_pack_len);
    value_temp = strtod(pack_temp, NULL);
    pid_hip.target = value_temp;

    Serial.find('k');
    Serial.readBytes(pack_temp, receive_pack_len);
    value_temp = strtod(pack_temp, NULL);
    pid_knee.target = value_temp;

    Serial.find('a');
    Serial.readBytes(pack_temp, receive_pack_len);
    value_temp = strtod(pack_temp, NULL);
    pid_ab.target = value_temp;
  }
}*/


void loop()
{
  encoder1.GetAngleraw();
  encoder2.GetAngleraw();
  encoder3.GetAngleraw();

  //Serial.println("test");
  //PrintData();
  data_sender();
  //data_receiver();
  delay(5);
}
