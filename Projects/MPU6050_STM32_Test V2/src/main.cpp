#include <Arduino.h>
#include <Wire.h>

int16_t gyroRaw_X, gyroRaw_Y, gyroRaw_Z;




void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("Check 0");
  delay(20);
  Wire.setClock(400000);
  delay(20);
  Serial.println("Check 1");
  delay(20);
  Wire.begin();
  delay(250);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x00);
  Wire.endTransmission();
}


void loop(){
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  gyroRaw_X = Wire.read() << 8 | Wire.read();
  gyroRaw_Y = Wire.read() << 8 | Wire.read();
  gyroRaw_Z = Wire.read() << 8 | Wire.read();
  
  Serial.print("X = ");
  Serial.print(gyroRaw_X);
  Serial.print("Y = ");
  Serial.print(gyroRaw_Y);
  Serial.print("Z = ");
  Serial.println(gyroRaw_Z);
  delay(250);
}
