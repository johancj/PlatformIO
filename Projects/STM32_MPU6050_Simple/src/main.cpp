#include <Arduino.h>

#include <Wire.h>

//int16_t gyroRaw[3];

void setup() {
  
  Serial1.begin(115200);
  delay(2500);
  Serial1.println("HEIHEI 1! ");
  delay(25);
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Serial1.println("HEIHEI 2! ");
  delay(5);

  Wire.setClock(400000);
  Serial1.println("HEIHEI 3! ");
  Wire.begin();
  delay(250);
  Serial1.println("HEIHEI 4! ");
/*
  //Skrur på MPU6050en
  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x00);
  Wire.endTransmission();
  */

 
}

void loop() {
  /* 
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  gyroRaw[0] = Wire.read() << 8 | Wire.read();
  gyroRaw[1] = Wire.read() << 8 | Wire.read();
  gyroRaw[2] = Wire.read() << 8 | Wire.read();
  Serial1.print("X = ");
  Serial1.print(gyroRaw[0]);
  Serial1.print(", Y = ");
  Serial1.print(gyroRaw[1]);
  Serial1.print(", Z = ");
  Serial1.println(gyroRaw[2]);
  delay(250);
   */
}

