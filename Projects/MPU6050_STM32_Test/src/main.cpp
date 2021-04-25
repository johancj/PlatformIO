#include <Arduino.h>
#include <Wire.h>

int16_t accRaw[3], tempRaw, gyroRaw[3], gyroRawCal[3];

int16_t acc[3], temp;
double angle[3], tempAngle;
long t;

int k = 0;

void MPU6050_setup();
void read_MPU6050_data();

TwoWire Wire = TwoWire();

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("Check 0");
  delay(20);
  Wire.setClock(400000);
  delay(500);
  Serial.println("Check 1");
  Wire.begin();
  
  delay(250);
  Serial.println("Check 2");
  MPU6050_setup();

  delay(500);
  Serial.println("Kaliberer. Ikke rør sensoren!");
  delay(250);
  for (int i = 0; i < 2000; i++)
  {
    read_MPU6050_data();

    for (int j = 0; j < 3; j++)
    {
      gyroRawCal[j] += gyroRaw[j];
    }
    delay(1);
  }
  gyroRawCal[0] /= 2000;
  gyroRawCal[1] /= 2000;
  gyroRawCal[2] /= 2000;
}


void loop()
{
  k++;
  t = micros();

  read_MPU6050_data();

  for (int i = 0; i < 3; i++)
  {
    acc[i] = accRaw[i] / 8192.0;
    gyroRaw[i] -= gyroRawCal[i];
    angle[i] += gyroRaw[i] * 0.0000611; // 1/(250Hz*65.5)=0.0000611
  }

  tempAngle = angle[0];
  angle[0] += angle[1] * sin(gyroRaw[2] * 0.000001067); //degrees to rad: 0.0000611 * 3.142/180 = 0.000001067
  angle[1] -= tempAngle * sin(gyroRaw[2] * 0.000001067);

  temp = tempRaw / 340.0 + 36.53;

  //    while (micros() < t + 4000) {}            //for at least 250Hz loop
  if (k > 500)
  {
    k = 0;
    Serial.print("temp:\t"), Serial.println(temp);
    Serial.print("Angle x:\t"), Serial.println(angle[0]);
    Serial.print("Angle y:\t"), Serial.println(angle[1]), Serial.println();
    //Serial.print("Angle z:\t"), Serial.println(angle[2]);
  }
}

void MPU6050_setup()
{
  //Skrur på MPU6050en
  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x00);
  Wire.endTransmission();

  //Konfigurerer gyroen
  Wire.beginTransmission(0x68);
  Wire.write(0x1b);
  Wire.write(0x08); //500grader/s
  Wire.endTransmission();

  //Konfigurerer akselerometeret
  Wire.beginTransmission(0x68);
  Wire.write(0x1c);
  Wire.write(0x08); //4g
  Wire.endTransmission();
}

void read_MPU6050_data()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3b);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  while (Wire.available())
  {
    accRaw[0] = Wire.read() << 8 | Wire.read();
    accRaw[1] = Wire.read() << 8 | Wire.read();
    accRaw[2] = Wire.read() << 8 | Wire.read();

    tempRaw = Wire.read() << 8 | Wire.read();

    gyroRaw[0] = Wire.read() << 8 | Wire.read();
    gyroRaw[1] = Wire.read() << 8 | Wire.read();
    gyroRaw[2] = Wire.read() << 8 | Wire.read();
  }
}