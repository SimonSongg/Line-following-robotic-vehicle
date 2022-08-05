#include <Wire.h>
#include <MsTimer2.h>

#define COM_BASEBOARD_ADDRESS 42
#define LED_PIN 10

volatile long unsigned int target = 10000 + 0x80000000;
volatile long unsigned int encoderValue = 0;
volatile bool flag = false;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  pinMode(LED_PIN, OUTPUT);
  readEncoder();
  MsTimer2::set(5, flagChange);
  MsTimer2::start();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(flag)
  {
    digitalWrite(LED_PIN, HIGH);
    MoveForward(20);
    readEncoder();
    digitalWrite(LED_PIN, LOW);
    if(encoderValue > target)
    {
      Stop();
      MsTimer2::stop();
    }
    flag = false;
  }
}

inline void flagChange()
{
  flag = true;
}

inline void Stop()
{
  Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
  Wire.write("ha");
  Wire.endTransmission();
}

inline void MoveForward(byte motorSpeed)
{
  Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
  //Wire.write("sa");
  Wire.write("bafrfr");
  Wire.write(motorSpeed - 4);
  Wire.write(0);
  Wire.write(motorSpeed);
  Wire.write(0);
  Wire.write(motorSpeed);
  Wire.write(0);
  Wire.write(motorSpeed - 4);
  Wire.write(0);
  Wire.endTransmission();
}

inline void readEncoder()  //true == Right  //false == Left
{
  long unsigned int encoder1 = 0;
  long unsigned int encoder2 = 0;
  Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
  Wire.write("i");  //Maybe one command, but it isn't recorded in command file.
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(COM_BASEBOARD_ADDRESS,8);
  delay(1);
  if(Wire.available()==8)
  {
    encoder1 = (long unsigned int) Wire.read();
    encoder1 += ((long unsigned int) Wire.read() <<8);
    encoder1 += ((long unsigned int) Wire.read() <<16);
    encoder1 += ((long unsigned int) Wire.read() <<24);
    encoder2 = (long unsigned int) Wire.read();
    encoder2 += ((long unsigned int) Wire.read() <<8);
    encoder2 += ((long unsigned int) Wire.read() <<16);
    encoder2 += ((long unsigned int) Wire.read() <<24);
  }

  encoderValue = encoder1;
}
