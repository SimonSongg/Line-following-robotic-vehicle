#include <Wire.h>

double magicNumber1 =  220 * 0.8/ (0.068 * 3.1415926);  //234.256 * 1/ (0.068 * 3.1415926)
double magicNumber2 =  220 * 1.4/ (0.068 * 3.1415926);  //234.256 * 1/ (0.068 * 3.1415926)
double magicNumber3 =  220 * 1.4/ (0.068 * 3.1415926);  //234.256 * 1/ (0.068 * 3.1415926)
double magicNumber4 =  220 * 1.4/ (0.068 * 3.1415926);  //234.256 * 1/ (0.068 * 3.1415926)

#define Forward1 (magicNumber1)
#define Forward2 (magicNumber2)
#define Forward3 (magicNumber3)
#define Forward4 (magicNumber4)
#define Turn1 (105) //105 is perfect number!!!!!!!!!!!!!!!!!
#define Turn2 (105)
#define Turn3 (105)
#define Turn4 (105)

long unsigned int encoder1Value = 0;
long unsigned int encoder2Value = 0;

long unsigned int target1 = 0;

byte mode = 0;
byte turnNum = 1;

void runForward()
{
    Wire.write("sa");
    Wire.write(25);
    Wire.write(0);
    Wire.write(25);
    Wire.write(0);
    Wire.write(30);
    Wire.write(0);
    Wire.write(30);
    Wire.write(0);
}

void runRight()
{
    Wire.write("baffrr");
    Wire.write(50);
    Wire.write(0);
    Wire.write(50);
    Wire.write(0);
    Wire.write(50);
    Wire.write(0);
    Wire.write(50);
    Wire.write(0);
}

void setup() {
  //Setup the serial ports
  Serial.begin(57600);    // This is the hardware port connected to the PC through the USB connection
  Wire.begin();
  target1 = 0x80000000 + Forward1;

  // wait 5 seconds
  delay(2000);
  
  Serial.println("software serial simple test!");  // Send some text to the PC
  Wire.beginTransmission(42);
  runForward();

  Wire.endTransmission();
}

void loop() {

  readEncoder();
  Serial.println("Encoder 1 read text!");  // Send some text to the PC
  Serial.println(encoder1Value);  // Send some text to the PC
  Serial.println("Encoder 2 read text!");  // Send some text to the PC
  Serial.println(encoder2Value);  // Send some text to the PC
  
  if (encoder1Value > 0x80040000)
  {
    encoder1Value=0x80000000;
  }
  else if (encoder1Value < 0x7FFC0000)
  {
    encoder1Value=0x80000000;
  }

  if(encoder1Value > target1)
  {
    Serial.println("Traget!!!!!!!!");
  if (mode == 0)
  {
    //Stop all the motors
    Wire.beginTransmission(42);
    Wire.write("ha");
    Wire.endTransmission();
    digitalWrite(13,HIGH);
    delay(500);
    Wire.beginTransmission(42);
    runRight();
    Wire.endTransmission();
    digitalWrite(13,HIGH);
    if(turnNum == 1)
    {
      target1 += Turn1;
      turnNum++;
    }
    else if(turnNum == 2)
    {
      target1 += Turn2;
      turnNum++;
    }
    else if(turnNum == 3)
    {
      target1 += Turn3;
      turnNum++;
    }
    else if(turnNum == 4)
    {
      target1 += Turn4;
      turnNum++;
    }
    mode = 1;
  }
  else if(mode == 1)
  {
    if(turnNum == 5)
    {
      //Stop all the motors
    Wire.beginTransmission(42);
    Wire.write("ha");
    Wire.endTransmission();
    digitalWrite(13,HIGH);
    while(1);
    }
    //Stop all the motors
    Wire.beginTransmission(42);
    Wire.write("ha");
    Wire.endTransmission();
    digitalWrite(13,HIGH);
    delay(500);
    Wire.beginTransmission(42);
    runForward();
    Wire.endTransmission();
    digitalWrite(13,HIGH);
    if(turnNum == 1)
    {
      target1 += Forward1;
    }
    if(turnNum == 2)
    {
      target1 += Forward2;
    }
    if(turnNum == 3)
    {
      target1 += Forward3;
    }
    if(turnNum == 4)
    {
      target1 += Forward4;
    }
    mode = 0;
  }
  }
}

void readEncoder()
{
  long unsigned int encoder1 = 0;
  long unsigned int encoder2 = 0;
  Wire.beginTransmission(42);
  Wire.write("i");
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(42,8);
  delay(10);
  //if(Wire.available()==8)
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
  encoder1Value = encoder1;
  encoder2Value = encoder2;
}
