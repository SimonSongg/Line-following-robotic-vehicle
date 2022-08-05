#include <Wire.h>
#define Period (650)

void Spin()
{
    Wire.write("baffrr");
    Wire.write(50);
    Wire.write(0);
    Wire.write(50);
    Wire.write(0);
    Wire.write(48);
    Wire.write(0);
    Wire.write(48);
    Wire.write(0);
}

long unsigned int encoder1Value = 0;
long unsigned int encoder2Value = 0;

long unsigned int target1 = 0;

void setup() {
  
  //Setup the serial ports
  Serial.begin(57600);    // This is the hardware port connected to the PC through the USB connection
  Wire.begin();
  target1 = 0x80000000 + Period;

  // wait 5 seconds
  delay(2000);
  
  Serial.println("software serial simple test!");  // Send some text to the PC
  
  Wire.beginTransmission(42);
  Spin();
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
    //Stop all the motors
    Wire.beginTransmission(42);
    Wire.write("ha");
    Wire.endTransmission();
    digitalWrite(13,HIGH);
    while(1); //Stop running
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
