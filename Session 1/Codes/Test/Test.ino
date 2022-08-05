#include <Wire.h>

#define StartPause 3500  //ms
#define Distance 2089  //100 --> 13cm
#define CommunicationAddress 42  //default. No need to change.

long unsigned int target = 0x80000000;  //The initial value of the encoder is 0x80000000. Do not change!!!!!!!!

void setup() 
{
  Initialize();
  MoveForward(20);
  while(1)
  {
    if(Distance + target < readEncoder(false, false))
    {
      break;
    }
  }
  Stop();
}

void loop() {
}

void Initialize()
{
  // Setup the serial ports
  Serial.begin(57600);    // This is the hardware port connected to the PC through the USB connection
  Wire.begin();

  // Wait 3.5 seconds
  delay(3500);
  
  Serial.println("Computer communication completed!");  // Send some text to the PC
  Serial.println("Initialized successfully!");  // Send some text to the PC
}

void MoveForward(int intMotorSpeed)
{
  Wire.beginTransmission(CommunicationAddress);
  Wire.write("sa");
  Wire.write(intMotorSpeed - 4);
  Wire.write(0);
  Wire.write(intMotorSpeed - 4);
  Wire.write(0);
  Wire.write(intMotorSpeed);
  Wire.write(0);
  Wire.write(intMotorSpeed);
  Wire.write(0);
  Wire.endTransmission();
}

void Turn(bool LR, int intMotorSpeed)  //false == Left  //true == right
{
  Wire.beginTransmission(CommunicationAddress);
  if(LR == false)
  {
    Wire.write("barrff");
  }
  else
  {
    Wire.write("baffrr");
  }
  Wire.write(intMotorSpeed - 4);
  Wire.write(0);
  Wire.write(intMotorSpeed - 4);
  Wire.write(0);
  Wire.write(intMotorSpeed);
  Wire.write(0);
  Wire.write(intMotorSpeed);
  Wire.write(0);
  Wire.endTransmission();
}

void Stop()
{
  Wire.beginTransmission(CommunicationAddress);
  Wire.write("ha");
  Wire.endTransmission();
}

long unsigned int readEncoder(bool LR, bool isExportData)  //true == Right  //false == Left
{
  long unsigned int encoder1 = 0;
  long unsigned int encoder2 = 0;
  Wire.beginTransmission(CommunicationAddress);
  Wire.write("i");  //Maybe one command, but it isn't recorded in command file.
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(CommunicationAddress,8);
  delay(10);
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

  if(isExportData == true)
  {
    Serial.println("Encoder Left read text!");  // Send some text to the PC
    Serial.println(encoder1);  // Send some text to the PC
    Serial.println("Encoder Right read text!");  // Send some text to the PC
    Serial.println(encoder2);  // Send some text to the PC
  }

  if(LR == true)  //true == Right
  {
    return encoder2;
  }
  else  //false == Left
  {
    return encoder1;
  }
}
