#include <Wire.h>

unsigned char t;
unsigned int data[16];

void setup()
{
  Wire.begin();       // join i2c bus (address optional for master)
  Serial.begin(9600); // start serial for output
  t = 0;
}
void loop()
{
  Wire.requestFrom(9, 16); // request 16 bytes from slave device #9
  while (Wire.available()) // slave may send less than requested
  {
    data[t] = Wire.read(); // receive a byte as character
    if (t < 15)
      t++;
    else
      t = 0;
  }

  int i = 0;
  for (i = 0; i < 8; ++i)
  {
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(data[i * 2] << 2 | data[i * 2 + 1]);
    Serial.print("\t");
  }
  Serial.println("");

  delay(200);
}
