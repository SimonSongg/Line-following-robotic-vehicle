#include <Wire.h>
#include <MsTimer2.h>
#include <LiquidCrystal.h>

//#define DEBUG

#define PIN_LCD_RS 4
#define PIN_LCD_E 5
#define PIN_LCD_DB4 6
#define PIN_LCD_DB5 7
#define PIN_LCD_DB6 8
#define PIN_LCD_DB7 9

#define SPEED_BASE 25

#define COM_BASEBOARD_ADDRESS 42

volatile double distance = 0;

volatile unsigned long encoderp1 = 0;
volatile unsigned long encoderp2 = 0;
volatile unsigned long encoder1 = 0;
volatile unsigned long encoder2 = 0;

volatile int speedL = 0;
volatile int speedR = 0;

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_DB4, PIN_LCD_DB5, PIN_LCD_DB6, PIN_LCD_DB7);

void setup()
{
#ifdef DEBUG
    Serial.begin(9600);
    Serial.println("Start!");
#endif

    //Wire
    Wire.begin();

    //LCD
    lcd.begin(16, 2);

    MsTimer2::set(200, LCDPrint);
    MsTimer2::start();
}

void loop()
{
    DistanceHandler();
}

inline void DistanceHandler()
{
    ReadEncoder();
    distance += ((signed long)(encoder1 - encoderp1) + (signed long)(encoder2 - encoderp2)) / (double)2 * 0.1143;
    if (distance > 60000)
    {
        distance = 0;
    }
}

void LCDPrint()
{
    lcd.clear();
    lcd.print(distance);
}

inline void ReadEncoder() //true == Right  //false == Left
{
    encoderp1 = encoder1;
    encoderp2 = encoder2;
    Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
    Wire.write("i");
    Wire.endTransmission();
    Wire.requestFrom(COM_BASEBOARD_ADDRESS, 8);
    while (Wire.available() == 8)
    {
        encoder1 = (unsigned long)Wire.read();
        encoder1 += ((unsigned long)Wire.read() << 8);
        encoder1 += ((unsigned long)Wire.read() << 16);
        encoder1 += ((unsigned long)Wire.read() << 24);
        encoder2 = (unsigned long)Wire.read();
        encoder2 += ((unsigned long)Wire.read() << 8);
        encoder2 += ((unsigned long)Wire.read() << 16);
        encoder2 += ((unsigned long)Wire.read() << 24);
    }
}
