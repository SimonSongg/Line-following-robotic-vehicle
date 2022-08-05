#include <Wire.h>
#include <MsTimer2.h>
#include <TimerOne.h>
#include <LiquidCrystal.h>

#define DEBUG

#define PIN_LCD_RS 4
#define PIN_LCD_E 5
#define PIN_LCD_DB4 6
#define PIN_LCD_DB5 7
#define PIN_LCD_DB6 8
#define PIN_LCD_DB7 9
#define PIN_TCRT_L A6
#define PIN_TCRT_R A7

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_DB4, PIN_LCD_DB5, PIN_LCD_DB6, PIN_LCD_DB7);

void setup()
{
#ifdef DEBUG
    Serial.begin(9600);
#endif

    //LCD
    lcd.begin(16, 2);

    //TimerOne (LCD refresh)
    Timer1.initialize(200000);
    Timer1.attachInterrupt(LCDPrint);
}

void loop()
{
}

void LCDPrint()
{
    int sensorL = analogRead(A6);
    int sensorR = analogRead(A7);
    float error = sensorL - sensorR;
    lcd.clear();
    lcd.print("L:");
    lcd.print(sensorL);
    lcd.setCursor(8, 0);
    lcd.print("R:");
    lcd.print(sensorR);
    lcd.setCursor(0, 1);
    lcd.print(String(error, 2));
}