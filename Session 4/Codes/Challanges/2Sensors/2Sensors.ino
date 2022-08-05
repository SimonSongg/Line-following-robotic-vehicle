#include <Wire.h>
#include <MsTimer2.h>
#include <TimerOne.h>
#include <LiquidCrystal.h>

//#define DEBUG

#define PIN_PM_KP A0
#define PIN_PM_KI A1
#define PIN_PM_KD A2
#define PIN_BTN_START 2
#define PIN_LED_START 10
#define PIN_LCD_RS 4
#define PIN_LCD_E 5
#define PIN_LCD_DB4 6
#define PIN_LCD_DB5 7
#define PIN_LCD_DB6 8
#define PIN_LCD_DB7 9
#define PIN_TCRT_L A7
#define PIN_TCRT_R A6

#define SPEED_BASE 18

#define COM_BASEBOARD_ADDRESS 42

volatile int speedDiff = 0;

float Ki = 0;
float Kp = 0;
float Kd = 0;

float error = 0;
float errorSum = 0;
float errorOld = 0;

volatile int speedL = 0;
volatile int speedR = 0;

bool vehicleState = false;

volatile bool pidStart = false;
bool vehicleStart = false;

unsigned int leftValue = 0;
unsigned int rightValue = 0;

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
    lcd.begin(16,2);

    //Start LED
    pinMode(PIN_LED_START, OUTPUT);

    //Start Button
    pinMode(PIN_BTN_START, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_START), VehicleStart, FALLING);

    //TimerOne (LCD refresh)
    Timer1.initialize(200000);
    Timer1.attachInterrupt(LCDPrint);

    //MsTimer2
    MsTimer2::set(4, PIDStart);
}

void loop()
{
    if (vehicleStart)
    {
        if(pidStart)
        {
            PID();
            MotorMove(speedL, speedR, speedL, speedR);
            pidStart = false;
        }
    }
    else
    {
        PIDParametersRead();
    }
    
}

void PIDStart()
{
    pidStart = true;
}

void VehicleStop()
{
    MotorStop();
    digitalWrite(PIN_LED_START, LOW);
    vehicleStart = false;
    MsTimer2::stop();
}

inline void VehicleStart()
{
    vehicleStart = true;
    digitalWrite(PIN_LED_START, HIGH);
    MsTimer2::start();
}

void PIDParametersRead()
{
    Kp = analogRead(PIN_PM_KP) / (float)10.23;
    Ki = analogRead(PIN_PM_KI) / (float)10.23;
    Kd = analogRead(PIN_PM_KD) / (float)10.23;
}

void PID()
{
    leftValue = analogRead(PIN_TCRT_L);
    rightValue = analogRead(PIN_TCRT_R);
    errorOld = error;
    error = (leftValue * (-1.75) + rightValue * 1.75) / (leftValue + rightValue);
    errorSum += error;
    float speedDiff = error * Kp + errorSum * Ki + (error - errorOld) * Kd; // Calculate the result
    #ifdef DEBUG
    Serial.println(speedDiff);
    #endif
    if(speedDiff > 0)
    {
        speedL = constrain(SPEED_BASE - speedDiff, -90, 90);
        speedR = constrain(SPEED_BASE + speedDiff, -90, 90);
    }
    else if(speedDiff < 0)
    {
        speedL = constrain(SPEED_BASE - speedDiff, -90, 90);
        speedR = constrain(SPEED_BASE + speedDiff, -90, 90);
    }
    else
    {
        speedL = SPEED_BASE;
        speedR = SPEED_BASE;
    }
}

void LCDPrint()
{
    lcd.clear();
    lcd.print(vehicleState ? "M" : "S");
    lcd.setCursor(2, 0);
    lcd.print(speedL);
    lcd.setCursor(6, 0);
    lcd.print(speedR);
    lcd.setCursor(10, 0);
    lcd.print(String(error, 2));
    lcd.setCursor(0, 1);
    lcd.print(String(Kp, 2));
    lcd.setCursor(5, 1);
    lcd.print(String(Ki, 2));
    lcd.setCursor(10, 1);
    lcd.print(String(Kd, 2));
}

inline void MotorMove(char motor1, char motor2, char motor3, char motor4)
{
    Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
    Wire.write("ba");
    Wire.write((motor1 > 0) ? "f" : "r");
    Wire.write((motor2 > 0) ? "r" : "f");
    Wire.write((motor3 > 0) ? "r" : "f");
    Wire.write((motor4 > 0) ? "f" : "r");
    Wire.write(abs(motor1));
    Wire.write(0);
    Wire.write(abs(motor2));
    Wire.write(0);
    Wire.write(abs(motor3));
    Wire.write(0);
    Wire.write(abs(motor4));
    Wire.write(0);
    Wire.endTransmission();
    vehicleState = true;
}

inline void MotorStop()
{
    speedL = 0;
    speedR = 0;
    speedDiff = 0;
    Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
    Wire.write("ha");
    Wire.endTransmission();
    vehicleState = false;
}
