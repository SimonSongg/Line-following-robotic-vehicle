#include <Wire.h>
#include <MsTimer2.h>
#include <TimerOne.h>
#include <LiquidCrystal.h>

//#define DEBUG
//#define VF

#define PIN_FREQ_VF 11

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

#define SPEED_BASE 15

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

#ifdef VF
    pinMode(PIN_FREQ_VF, OUTPUT);
#endif

    //Wire
    Wire.begin();

    //LCD
    lcd.begin(16, 2);

    //Start LED
    pinMode(PIN_LED_START, OUTPUT);

    //Start Button
    pinMode(PIN_BTN_START, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_START), VehicleStartStop, FALLING);

    //TimerOne (LCD refresh)
    Timer1.initialize(200000);
    Timer1.attachInterrupt(LCDPrint);

    //MsTimer2
    MsTimer2::set(4, PID);
    MsTimer2::start();
}

void loop()
{
    //empty!!!
}

inline void VehicleStartStop()
{
    if (vehicleStart)
    {
        pidStart = false;
        vehicleStart = false;
        digitalWrite(PIN_LED_START, LOW);
        error = 0;
        errorSum = 0;
        errorOld = 0;
        speedL = 0;
        speedR = 0;
    }
    else
    {
        vehicleStart = true;
        pidStart = true;
        digitalWrite(PIN_LED_START, HIGH);
    }
}

void PIDParametersRead()
{
    Kp = analogRead(PIN_PM_KP) / (float)6.82;
    Ki = analogRead(PIN_PM_KI) / (float)102.3;
    Kd = analogRead(PIN_PM_KD) / (float)102.3;
}

inline void PID()
{
    if (pidStart)
    {
#ifdef VF
        digitalWrite(PIN_FREQ_VF, HIGH);
#endif

        leftValue = analogRead(PIN_TCRT_L);
        rightValue = analogRead(PIN_TCRT_R);
        errorOld = error;
        error = (leftValue * (-1.75) + rightValue * 1.75) / (leftValue + rightValue);
        errorSum += error;
        float speedDiff = error * Kp + errorSum * Ki + (error - errorOld) * Kd; // Calculate the result
#ifdef DEBUG
        Serial.println(speedDiff);
#endif

        speedL = constrain(SPEED_BASE - speedDiff, -90, 90);
        speedR = constrain(SPEED_BASE + speedDiff, -90, 90);

#ifdef DEBUG
        Serial.print("L:");
        Serial.print(speedL);
        Serial.print("\t");
        Serial.print("R:");
        Serial.println(speedR);
#endif

        interrupts();
        MotorMove(speedL, speedR, speedL, speedR);

#ifdef VF
        digitalWrite(PIN_FREQ_VF, LOW);
#endif
    }
    else
    {
        interrupts();
        MotorStop();
        PIDParametersRead();
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
    lcd.setCursor(6, 1);
    lcd.print(String(Ki, 2));
    lcd.setCursor(11, 1);
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
