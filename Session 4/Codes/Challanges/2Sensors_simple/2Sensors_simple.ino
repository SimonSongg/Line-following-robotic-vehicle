#include <Wire.h>
#include <MsTimer2.h>
#include <TimerOne.h>
#include <LiquidCrystal.h>

//#define DEBUG
//#define VF

#define PIN_PM_KP A0
#define PIN_BTN_START 2
#define PIN_LED_START 10
#define PIN_LCD_RS 4
#define PIN_LCD_E 5
#define PIN_LCD_DB4 6
#define PIN_LCD_DB5 7
#define PIN_LCD_DB6 8
#define PIN_LCD_DB7 9
#define PIN_TCRT_L A6
#define PIN_TCRT_R A7

#define SPEED_FORWARD 20

#define SPEED_SPIN 50

#define COM_BASEBOARD_ADDRESS 42

#define TCRT_BLK_VALUE 700

volatile int speedDiff = 0;

float Kp = 0;

unsigned int AccNum = 0;

volatile int speedL = 0;
volatile int speedR = 0;

bool vehicleState = false;
volatile unsigned char vehicleMoveState = 0; //0->forward    //1->left   2->right

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
    pinMode(13, OUTPUT);
#endif

    //Wire
    Wire.begin();

    //LCD
    lcd.begin(16, 2);

    //Start LED
    pinMode(PIN_LED_START, OUTPUT);

    //Start Button
    pinMode(PIN_BTN_START, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_START), VehicleStart, FALLING);

    //TimerOne (LCD refresh)
    Timer1.initialize(200000);
    Timer1.attachInterrupt(LCDPrint);

    //MsTimer2
    MsTimer2::set(10, PIDStart);
}

void loop()
{
    if (vehicleStart)
    {
        if (pidStart)
        {
#ifdef VF
            digitalWrite(13, HIGH);
#endif
            PID();
#ifdef DEBUG
            Serial.print("L:");
            Serial.print(speedL);
            Serial.print("\t");
            Serial.print("R:");
            Serial.println(speedR);
#endif
            MotorMove(speedL, speedR, speedL, speedR);
            pidStart = false;
#ifdef VF
            digitalWrite(13, LOW);
#endif
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

inline void PIDParametersRead()
{
    Kp = analogRead(PIN_PM_KP) / (float)51.15;
}

inline void PID()
{
    leftValue = analogRead(PIN_TCRT_L);
    rightValue = analogRead(PIN_TCRT_R);
    if (leftValue > TCRT_BLK_VALUE && rightValue < TCRT_BLK_VALUE)
    {
        vehicleMoveState = 1;
    }
    else if (leftValue < TCRT_BLK_VALUE && rightValue > TCRT_BLK_VALUE)
    {
        vehicleMoveState = 2;
    }
    else if (leftValue < TCRT_BLK_VALUE && rightValue < TCRT_BLK_VALUE)
    {
        vehicleMoveState = 0;
    }

    switch (vehicleMoveState)
    {
    case 0:
        speedL = SPEED_FORWARD;
        speedR = SPEED_FORWARD;
        break;

    case 1:
        speedL = -SPEED_SPIN;
        speedR = SPEED_SPIN;
        break;

    case 2:
        speedL = SPEED_SPIN;
        speedR = -SPEED_SPIN;
        break;
    }
}

inline void LCDPrint()
{
    lcd.clear();
    lcd.print(vehicleState ? "M" : "S");
    lcd.setCursor(2, 0);
    lcd.print(speedL);
    lcd.setCursor(5, 0);
    lcd.print(speedR);
    lcd.setCursor(8, 0);
    lcd.print(AccNum);
    lcd.setCursor(0, 1);
    lcd.print(String(Kp, 2));
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
