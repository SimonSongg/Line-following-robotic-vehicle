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

#define SPEED_BASE 45

#define COM_BASEBOARD_ADDRESS 42

volatile int speedDiff = 0;

float Ki = 0;
float Kp = 0;
float Kd = 0;

volatile double distance = 0;

volatile unsigned long encoderp1 = 0;
volatile unsigned long encoderp2 = 0;
volatile unsigned long encoder1 = 0;
volatile unsigned long encoder2 = 0;

volatile float error = 0;
volatile float errorSum = 0;
volatile float errorOld = 0;

volatile int speedL = 0;
volatile int speedR = 0;

bool vehicleState = false;

volatile bool pidStart = false;
bool vehicleStart = false;

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_DB4, PIN_LCD_DB5, PIN_LCD_DB6, PIN_LCD_DB7);

void setup()
{
#ifdef DEBUG
    Serial.begin(9600);
    Serial.println("Start!");
#endif

    //Wire
    Wire.begin();

#ifdef VF
    pinMode(PIN_FREQ_VF, OUTPUT);
#endif

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
    //empty!
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
        //distance = 0;
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
    Kp = analogRead(PIN_PM_KP) / (float)10.23;
    Ki = analogRead(PIN_PM_KI) / (float)10.23;
    Kd = analogRead(PIN_PM_KD) / (float)10.23;
}

void PID()
{
    interrupts();
    DistanceHandler();
    if (pidStart)
    {
        float speedDiff = 0;
#ifdef VF
        digitalWrite(PIN_FREQ_VF, HIGH);
#endif
        if (distance > 147.85)
        {
            bool blackFlag = false;
            float sumUpper = 0;
            float sumLower = 0;
            unsigned int fullData[8];
            unsigned int rawData[16];
            Wire.requestFrom(9, 16);
            for (int i = 0; i < 16; ++i)
            {
                rawData[i] = Wire.read();
            }
            for (int i = 0; i < 8; ++i)
            {
                fullData[i] = 1023 - (rawData[i * 2] << 2 | rawData[i * 2 + 1]);

#ifdef DEBUG
                Serial.print(i);
                Serial.print(":");
                Serial.print(fullData[i]);
                Serial.print("\t");
#endif

                if (fullData[i] > 150)
                {
                    blackFlag = true;
                }
                sumUpper += fullData[i] * (-4.55 + 1.3 * i);
                sumLower += fullData[i];
            }

#ifdef DEBUG
            Serial.println("");
#endif

            if (blackFlag)
            {
                errorOld = error;
                error = sumUpper / sumLower;
            }
            errorSum += error;
            speedDiff = error * Kp + errorSum * Ki + (error - errorOld) * Kd; // Calculate the result

#ifdef DEBUG
            Serial.println(speedDiff);
#endif
            speedL = constrain(SPEED_BASE + speedDiff, -90, 90);
            speedR = constrain(SPEED_BASE - speedDiff, -90, 90);
        }
        else if (distance > 119.09)
        {
            speedDiff = -30;
            speedL = constrain(SPEED_BASE + speedDiff, -90, 90);
            speedR = constrain(SPEED_BASE - speedDiff, -90, 90);
        }
        else
        {
            speedDiff = 0;
            speedL = 60;
            speedR = 60;
        }

#ifdef VF
        digitalWrite(PIN_FREQ_VF, LOW);
#endif

        MotorMove(speedL, speedR, speedL, speedR);
    }
    else
    {
        interrupts();
        MotorStop();
        PIDParametersRead();
    }
}

inline void DistanceHandler()
{
    ReadEncoder();
    distance += (((signed long)(encoder1 - encoderp1) + (signed long)(encoder2 - encoderp2)) / (double)2) * 0.1143;
    if (distance > 1701.85)
    {
        distance = 0;
    }
}

inline void LCDPrint()
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
