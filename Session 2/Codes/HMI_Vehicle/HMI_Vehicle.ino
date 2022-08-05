#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <TimerOne.h>
#include <LiquidCrystal.h>
#include "pitches.h"

//Pin Define
#define PIN_BTN_TASK_START 2
#define PIN_BTN_TASK_SELECT 12
#define PIN_SWH_DIRECTION_SELECT 4
#define PIN_EC_SPEED_SET_EA 3
#define PIN_EC_SPEED_SET_EB 7
#define PIN_EC_TARGET_E A0
#define PIN_PM_SPEED_LIMIT A6
#define PIN_MICROPHONE A7
#define PIN_LCD_RS 5
#define PIN_LCD_E 6
#define PIN_LCD_DB4 8
#define PIN_LCD_DB5 9
#define PIN_LCD_DB6 10
#define PIN_LCD_DB7 11
#define PIN_LED_MOVING 13
#define PIN_LED_STOP A1
#define PIN_LED_TASK1 A2
#define PIN_LED_TASK2 0
#define PIN_LED_TASK3 1
#define PIN_BUZZER A3
//PIN_BASEBOARD_SDA A4
//PIN_BASEBOARD_SCL A5

#define COM_BASEBOARD_ADDRESS 42  //Defined in the baseboard! DO NOT CHANGE!!!
#define COM_PC_ADDRESS 57600  // This is the hardware port connected to the PC through the USB connection.
#define MAX_TARGET 3.95
#define MIN_TARGET 2.05
#define MAX_VOICE 700
#define MIN_VOICE 10
#define START_PAUSE_TIME 1000 //ms
#define TASK_2_TURN_TARGET 0.5
#define TASK_2_TURN_SPEED 50
#define NOTE_AMOUNT 90
#define BPM 92
#define TASK_OR_FUN 1 //0=>task //1=>fun
#define SINGLE_NOTE_FREQ NOTE_C4

const unsigned int notes[NOTE_AMOUNT][2] = {
  {NOTE_C5, 8}, {0, 8}, {NOTE_C5, 8}, {0, 8}, {NOTE_G5, 8}, {0, 8}, {NOTE_G5, 8}, {0, 8}, {NOTE_A5, 8}, {0, 8}, {NOTE_A5, 8}, {0, 8}, {NOTE_G5, 8}, {0, 8}, {0, 4},
  {NOTE_F5, 8}, {0, 8}, {NOTE_F5, 8}, {0, 8}, {NOTE_E5, 8}, {0, 8}, {NOTE_E5, 8}, {0, 8}, {NOTE_D5, 8}, {0, 8}, {NOTE_D5, 8}, {0, 8}, {NOTE_C5, 8}, {0, 8}, {0, 4},
  {NOTE_G5, 8}, {0, 8}, {NOTE_G5, 8}, {0, 8}, {NOTE_F5, 8}, {0, 8}, {NOTE_F5, 8}, {0, 8}, {NOTE_E5, 8}, {0, 8}, {NOTE_E5, 8}, {0, 8}, {NOTE_D5, 8}, {0, 8}, {0, 4},
  {NOTE_G5, 8}, {0, 8}, {NOTE_G5, 8}, {0, 8}, {NOTE_F5, 8}, {0, 8}, {NOTE_F5, 8}, {0, 8}, {NOTE_E5, 8}, {0, 8}, {NOTE_E5, 8}, {0, 8}, {NOTE_D5, 8}, {0, 8}, {0, 4},
  {NOTE_C5, 8}, {0, 8}, {NOTE_C5, 8}, {0, 8}, {NOTE_G5, 8}, {0, 8}, {NOTE_G5, 8}, {0, 8}, {NOTE_A5, 8}, {0, 8}, {NOTE_A5, 8}, {0, 8}, {NOTE_G5, 8}, {0, 8}, {0, 4},
  {NOTE_F5, 8}, {0, 8}, {NOTE_F5, 8}, {0, 8}, {NOTE_E5, 8}, {0, 8}, {NOTE_E5, 8}, {0, 8}, {NOTE_D5, 8}, {0, 8}, {NOTE_D5, 8}, {0, 8}, {NOTE_C5, 8}, {0, 8}, {0, 4}
};

volatile float target = 3;  //unit: m
volatile byte speedValue = 20;
volatile byte speedLimit = 70;
volatile bool directionValue = false;  //true=>right  //false=>left
volatile byte taskIndex = 1;
volatile bool startFlag = false;

const byte debounceDelay = 100;

//LCD Initialize
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_DB4, PIN_LCD_DB5, PIN_LCD_DB6, PIN_LCD_DB7);

void setup() 
{
  Initialize();
}

void loop() {
  if(startFlag == false)
  {
    delay(100);
    readSpeedLimit();
    readVoice();
  }
  else
  {
    delay(START_PAUSE_TIME);
    //noInterrupts();
    //disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SWH_DIRECTION_SELECT));
    //disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BTN_TASK_SELECT));
    switch(taskIndex)
    {
      case 1:
        task1();
        break;
      case 2:
        task2();
        break;
      case 3:
        task3();
        break;
    }
    //Turn off taskLED / buzzer / MsTimer2
    digitalWrite(PIN_LED_TASK1, LOW);
    digitalWrite(PIN_LED_TASK2, LOW);
    digitalWrite(PIN_LED_TASK3, LOW);
    noTone(PIN_BUZZER);
    Timer1.stop();
    
    lcd.setCursor(10, 1);
    lcd.print("    ");

    startFlag = false;

    //interrupts();
    //enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SWH_DIRECTION_SELECT));
    //enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BTN_TASK_SELECT));
  }
}

void Initialize()
{
  //Setup serial ports
  Serial.begin(COM_PC_ADDRESS);
  Serial.println("Communicate to PC successfully!");
  Wire.begin();
  Serial.println("Communicate to baseboard successfully!");

  //Setup pin modes
  pinMode(PIN_BTN_TASK_START, INPUT_PULLUP);
  pinMode(PIN_BTN_TASK_SELECT, INPUT_PULLUP);
  pinMode(PIN_SWH_DIRECTION_SELECT, INPUT_PULLUP);
  pinMode(PIN_EC_SPEED_SET_EA, INPUT_PULLUP);
  pinMode(PIN_EC_SPEED_SET_EB, INPUT_PULLUP);
  pinMode(PIN_EC_TARGET_E, INPUT_PULLUP);
  //pinMode(PIN_PM_SPEED_LIMIT, INPUT);
  pinMode(PIN_LED_MOVING, OUTPUT);
  pinMode(PIN_LED_STOP, OUTPUT);
  pinMode(PIN_LED_TASK1, OUTPUT);
  pinMode(PIN_LED_TASK2, OUTPUT);
  pinMode(PIN_LED_TASK3, OUTPUT);
  Serial.println("Set pin mode successfully!");

  //Setup speed limit and direction
  speedLimit = analogRead(PIN_PM_SPEED_LIMIT) / 51.15 + 60;
  
  Serial.println("Set speed limit successfully!");

  //Setup LCD
  lcd.begin(16, 2);
  //Task
  lcd.print('T');
  lcd.print(taskIndex);
  //Speed value / limit
  lcd.setCursor(3, 0);
  lcd.print('S');
  lcd.print(speedValue);
  lcd.print('/');
  lcd.print(speedLimit);
  //Direction
  lcd.setCursor(12, 0);
  lcd.print('D');
  if(directionValue)
  {
    lcd.print('R');
  }
  else
  {
    lcd.print('L');
  }
  //Target
  lcd.setCursor(0, 1);
  lcd.print("TGT");
  lcd.print(String(target, 1));
  Serial.println("Set LCD successfully!");

  //Setup interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_TASK_START), ISR_TaskStart, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_EC_SPEED_SET_EA), ISR_SpeedSetAndTarget, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SWH_DIRECTION_SELECT), ISR_DirecSelect, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BTN_TASK_SELECT), ISR_TaskSelect, FALLING);
  interrupts();
  Serial.println("Set interrupts successfully!");

  //Setup StopLED
  digitalWrite(PIN_LED_STOP, HIGH);
  
  Serial.println("Initialize Complete!");
  Serial.end();

  digitalWrite(PIN_LED_TASK2, LOW);
  digitalWrite(PIN_LED_TASK3, LOW);
}

void ISR_TaskStart()
{
  if(startFlag == false)
  {
    static unsigned long lastDebounceTime = 0;
    if(millis() - lastDebounceTime > debounceDelay)
    {
      lastDebounceTime = millis();
      lcd.setCursor(10, 1);
      lcd.print("ST!!");
      startFlag = true;
    }
  }
}

void ISR_TaskSelect()
{
  if(startFlag == false)
  {
    static unsigned long lastDebounceTime = 0;
    if(millis() - lastDebounceTime > debounceDelay)
    {
      lastDebounceTime = millis();
      if(taskIndex == 3)
      {
        taskIndex = 1;
      }
      else
      {
        ++taskIndex;
      }
      lcd.setCursor(0, 0);
      lcd.print('T');
      lcd.print(taskIndex);
    }
  }
}

void ISR_DirecSelect()
{
  if(startFlag == false)
  {
    lcd.setCursor(12, 0);
    if(digitalRead(PIN_SWH_DIRECTION_SELECT) == HIGH)
    {
      directionValue = true;
      lcd.print("DR");
    }
    else
    {
      directionValue = false;
      lcd.print("DL");
    }
  }
}

void ISR_SpeedSetAndTarget()
{
  if(startFlag == false)
  {
    bool isTarget;
    if(digitalRead(PIN_EC_TARGET_E) == HIGH)
    {
      isTarget = false;
    }
    else
    {
      isTarget = true;
    }
  
    if(digitalRead(PIN_EC_SPEED_SET_EB) == HIGH)
    {
      if(isTarget)
      {
        if(target < (float)MAX_TARGET)
        {
          target += 0.1;
        }
      }
      else
      {
        if(speedValue < speedLimit)
        {
          ++speedValue;
        }
      }
    }
    else
    {
      if(isTarget)
      {
        if(target > (float)MIN_TARGET)
        {
          target -= 0.1;
        }
      }
      else
      {
        if(speedValue > 0)
        {
          --speedValue;
        }
      }
    }
  

    if(isTarget)
    {
      lcd.setCursor(0, 1);
      lcd.print("          ");
      lcd.setCursor(0, 1);
      lcd.print("TGT");
      lcd.print(String(target, 1));
    }
    else
    {
      lcd.setCursor(3, 0);
      lcd.print("        ");
      lcd.setCursor(3, 0);
      lcd.print('S');
      lcd.print(speedValue);
      lcd.print('/');
      lcd.print(speedLimit);
    }
  }
}

inline void readSpeedLimit()
{
  int tempValue = analogRead(PIN_PM_SPEED_LIMIT) / 51.15 + 60;
  if(tempValue != speedLimit)
  {
    speedLimit = tempValue;
    if(speedValue > speedLimit)
    {
      speedValue = speedLimit;
    }
    lcd.setCursor(3, 0);
    lcd.print("        ");
    lcd.setCursor(3, 0);
    lcd.print('S');
    lcd.print(speedValue);
    lcd.print('/');
    lcd.print(speedLimit);
  }
}

inline void readVoice()
{
  int tempValue = analogRead(PIN_MICROPHONE);
  if(tempValue > MAX_VOICE || tempValue < MIN_VOICE)
  {
    startFlag = true;
  }
}

inline void playMelody()
{
  static bool LEDstatus = true;
  static unsigned int noteIndex = 0;
  if(LEDstatus)
  {
    digitalWrite(PIN_LED_TASK1, HIGH);
  }
  else
  {
    digitalWrite(PIN_LED_TASK1, LOW);
  }
  LEDstatus = !LEDstatus;
  tone(PIN_BUZZER, notes[noteIndex][0]);
  ++noteIndex;
  Timer1.setPeriod((unsigned long)(240000000 / (notes[noteIndex - 1][1] * BPM)));
  if(noteIndex == NOTE_AMOUNT)
  {
    noteIndex = 0;
  }
}

inline void playNote()
{
  static bool enable = true;
  if(enable)
  {
    switch(taskIndex)
    {
      case 2:
        digitalWrite(PIN_LED_TASK2, HIGH);
        break;
      case 3:
        digitalWrite(PIN_LED_TASK3, HIGH);
        break;
    }
    tone(PIN_BUZZER, SINGLE_NOTE_FREQ);
  }
  else
  {
    switch(taskIndex)
    {
      case 2:
        digitalWrite(PIN_LED_TASK2, LOW);
        break;
      case 3:
        digitalWrite(PIN_LED_TASK3, LOW);
        break;
    }
    noTone(PIN_BUZZER);
  }
  enable = !enable;
}

void task1()
{
  if(TASK_OR_FUN)
  {
    Timer1.initialize(1000000);
    Timer1.attachInterrupt(playMelody);
  }
  else
  {
    digitalWrite(PIN_LED_TASK1, HIGH);
    tone(PIN_BUZZER, SINGLE_NOTE_FREQ);
  }

  Turn(directionValue, speedValue);
  WaitTarget(target, !directionValue, false);
  Stop();
}

void task2()
{
  if(TASK_OR_FUN)
  {
    Timer1.initialize(1000000);
    Timer1.attachInterrupt(playMelody);
  }
  else
  {
    Timer1.initialize(250000);
    Timer1.attachInterrupt(playNote);
  }

  for(int i = 0; i < 4; ++i)
  {
    MoveForward(speedValue);
    WaitTarget(target, !directionValue, false);
    Stop();
    Turn(directionValue, TASK_2_TURN_SPEED);
    WaitTarget(TASK_2_TURN_TARGET, !directionValue, false);
    Stop();
  }
  Stop();
}

void task3()
{
  if(TASK_OR_FUN)
  {
    Timer1.initialize(1000000);
    Timer1.attachInterrupt(playMelody);
  }
  else
  {
    Timer1.initialize(500000);
    Timer1.attachInterrupt(playNote);
  }

  MoveForward(speedValue);
  WaitTarget(target, !directionValue, false);
  Stop();
}

inline void MoveForward(byte motorSpeed)
{
  Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
  //Wire.write("sa");
  Wire.write("bafrfr");
  Wire.write(motorSpeed - 4);
  Wire.write(0);
  Wire.write(motorSpeed);
  Wire.write(0);
  Wire.write(motorSpeed);
  Wire.write(0);
  Wire.write(motorSpeed - 4);
  Wire.write(0);
  Wire.endTransmission();
  digitalWrite(PIN_LED_MOVING, HIGH);
  digitalWrite(PIN_LED_STOP, LOW);
  lcd.setCursor(12, 1);
  lcd.print("MF");
}

inline void Turn(bool LR, byte motorSpeed)  //false == Left  //true == right
{
  Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
  if(LR == false)
  {
    Wire.write("barrff");
  }
  else
  {
    Wire.write("baffrr");
  }
  Wire.write(motorSpeed - 4);
  Wire.write(0);
  Wire.write(motorSpeed);
  Wire.write(0);
  Wire.write(motorSpeed);
  Wire.write(0);
  Wire.write(motorSpeed - 4);
  Wire.write(0);
  Wire.endTransmission();
  digitalWrite(PIN_LED_MOVING, HIGH);
  digitalWrite(PIN_LED_STOP, LOW);
  lcd.setCursor(12, 1);
  if(LR)
  {
    lcd.print("TR");
  }
  else
  {
    lcd.print("TL");
  }
}

inline void Stop()
{
  Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
  Wire.write("ha");
  Wire.endTransmission();
  digitalWrite(PIN_LED_MOVING, LOW);
  digitalWrite(PIN_LED_STOP, HIGH);
  lcd.setCursor(12, 1);
  lcd.print("ST");
}

inline void WaitTarget(float target, bool LR, bool isExportData)//target(unit: m) //should be positive
{
  long unsigned int startPoint = readEncoder(LR, isExportData);
  while(true)
  {
    if((target * 874.8932 - 10.5749) < abs(readEncoder(LR, isExportData) - startPoint))
    {
      break;
    }
  }
}

inline long unsigned int readEncoder(bool LR, bool isExportData)  //true == Right  //false == Left
{
  long unsigned int encoder1 = 0;
  long unsigned int encoder2 = 0;
  Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
  Wire.write("i");  //Maybe one command, but it isn't recorded in command file.
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(COM_BASEBOARD_ADDRESS,8);
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
    Serial.println("Encoder Left read text!");
    Serial.println(encoder1);
    Serial.println("Encoder Right read text!");
    Serial.println(encoder2);
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
