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
#define REAL_TIME_PERIOD 5000 //microsecond
#define MAX_TARGET 3.95
#define MIN_TARGET 2.05
#define MICROPHONE_ENABLE 0
#define MAX_VOICE 700
#define MIN_VOICE 10
#define START_PAUSE_TIME 1000 //ms
#define TASK_2_TURN_TARGET 0.5
#define TASK_2_TURN_SPEED 50
#define NOTE_AMOUNT 90
#define BPM 92
#define TASK_OR_FUN 0 //0=>task //1=>fun
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

volatile bool realTimeControl = false;
volatile byte carState = 0;   //0=>stop   //1=>moveforward   //2=>turn
volatile byte dutySpeedValue = 0;
volatile unsigned long encoderValue = 0x80000000;
volatile unsigned long encoderStartPoint = 0;

volatile unsigned long runTime = 0;

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
    readSpeedLimit();
    if(MICROPHONE_ENABLE)
    {
      readVoice();
    }
  }

  if(realTimeControl)
  {
    switch(carState)
    {
      case 0:
        Stop();
        break;
      case 1:
        MoveForward(dutySpeedValue);
        break;
      case 2:
        Turn(directionValue, dutySpeedValue);
        break;
    }

    encoderValue = readEncoder(!directionValue, true);
    realTimeControl = false;
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
  //Serial.end();

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

      realTimeControl = true;
      
      Timer1.initialize(1000);
      switch(taskIndex)
      {
        case 1:
          Timer1.attachInterrupt(task1);
          break;
        case 2:
          Timer1.attachInterrupt(task2);
         break;
        case 3:
          Timer1.attachInterrupt(task3);
          break;
      }
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
  static bool firstFlag = true;
  static bool LEDstatus = true;
  static unsigned int noteIndex = 0;
  static unsigned long notePeriodCount = 0;
  notePeriodCount += REAL_TIME_PERIOD;
  if(((unsigned long)(240000000 / (notes[noteIndex][1] * BPM)) <= notePeriodCount) || (firstFlag))
  {
    if(firstFlag)
    {
      firstFlag = false;
    }
    else
    {
      noteIndex++;
      if(noteIndex == NOTE_AMOUNT)
      {
        noteIndex = 0;
      }
    }
    notePeriodCount = 0;
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
  static bool firstTime = true;
  runTime += REAL_TIME_PERIOD;
  if(firstTime)
  {
    encoderStartPoint = encoderValue;
    firstTime = false;
  }
  else
  {
  if(TASK_OR_FUN)
  {
    playMelody();
  }
  else
  {
    digitalWrite(PIN_LED_TASK1, HIGH);
    tone(PIN_BUZZER, SINGLE_NOTE_FREQ);
  }
  if(encoderValue - encoderStartPoint > Target())
  {
    firstTime = true;
    carState = 0;
    realTimeControl = true;
    startFlag = false;
    //Turn off taskLED / buzzer / MsTimer2
    digitalWrite(PIN_LED_TASK1, LOW);
    noTone(PIN_BUZZER);
    Timer1.stop();
  }
  else
  {
    carState = 2;
    dutySpeedValue = speedValue;
    realTimeControl = true;
  }
  }
}

void task2()
{
  static bool firstTime = true;
  static byte count = 0;
  static bool turnOrMove = false; //false=>MoveForward  //true=>turn
  runTime += REAL_TIME_PERIOD;
  if(firstTime)
  {
    encoderStartPoint = encoderValue;
    firstTime = false;
  }
  
  if(TASK_OR_FUN)
  {
    playMelody();
  }
  else
  {
    if(runTime % 250000 == 0)
    {
      playNote();
    }
  }
  
  if(turnOrMove)
  {
    carState = 2;
    dutySpeedValue = TASK_2_TURN_SPEED;
    realTimeControl = true;
    if(encoderValue - encoderStartPoint > TASK_2_TURN_TARGET)
    {
      turnOrMove = !turnOrMove;
      encoderStartPoint = encoderValue;
      count++;
    }
  }
  else
  {
    carState = 1;
    dutySpeedValue = speedValue;
    realTimeControl = true;
    if(encoderValue - encoderStartPoint > Target())
    {
      turnOrMove = !turnOrMove;
      encoderStartPoint = encoderValue;
      count++;
    }
  }
  if(count >= 8)
  {
    firstTime = true;
    carState = 0;
    realTimeControl = true;
    startFlag = false;
    Timer1.stop();
    //Turn off taskLED / buzzer / MsTimer2
    digitalWrite(PIN_LED_TASK2, LOW);
    noTone(PIN_BUZZER);
  }
}

void task3()
{
  static bool firstTime = true;
  runTime += REAL_TIME_PERIOD;
  if(firstTime)
  {
    encoderStartPoint = encoderValue;
    firstTime = false;
  }
  if(TASK_OR_FUN)
  {
    playMelody();
  }
  else
  {
    if(runTime % 500000 == 0)
    {
      playNote();
    }
  }
  if(encoderValue - encoderStartPoint > Target())
  {
    firstTime = true;
    carState = 0;
    realTimeControl = true;
    startFlag = false;
    Timer1.stop();
    //Turn off taskLED / buzzer / MsTimer2
    digitalWrite(PIN_LED_TASK3, LOW);
    noTone(PIN_BUZZER);
  }
  else
  {
    carState = 1;
    dutySpeedValue = speedValue;
    realTimeControl = true;
  }
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

inline unsigned long Target()
{
  return (unsigned long)(target * 874.8932 - 10.5749);
}

inline unsigned long readEncoder(bool LR, bool isExportData)  //true == Right  //false == Left
{
  unsigned long encoder1 = 0;
  unsigned long encoder2 = 0;
  Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
  Wire.write("i");  //Maybe one command, but it isn't recorded in command file.
  Wire.endTransmission();
  //delay(1);
  Wire.requestFrom(COM_BASEBOARD_ADDRESS,8);
  //delay(10);
  if(Wire.available()==8)
  {
    encoder1 = (unsigned long) Wire.read();
    encoder1 += ((unsigned long) Wire.read() <<8);
    encoder1 += ((unsigned long) Wire.read() <<16);
    encoder1 += ((unsigned long) Wire.read() <<24);
    encoder2 = (unsigned long) Wire.read();
    encoder2 += ((unsigned long) Wire.read() <<8);
    encoder2 += ((unsigned long) Wire.read() <<16);
    encoder2 += ((unsigned long) Wire.read() <<24);
  }

  if(isExportData == true)
  {
    Serial.println("Encoder Left read text!");
    Serial.println(encoder1);
    Serial.println("Encoder Right read text!");
    Serial.println(encoder2);
    Serial.println("Encoder Start Point!");
    Serial.println(encoderStartPoint);
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
