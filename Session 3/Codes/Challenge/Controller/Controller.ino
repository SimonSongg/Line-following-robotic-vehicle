#include <SPI.h>
#include <RF24.h>
#include <LiquidCrystal.h>
#include <MsTimer2.h>

//#define DEBUG

#define PIN_LCD_RS 3
#define PIN_LCD_E 4
#define PIN_LCD_DB4 5
#define PIN_LCD_DB5 6
#define PIN_LCD_DB6 7
#define PIN_LCD_DB7 8
#define PIN_JS_FB A7
#define PIN_JS_LR A6
#define PIN_JS_BTN_SPIN A3
#define PIN_LED_SIG A1
#define PIN_RF_CE 9
#define PIN_RF_CSN 10
#define PIN_SS_MODE 2
#define PIN_BTN_AUTOSTART A0

#define MSG_T_INDEX_MODE 0
#define MSG_T_INDEX_LSPEED 1
#define MSG_T_INDEX_RSPEED 2
#define MSG_R_INDEX_ST_SLOPE 0
#define MSG_R_INDEX_DISTANCE1 1
#define MSG_R_INDEX_DISTANCE2 2

#define RF_PIPE 1
#define RF_ADDR_T 0
#define RF_ADDR_R 1
#define RF_CHANNEL 117

#define SPEED_DIFF 30
#define SPEED_MIN 10
#define SPEED_MAX 90

#define SIG_LOST_TOLERANCE 11
#define SIG_RT_PERIOD 10

volatile bool mode = false;                                    //false->mannual    //true->auto
volatile char messageTransmit[3] = {0x00, 0x00, 0x00};         //mode + Left speed + Right speed
volatile unsigned char messageRecieve[3] = {0x00, 0x00, 0x00}; //1bit state + 7 bit slope + 2 bytes distance

volatile char leftSpeed = 0;
volatile char rightSpeed = 0;

//First address is used for message transmit
//Second address is used for message recieve
const byte address[2][5] = {"1Node", "2Node"};

volatile unsigned char lostCounter = 0;

RF24 rf24(PIN_RF_CE, PIN_RF_CSN); // CE, CSN
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_DB4, PIN_LCD_DB5, PIN_LCD_DB6, PIN_LCD_DB7);

void setup()
{
#ifdef DEBUG
    Serial.begin(9600);
#endif
    //LCD
    lcd.begin(16, 2);
    lcd.print("Initializing...");

    //RF24
    rf24.begin();
    rf24.setChannel(RF_CHANNEL);
    rf24.setPALevel(RF24_PA_MAX);
    rf24.setDataRate(RF24_2MBPS);
    rf24.openReadingPipe(RF_PIPE, address[RF_ADDR_R]);
    rf24.openWritingPipe(address[RF_ADDR_T]);

    //Signal LED
    pinMode(PIN_LED_SIG, OUTPUT);

    //Spin Button
    pinMode(PIN_JS_BTN_SPIN, INPUT_PULLUP);

    //AutoStart Button
    pinMode(PIN_BTN_AUTOSTART, INPUT_PULLUP);

    //Mode Switch
    pinMode(PIN_SS_MODE, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SS_MODE), ModeChange, CHANGE);

    //LCD info. initialize
    lcd.clear();
    lcd.print("ST:");
    lcd.setCursor(6, 0);
    lcd.print("ANG:");
    lcd.setCursor(14, 0);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("DIST:");

    //MsTimer2 (used to LCD print)
    MsTimer2::set(10, LCDPrint);
    MsTimer2::start();

    ModeChange();
}

void loop()
{
    MessageGenerator();
    if (rf24.available())
    {
        rf24.read(messageRecieve, sizeof(messageRecieve));
        lostCounter = 0;
        digitalWrite(PIN_LED_SIG, LOW);
    }
    else
    {
#ifdef DEBUG
        Serial.println("Lost!!!!!");
#endif
        ++lostCounter;
    }
    if (lostCounter >= SIG_LOST_TOLERANCE)
    {
        digitalWrite(PIN_LED_SIG, HIGH);
        lostCounter = SIG_LOST_TOLERANCE;
    }
    rf24.stopListening();

#ifdef DEBUG
    Serial.print((int)messageTransmit[0]);
    Serial.print("\t");
    Serial.print((int)messageTransmit[1]);
    Serial.print("\t");
    Serial.println((int)messageTransmit[2]);
#endif

    rf24.write(messageTransmit, sizeof(messageTransmit));
    rf24.startListening();
    
    delay(SIG_RT_PERIOD);
}

inline void MessageGenerator()
{
    ReadJoystick();
    messageTransmit[MSG_T_INDEX_MODE] = ((mode && digitalRead(PIN_BTN_AUTOSTART) == LOW) ? 1 : 0);
    messageTransmit[MSG_T_INDEX_LSPEED] = (mode ? 0 : leftSpeed);
    messageTransmit[MSG_T_INDEX_RSPEED] = (mode ? 0 : rightSpeed);
}

inline void LCDPrint()
{
    //data processing
    bool state = (messageRecieve[MSG_R_INDEX_ST_SLOPE] & 0x80) >> 7;
    byte slope = (messageRecieve[MSG_R_INDEX_ST_SLOPE] & 0x7F);
    unsigned int distance = ((unsigned int)messageRecieve[MSG_R_INDEX_DISTANCE1] << 8) | ((unsigned int)messageRecieve[MSG_R_INDEX_DISTANCE2]);

    lcd.setCursor(3, 0);
    lcd.print(state ? "MO" : "ST"); //1->move    //0->stop
    lcd.setCursor(10, 0);
    lcd.print("  ");
    lcd.setCursor(10, 0);
    lcd.print(slope);
    lcd.setCursor(5, 1);
    lcd.print("           ");
    lcd.setCursor(5, 1);
    lcd.print(String((float)distance / 100, 2));
    lcd.print("m");
}

void ModeChange()
{
    if (digitalRead(PIN_SS_MODE) == HIGH) //Auto
    {
        mode = true;
        lcd.setCursor(15, 0);
        lcd.print("2");
    }
    else //mannual
    {
        mode = false;
        lcd.setCursor(15, 0);
        lcd.print("1");
    }
}

inline void ReadJoystick()
{
    int xValue = analogRead(PIN_JS_LR);
    char yValue = map(analogRead(PIN_JS_FB), 0, 1023, -SPEED_MAX, SPEED_MAX);
    int tempLeftSpeed;
    int tempRightSpeed;
    if (digitalRead(PIN_JS_BTN_SPIN) == HIGH)
    {
        char absYValue = abs(yValue);
        char xCommonValue = map(xValue, 0, 1023, -SPEED_DIFF, SPEED_DIFF);
        if (absYValue >= SPEED_MIN)
        {
            tempRightSpeed = absYValue - xCommonValue;
            tempLeftSpeed = absYValue + xCommonValue;
            if (xCommonValue > 0) //right
            {
                if (tempRightSpeed < 0)
                {
                    tempRightSpeed = 0;
                    tempLeftSpeed = 0 + 2 * xCommonValue;
                }
                else if (tempLeftSpeed > SPEED_MAX)
                {
                    tempLeftSpeed = SPEED_MAX;
                    tempRightSpeed = SPEED_MAX - 2 * xCommonValue;
                }
            }
            else if (xCommonValue < 0) //left
            {
                if (tempRightSpeed > SPEED_MAX)
                {
                    tempRightSpeed = SPEED_MAX;
                    tempLeftSpeed = SPEED_MAX + 2 * xCommonValue;
                }
                else if (tempLeftSpeed < 0)
                {
                    tempLeftSpeed = 0;
                    tempRightSpeed = 0 - 2 * xCommonValue;
                }
            }

            if (yValue < 0)
            {
                tempRightSpeed = -tempRightSpeed;
                tempLeftSpeed = -tempLeftSpeed;
            }
        }
        else
        {
            tempLeftSpeed = 0;
            tempRightSpeed = 0;
        }
    }
    else
    {
        char xSpinValue = map(xValue, 0, 1023, -SPEED_MAX, SPEED_MAX);
        if (abs(xSpinValue) >= SPEED_MIN)
        {
            tempLeftSpeed = 0 + xSpinValue;
            tempRightSpeed = 0 - xSpinValue;
        }
        else
        {
            tempLeftSpeed = 0;
            tempRightSpeed = 0;
        }
    }

    leftSpeed = tempLeftSpeed;
    rightSpeed = tempRightSpeed;
}
