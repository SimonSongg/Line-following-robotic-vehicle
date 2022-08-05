#include <SPI.h>
#include <RF24.h>
#include <LiquidCrystal.h>
#include <MsTimer2.h>

RF24 rf24(9, 10); // CE, CSN
LiquidCrystal lcd(3, 4, 5, 6, 7, 8);

const byte pipe = 1;

const byte addr[] = "1Node";

int msg[3] = {0, 0, 0};

void refresh()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Y:");
    lcd.print(msg[0]);
    lcd.setCursor(8, 0);
    lcd.print("P:");
    lcd.print(msg[1]);
    lcd.setCursor(0, 1);
    lcd.print("R:");
    lcd.print(msg[2]);
}

void setup()
{
    MsTimer2::set(200, refresh);
    MsTimer2::start();
    lcd.begin(16, 2);
    rf24.begin();
    rf24.setChannel(117);
    rf24.setPALevel(RF24_PA_MAX);
    rf24.setDataRate(RF24_2MBPS);
    rf24.openReadingPipe(pipe, addr);
    rf24.startListening();
    pinMode(2, OUTPUT);
}

void loop()
{
    if (rf24.available(&pipe))
    {
        digitalWrite(2, HIGH);
        rf24.read(&msg, sizeof(msg));
        digitalWrite(2, LOW);
    }
}
