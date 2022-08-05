#include <wiringPi.h>
#include <lcd.h>
#include <iostream>
#include <string>

using namespace std;

const int PIN_LCD_RS = 26;
const int PIN_LCD_E = 21;
const int PIN_LCD_D4 = 22;
const int PIN_LCD_D5 = 23;
const int PIN_LCD_D6 = 24;
const int PIN_LCD_D7 = 25;

int main()
{
    if(wiringPiSetup() == -1)
    {
        cout << "WiringPi initialize error!" << endl;
    }

    int fdLCD = lcdInit(2, 16, 4, PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7, 0, 0, 0, 0);
    //Initialise LCD
    if(fdLCD < 0)
    {
        cout << "LCD Initialize failed" << endl;
        return -1;
    }

    lcdHome(fdLCD);
    lcdPuts(fdLCD, "Hello World!");

    unsigned int index = 0;
    while(1)
    {
        lcdPosition(fdLCD, 0, 1);
        lcdPrintf(fdLCD, "%d", index);
        ++index;
        delay(1000);
    }
    
    return 0;
}