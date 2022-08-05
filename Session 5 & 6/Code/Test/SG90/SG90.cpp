#include <wiringPi.h>
#include <iostream>

const int PIN_SG90_PWM = 6;

const unsigned int SG90_UP_VALUE = 1430;    //<-microseconds
const unsigned int SG90_DOWN_VALUE = 2000;  //These two should be less than 20000
const unsigned int PWM_PERIOD = 1000;

using namespace std;

int main()
{
    if(wiringPiSetup() == -1)
    {
        cout << "WiringPi initialize error!" << endl;
    }

    pinMode(PIN_SG90_PWM, PWM_OUTPUT);
    TurnServo(false);

    int input;
    while(1)
    {
        cin >> input;
        switch(input)
        {
            case 1:
            TurnServo(true);
            break;
            case 2:
            TurnServo(false);
            break;
        }
    }

    return 0;
}

void TurnServo(unsigned int angle)
{
    unsigned int time = 0;
    while (time < PWM_PERIOD)
    {
        digitalWrite(PIN_SG90_PWM, HIGH);
        delayMicroseconds(angle);
        digitalWrite(PIN_SG90_PWM, LOW);
        delayMicroseconds(20000 - angle);
        time += 20;
    }
    return;
}