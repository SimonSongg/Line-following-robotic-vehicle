#include <wiringPi.h>
#include <iostream>

const int PIN_SG90_PWM = 1;

using namespace std;

int main()
{
    if(wiringPiSetup() == -1)
    {
        cout << "WiringPi initialize error!" << endl;
    }

    pinMode(PIN_SG90_PWM, PWM_OUTPUT);

    int dutyRatio = 512;
    while(1)
    {
        cin >> dutyRatio;
        pwmWrite(PIN_SG90_PWM, dutyRatio);
    }

    return 0;
}