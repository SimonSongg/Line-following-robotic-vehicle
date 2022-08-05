#include <wiringPi.h>
#include <iostream>

const int PIN_HC_SR04_TRIG = 4;
const int PIN_HC_SR04_ECHO = 5;

using namespace std;

int main()
{
    if(wiringPiSetup() == -1)
    {
        cout << "WiringPi initialize error!" << endl;
    }
    
    pinMode(PIN_HC_SR04_TRIG, OUTPUT);
    pinMode(PIN_HC_SR04_ECHO, INPUT);

    while(1)
    {
        cout << "Distance: " << DetectDistance() << endl;
    }

    return 0;
}

double DetectDistance()
{
    unsigned int startTime = 0;
    unsigned int endTime = 0;

    digitalWrite(PIN_HC_SR04_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_HC_SR04_TRIG, LOW);

    while(digitalRead(PIN_HC_SR04_ECHO) == 0);
    startTime = micros();

    while(digitalRead(PIN_HC_SR04_ECHO) == 1);
    endTime = micros();

    return (endTime - startTime) * 0.0173; // (endTime - startTime) / 1E6 * 346 * 100 / 2
}

