#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <cstdlib>

inline void MotorMove(char motorL, char motorR);
inline void MotorStop();

const int BAUD_BASEBOARD = 57600;

using namespace std;

int fdSerialBaseboard = 0;

int main()
{
    if(wiringPiSetupGpio() == -1)
    {
        cout << "WiringPi initialize error!" << endl;
        return -1;
    }

    fdSerialBaseboard = serialOpen("/dev/ttyAMA0", BAUD_BASEBOARD);

    if(fdSerialBaseboard == -1)
    {
        cout << "WiringPi Serial communication initialize error!" << endl;
        return -1;
    }

    MotorMove(50, 50);
    MotorStop();
    
    return 0;
}

inline void MotorMove(char motorL, char motorR)
{
    serialPrintf(fdSerialBaseboard, "#ba%c%c%c%c%3d,%3d,%3d,%3d",
    motorL >= 0 ? 'f' : 'r',
    motorR >= 0 ? 'f' : 'r',
    motorL >= 0 ? 'f' : 'r',
    motorR >= 0 ? 'f' : 'r',
    abs(motorL), abs(motorR), abs(motorL), abs(motorR));
    return;
}

inline void MotorStop()
{
    serialPrintf(fdSerialBaseboard, "#ha");
    return;
}