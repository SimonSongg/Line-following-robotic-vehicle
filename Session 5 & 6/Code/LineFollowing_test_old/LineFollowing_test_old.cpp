#include <wiringPi.h>
#include <wiringSerial.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <cstdlib>

//#define DEBUG_IMSHOW
//#define DEBUG_CONSOLE
//#define FPS

#ifdef DEBUG_IMSHOW
#include <opencv2/highgui.hpp>
#endif

#ifdef FPS
#include <opencv2/core/utility.hpp>
#endif

using namespace std;
using namespace cv;

int initialize();
void FollowLinePID(Scalar lower, Scalar upper, double basespeed, double kP, double kI, double kD, bool pidZero);
int constrain(int raw, int lower, int upper);
inline void MotorMove(int motorL, int motorR);
inline void MotorStop();

const int BAUD_BASEBOARD = 57600;

const double LINE_FOLLOWING_MAT_Y = 0.5;       //<-
const double LINE_FOLLOWING_MAT_Y_RANGE = 0.2; //These two range added should be less than one
const Scalar LINE_FOLLOWING_BKCOLOR_UPPER(179, 255, 50);
const Scalar LINE_FOLLOWING_BKCOLOR_LOWER(0, 0, 0);
const double LINE_FOLLOWING_TOTAL_WIDTH = 30;
const double LINE_FOLLOWING_OFFSET = 0;
const double LINE_FOLLOWING_BASESPEED = 30;
const double LINE_FOLLOWING_UPPERSPEED = 90;
const double LINE_FOLLOWING_LOWERSPEED = -90;
const double LINE_FOLLOWING_PID_KP = 2;
const double LINE_FOLLOWING_PID_KI = 0.01;
const double LINE_FOLLOWING_PID_KD = 0;

int fdSerialBaseboard = 0;

VideoCapture camera(0);
Mat originalMat;

int main()
{
    camera.set(CAP_PROP_FPS, 40);
    if (initialize() == -1)
        return -1;

    cout << "Press enter to start!";
#ifdef DEBUG_CONSOLE
    cout << " (DEBUG_CONSOLE ON)";
#endif
#ifdef DEBUG_IMSHOW
    cout << " (DEBUG_IMSHOW ON)";
#endif
    cin.get();

#ifdef FPS
    TickMeter tm;
#endif

    while (1)
    {
#ifdef FPS
        tm.reset();
        tm.start();
#endif

        camera >> originalMat;
        flip(originalMat, originalMat, -1);
        FollowLinePID(LINE_FOLLOWING_BKCOLOR_LOWER,
                      LINE_FOLLOWING_BKCOLOR_UPPER,
                      LINE_FOLLOWING_BASESPEED,
                      LINE_FOLLOWING_PID_KP,
                      LINE_FOLLOWING_PID_KI,
                      LINE_FOLLOWING_PID_KD,
                      false);

#ifdef FPS
        tm.stop();
        cout << 1 / tm.getTimeSec() << "fps" << endl;
#endif
    }
    return 0;
}

int initialize()
{
    //GPIO initialize
#ifdef DEBUG_CONSOLE
    if (wiringPiSetupGpio() == -1)
    {
        cout << "WiringPi initialize error!" << endl;
        return -1;
    }
#else
    wiringPiSetupGpio();
#endif

    //Serial initialize
    fdSerialBaseboard = serialOpen("/dev/ttyAMA0", BAUD_BASEBOARD);

#ifdef DEBUG_CONSOLE
    if (fdSerialBaseboard == -1)
    {
        cout << "WiringPi Serial communication initialize error!" << endl;
        return -1;
    }
#endif
}

void FollowLinePID(Scalar lower, Scalar upper, double basespeed, double kP, double kI, double kD, bool pidZero)
{
    static double pidError = 0;
    static double pidErrorSum = 0;
    static double pidErrorOld = 0;

    if (pidZero)
    {
        pidError = 0;
        pidErrorSum = 0;
        pidErrorOld = 0;
    }

    bool blackFlag = false;
    unsigned int sumUpper = 0;
    unsigned int sumLower = 0;

    Mat cutMat, cutMatHSV, cutMatFilted;
    cutMat = originalMat.rowRange((int)(LINE_FOLLOWING_MAT_Y * originalMat.rows),
                                  (int)((LINE_FOLLOWING_MAT_Y + LINE_FOLLOWING_MAT_Y_RANGE) * originalMat.rows));
    cvtColor(cutMat, cutMatHSV, COLOR_BGR2HSV, 0);
    inRange(cutMatHSV, lower, upper, cutMatFilted);

#ifdef DEBUG_IMSHOW
    imshow("originalMat", originalMat);
    imshow("cutMat", cutMat);
    imshow("Linefiltering", cutMatFilted);
    waitKey(1);
#endif

       unsigned int colAvgValue = 0;
       for (int i = 0; i < cutMatFilted.cols; ++i)
       {
           for (int j = 0; j < cutMatFilted.rows; ++j)
           {
               colAvgValue += cutMatFilted.at<uchar>(j, i);
           }
    
           colAvgValue = colAvgValue / cutMatFilted.rows;
    
           if (colAvgValue > 125)
           {
               sumUpper += i * colAvgValue;
               blackFlag = true;
               sumLower += colAvgValue;
           }
           colAvgValue = 0;
       }

    if (blackFlag)
    {
        pidErrorOld = pidError;
        pidError = ((int)(sumUpper / sumLower) - cutMatFilted.cols / 2) * LINE_FOLLOWING_TOTAL_WIDTH / cutMatFilted.cols;
    }

    pidErrorSum += pidError;
    double speedDiff = pidError * kP + pidErrorSum * kI + (pidError - pidErrorOld) * kD; //PID Calculate the result

    int speedL = 0;
    int speedR = 0;

    speedL = constrain(basespeed + speedDiff, LINE_FOLLOWING_LOWERSPEED, LINE_FOLLOWING_UPPERSPEED);
    speedR = constrain(basespeed - speedDiff, LINE_FOLLOWING_LOWERSPEED, LINE_FOLLOWING_UPPERSPEED);

#ifdef DEBUG_CONSOLE
    cout << "PID: LOWER-" << lower[0] << ',' << lower[1] << ',' << lower[2]
         << " UPPER-" << upper[0] << ',' << upper[1] << ',' << upper[2]
         << " pidZero-" << pidZero << " rows-" << originalMat.rows << endl;
    cout << "PID: sumUpper-" << sumUpper << " sumLower-" << sumLower << endl;
    cout << "PID: error-" << pidError << " sum-" << pidErrorSum << " old-" << pidErrorOld << " Diff-" << speedDiff << " L-" << speedL << " R-" << speedR << endl;
#endif

    MotorMove(speedL, speedR);
    return;
}

int constrain(int raw, int lower, int upper)
{
    if (raw > upper)
        return upper;
    if (raw < lower)
        return lower;
    else
        return raw;
}

inline void MotorMove(int motorL, int motorR)
{
    serialPrintf(fdSerialBaseboard, "#Ba%c%c%c%c%03d,%03d,%03d,%03d",
                 motorL >= 0 ? 'f' : 'r',
                 motorR >= 0 ? 'r' : 'f',
                 motorL >= 0 ? 'r' : 'f',
                 motorR >= 0 ? 'f' : 'r',
                 abs(motorL), abs(motorR), abs(motorL), abs(motorR));
    return;
}

inline void MotorStop()
{
    serialPrintf(fdSerialBaseboard, "#ha");
    return;
}
