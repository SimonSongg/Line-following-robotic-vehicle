#include <wiringPi.h>
#include <wiringSerial.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <cstdlib>

//#define DEBUG

#ifdef DEBUG
#include <opencv2/highgui.hpp>
#endif

using namespace std;
using namespace cv;

int initialize();
void FollowLinePID(Scalar lower, Scalar upper, bool pidZero);
int constrain(int raw, int lower, int upper);
inline void MotorMove(int motorL, int motorR);
inline void MotorStop();

const int BAUD_BASEBOARD = 57600;

const double LINE_FOLLOWING_MAT_Y = 0.5;       //<-
const double LINE_FOLLOWING_MAT_Y_RANGE = 0.1; //These two range added should be less than one
const Scalar LINE_FOLLOWING_BKCOLOR_UPPER(179, 255, 50);
const Scalar LINE_FOLLOWING_BKCOLOR_LOWER(0, 0, 0);
const double LINE_FOLLOWING_TOTAL_WIDTH = 30;
const double LINE_FOLLOWING_OFFSET = 0;
const double LINE_FOLLOWING_BASESPEED = 20;
const double LINE_FOLLOWING_UPPERSPEED = 90;
const double LINE_FOLLOWING_LOWERSPEED = -90;
const double LINE_FOLLOWING_PID_KI = 0.0;
const double LINE_FOLLOWING_PID_KP = 1.8;
const double LINE_FOLLOWING_PID_KD = 0;

int fdSerialBaseboard = 0;

VideoCapture camera(0);
Mat originalMat;

int main()
{
    camera.set(CAP_PROP_FPS, 40);
    //camera.set(CAP_PROP_FRAME_HEIGHT, 48);
    //camera.set(CAP_PROP_FRAME_WIDTH, 640);
    //camera >> originalMat;
    //imwrite("output.jpg", originalMat);
    if (initialize() == -1)
        return -1;

    cout << "Press enter to start!";
#ifdef DEBUG
    cout << " (DEBUG MODE)";
#endif
    cin.get();

    TickMeter tm;

    int i = 0;
    while (1)
    {
        tm.reset();
        tm.start();
        camera >> originalMat;
        flip(originalMat, originalMat, -1);
        FollowLinePID(LINE_FOLLOWING_BKCOLOR_LOWER, LINE_FOLLOWING_BKCOLOR_UPPER, false);
        tm.stop();
        cout << 1 / tm.getTimeSec() << "fps" << endl;
        //i++;
    }
    return 0;
}

int initialize()
{
    //GPIO initialize
#ifdef DEBUG
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

#ifdef DEBUG
    if (fdSerialBaseboard == -1)
    {
        cout << "WiringPi Serial communication initialize error!" << endl;
        return -1;
    }
#endif
}

void FollowLinePID(Scalar lower, Scalar upper, bool pidZero)
{
#ifdef DEBUG
    if (pidZero)
    {
        destroyAllWindows();
    }
#endif

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
    //unsigned int sumUpper = 0;
    //unsigned int sumLower = 0;
    int sumUpper = 0;
    int sumLower = 0;

    Mat cutMat, cutMatHSV, cutMatFilted;
    cutMat = originalMat.rowRange((int)(LINE_FOLLOWING_MAT_Y * originalMat.rows),
                                  (int)((LINE_FOLLOWING_MAT_Y + LINE_FOLLOWING_MAT_Y_RANGE) * originalMat.rows));
    cvtColor(cutMat, cutMatHSV, COLOR_BGR2HSV, 0);
    inRange(cutMatHSV, lower, upper, cutMatFilted);

#ifdef DEBUG
    imshow("originalMat", originalMat);
    imshow("cutMat", cutMat);
    imshow("Linefiltering", cutMatFilted);
    waitKey(1);
#endif

//    unsigned int colAvgValue = 0;
//    for (int i = 0; i < cutMatFilted.cols; ++i)
//    {
//        for (int j = 0; j < cutMatFilted.rows; ++j)
//        {
//            colAvgValue += cutMatFilted.at<uchar>(j, i);
//        }
//
//        colAvgValue = colAvgValue / cutMatFilted.rows;
//
//        if (colAvgValue > 125)
//        {
//            //sumUpper += i * colAvgValue;
//            sumUpper += i - cutMatFilted.cols / 2;
//            blackFlag = true;
//            //sumLower += colAvgValue;
//        }
//        colAvgValue = 0;
//    }

    for (int i = 0; i < cutMatFilted.cols; ++i)
    {
        for (int j = 0; j < cutMatFilted.rows; ++j)
        {
            if(cutMatFilted.at<uchar>(j, i) > 125)
            {
                sumUpper += i - cutMatFilted.cols / 2;
                blackFlag = true;
                break;
            }
        }
    }

    if (blackFlag)
    {
        pidErrorOld = pidError;
        //pidError = ((int)(sumUpper / sumLower) - cutMatFilted.cols / 2) * LINE_FOLLOWING_TOTAL_WIDTH / cutMatFilted.cols;
        pidError = sumUpper / cutMatFilted.cols;
    }

    pidErrorSum += pidError;
    double speedDiff = pidError * LINE_FOLLOWING_PID_KP + pidErrorSum * LINE_FOLLOWING_PID_KI + (pidError - pidErrorOld) * LINE_FOLLOWING_PID_KD; //PID Calculate the result

    int speedL = 0;
    int speedR = 0;

    speedL = constrain(LINE_FOLLOWING_BASESPEED + speedDiff, LINE_FOLLOWING_LOWERSPEED, LINE_FOLLOWING_UPPERSPEED);
    speedR = constrain(LINE_FOLLOWING_BASESPEED - speedDiff, LINE_FOLLOWING_LOWERSPEED, LINE_FOLLOWING_UPPERSPEED);

//#ifdef DEBUG
    cout << "PID: LOWER-" << lower[0] << ',' << lower[1] << ',' << lower[2]
         << " UPPER-" << upper[0] << ',' << upper[1] << ',' << upper[2]
         << " pidZero-" << pidZero << " rows-" << originalMat.rows << endl;
    cout << "PID: sumUpper-" << sumUpper << " sumLower-" << sumLower << endl;
    cout << "PID: error-" << pidError << " sum-" << pidErrorSum << " old-" << pidErrorOld << " Diff-" << speedDiff << " L-" << speedL << " R-" << speedR << endl;
//#endif

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
