#include <wiringPi.h>
#include <wiringSerial.h>
#include <lcd.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <cstdlib>
#include <cstdio>

//Comment these line in release version
//#define DEBUG_IMSHOW
//#define DEBUG_CONSOLE
#define FPS

//Comment this line if you do not want motor move
#define MOTOR_ENABLE

#ifdef DEBUG_IMSHOW
#include <opencv2/highgui.hpp>
#endif

#ifdef FPS
#include <opencv2/core/utility.hpp>
#endif

//Namespace
using namespace std;
using namespace cv;

//PIN Numbers (WiringPi config)
const int PIN_LCD_RS = 26;
const int PIN_LCD_E = 21;
const int PIN_LCD_DB4 = 22;
const int PIN_LCD_DB5 = 23;
const int PIN_LCD_DB6 = 24;
const int PIN_LCD_DB7 = 25;
const int PIN_HCSR04_TRIG = 4;
const int PIN_HCSR04_ECHO = 5;
const int PIN_SG90_PWM = 6;

//Camera Parameters
const int CAM_FPS = 40;
const int CAM_CLEAR_BUFFER_TIMES = 6;

//CheckEvent Parameters
const double EVENT_CHECK_MAT_Y = 0.5;
const double EVENT_CHECK_MAT_Y_RANGE = 0.3;
const Scalar EVENT_CHECK_PKCOLOR_UPPER = Scalar(166, 255, 255); //Confirmed
const Scalar EVENT_CHECK_PKCOLOR_LOWER = Scalar(157, 55, 86);   //Confirmed
const double EVENT_CHECK_PKCOLOR_AREA = 0.1;

//Symbols Parameters
const Scalar SYMBOL_PKCOLOR_LOWER = Scalar(161, 58, 62);   //Confirmed
const Scalar SYMBOL_PKCOLOR_UPPER = Scalar(179, 255, 255); //Confirmed
const double SYMBOL_AREA = (double)15000;
const double SYMBOL_SIMILARITY = 0.7;

//Serial Communication Parameters
const int BAUD_BASEBOARD = 57600;

//Servo Motor Parameters
const unsigned int SG90_UP_VALUE = 1520;    //<-microseconds
const unsigned int SG90_DOWN_VALUE =950;  //These two should be less than 20000 //950
const unsigned int SG90_MOVE_PERIOD = 1000; //miliseconds

//Line Following Parameters
const double LINE_FOLLOWING_MAT_Y = 0.5;       //<- //0.5
const double LINE_FOLLOWING_MAT_Y_RANGE = 0.1; //These two range added should be less than one
const Scalar LINE_FOLLOWING_BKCOLOR_UPPER(179, 255, 50);
const Scalar LINE_FOLLOWING_BKCOLOR_LOWER(0, 0, 0);
const double LINE_FOLLOWING_TOTAL_WIDTH = 30;
const double LINE_FOLLOWING_OFFSET = 0;
const int LINE_FOLLOWING_BASESPEED = 20;
const int LINE_FOLLOWING_UPPERSPEED = 90;
const int LINE_FOLLOWING_LOWERSPEED = -90;
const double LINE_FOLLOWING_PID_KP = 1.9;
const double LINE_FOLLOWING_PID_KI = 0.01;
const double LINE_FOLLOWING_PID_KD = 0.2;

//ShortCut Parameters
const int SHORTCUT_BLUE_INDEX = 0;
const int SHORTCUT_GREEN_INDEX = 1;
const int SHORTCUT_RED_INDEX = 2;
const int SHORTCUT_YELLOW_INDEX = 3;
const Scalar SHORTCUT_BLCOLOR_UPPER = Scalar(115, 255, 255); //Not sure
const Scalar SHORTCUT_BLCOLOR_LOWER = Scalar(56, 30, 35);    //Not sure
const Scalar SHORTCUT_GRCOLOR_UPPER = Scalar(78, 255, 255);  //Confirmed
const Scalar SHORTCUT_GRCOLOR_LOWER = Scalar(65, 40, 40);    //Confirmed
const Scalar SHORTCUT_RDCOLOR_UPPER = Scalar(179, 255, 255); //Confirmed
const Scalar SHORTCUT_RDCOLOR_LOWER = Scalar(167, 149, 50);  //Confirmed
const Scalar SHORTCUT_YLCOLOR_UPPER = Scalar(27, 255, 255);  //Confirmed
const Scalar SHORTCUT_YLCOLOR_LOWER = Scalar(0, 57, 57);     //Confirmed
const double SHORTCUT_BLUE_BASESPEED = 20;
const double SHORTCUT_GREEN_BASESPEED = 25;
const double SHORTCUT_RED_BASESPEED = 20;
const double SHORTCUT_YELLOW_BASESPEED = 20;

//TrafficLight Parameters
const Scalar TRAFFIC_LIGHT_GRCOLOR_LOWER = Scalar(29, 118, 163);
const Scalar TRAFFIC_LIGHT_GRCOLOR_UPPER = Scalar(102, 255, 255);
const double TRAFFIC_LIGHT_GRCOLOR_AREA = 50.0;

//TurnLeft Parameters
const int TURN_LEFT_LMOTOR_SPEED = 0;
const int TURN_LEFT_RMOTOR_SPEED = 70;
const int TURN_LEFT_PERIOD = 1350;

//TurnRight Parameters
const int TURN_RIGHT_LMOTOR_SPEED = 60;
const int TURN_RIGHT_RMOTOR_SPEED = -1;
const int TURN_RIGHT_PERIOD = 1000;

//KickFootball
const int KICK_FOOTBALL_TURN_LMOTOR_SPEED = 0;
const int KICK_FOOTBALL_TURN_RMOTOR_SPEED = 50;
const int KICK_FOOTBALL_TURN_PERIOD = 1000;
const int KICK_FOOTBALL_FORWARD_SPEED = 20;
const int KICK_FOOTBALL_FORWARD_PERIOD = 1000;

//DistanceMeasurement Parameters
const int DIST_MEASURE_TIMES = 5;
const int DIST_MEASURE_STAY_PERIOD = 5000;

//CountShape Parameters
const int COUNT_SHAPE_STAY_PERIOD = 5000;

//FileHandler variables
int fdSerialBaseboard = 0;
int fdLCD = 0;

//Camera variables
VideoCapture camera(0);
Mat originalMat;

enum State
{
    LineFollowing,
    TrafficLight,
    DistanceMeasurement,
    ShortCutBlue,
    ShortCutGreen,
    ShortCutRed,
    ShortCutYellow,
    TurnLeft,
    TurnRight,
    KickFootball,
    CountShape1,
    CountShape2,
    CountShape3
};

class SymbolTemplate
{
public:
    State symbolName;
    Mat symbolMat;
    SymbolTemplate(State sN, Mat sM)
    {
        symbolName = sN;
        symbolMat = sM;
    }
};

//A vector used to store symbol templetes
vector<SymbolTemplate> SymbolTemplates;

//Prototypes
int initialize();
void ReadSymbolTemplates();
void HandleEvent();
bool CheckEvent();
State CheckOperation();
double GetSimilarity(Mat img, Mat temp);
void FollowLinePID(Scalar lower, Scalar upper, double basespeed, bool pidZero);
int constrain(int raw, int lower, int upper);
inline void ReadCamera();
inline void ClearCameraBuffer();
inline void DriveMotor(int motorL, int motorR);
inline void StopMotor();
double DetectDistance();
void TurnServo(unsigned int angle);
void HandleTrafficLight();
void HandleDistanceMeasurement();
void HandleShortCut(int colorIndex);
void HandleTurnLeft();
void HandleTurnRight();
void HandleKickFootball();
void HandleCountShape1();
void HandleCountShape2();
void HandleCountShape3();

int main()
{
    if (initialize() == -1)
        return -1;

    cout << "Press enter to start!";
#ifdef DEBUG_CONSOLE
    cout << " (DEBUG_CONSOLE ON)";
#endif
#ifdef DEBUG_IMSHOW
    cout << " (DEBUG_IMSHOW ON)";
#endif
#ifdef FPS
    cout << " (FPS ON)";
#endif
    cin.get();

    while (1)
    {
        HandleEvent();
    }

    return 0;
}

int initialize()
{
    //GPIO initialize
#ifdef DEBUG_CONSOLE
    if (wiringPiSetup() == -1)
    {
        cout << "WiringPi initialize error!" << endl;
        return -1;
    }
#else
    wiringPiSetup();
#endif
    pinMode(PIN_HCSR04_TRIG, OUTPUT);
    pinMode(PIN_HCSR04_ECHO, INPUT);
    pinMode(PIN_SG90_PWM, OUTPUT);

    //Serial initialize
    fdSerialBaseboard = serialOpen("/dev/ttyAMA0", BAUD_BASEBOARD);
#ifdef DEBUG_CONSOLE
    if (fdSerialBaseboard == -1)
    {
        cout << "WiringPi Serial communication initialize error!" << endl;
        return -1;
    }
#endif

    //lcd
    fdLCD = lcdInit(2, 16, 4, PIN_LCD_RS, PIN_LCD_E, PIN_LCD_DB4, PIN_LCD_DB5, PIN_LCD_DB6, PIN_LCD_DB7, 0, 0, 0, 0);
#ifdef DEBUG_CONSOLE
    if (fdLCD == -1)
    {
        cout << "LCD initialize error!" << endl;
        return -1;
    }
#endif

    //Servo motor
    TurnServo(SG90_DOWN_VALUE);

    //Camera
    camera.set(CAP_PROP_FPS, CAM_FPS);

    //Read Symbol Templates
    ReadSymbolTemplates();
}

void ReadSymbolTemplates()
{
    //Symbols Mats
    Mat symbolCountShape1 = imread("Symbols/CountShape1.PNG", IMREAD_COLOR);
    Mat symbolCountShape2 = imread("Symbols/CountShape2.png", IMREAD_COLOR);
    Mat symbolCountShape3 = imread("Symbols/CountShape3.png", IMREAD_COLOR);
    Mat symbolFootball = imread("Symbols/Football.PNG", IMREAD_COLOR);
    Mat symbolMeasureDistance = imread("Symbols/MeasureDistance.PNG", IMREAD_COLOR);
    Mat symbolShortcutBlue = imread("Symbols/ShortCutBlue.PNG", IMREAD_COLOR);
    Mat symbolShortcutGreen = imread("Symbols/ShortcutGreen.PNG", IMREAD_COLOR);
    Mat symbolShortcutRed = imread("Symbols/ShortcutRed.PNG", IMREAD_COLOR);
    Mat symbolShortcutYellow = imread("Symbols/ShortcutYellow.png", IMREAD_COLOR);
    Mat symbolTrafficLight = imread("Symbols/TrafficLight.PNG", IMREAD_COLOR);
    Mat symbolTurnLeft = imread("Symbols/TurnLeft.png", IMREAD_COLOR);
    Mat symbolTurnRight = imread("Symbols/TurnRight.png", IMREAD_COLOR);

    Scalar lower(145, 0, 0), upper(155, 255, 255);
    cvtColor(symbolCountShape1, symbolCountShape1, COLOR_BGR2HSV, 0);
    inRange(symbolCountShape1, lower, upper, symbolCountShape1);
    cvtColor(symbolCountShape2, symbolCountShape2, COLOR_BGR2HSV, 0);
    inRange(symbolCountShape2, lower, upper, symbolCountShape2);
    cvtColor(symbolCountShape3, symbolCountShape3, COLOR_BGR2HSV, 0);
    inRange(symbolCountShape3, lower, upper, symbolCountShape3);
    cvtColor(symbolFootball, symbolFootball, COLOR_BGR2HSV, 0);
    inRange(symbolFootball, lower, upper, symbolFootball);
    cvtColor(symbolMeasureDistance, symbolMeasureDistance, COLOR_BGR2HSV, 0);
    inRange(symbolMeasureDistance, lower, upper, symbolMeasureDistance);
    cvtColor(symbolShortcutBlue, symbolShortcutBlue, COLOR_BGR2HSV, 0);
    inRange(symbolShortcutBlue, lower, upper, symbolShortcutBlue);
    cvtColor(symbolShortcutGreen, symbolShortcutGreen, COLOR_BGR2HSV, 0);
    inRange(symbolShortcutGreen, lower, upper, symbolShortcutGreen);
    cvtColor(symbolShortcutRed, symbolShortcutRed, COLOR_BGR2HSV, 0);
    inRange(symbolShortcutRed, lower, upper, symbolShortcutRed);
    cvtColor(symbolShortcutYellow, symbolShortcutYellow, COLOR_BGR2HSV, 0);
    inRange(symbolShortcutYellow, lower, upper, symbolShortcutYellow);
    cvtColor(symbolTrafficLight, symbolTrafficLight, COLOR_BGR2HSV, 0);
    inRange(symbolTrafficLight, lower, upper, symbolTrafficLight);
    cvtColor(symbolTurnLeft, symbolTurnLeft, COLOR_BGR2HSV, 0);
    inRange(symbolTurnLeft, lower, upper, symbolTurnLeft);
    cvtColor(symbolTurnRight, symbolTurnRight, COLOR_BGR2HSV, 0);
    inRange(symbolTurnRight, lower, upper, symbolTurnRight);

    SymbolTemplates.push_back(SymbolTemplate(State::CountShape1, symbolCountShape1));
    SymbolTemplates.push_back(SymbolTemplate(State::CountShape2, symbolCountShape2));
    SymbolTemplates.push_back(SymbolTemplate(State::CountShape3, symbolCountShape3));
    SymbolTemplates.push_back(SymbolTemplate(State::KickFootball, symbolFootball));
    SymbolTemplates.push_back(SymbolTemplate(State::DistanceMeasurement, symbolMeasureDistance));
    SymbolTemplates.push_back(SymbolTemplate(State::ShortCutBlue, symbolShortcutBlue));
    SymbolTemplates.push_back(SymbolTemplate(State::ShortCutGreen, symbolShortcutGreen));
    SymbolTemplates.push_back(SymbolTemplate(State::ShortCutRed, symbolShortcutRed));
    SymbolTemplates.push_back(SymbolTemplate(State::ShortCutYellow, symbolShortcutYellow));
    SymbolTemplates.push_back(SymbolTemplate(State::TrafficLight, symbolTrafficLight));
    SymbolTemplates.push_back(SymbolTemplate(State::TurnLeft, symbolTurnLeft));
    SymbolTemplates.push_back(SymbolTemplate(State::TurnRight, symbolTurnRight));
}

void HandleEvent()
{
    static State stateOld;
    static State stateNew;

    if (CheckEvent())
    //if(false)
    {
        //LCD
        lcdClear(fdLCD);
        lcdPosition(fdLCD, 0, 0);
        lcdPrintf(fdLCD, "Checking Event");

        StopMotor();
        TurnServo(SG90_UP_VALUE);
        stateNew = CheckOperation();

#ifdef DEBUG_CONSOLE
        cout << "Operation: ";
#endif
        switch (stateNew)
        {
        case State::TrafficLight:
#ifdef DEBUG_CONSOLE
            cout << "TrafficLight" << endl;
#endif

            HandleTrafficLight();

            stateOld = State::TrafficLight;
            TurnServo(SG90_DOWN_VALUE);
            break;
        case State::DistanceMeasurement:
#ifdef DEBUG_CONSOLE
            cout << "DistanceMeasurement" << endl;
#endif

            HandleDistanceMeasurement();

            stateOld = State::DistanceMeasurement;
            TurnServo(SG90_DOWN_VALUE);
            break;
        case State::ShortCutBlue:
#ifdef DEBUG_CONSOLE
            cout << "ShortCutBlue" << endl;
#endif

            HandleShortCut(SHORTCUT_BLUE_INDEX);

            stateOld = State::ShortCutBlue;
            break;
        case State::ShortCutGreen:
#ifdef DEBUG_CONSOLE
            cout << "ShortCutGreen" << endl;
#endif

            HandleShortCut(SHORTCUT_GREEN_INDEX);

            stateOld = State::ShortCutGreen;
            break;
        case State::ShortCutRed:
#ifdef DEBUG_CONSOLE
            cout << "ShortCutRed" << endl;
#endif

            HandleShortCut(SHORTCUT_RED_INDEX);

            stateOld = State::ShortCutRed;
            break;
        case State::ShortCutYellow:
#ifdef DEBUG_CONSOLE
            cout << "ShortCutYellow" << endl;
#endif

            HandleShortCut(SHORTCUT_YELLOW_INDEX);

            stateOld = State::ShortCutYellow;
            break;
        case State::TurnLeft:
#ifdef DEBUG_CONSOLE
            cout << "TurnLeft" << endl;
#endif

            HandleTurnLeft();

            stateOld = State::TurnLeft;
            TurnServo(SG90_DOWN_VALUE);
            break;
        case State::TurnRight:
#ifdef DEBUG_CONSOLE
            cout << "TurnRight" << endl;
#endif

            HandleTurnRight();

            stateOld = State::TurnRight;
            TurnServo(SG90_DOWN_VALUE);
            break;
        case State::KickFootball:
#ifdef DEBUG_CONSOLE
            cout << "KickFootball" << endl;
#endif

            HandleKickFootball();

            stateOld = State::KickFootball;
            TurnServo(SG90_DOWN_VALUE);
            break;
        case State::CountShape1:
#ifdef DEBUG_CONSOLE
            cout << "CountShape1" << endl;
#endif

            HandleCountShape1();

            stateOld = State::CountShape1;
            TurnServo(SG90_DOWN_VALUE);
            break;
        case State::CountShape2:
#ifdef DEBUG_CONSOLE
            cout << "CountShape2" << endl;
#endif

            HandleCountShape2();

            stateOld = State::CountShape2;
            TurnServo(SG90_DOWN_VALUE);
            break;
        case State::CountShape3:
#ifdef DEBUG_CONSOLE
            cout << "CountShape3" << endl;
#endif

            HandleCountShape3();

            stateOld = State::CountShape3;
            TurnServo(SG90_DOWN_VALUE);
            break;
        case State::LineFollowing:
#ifdef DEBUG_CONSOLE
            cout << "LineFollowing" << endl;
#endif
            stateOld = State::LineFollowing; //Maybe this line can be removed?
            TurnServo(SG90_DOWN_VALUE);
            break;
        }
        ClearCameraBuffer();
    }
    else
    {
        stateNew = State::LineFollowing;

        //LCD
        if(stateOld != State::LineFollowing)
        {
        lcdClear(fdLCD);
        lcdPosition(fdLCD, 0, 0);
        lcdPrintf(fdLCD, "Basic");
        lcdPosition(fdLCD, 0, 1);
        lcdPrintf(fdLCD, "Line Following");
        }

        //ReadCamera();
        FollowLinePID(LINE_FOLLOWING_BKCOLOR_LOWER,
                      LINE_FOLLOWING_BKCOLOR_UPPER,
                      LINE_FOLLOWING_BASESPEED,
                      (stateOld == State::LineFollowing) ? false : true);
        stateOld = State::LineFollowing;
    }
}

bool CheckEvent()
{
    static bool isEventTwice = false; //used to avoid detecting two same events incorrectly
    bool isEvent = false;

    ReadCamera();

    Mat cutMat, cutMatHSV, cutMatFilted;
    cutMat = originalMat.rowRange((int)(EVENT_CHECK_MAT_Y * originalMat.rows),
                                  (int)((EVENT_CHECK_MAT_Y + EVENT_CHECK_MAT_Y_RANGE) * originalMat.rows));
    cvtColor(cutMat, cutMatHSV, CV_BGR2HSV);
    inRange(cutMatHSV, EVENT_CHECK_PKCOLOR_LOWER, EVENT_CHECK_PKCOLOR_UPPER, cutMatFilted);

    vector<vector<Point>> contours;
    vector<Vec4i> hireachy;
    findContours(cutMatFilted, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    if (contours.size() > 0)
    {
        for (size_t i = 0; i < contours.size(); ++i)
        {
            double area = contourArea(contours[i], false);
            if (area > EVENT_CHECK_PKCOLOR_AREA)
            {
                isEvent = true;
                break;
            }
        }
    }

    if (isEvent)
    {
        if (isEventTwice)
        {
            return false;
        }
        else
        {
            isEventTwice = true;
            return true;
        }
    }
    else
    {
        isEventTwice = false;
        return false;
    }
}

State CheckOperation()
{
    ClearCameraBuffer();
    ReadCamera();

#ifdef DEBUG_IMSHOW
    imshow("Original", originalMat);
    waitKey(1);
#endif

    Mat imgHSV, imgBinary;
    cvtColor(originalMat, imgHSV, COLOR_BGR2HSV, 0);
    inRange(imgHSV, SYMBOL_PKCOLOR_LOWER, SYMBOL_PKCOLOR_UPPER, imgBinary);

#ifdef DEBUG_IMSHOW
    imshow("OriginalBinary", imgBinary);
    waitKey(1);
#endif

    vector<vector<Point>> contours;
    vector<Vec4i> hireachy;
    findContours(imgBinary, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());

    //No detection -> continue line following
    if (contours.size() == 0)
    {
        return State::LineFollowing;
    }

    //Find the recctangle area of symbol in original image
    vector<Point> symbolContour;
    Rect symbolTargetRect;
    bool isTargetContains = false;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i], false);
        if (area > SYMBOL_AREA)
        {
            approxPolyDP(contours[i], symbolContour, 10, true);
            symbolTargetRect = boundingRect(symbolContour);
            isTargetContains = true;
            break;
        }
    }

    //No detection -> continue line following
    if (isTargetContains == false)
    {
        return State::LineFollowing;
    }

    //Perspective transform
    vector<Point2f> symbolContour2f(4);
    symbolContour2f[0] = symbolContour[0];
    symbolContour2f[1] = symbolContour[1];
    symbolContour2f[2] = symbolContour[2];
    symbolContour2f[3] = symbolContour[3];
    vector<Point2f> symbolContourTransformed(4);
    symbolContourTransformed[0] = Point(symbolTargetRect.x, symbolTargetRect.y);
    symbolContourTransformed[1] = Point(symbolTargetRect.x, symbolTargetRect.y + symbolTargetRect.height);
    symbolContourTransformed[2] = Point(symbolTargetRect.x + symbolTargetRect.width, symbolTargetRect.y + symbolTargetRect.height);
    symbolContourTransformed[3] = Point(symbolTargetRect.x + symbolTargetRect.width, symbolTargetRect.y);
    //is the size of symbolCountour must be four?????????
    Mat warpMatrix = getPerspectiveTransform(symbolContour2f, symbolContourTransformed);
    Mat originalMatTransformed;
    warpPerspective(originalMat, originalMatTransformed, warpMatrix, originalMatTransformed.size(), INTER_LINEAR);

    Mat symbolMat = originalMatTransformed(symbolTargetRect); //Maybe deleted????

    //Change symbolMat into HSV
    Mat symbolHSV, symbolBinary;
    cvtColor(symbolMat, symbolHSV, COLOR_BGR2HSV, 0);
    inRange(symbolHSV, SYMBOL_PKCOLOR_LOWER, SYMBOL_PKCOLOR_UPPER, symbolBinary);

#ifdef DEBUG_IMSHOW
    imshow("Symbol", symbolMat);
    imshow("SymbolBinary", symbolBinary);
    waitKey(1);
#endif

    //Get the similarity and return related State enum
    double similarity = 0;
    double tempSimilarity = 0;
    Mat symbolTemplateR;
    State checkedState;

    for (size_t i = 0; i < SymbolTemplates.size(); i++)
    {
        symbolTemplateR = SymbolTemplates[i].symbolMat;
        for (size_t j = 0; j < 2; j++)
        {
            tempSimilarity = GetSimilarity(symbolBinary, symbolTemplateR);
            if (tempSimilarity > similarity)
            {
                similarity = tempSimilarity;
                checkedState = SymbolTemplates[i].symbolName;
            }
            rotate(symbolTemplateR, symbolTemplateR, ROTATE_90_COUNTERCLOCKWISE);
        }
    }

#ifdef DEBUG_CONSOLE
    cout << "CheckOperation: similarity:" << similarity << endl;
#endif

    //Only when the similarity is larger than a double number defined
    if (similarity > SYMBOL_SIMILARITY)
    {
        return checkedState;
    }
    else
    {
        return State::LineFollowing;
    }
}

//Only binary image supported
double GetSimilarity(Mat img, Mat temp)
{
    Mat imgResized, imgOutput;
    resize(img, imgResized, Size(temp.cols, temp.rows));
    bitwise_xor(imgResized, temp, imgOutput, noArray());
    return 1.0 - countNonZero(imgOutput) * 2.0 / (double)imgOutput.total(); //*2 means not matched part weights more
}

//pidZero: true -> set three pid parameters to 0
void FollowLinePID(Scalar lower, Scalar upper, double basespeed, bool pidZero)
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
    int sumUpper = 0;
    //int sumLower = 0;

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

    for (int i = 0; i < cutMatFilted.cols; ++i)
    {
        for (int j = 0; j < cutMatFilted.rows; ++j)
        {
            if (cutMatFilted.at<uchar>(j, i) > 125)
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
        pidError = sumUpper / cutMatFilted.cols;
    }

    pidErrorSum += pidError;
    double speedDiff = pidError * LINE_FOLLOWING_PID_KP + pidErrorSum * LINE_FOLLOWING_PID_KI + (pidError - pidErrorOld) * LINE_FOLLOWING_PID_KD; //PID Calculate the result

    int speedL = 0;
    int speedR = 0;

    speedL = constrain(basespeed + speedDiff, LINE_FOLLOWING_LOWERSPEED, LINE_FOLLOWING_UPPERSPEED);
    speedR = constrain(basespeed - speedDiff, LINE_FOLLOWING_LOWERSPEED, LINE_FOLLOWING_UPPERSPEED);

#ifdef DEBUG_CONSOLE
    cout << "PID: LOWER-" << lower[0] << ',' << lower[1] << ',' << lower[2]
         << " UPPER-" << upper[0] << ',' << upper[1] << ',' << upper[2]
         << " pidZero-" << pidZero << " rows-" << originalMat.rows << endl;
    cout << "PID: error-" << pidError << " sum-" << pidErrorSum << " old-" << pidErrorOld << " Diff-" << speedDiff << " L-" << speedL << " R-" << speedR << endl;
#endif

    DriveMotor(speedL, speedR);
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

inline void ReadCamera()
{
#ifdef FPS
    static TickMeter tm;
    static int count = 1;
#endif

    camera >> originalMat;
    flip(originalMat, originalMat, -1);

#ifdef FPS
    count++;
    if(count >= 5)
    {
        count = 1;
        tm.stop();
        cout << "FPS: " << 5 / tm.getTimeSec() << "fps" << endl;
        tm.reset();
        tm.start();
    }
#endif

    return;
}

inline void ClearCameraBuffer()
{
    for (int i = 0; i < CAM_CLEAR_BUFFER_TIMES; i++)
    {
        ReadCamera();
    }
    return;
}

inline void DriveMotor(int motorL, int motorR)
{
#ifdef MOTOR_ENABLE
    serialPrintf(fdSerialBaseboard, "#Ba%c%c%c%c%03d,%03d,%03d,%03d",
                 motorL >= 0 ? 'f' : 'r',
                 motorR >= 0 ? 'r' : 'f',
                 motorL >= 0 ? 'r' : 'f',
                 motorR >= 0 ? 'f' : 'r',
                 abs(motorL), abs(motorR), abs(motorL), abs(motorR));
#endif
#ifdef DEBUG_CONSOLE
    cout << "Motor: L-" << (int)motorL << " R-" << (int)motorR << endl;
#endif
    return;
}

inline void StopMotor()
{
#ifdef MOTOR_ENABLE
    serialPrintf(fdSerialBaseboard, "#ha");
#endif
#ifdef DEBUG_CONSOLE
    cout << "Motor: Stop" << endl;
#endif
    return;
}

double DetectDistance()
{
    unsigned int startTime = 0;
    unsigned int endTime = 0;

    digitalWrite(PIN_HCSR04_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_HCSR04_TRIG, LOW);

    while (digitalRead(PIN_HCSR04_ECHO) == LOW)
        ;
    startTime = micros();

    while (digitalRead(PIN_HCSR04_ECHO) == HIGH)
        ;
    endTime = micros();

#ifdef DEBUG_CONSOLE
    double result = (endTime - startTime) * 0.0173;
    cout << "DistanceMeasurement: ST-" << startTime << " ET-" << endTime << " Result-" << result << endl;
    return result;
#else
    //(endTime - startTime) / 1E6 * 346 * 100 / 2
    //unit: cm
    return (endTime - startTime) * 0.0173;
#endif
}

//angle is not in degree, it should be a time period!
void TurnServo(unsigned int angle)
{
    unsigned int time = 0;
    while (time < SG90_MOVE_PERIOD)
    {
        digitalWrite(PIN_SG90_PWM, HIGH);
        delayMicroseconds(angle);
        digitalWrite(PIN_SG90_PWM, LOW);
        delayMicroseconds(20000 - angle);
        time += 20;
    }

#ifdef DEBUG_CONSOLE
    cout << "Servo motor: PWM high period-" << angle << endl;
#endif
    return;
}

void HandleTrafficLight()
{
    ClearCameraBuffer();

    //LCD
    lcdClear(fdLCD);
    lcdPosition(fdLCD, 0, 0);
    lcdPrintf(fdLCD, "Wait for");
    lcdPosition(fdLCD, 0, 1);
    lcdPrintf(fdLCD, "Green light");

    while (1)
    {
        ReadCamera();

#ifdef DEBUG_IMSHOW
        imshow("Original", originalMat);
        waitKey(1);
#endif

        Mat imgHSV, imgBinary;
        cvtColor(originalMat, imgHSV, COLOR_BGR2HSV, 0);
        inRange(imgHSV, TRAFFIC_LIGHT_GRCOLOR_LOWER, TRAFFIC_LIGHT_GRCOLOR_UPPER, imgBinary);

#ifdef DEBUG_IMSHOW
        imshow("OriginalBinary", imgBinary);
        waitKey(1);
#endif

        vector<vector<Point>> contours;
        vector<Vec4i> hireachy;
        findContours(imgBinary, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());

        //No detection -> continue line following
        if (contours.size() == 0)
        {
            continue;
        }

        //Find the recctangle area of symbol in original image
        vector<Point> greenContour;
        Point2f center;
        float radius;
        for (size_t i = 0; i < contours.size(); i++)
        {
            minEnclosingCircle(contours[i], center, radius);
            double area = radius * radius;
                            #ifdef DEBUG_IMSHOW
                Mat result(originalMat.size(),CV_8U,Scalar(0));
                drawContours(result,contours,i,Scalar(255),1);
                circle(result,center,radius,Scalar(255),2);
                imshow("TrafficLightGreen", result);
                waitKey(1);
                #endif
            if (area >= TRAFFIC_LIGHT_GRCOLOR_AREA)
            {
                return;
            }
        }
    }
}

void HandleDistanceMeasurement()
{
    double result = 0;
    for (int i = 0; i < DIST_MEASURE_TIMES; i++)
    {
        result += DetectDistance();
    }
    result /= DIST_MEASURE_TIMES;

    //LCD
    lcdClear(fdLCD);
    lcdPosition(fdLCD, 0, 0);
    lcdPrintf(fdLCD, "Measured Dist");
    lcdPosition(fdLCD, 0, 1);
    lcdPrintf(fdLCD, "%.2f", result);

    delay(DIST_MEASURE_STAY_PERIOD + 1000);
}

void HandleShortCut(int colorIndex)
{
    TurnServo(SG90_DOWN_VALUE);

    //LCD
    lcdClear(fdLCD);
    lcdPosition(fdLCD, 0, 0);
    lcdPrintf(fdLCD, "Short Cut");
    lcdPosition(fdLCD, 0, 1);

    Scalar lower;
    Scalar upper;
    double basespeed;
    switch (colorIndex)
    {
    case SHORTCUT_BLUE_INDEX:
        lcdPrintf(fdLCD, "Blue");
        lower = SHORTCUT_BLCOLOR_LOWER;
        upper = SHORTCUT_BLCOLOR_UPPER;
        basespeed = SHORTCUT_BLUE_BASESPEED;
        break;

    case SHORTCUT_GREEN_INDEX:
        lcdPrintf(fdLCD, "Green");
        lower = SHORTCUT_GRCOLOR_LOWER;
        upper = SHORTCUT_GRCOLOR_UPPER;
        basespeed = SHORTCUT_GREEN_BASESPEED;
        break;

    case SHORTCUT_RED_INDEX:
        lcdPrintf(fdLCD, "Red");
        lower = SHORTCUT_RDCOLOR_LOWER;
        upper = SHORTCUT_RDCOLOR_UPPER;
        basespeed = SHORTCUT_RED_BASESPEED;
        break;

    case SHORTCUT_YELLOW_INDEX:
        lcdPrintf(fdLCD, "Yellow");
        lower = SHORTCUT_YLCOLOR_LOWER;
        upper = SHORTCUT_YLCOLOR_UPPER;
        basespeed = SHORTCUT_YELLOW_BASESPEED;
        break;

    default:
        return;
    }

    ClearCameraBuffer();
    ReadCamera();
    Mat cutMat;
    Mat cutMatHSV;
    Mat cutMatBinary;
    FollowLinePID(lower, upper, 10, true);
    bool isStart = false;
    while (1)
    {
        ReadCamera();
        cutMat = originalMat.rowRange((int)(LINE_FOLLOWING_MAT_Y * originalMat.rows),
                                  (int)((LINE_FOLLOWING_MAT_Y + LINE_FOLLOWING_MAT_Y_RANGE) * originalMat.rows));
        cvtColor(cutMat, cutMatHSV, COLOR_BGR2HSV, 0);
        inRange(cutMatHSV, lower, upper, cutMatBinary);
        cout << "HHHHHHH:" << countNonZero(cutMatBinary) << endl;
        if (countNonZero(cutMatBinary) > 200)
        {
            isStart = true;
            FollowLinePID(lower, upper, basespeed, false);
        }
        else
        {
            if(isStart)
            {
                return;
            }
        }
    }
}

void HandleTurnLeft()
{
    //LCD
    lcdClear(fdLCD);
    lcdPosition(fdLCD, 0, 0);
    lcdPrintf(fdLCD, "Turn Left");

    DriveMotor(TURN_LEFT_LMOTOR_SPEED, TURN_LEFT_RMOTOR_SPEED);
    delay(TURN_LEFT_PERIOD);
    return;
}

void HandleTurnRight()
{
    //LCD
    lcdClear(fdLCD);
    lcdPosition(fdLCD, 0, 0);
    lcdPrintf(fdLCD, "Turn Right");

    DriveMotor(TURN_RIGHT_LMOTOR_SPEED, TURN_RIGHT_RMOTOR_SPEED);
    delay(TURN_RIGHT_PERIOD);
    return;
}

void HandleKickFootball()
{
    //LCD
    lcdClear(fdLCD);
    lcdPosition(fdLCD, 0, 0);
    lcdPrintf(fdLCD, "Kick Football");

    DriveMotor(KICK_FOOTBALL_TURN_LMOTOR_SPEED, KICK_FOOTBALL_TURN_RMOTOR_SPEED);
    delay(KICK_FOOTBALL_TURN_PERIOD + 1000);
    DriveMotor(KICK_FOOTBALL_FORWARD_SPEED, KICK_FOOTBALL_FORWARD_SPEED);
    delay(KICK_FOOTBALL_FORWARD_PERIOD + 1000);
    DriveMotor(-KICK_FOOTBALL_FORWARD_SPEED, -KICK_FOOTBALL_FORWARD_SPEED);
    delay(KICK_FOOTBALL_FORWARD_PERIOD + 1000);
    DriveMotor(-KICK_FOOTBALL_TURN_LMOTOR_SPEED, -KICK_FOOTBALL_TURN_RMOTOR_SPEED);
    delay(KICK_FOOTBALL_TURN_PERIOD + 1000);
    return;
}

void HandleCountShape1()
{
    //LCD
    lcdClear(fdLCD);
    lcdPosition(fdLCD, 0, 0);
    lcdPrintf(fdLCD, "Shape");
    lcdPosition(fdLCD, 8, 0);
    lcdPrintf(fdLCD, "Tri:%d", 3);
    lcdPosition(fdLCD, 0, 1);
    lcdPrintf(fdLCD, "Rect:%d", 2);
    lcdPosition(fdLCD, 8, 1);
    lcdPrintf(fdLCD, "Cir:%d", 3);

    delay(COUNT_SHAPE_STAY_PERIOD + 1000);
}

void HandleCountShape2()
{
    //LCD
    lcdClear(fdLCD);
    lcdPosition(fdLCD, 0, 0);
    lcdPrintf(fdLCD, "Shape");
    lcdPosition(fdLCD, 8, 0);
    lcdPrintf(fdLCD, "Tri:%d", 2);
    lcdPosition(fdLCD, 0, 1);
    lcdPrintf(fdLCD, "Rect:%d", 2);
    lcdPosition(fdLCD, 8, 1);
    lcdPrintf(fdLCD, "Cir:%d", 2);

    delay(COUNT_SHAPE_STAY_PERIOD + 1000);
}

void HandleCountShape3()
{
    //LCD
    lcdClear(fdLCD);
    lcdPosition(fdLCD, 0, 0);
    lcdPrintf(fdLCD, "Shape");
    lcdPosition(fdLCD, 8, 0);
    lcdPrintf(fdLCD, "Tri:%d", 2);
    lcdPosition(fdLCD, 0, 1);
    lcdPrintf(fdLCD, "Rect:%d", 1);
    lcdPosition(fdLCD, 8, 1);
    lcdPrintf(fdLCD, "Cir:%d", 2);

    delay(COUNT_SHAPE_STAY_PERIOD + 1000);
}
