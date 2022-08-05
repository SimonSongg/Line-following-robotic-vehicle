#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

//Comment these line in release version
#define DEBUG_IMSHOW
#define DEBUG_CONSOLE
//#define FPS

#ifdef DEBUG_IMSHOW
#include <opencv2/highgui.hpp>
#endif

#ifdef FPS
#include <opencv2/core/utility.hpp>
#endif

//Namespace
using namespace std;
using namespace cv;

//Camera Parameters
const int CAM_FPS = 60;
const int CAM_CLEAR_BUFFER_TIMES = 6;

//Symbols Parameters
const Scalar SYMBOL_PKCOLOR_LOWER = Scalar(160, 128, 77);
const Scalar SYMBOL_PKCOLOR_UPPER = Scalar(255, 255, 255);
const double SYMBOL_AREA = (double)50000;
const double SYMBOL_SIMILARITY = 0.7;

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

vector<SymbolTemplate> SymbolTemplates;

void initialize();
void ReadSymbolTemplates();
State CheckOperation();
double GetSimilarity(Mat img, Mat temp);
inline void ReadCamera();
inline void ClearCameraBuffer();

int main()
{
    initialize();

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
        State result = CheckOperation();
#ifdef DEBUG_CONSOLE
        cout << "Result: ";
        switch (result)
        {
        case State::LineFollowing:
            cout << "LineFollowing";
            break;
        case State::TrafficLight:
            cout << "TrafficLight";
            break;
        case State::DistanceMeasurement:
            cout << "DistanceMeasurement";
            break;
        case State::ShortCutBlue:
            cout << "ShortCutBlue";
            break;
        case State::ShortCutGreen:
            cout << "ShortCutGreen";
            break;
        case State::ShortCutRed:
            cout << "ShortCutRed";
            break;
        case State::ShortCutYellow:
            cout << "ShortCutYellow";
            break;
        case State::TurnLeft:
            cout << "TurnLeft";
            break;
        case State::TurnRight:
            cout << "TurnRight";
            break;
        case State::KickFootball:
            cout << "KickFootball";
            break;
        case State::CountShape1:
            cout << "CountShape1";
            break;
        case State::CountShape2:
            cout << "CountShape2";
            break;
        case State::CountShape3:
            cout << "CountShape3";
            break;
        }
        cout << endl;
#endif
#ifdef DEBUG_IMSHOW
        waitKey(0);
#endif
    }

    return 0;
}

void initialize()
{
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

State CheckOperation()
{
    ClearCameraBuffer();
    ReadCamera();

#ifdef DEBUG_IMSHOW
    imshow("Original", originalMat);
#endif

    Mat imgHSV, imgBinary;
    cvtColor(originalMat, imgHSV, COLOR_BGR2HSV, 0);
    inRange(imgHSV, SYMBOL_PKCOLOR_LOWER, SYMBOL_PKCOLOR_UPPER, imgBinary);

#ifdef DEBUG_IMSHOW
    imshow("OriginalBinary", imgBinary);
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

#ifdef DEBUG_IMSHOW
    imshow("Symbol", symbolMat);
    imshow("SymbolBinary", symbolBinary);
#endif

#ifdef DEBUG_CONSOLE
    cout << "Similarity: " << similarity << endl;
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

inline void ReadCamera()
{
#ifdef FPS
    static TickMeter tm;
    tm.stop();
    cout << "FPS: " << 1 / tm.getTimeSec() << "fps" << endl;
#endif

    camera >> originalMat;
    flip(originalMat, originalMat, -1);

#ifdef FPS
    tm.reset();
    tm.start();
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
