#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

//Comment these line in release version
#define DEBUG_IMSHOW
//#define DEBUG_CONSOLE
//#define FPS

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

//Camera Parameters
const int CAM_FPS = 40;
const int CAM_CLEAR_BUFFER_TIMES = 6;

//TrafficLight Parameters
const Scalar TRAFFIC_LIGHT_GRCOLOR_LOWER = Scalar(29, 118, 163);
const Scalar TRAFFIC_LIGHT_GRCOLOR_UPPER = Scalar(102, 255, 255);
const double TRAFFIC_LIGHT_GRCOLOR_AREA = 50.0;

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
inline void ReadCamera();
inline void ClearCameraBuffer();
void HandleTrafficLight();

int main()
{
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
        HandleTrafficLight();
#ifdef DEBUG_IMSHOW
        cout << "Green Light" << endl;
#endif
    }

    return 0;
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

void HandleTrafficLight()
{
    ClearCameraBuffer();

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
            if (area >= TRAFFIC_LIGHT_GRCOLOR_AREA)
            {
                return;
            }
        }
    }
}