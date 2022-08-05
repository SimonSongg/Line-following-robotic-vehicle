#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

int main()
{
    VideoCapture cap(0);
    Mat originalMat, hsvMat, resizeMat, inrangeMat, filtedMat;
    int *lowerHptr, *lowerSptr, *lowerVptr, *upperHptr, *upperSptr, *upperVptr;
    int lowerH = 0, lowerS = 0, lowerV = 0, upperH = 179, upperS = 255, upperV = 255;
    lowerHptr = &lowerH;
    lowerSptr = &lowerS;
    lowerVptr = &lowerV;
    upperHptr = &upperH;
    upperSptr = &upperS;
    upperVptr = &upperV;

    String winName = String("HSV_Range_Tool");
    namedWindow(winName, WINDOW_NORMAL);
    createTrackbar("LowerH", winName, lowerHptr, 255);
    createTrackbar("LowerS", winName, lowerSptr, 255);
    createTrackbar("LowerV", winName, lowerVptr, 255);
    createTrackbar("UpperH", winName, upperHptr, 179);
    createTrackbar("UpperS", winName, upperSptr, 255);
    createTrackbar("UpperV", winName, upperVptr, 255);

    while (1)
    {
        cap >> originalMat;
        if (originalMat.empty())
            continue;
        resize(originalMat, resizeMat, Size(), 1, 1);
        cvtColor(resizeMat, hsvMat, COLOR_BGR2HSV);
        inRange(hsvMat, Scalar(lowerH, lowerS, lowerV), Scalar(upperH, upperS, upperV), inrangeMat);
        Mat resultMat(resizeMat.rows, resizeMat.cols, CV_8UC3, Scalar(0, 0, 0));
        bitwise_and(resizeMat, resizeMat, filtedMat, inrangeMat);
        filtedMat.copyTo(resultMat, inrangeMat);
        imshow("Output", resultMat);
        imshow("Mask", inrangeMat);
        if (waitKey(1) == 'q')
            break;
    }
}
