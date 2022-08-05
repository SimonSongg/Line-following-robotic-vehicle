#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

int main()
{
    VideoCapture cap(0);
    Mat originalMat, resizeMat, inrangeMat, filtedMat;
    int *lowerBptr, *lowerGptr, *lowerRptr, *upperBptr, *upperGptr, *upperRptr;
    int lowerB = 0, lowerG = 0, lowerR = 0, upperB = 179, upperG = 255, upperR = 255;
    lowerBptr = &lowerB;
    lowerGptr = &lowerG;
    lowerRptr = &lowerR;
    upperBptr = &upperB;
    upperGptr = &upperG;
    upperRptr = &upperR;

    String winName = String("HSV_Range_Tool");
    namedWindow(winName, WINDOW_NORMAL);
    createTrackbar("LowerB", winName, lowerBptr, 255);
    createTrackbar("LowerG", winName, lowerGptr, 255);
    createTrackbar("LowerR", winName, lowerRptr, 255);
    createTrackbar("UpperB", winName, upperBptr, 255);
    createTrackbar("UpperG", winName, upperGptr, 255);
    createTrackbar("UpperR", winName, upperRptr, 255);

    while (1)
    {
        cap >> originalMat;
        if (originalMat.empty())
            continue;
        resize(originalMat, resizeMat, Size(), 1, 1);
        inRange(resizeMat, Scalar(lowerB, lowerG, lowerR), Scalar(upperB, upperG, upperR), inrangeMat);
        Mat resultMat(resizeMat.rows, resizeMat.cols, CV_8UC3, Scalar(0, 0, 0));
        bitwise_and(resizeMat, resizeMat, filtedMat, inrangeMat);
        filtedMat.copyTo(resultMat, inrangeMat);
        imshow("Output", resultMat);
        imshow("Mask", inrangeMat);
        if (waitKey(1) == 'q')
            break;
    }
}
