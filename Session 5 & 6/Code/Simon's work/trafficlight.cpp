#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include <iostream>
#include"stdlib.h"


using namespace std;
using namespace cv;


int main()
{
    Mat imggray;
    Mat img=imread("C:\\Users\\Tianchen\\Desktop\\red.jpg");
    Mat mask(img.rows,img.cols,CV_8UC1,Scalar(0,0,0));
	cout<<img.elemSize()<<endl<<img.channels();
	Mat HSVimg,merged,mergedbgr,MASK,op,binary,opgray;
	cvtColor(img,HSVimg,COLOR_BGR2HSV);
	inRange(HSVimg,Scalar(29,118,163),Scalar(102,255,255),mask);
	bitwise_and(img,img,op,mask);
	
	//imshow("1",op);
	//imshow("mask",mask);
	vector<vector<Point> > contoursext;
    vector<Point> pointext;
    vector<Vec4i> hireachyext;
	cout<<"1";
    Mat result(img.size(),CV_8U,Scalar(0));
    findContours(mask, contoursext, hireachyext, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    cout<<contoursext.size()<<endl;
    if(contoursext.size()==0)
    {
        cout<<"bye~";
        waitKey(0);
        
    }
    for (size_t t = 0; t < contoursext.size(); t++)
    {
        
        //approxPolyDP(contours[t], point, 2, true);
        if(contourArea(contoursext[t])>100)
        {
            drawContours(result,contoursext,t,Scalar(255),1);
            Point2f center;float radius;
            minEnclosingCircle(contoursext[t],center,radius);
            circle(result,center,radius,Scalar(255),2);
            //cout<<contoursext[t];
        }
		//drawContours(result,contoursext,t,Scalar(255),1);
        cout<<contourArea(contoursext[t])<<endl;
    }
    //cout<<contoursext[0];
    resize(result, result, Size(1080, (1080 * result.rows / result.cols)));
    imshow("results",result);
    waitKey(0);
}