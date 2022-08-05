#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include<opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main()
{

    vector<Mat> channels_HSV;
	vector<Mat> mv;
	Mat img=imread("C:\\Users\\Tianchen\\Desktop\\new.jpg");
	Mat mask(img.rows,img.cols,CV_8UC1,Scalar(0,0,0));
	cout<<img.elemSize()<<endl<<img.channels();
	Mat HSVimg,merged,mergedbgr,MASK,op,binary,opgray;
	cvtColor(img,HSVimg,COLOR_BGR2HSV);
	inRange(HSVimg,Scalar(160,128,77),Scalar(255,255,255),mask);
	bitwise_and(img,img,op,mask);
	
	//imshow("1",op);
	//imshow("mask",mask);
	vector<vector<Point> > contoursext;
    vector<Point> pointext;
    vector<Vec4i> hireachyext;
	cout<<"1";
    findContours(mask, contoursext, hireachyext, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    cout<<contoursext.size()<<endl;
     if(contoursext.size()==0)
     {
         continue;
     }
	cout<<"1";
	Mat result(img.size(),CV_8U,Scalar(0));
	Rect kuang;
	Mat kuangtu,kuanggray;
	for (size_t t = 0; t < contoursext.size(); t++)
    {
        
        //approxPolyDP(contours[t], point, 2, true);
		drawContours(result,contoursext,t,Scalar(255),1);
		double area = contourArea(contoursext[t]);
		cout<<area<<endl;
        //cout<<point.size()<<endl;
        
		if(area>50000)
		{
			approxPolyDP(contoursext[t], pointext, 10, true);
			kuang = boundingRect(pointext);
			kuangtu = img(kuang);
			imshow("kuang)",kuangtu);
            cout<<pointext.size()<<endl;
            cout<<"1 "<<pointext[0]<<endl<<"2 "<<pointext[1]<<endl<<"3"<<pointext[2]<<endl<<"4"<<pointext[3]<<endl;

		}
     }
     waitKey(0);
    vector<Point2f> src_corners(4);
    src_corners[0] = pointext[0];
    src_corners[1] = pointext[1];
    src_corners[2] = pointext[2];
    src_corners[3] = pointext[3];
     vector<Point2f> dst_corners(4);
     dst_corners[0]=Point(kuang.x,kuang.y);
     dst_corners[1]=Point(kuang.x,kuang.y+kuang.height);
     dst_corners[2]=Point(kuang.x+kuang.width,kuang.y+kuang.height);
     dst_corners[3]=Point(kuang.x+kuang.width,kuang.y);
     Mat resultImage;
    Mat warpmatrix = getPerspectiveTransform(src_corners, dst_corners);
    cout<<"ttttttttttttttttttttttttt";
    warpPerspective(img, resultImage, warpmatrix, resultImage.size(), INTER_LINEAR);
    //namedWindow("Final Result", CV_WINDOW_AUTOSIZE);
    imshow("Final Result", resultImage);
    Mat kuangtu2=resultImage(kuang);
    imshow("kuangtu trans",kuangtu2);
    if(kuangtu.empty())
    {
        waitKey(0);

    }
	 //imshow("con",result);
	 Mat imggray,bin,dst;
	cvtColor(kuangtu2, imggray, COLOR_BGR2GRAY);
    imwrite("111.jpg",imggray);
   //Laplacian(imggray,dst,CV_16S,1,1,0,BORDER_DEFAULT);
   //Mat dst_abs;
   //convertScaleAbs(dst,dst_abs);
    //imshow("lap",dst_abs);
     threshold(imggray, bin, 150, 255, THRESH_BINARY);
     bin = ~bin;
	medianBlur(bin,bin,3);
    imshow("binary", bin);
    waitKey(0);
	 vector<vector<Point> > contours;
    vector<Point> point;
    vector<Vec4i> hireachy;
    vector<vector<Point> > contours1;
    vector<Point> point1;
    vector<Vec4i> hireachy1;
	cvtColor(kuangtu,kuanggray,COLOR_BGR2GRAY);
	imshow("kuanggray",kuanggray);
    findContours(bin, contours, hireachy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
    if(contours.size()==0)
    {
        cout<<"TTTTTTTTTTTTTTTTTTTTTT";
    }
     Mat results(img.size(),CV_8U,Scalar(0));
    for(int j=0;j<contours.size();j++)
    {


        cout<<hireachy[j][0]<<hireachy[j][1]<<hireachy[j][2]<<hireachy[j][3]<<endl;


        cout<<"yes"<<endl;
    }
         vector<vector<Point> > contours_poly(contours.size());
         Mat smt(kuangtu.size(), CV_8UC3, Scalar::all(0));
         cout<<"TTTTTTTTTTTTTTTTTTTTTT";
         waitKey(0);
    for(int j=0;j<contours.size();j++)
    {
        double area1 = contourArea(contours[j]);
        cout<<area1<<" -area"<<endl;
        if(area1<100||area1>90000)
        {
            continue;
        }
        
        approxPolyDP(contours[j], contours_poly[j],14, true);
        
        drawContours(smt, contours_poly, j, Scalar(0, 255, 255), 2, 8);
        cout<<contours_poly[j].size()<<endl;
        //cout<<contours_poly[j].size()<<endl;
    }
    imshow("smt",smt);
    waitKey(0);
    Mat smtgray;
    // for(int j=0;j<contours.size();j++)
    // {
    //     if(hireachy[j][2]==-1)
    //     {
    //         drawContours(results,contours,j,Scalar(255),0);
    //     }
    // }
    // cvtColor(smt,smtgray,CV_BGR2GRAY);
    // findContours(smtgray, contours1, hireachy1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    // vector<vector<Point> > contours_poly1(contours1.size());
    // Mat smt1(kuangtu.size(), CV_8UC3, Scalar::all(0));
    // cout<<contours1.size()<<endl;
    // for(int j=0;j<contours1.size();j++)
    // {
    //     double area1 = contourArea(contours1[j]);
    //     cout<<area1;
    //     //if(area1)
    //     approxPolyDP(Mat(contours1[j]), contours_poly1[j], 2, true);
    //     drawContours(smt1, contours_poly1, j, Scalar(0, 255, 255), 2, 8);
    //     cout<<contours_poly1[j].size()<<endl;
    // }
	//imshow("smt",smt1);
	// split(img,channels_HSV);
	// threshold(channels_HSV[0],mv[0],0,180,THRESH_TRUNC);
	// threshold(channels_HSV[1],mv[1],0,255,THRESH_TRUNC);
	// threshold(channels_HSV[2],mv[2],0,185,THRESH_TRUNC);
	// merge(mv,merged);
	// cvtColor(merged,mergedbgr,CV_HSV2BGR);
	// imshow("result",merged);
	waitKey(0);
    
	
}