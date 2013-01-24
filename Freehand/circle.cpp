#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <SerialStream.h>

using namespace cv;
using namespace std;
using namespace LibSerial;

void checkcircle(Point2f p);
void makecircle();
float getSlope(Point p1, Point p2, Point2f p);
void initArduino();
void writechar(char next_char);
void exitArduino();

Point initial,final;
Point2f point1, point2,point3;
int flag=0;
float h,k;
Mat img;
float radius;
int write_flag = 0;
char next_char ;
SerialStream usart;

int main(int argc, char *argv[])
{
    initArduino();
	if(strcmp(argv[1],"circle")==0);
	{
		Mat thresh, hsv, thresh_hsv;
		point1=Point(0,0);
		point2=Point(0,0);
		point3=Point(0,0);
		VideoCapture cap = VideoCapture(1);
		cap >> img;
		imshow("image", img);
		int key,wh=0,f=0;
		while(1)
		{
			wh++;
			cap >> img;
			GaussianBlur(img, img, Size(9, 9), 2, 2);
			cvtColor(img, hsv, CV_BGR2HSV);
			inRange(img, Scalar(15, 11, 10), Scalar(40, 25, 20), thresh);
			inRange(hsv,Scalar(100, 60, 25), Scalar(130, 200, 200), thresh_hsv);
			Moments mu;
			Point2f mc;
			mu = moments(thresh_hsv, true);
			mc =Point2f(mu.m10/mu.m00,mu.m01/mu.m00);
			if (mu.m10 < 1000000)
			{
				wh--;
				imshow("image", img);;
				key=waitKey(30);   
				if(key==27)
				{
					break;
				}
				continue;
			}
			if(wh==1) 
			{
				point1=Point2f(mc.x,mc.y);
				printf("point1 %f,%f\n", point1.x,point1.y);
			}
			if(f==2)
			{
				checkcircle(mc);
				printf("h=%d k=%d\n",int(h), int(k));
				circle(img, Point(int(h), int(k)), int(radius), Scalar(0,0,255), 2, 8, 0);
			}
			if(f==0 && (point1.x-(mc.x))*(point1.x-(mc.x))+(point1.y-(mc.y))*(point1.y-(mc.y))>5000.00 /*&& point1.y!=mc.y*/)
			{
				f=1;
				point2.x=(mc.x);
				point2.y=(mc.y);
				printf("point2 %f, %f\n",point2.x,point2.y);
				
			}
			
			if(f==1 && (point2.x-(mc.x))*(point2.x-(mc.x))+(point2.y-(mc.y))*(point2.y-(mc.y))>5000.00 /*&& point2.y!=mc.y && point1.y!=mc.y*/)
			{
				f=2;
				point3.x=(mc.x);
				point3.y=(mc.y);
				makecircle();
				printf("point3 %f, %f\n",point3.x,point3.y);
			}
			circle(img, Point(int(mc.x), int(mc.y)), 4, Scalar(0,0,255), -1, 8, 0);  
			imshow("image", img);;
			imshow("thresh_hsv", thresh_hsv);
			key=waitKey(30);   
			if(key==27)
			{
				break;
			}
		}
	}
    if(strcmp(argv[1],"line")==0)
    {
		Mat thresh, hsv, thresh_hsv;
		final=Point(0,0);
		initial=Point(0,0);
		VideoCapture cap = VideoCapture(1);
		cap >> img;
		imshow("image", img);
		int k,wh=0,f=0;
		while(1)
		{
			wh++;
			cap >> img;
			GaussianBlur(img, img, Size(9, 9), 2, 2);
			cvtColor(img, hsv, CV_BGR2HSV);
			inRange(hsv,Scalar(100, 80, 60), Scalar(130, 200, 200), thresh_hsv);
			Moments mu;
			Point2f mc;
			mu = moments(thresh_hsv, true);
			mc =Point2f(mu.m10/mu.m00,mu.m01/mu.m00);
			if(wh==1) initial=Point(int(mc.x),int(mc.y));
			if (f==1)
			{
				getSlope(initial, final, mc);
			}	
			if(f==0 && (initial.x-int(mc.x))*(initial.x-int(mc.x))+(initial.y-int(mc.y))*(initial.y-int(mc.y))>5000)
			{
				f=1;
				final.x=int(mc.x);
				final.y=int(mc.y);
			}
			circle(img, Point(int(mc.x), int(mc.y)), 4, Scalar(0,0,255), -1, 8, 0);  
			imshow("image", img);
			imshow("thresh_hsv", thresh_hsv);
			k=waitKey(30);   
			if(k==27)
			{
				break;
			}
		}
	}
    exitArduino();
    return 0;
}
void checkcircle(Point2f p)
{
	float d1=(point1.x-h)*(point1.x-h)+(point1.y-k)*(point1.y-k);
	float d2=(p.x-h)*(p.x-h)+(p.y-k)*(p.y-k);
	if(abs(sqrt(d1)-sqrt(d2))<	30.00)
    {
            writechar('r');
            printf("on path\n" );
			printf("d1=%f d2=%f %f %f\n",d1, d2,sqrt(d1),sqrt(d2));
    }
    else
    {   
        writechar('w');
        printf("out of path\n");
        printf("d1=%f d2=%f %f %f\n",d1,d2, sqrt(d1), sqrt(d2));
    }
    
}
float getSlope(Point p1, Point p2, Point2f p)
{
    float m1,m2;
   // p.y=p.y+ caliber_dist;
    m1=(p1.y-p2.y)/float(p1.x-p2.x);
    m2=(p1.y-p.y)/float(p1.x-p.x);
    //circle(img, Point(int(p.x), int(p.y)+caliber_dist), 4, Scalar(0,0,255), -1, 8, 0);  
    if(abs(m1-m2)<0.8)
    {
        printf("on path\n" );
    }
    else
    {
        printf("out of path\n");
    }
}
void makecircle()
{
	float x1=point1.x,y1=point1.y,x2=point2.x,y2=point2.y,x3=point3.x,y3=point3.y;
	{
		h=((x1*x1+y1*y1-x2*x2-y2*y2)*(y1-y3)-(y1-y2)*(x1*x1+y1*y1-x3*x3-y3*y3))/2/((y1-y3)*(x1-x2)-(x1-x3)*(y1-y2));
		k=((x1*x1+y1*y1-x2*x2-y2*y2)*(x1-x3)-(x1-x2)*(x1*x1+y1*y1-x3*x3-y3*y3))/2/((x1-x3)*(y1-y2)-(y1-y3)*(x1-x2));
		printf("h=%f k=%f\n", h,k);
		radius = sqrt((point1.x-h)*(point1.x-h)+(point1.y-k)*(point1.y-k));
		printf("radius=%f\n", radius);
	}

}

void initArduino()
{
    usart.Open("/dev/ttyACM0");
    usart.SetBaudRate(SerialStreamBuf::BAUD_9600);
    usart.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    usart.SetNumOfStopBits( 1 );
    usart.SetParity(SerialStreamBuf::PARITY_NONE);
    usart.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
}

void writechar(char next_char)
{
    usart << next_char;
    printf("put %c\n", next_char);
}

void exitArduino()
{
    writechar('r');
    usart.Close();
}