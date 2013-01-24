#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <math.h>
#include <SerialStream.h>

using namespace cv;
using namespace std;
using namespace LibSerial ;

void mouseHandler(int event, int x, int y, int flags, void* param);
float getSlope(Point p1, Point p2, Point2f p);
void initArduino();
void writechar(char next_char);
void exitArduino(); 

Point point1, point2,initial,final;
int flag=0;
Mat img;
int write_flag = 0;
char next_char ;
SerialStream usart;

int main(int argc, char *argv[])
{
    initArduino();
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
		//printf("finalx=%d finaly=%d initialx=%d initialy=%d\n",final.x,final.y,initial.x,initial.y);
        cap >> img;
        GaussianBlur(img, img, Size(9, 9), 2, 2);
        
        cvtColor(img, hsv, CV_BGR2HSV);
        inRange(hsv,Scalar(100, 45, 15), Scalar(130, 200, 200), thresh_hsv);
        Moments mu;
        Point2f mc;
        mu = moments(thresh_hsv, true);
        mc = Point2f(mu.m10/mu.m00,mu.m01/mu.m00);
        if(wh==1) initial=Point(int(mc.x),int(mc.y));
        if (f==1)
        {
            getSlope(initial, final, mc);
        }
      //  printf("%d\n", (initial.x-int(mc.x))*(initial.x-int(mc.x))+(initial.y-int(mc.y))*(initial.y-int(mc.y)));
        if(f==0 && (initial.x-int(mc.x))*(initial.x-int(mc.x))+(initial.y-int(mc.y))*(initial.y-int(mc.y))>3000)
        {
			f=1;
			final.x=int(mc.x);
			final.y=int(mc.y);
		}
        
        //printf("%f %f\n", mc.y, mc.x);
        circle(img, Point(int(mc.x), int(mc.y)), 4, Scalar(0,0,255), -1, 8, 0);  
        imshow("image", img);
        imshow("thresh_hsv", thresh_hsv);
        k=waitKey(30);   
        if(k==27)
        {
            break;
        }
    }
    exitArduino();
    return 0;
}

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN && flag == 0)
    {
        /* left button clicked. ROI selection begins */
        point1 = Point(x, y);
        flag = 1;
        printf("point1 %d %d\n",x,y);
    }

    else if (event == CV_EVENT_LBUTTONDOWN && flag==1)
    {
        point2 = Point(x, y);
        flag = -1;
        printf("point 2 %d %d\n",x,y);
      
    }
    
}

float getSlope(Point p1, Point p2, Point2f p)
{
    float m1,m2;
   // p.y=p.y+ caliber_dist;
    m1=(p1.y-p2.y)/float(p1.x-p2.x);
    m2=(p1.y-p.y)/float(p1.x-p.x);
    //circle(img, Point(int(p.x), int(p.y)+caliber_dist), 4, Scalar(0,0,255), -1, 8, 0);  
    if(abs(abs(m1)-abs(m2))<0.1)
    {
        if(write_flag)
        {
            writechar('r');
            write_flag=0;
        }
        //printf("on path\n" );
    }
    else
    {
        if(!write_flag)
        {
            writechar('w');
            write_flag=1;    
        }
        //printf("out of path\n");
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