#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <core/core.hpp>
#include <highgui/highgui.hpp>
#include <imgproc/imgproc.hpp>
#include <features2d/features2d.hpp>
#include <calib3d/calib3d.hpp>
#include <nonfree/nonfree.hpp>
#include "cruise.h"
#include "CarCtrl.h"

using namespace std;
using namespace cv;

int edgeThresh = 1;
int lowThreshold=100;
//int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
Mat dst;
static void CannyThreshold(Mat &src_gray,Mat &detected_edges,int, void*);

const float PI=3.1415926;
//Preparation for kmeans
Scalar colorTab[] =
{
    Scalar(0, 0, 255),
    Scalar(0,255,0),
    Scalar(255,100,255),
    Scalar(255,100,100),
    Scalar(0,255,255)
};


class sampleNULL{};
int findCon(Mat &dila,Mat &frame, Mat &edge , Mat &sign_bi, int flag);//flag == 1 在frame上显示结果，否则不显示
int matchSign(Mat &frame);
int matchTem(Mat img);

int main()
{
	VideoCapture cap(0);//"test.avi"); // open the default camera
    if(!cap.isOpened()) return -1; // check if we succeeded
	//VideoWriter wri("result.avi",CV_FOURCC('P','I','M','1'),30,Point(640,480));//录像用的，注销这里和后面有wri的部分就可以录像了
    //if(!wri.isOpened()) return -1; // check if we succeeded

	Mat edges,frame_gray,frame_bi,erod,dila;
	Mat elem5(3,3,CV_8U,Scalar(1));
	float angle=0;
	
	CarControl car(3,9600);
	const int vec=5; //speed
	car.speed(vec);
	int back=0;
	//variables for controlling
	for(;;)
    {
		static Mat frame;
        cap >> frame; // get a new frame from camera
		if(frame.empty())break;
		//GaussianBlur(frame, frame_gray, Size(7,7), 1.5, 1.5);
		//===========预处理=========
		cvtColor(frame, frame_gray, CV_BGR2GRAY);
		threshold(frame_gray,frame_bi,50,255,THRESH_BINARY_INV);
	    erode(frame_bi,erod,elem5,Point(-1,-1),2);//这里使用腐蚀和膨胀消除干扰，在简陋的赛道下可以消除大部分干扰
													//理想赛道时可以使用这个来节省运算量
		dilate(erod,dila,elem5,Point(-1,-1),5);//open operation in order to curb the noise(effective)
		//morphologyEx(frame_bi,frame_bi,MORPH_OPEN,element5);
		//imshow("erode",erod);
		//imshow("dilate",dila);

		//=====findContours=========//找赛道边界
		Mat edge(dila.rows,dila.cols,CV_8U); //与输入二值图像大小相同
		Mat sign_bi;
		int detectSign;
		detectSign = findCon(dila,frame,edge,sign_bi,1); //edge返回含有赛道边界的图像
		bitwise_and(edge,dila,edges); //与运算
		imshow("edges",edges);

		//=======matchSign===========//特征匹配
	/*
		int signs;
		if(detectSign){
			signs = matchSign(sign_bi);
			cout << "检测到标志：";
			switch(signs){
			case 0: cout << " 左转 " << endl; break;
			case 1: cout << " 停止 " << endl; break;
			case 2: cout << " 右转 " << endl; break;
			default: break;
			}
		}
		*/

        //========赛道信息获取=========
		double midline[50][3];
		double r[48][2];
		getMid(edges,midline); 
		Curv(midline,r);
	/*
		int cnt1=0,cnt2=0;
		for(int i=0; i<30; i++)        //用于判断弯道大小，以调整预瞄点
		{
			if(r[i][1]!=0)
			{
				if(r[i][0]>0.01) cnt1++;
				else if(r[i][0]>0.005) cnt2++;
			}
		}
	*/
		//========小车控制==============
		dst = Scalar::all(0);
		static double k,err,err_last=0,err_prev=0,cur=0,cur_last=0;
		int Dir = car.getDir();
		int len = 2;  //预瞄点推后距离，小车接收信号有延时
		for(int i=10+len;i<20+len;++i)  //取midline中第10个点，如果没有的话就取11点，以此类推
		{
			if(midline[i][2]!=0 && r[i][1]!=0){
				if(back==1){back=0;car.speed(vec);}
				Point pt1(frame.cols/2+midline[i][0],frame.rows-midline[i][1]);
				line(frame,Point(frame.cols/2,frame.rows),pt1,Scalar(0,255,255),5); //黄线前进
				cur_last = cur;
				cur = r[i][0];
				err_prev = err_last;
				err_last = err;
				k = 1-atan2(midline[i][1],midline[i][0])*2/PI;
				err = k;
				break;
			}
			if(i==19)
			{
				car.speed(-vec);
				back=1;
			}
		}
		
		if(back==1)
		{
			for(int i=0;i<5;++i)
			{
			if(midline[i][2]!=0 && r[i][1]!=0){
				Point pt1(frame.cols/2+midline[i][0],frame.rows-midline[i][1]);
				line(frame,Point(frame.cols/2,frame.rows),pt1,Scalar(0,0,255),5);  //红线倒车
				cur_last = cur;
				cur = r[i][0];
				err_prev = err_last;
				err_last = err;
				k = 1-atan2(midline[i][1],midline[i][0])*2/PI;
				err = k;
				break;}
			}
		}
	
		//舵机方向控制
		static double steer, yaw, ctrl;
		if (midline[len][2]!=0 && midline[len+1][2]!=0)
			yaw = 1-atan2(midline[len+1][1]-midline[len][1],midline[len+1][0]-midline[len][0])*2/PI;  //偏航角
		else yaw = 0;
		ctrl = 8*err + 3*(err - err_last); //PD控制器
		steer = ctrl + 10*cur + 5*yaw; 
		if(back==1)car.direct(-int(steer));
		else	car.direct(int(steer));

		//cout << "dir:" << Dir << " steer:" << steer <<" r:" << cur <<" yaw:" << yaw << endl;
	
	/*
		//PID控制（增量式）
		static double sp, si, sd, spid;
		sp = err-err_last;
		si = err;
		sd = err-2*err_last+err_prev;
		spid = 0*sp+1.9*si+0*sd + cur-cur_last;
		//spid = 5*(0*k-0*k_last+0*k_prev);
		if(back==1) car.addDir(-int(spid));
		else car.addDir((int)(spid));
		cout << "dir:" << Dir << "spid:" << spid << " si:" << si << endl;
	*/
		//=========显示=========
		frame_gray.copyTo( dst, edges);
		imshow("source", frame);
		//wri<<frame;
         if(waitKey(30) >= 0) 
		{
			car.speed(0);
			break;
		}
	}
	//wri.release();
	
	//========matchTem==========//标志匹配
	
	int total=0,cntL=0,cntR=0,cntS=0;
	Mat img_sign;
	char img_name[50];
	int signs;
	while(1){
		sprintf(img_name,"./sign_bi/%d.jpg",++total);
		img_sign = imread(img_name);
		if(img_sign.empty())break;
		signs = matchTem(img_sign);
		switch(signs){
		case 0: sprintf(img_name,"./sign_rec/Left_%d.jpg",++cntL); imwrite(img_name,img_sign); break;
		case 1: sprintf(img_name,"./sign_rec/Stop_%d.jpg",++cntS); imwrite(img_name,img_sign); break;
		case 2: sprintf(img_name,"./sign_rec/Right_%d.jpg",++cntR); imwrite(img_name,img_sign); break;
		default: break;
		}
	}
	
    //===========================
	car.speed(0);
	return 0;
}

static void CannyThreshold(Mat &src_gray,Mat &detected_edges,int, void*)
{
    /// Reduce noise with a kernel 3x3
    blur( src_gray, detected_edges, Size(3,3) );

    /// Canny detector
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
}

int findCon(Mat &dila,Mat &frame, Mat &edge, Mat &sign_bi, int flag)
{
		vector<vector<Point>> contours,yelcon;
		vector<Point> poly;
		Mat frame_cpy,hsv;
		frame.copyTo(frame_cpy);
		
		//CannyThreshold(dila,dila,0, 0);
		//findContours的输入是二值图像
		findContours(dila,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE); 
		 //cout<<"size:"<<contours[0][1]<<endl;
		 //imshow("findContours",result);
		int q=0;//r=0;
		int x=0,y=0;
		int xf=0,yf=0;
		double p=(x-dila.cols/2)*(x-dila.cols/2)+( y-dila.rows)*( y-dila.rows);
		//p=new double[contours.size()]{0};
		for(int i=0;i<contours.size();++i)    //找出重心距小车坐标系原点最近的轮廓
		{
			x=0;y=0;
			for(int j=0;j<contours[i].size();++j)
			{
				x+=contours[i][j].x;
				y+=contours[i][j].y;
			}
			x/=contours[i].size();
			y/=contours[i].size();
			Point ipt(x,y);
			//circle( pear, ipt, 1,colorTab[0] , CV_FILLED, CV_AA );
			if(p>(x-dila.cols/2)*(x-dila.cols/2)+( y-dila.rows)*( y-dila.rows))
			{
				p=(x-dila.cols/2)*(x-dila.cols/2)+( y-dila.rows)*( y-dila.rows);
				//r=q;
				q=i;
				//xf=x;
				//yf=y;
			}
		}
		//circle( pear, Point(xf,yf),5,colorTab[0] , CV_FILLED, CV_AA );
		for(int i = 0; i < contours[q].size(); i++ )
        {

			Point ipt=contours[q][i];

			circle( edge, ipt, 1,Scalar(0, 0, 0) , CV_FILLED, CV_AA );
			//Point ipt=centers.at<Point2f>(i,0);
			if(flag==1)circle( frame, ipt, 3, colorTab[2], CV_FILLED, CV_AA );
		}
		threshold(edge,edge,0,255,THRESH_BINARY_INV);

		int dSign = 0;
		Mat yellow(frame_cpy.size(),CV_8U);
		//转换到HSV颜色空间
		cvtColor(frame_cpy,hsv,CV_BGR2HSV);
		vector<Mat> v;
		split(hsv,v);  //分割三通道
		for( int i = hsv.rows - 1; i >= 0; i--){
			for( int j = 0; j < hsv.cols; j++){
				if(*(v[0].data+i*v[0].step[0]+j*v[0].step[1])>20 && *(v[0].data+i*v[0].step[0]+j*v[0].step[1])< 40
					&& *(v[1].data+i*v[1].step[0]+j*v[1].step[1])> 50
					&& *(v[2].data+i*v[2].step[0]+j*v[2].step[1])> 80)
					*(yellow.data+i*yellow.step[0]+j*yellow.step[1]) = 255;
				else
					*(yellow.data+i*yellow.step[0]+j*yellow.step[1]) = 0;
			}
		}
		imshow("yellow",yellow);
		Mat yel_cpy;
		yellow.copyTo(yel_cpy);
		findContours(yellow,yelcon,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

		for(int i=0;i<yelcon.size();++i)
		{
			approxPolyDP(Mat(yelcon[i]),poly,8,true);
			if( poly.size() == 3 && contourArea(yelcon[i])>1400){
				int minX=640, minY=480, maxX=0, maxY=0;
				vector<Point>::const_iterator itp= poly.begin();
				while (itp!=(poly.end()-1)) {
					if(minX > (*itp).x) minX = (*itp).x;
					if(minY > (*itp).y) minY = (*itp).y;
					if(maxX < (*itp).x) maxX = (*itp).x;
					if(maxY < (*itp).y) maxY = (*itp).y;
					line(frame,*itp,*(itp+1),colorTab[1],2);
					++itp;
				}
				// last point linked to first point
				if(minX > (*itp).x) minX = (*itp).x;
				if(minY > (*itp).y) minY = (*itp).y;
				if(maxX < (*itp).x) maxX = (*itp).x;
				if(maxY < (*itp).y) maxY = (*itp).y;
				line(frame,*(poly.begin()),*(poly.end()-1),colorTab[1],2);
				//Save sign
				Mat sign;
				Rect rect( max(minX-5,0),max(minY-5,0),min(maxX-minX+10,639),min(maxY-minY+10,479));
				yel_cpy(rect).copyTo(sign_bi);
				frame_cpy(rect).copyTo(sign);
				static int count=0;
				char img_name[100];
				sprintf(img_name,"./sign/%d.jpg",++count);
				imwrite(img_name,sign);
				sprintf(img_name,"./sign_bi/%d.jpg",count);
				imwrite(img_name,sign_bi);
				dSign = 1;
			}
		}

		return dSign;
}

int matchSign(Mat &frame)
{
	Mat img_scene = frame;
	Mat img_object[3];
	img_object[0] = imread("left.jpg");
	img_object[1] = imread("stop.jpg");
	img_object[2] = imread("right.jpg");
//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;
	SurfFeatureDetector detector( minHessian );
	vector<KeyPoint> keypoints_object, keypoints_scene;
	double min = 100;
	int pos;
	for(int j=0;j<3;j++)
	{
		detector.detect( img_object[j], keypoints_object );
		detector.detect( img_scene, keypoints_scene );
//-- Step 2: Calculate descriptors (feature vectors)
		SurfDescriptorExtractor extractor;
		Mat descriptors_object, descriptors_scene;
		extractor.compute( img_object[j], keypoints_object, descriptors_object );
		extractor.compute( img_scene, keypoints_scene, descriptors_scene );
//-- Step 3: Matching descriptor vectors using FLANN matcher
		FlannBasedMatcher matcher;
		vector< DMatch > matches;
		matcher.match( descriptors_object, descriptors_scene, matches );
		double max_dist = 0; double min_dist = 100;
//-- Quick calculation of max and min distances between keypoints
		for( int i = 0; i < descriptors_object.rows; i++ )
		{ 
			double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}
		if( min > min_dist)
		{ 
			min = min_dist;
			pos = j;
		}
	}
	return pos;
}

int matchTem( Mat img ) //模板匹配
{
/// Source image to display
	//if((max_left>0.75 && temp==-1)||(max_right>0.75 && temp==1)||(max_stop>0.75 && temp==0))return 0;
	Mat temp[3];
	temp[0] = imread("left.jpg");
	temp[1] = imread("stop.jpg");
	temp[2] = imread("right.jpg");
	Size dsize = temp[0].size();
	Mat image = Mat(dsize,img.type());
	resize(img, image, dsize);
	Mat result;
	double max = -1;
	int pos;
/// Do the Matching and Normalize
	for(int i=0;i<3;i++)
	{
		matchTemplate( image, temp[i], result, CV_TM_CCOEFF_NORMED );
//normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
/// Localizing the best match with minMaxLoc
		double minVal, maxVal; 
		Point minLoc, maxLoc;
		minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
		if(maxVal>max)
		{
			max=maxVal;
			pos=i;
		}
	}
	return pos;
}