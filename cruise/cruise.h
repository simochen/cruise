#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <core/core.hpp>
#include <highgui/highgui.hpp>
#include <imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

void getMid(Mat edges, double midline[50][3])
{
	double X[480],Y[480],dis[480];
	dis[0] = 0;
	int Pcnt=0,sum=0,cnt=0,flag=0;
	double x,y;
	for( int i = edges.rows - 1; i >= 0; i--){
		for( int j = 0; j < edges.cols; j++){
			if((int)*(edges.data+i*edges.step[0]+j*edges.step[1])!=0){sum+=j;cnt++;}
		}
		if( cnt && cnt < 5 ) {
			x = sum*1.0/cnt - edges.cols*1.0/2;
			y = edges.rows - 1 - i;
			X[Pcnt] = x;
			Y[Pcnt] = y;
			if(Pcnt) {
				dis[Pcnt] = dis[Pcnt-1]+sqrt((x-X[Pcnt-1])*(x-X[Pcnt-1])+(y-Y[Pcnt-1])*(y-Y[Pcnt-1]));
			}
			Pcnt++;
		}
		sum = 0;
		cnt = 0;
	}
	int i = 0;
	double sumx = 0,sumy = 0;
	int count = 0;
	cnt = 0;
	for(int j = 0; j < 49; j++)
	{
		while((i < Pcnt) && ((int)(dis[i]/18) == j))
		{
			sumx += X[i];
			sumy += Y[i];
			cnt++;
			if(!flag){
				flag = 1;
				count++;
			}
			i++;
		}
		if(cnt){midline[j][0] = sumx/cnt; midline[j][1] = sumy/cnt; sumx = 0;sumy = 0;cnt = 0;}
		midline[j][2] = flag;
		flag = 0;
	}
	//cout << count <<endl;
}

void Curv(double midline[50][3],double r[48][2])
{
	double ax,ay,bx,by,cx,cy,lab,lbc,lac,S;
	int i = 0,cnt = 0,oft1 = 1,oft2 = 1;       //offset
	for (int j = 0; j< 48; ++j) r[j][1] = 0;
	while(i < 48)
	{
		if(!midline[i][2]){i++;continue;}
		ax = midline[i][0];                                   //计算前方i个单位的曲率
		ay = midline[i][1];
		while(!midline[i+oft1][2])oft1++;
		bx = midline[i+oft1][0];
		by = midline[i+oft1][1];
		while(!midline[i+oft1+oft2][2])oft2++;
		cx = midline[i+oft1+oft2][0];
		cy = midline[i+oft1+oft2][1];
		lab = sqrt((ax-bx)*(ax-bx)+(ay-by)*(ay-by));
		lbc = sqrt((bx-cx)*(bx-cx)+(by-cy)*(by-cy));
		lac = sqrt((ax-cx)*(ax-cx)+(ay-cy)*(ay-cy));
		S = ((cx-ax)*(by-ay)-(bx-ax)*(cy-ay))/2;            //三点顺时针分布，面积为正，逆时针分布，面积为负
		r[i][0] = 4*S/(lab*lac*lbc);                        //外接圆半径为R=abc/4S,曲率=1/R=4S/abc；向右曲率为正，向左曲率为负
		r[i][1] = 1;
		i += oft1;
		oft1 = 1;
		oft2 = 1;
		cnt++;
	}
	//cout << cnt << endl;
}