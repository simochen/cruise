#include <iostream>
#include "serial.h"
using namespace std;
class CarControl:public CSerial
{
private:
	int direction;  //当前前轮方向（-5~5 状态量）
	int dir;        //>0向右转 =0前进 <0向左转（int 叠加量）
	int velocity;   //速度PWM大小，被初始化为0，实际默认初始值不是0，待定
	int flag;       //=1向前转 =0停止 =-1向后转
public:
	CarControl(int Port=3,int Baud=9600):direction(0),dir(0),velocity(0),flag(1){
		Open(Port,Baud);Sleep(100);
		SendData("s",1); //前进
		Sleep(100);
		for(int i=0;i<15;++i)
		{
			SendData("[",1); //速度增大
			Sleep(55);
		}
	}
	//~CarControl(){};
	void addDir(const int n);
	void direct(const int n);
	int speed(const int s);
	void steer(const int n);
	int getDir();
	void brake();                //如有需要再实现
};

void CarControl::addDir(int n) //（n为转向叠加量）右转n>0,左转n<0
{
	if(n==0)return;
	direction += n;
	(direction<0)?direction=max(-5,direction):direction=min(5,direction);
	switch(direction)
	{
	case -5:SendData("q",1);break;
	case -4:SendData("w",1);break;
	case -3:SendData("e",1);break;
	case -2:SendData("r",1);break;
	case -1:SendData("t",1);break;
	case 0:SendData("l",1);break;
	case 1:SendData("y",1);break;
	case 2:SendData("u",1);break;
	case 3:SendData("i",1);break;
	case 4:SendData("o",1);break;
	case 5:SendData("p",1);break;
	default:break;
	}
	return;
}

void CarControl::direct(int n) //-5~5分别从左到右角度，不会自动回0
{
	if(direction==n) return;
	switch(n)
	{
	case -5:SendData("q",1);break;
	case -4:SendData("w",1);break;
	case -3:SendData("e",1);break;
	case -2:SendData("r",1);break;
	case -1:SendData("t",1);break;
	case 0:SendData("l",1);break;
	case 1:SendData("y",1);break;
	case 2:SendData("u",1);break;
	case 3:SendData("i",1);break;
	case 4:SendData("o",1);break;
	case 5:SendData("p",1);break;
	default:break;
	}
	direction = n;
	return;
}

int CarControl::speed(int s)
{
	
	if(flag*s<=0)
	{
		if(s<0)
		{
			SendData("v",1); //后退
			flag=-1;
		}
		else if(s>0)
		{
			SendData("s",1); //前进
			flag=1;
		}
		else
		{
			SendData("c",1); //停止
			flag=0;
		}
	}
	if(velocity==abs(s))return velocity;
	while(velocity!=abs(s))
	{
		if(abs(s)>velocity)
		{
			SendData("]",1); //速度增大
			Sleep(55);
			velocity++;
		}
		else
		{
			SendData("[",1); //速度减小
			Sleep(55);
			velocity--;
		}
	}
	return velocity;
}

void CarControl::steer(int n) //（n为转向叠加量）右转n>0,左转n<0
{
	if(n==0)return;
	int abs_n = abs(n);
	if(n<0)
	{
		for(int i=0; i<abs_n; i++)
		{
			if(dir<0)SendData("x",1); //左转角度增大
			else if(dir>0) SendData("n",1); //右转角度减小
			else SendData("t",1); //由直行向左转
			dir--;
		}
	}
	if(n>0)
	{
		for(int i=0; i<abs_n; i++)
		{
			if(dir>0) SendData("m",1); //右转角度增大
			else if(dir<0) SendData("z",1); //左转角度减小
			else SendData("u",1); //由直行向右转
			dir++;
		}
	}
	return;
}

int CarControl::getDir()
{
	return direction;
}