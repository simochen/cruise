//#ifndef _CarControl_h
//#define _CarControl_h
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
	int left_angle; //自动回零转向的左边角度值
	int right_angle;//自动回零转向的右边角度值
public:
	CarControl(int Port=3,int Baud=9600):direction(0),dir(0),velocity(0),flag(1),left_angle(90),right_angle(90){
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
	//void direct(const int n);
	int direct(const int d);
	int speed(const int s);
	//void steer(const int n);
	void steer(const int angle);
	void brake();                //如有需要再实现
};

int CarControl::direct(const int d) //-5~5分别从左到右角度，不会自动回0
{
	if(direction==d)return d;
	switch(d)
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
	return direction = d;
}

/*
void CarControl::direct(int n) //（n为转向叠加量）右转n>0,左转n<0
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
*/

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

/*
void CarControl::steer(int n) //（n为转向叠加量）右转n>0,左转n<0
{
	if(n==0)return;
	int abs_n = abs(n);
	if(n<0)
	{
		for(int i=0; i<abs_n; i++)
		{
			if(dir<=0)SendData("x",1); //左转角度增大
			else SendData("n",1); //右转角度减小 
			dir--;
		}
	}
	if(n>0)
	{
		for(int i=0; i<abs_n; i++)
		{
			if(dir>=0) SendData("m",1); //右转角度增大
			else SendData("z",1); //左转角度减小
			dir++;
		}
	}
	return;
}
*/

void CarControl::steer(int angle) //右转angle>0,左转angle<0,自动回零
{

    int abs_a=abs(angle);
    if(angle<0)
    {
        while(abs(left_angle-abs_a)>2)  //这里需要注意angle角度不合适的话会陷入死循环,修改方案为abs(left_angle-abs_a)>5
        {
            if(left_angle < abs_a)
            {
                SendData("x",1); //左转角度增大
                left_angle+=5;  //5是随便写的，具体数值待测
            }
            else
            {
                SendData("z",1); //左转角度减小
                left_angle-=5;
            }
        }
        //SendData("g",1); //左转一个直角
    }
    else if(angle>0)
    {
        while(abs(right_angle-abs_a)>2)
        {
            if(right_angle < abs_a)
            {
                SendData("m",1); //右转角度增大
                right_angle+=5; //5是随便写的，具体数值待测
            }
            else
            {
                SendData("n",1); //右转角度减小
                right_angle-=5;
            }
        }
        //SendData("h",1); //右转一个直角
    }
}


//#endif // _CarControl_h