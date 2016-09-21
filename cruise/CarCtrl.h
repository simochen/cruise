#include <iostream>
#include "serial.h"
using namespace std;
class CarControl:public CSerial
{
private:
	int direction;  //��ǰǰ�ַ���-5~5 ״̬����
	int dir;        //>0����ת =0ǰ�� <0����ת��int ��������
	int velocity;   //�ٶ�PWM��С������ʼ��Ϊ0��ʵ��Ĭ�ϳ�ʼֵ����0������
	int flag;       //=1��ǰת =0ֹͣ =-1���ת
public:
	CarControl(int Port=3,int Baud=9600):direction(0),dir(0),velocity(0),flag(1){
		Open(Port,Baud);Sleep(100);
		SendData("s",1); //ǰ��
		Sleep(100);
		for(int i=0;i<15;++i)
		{
			SendData("[",1); //�ٶ�����
			Sleep(55);
		}
	}
	//~CarControl(){};
	void addDir(const int n);
	void direct(const int n);
	int speed(const int s);
	void steer(const int n);
	int getDir();
	void brake();                //������Ҫ��ʵ��
};

void CarControl::addDir(int n) //��nΪת�����������תn>0,��תn<0
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

void CarControl::direct(int n) //-5~5�ֱ�����ҽǶȣ������Զ���0
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
			SendData("v",1); //����
			flag=-1;
		}
		else if(s>0)
		{
			SendData("s",1); //ǰ��
			flag=1;
		}
		else
		{
			SendData("c",1); //ֹͣ
			flag=0;
		}
	}
	if(velocity==abs(s))return velocity;
	while(velocity!=abs(s))
	{
		if(abs(s)>velocity)
		{
			SendData("]",1); //�ٶ�����
			Sleep(55);
			velocity++;
		}
		else
		{
			SendData("[",1); //�ٶȼ�С
			Sleep(55);
			velocity--;
		}
	}
	return velocity;
}

void CarControl::steer(int n) //��nΪת�����������תn>0,��תn<0
{
	if(n==0)return;
	int abs_n = abs(n);
	if(n<0)
	{
		for(int i=0; i<abs_n; i++)
		{
			if(dir<0)SendData("x",1); //��ת�Ƕ�����
			else if(dir>0) SendData("n",1); //��ת�Ƕȼ�С
			else SendData("t",1); //��ֱ������ת
			dir--;
		}
	}
	if(n>0)
	{
		for(int i=0; i<abs_n; i++)
		{
			if(dir>0) SendData("m",1); //��ת�Ƕ�����
			else if(dir<0) SendData("z",1); //��ת�Ƕȼ�С
			else SendData("u",1); //��ֱ������ת
			dir++;
		}
	}
	return;
}

int CarControl::getDir()
{
	return direction;
}