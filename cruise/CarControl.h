//#ifndef _CarControl_h
//#define _CarControl_h
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
	int left_angle; //�Զ�����ת�����߽Ƕ�ֵ
	int right_angle;//�Զ�����ת����ұ߽Ƕ�ֵ
public:
	CarControl(int Port=3,int Baud=9600):direction(0),dir(0),velocity(0),flag(1),left_angle(90),right_angle(90){
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
	//void direct(const int n);
	int direct(const int d);
	int speed(const int s);
	//void steer(const int n);
	void steer(const int angle);
	void brake();                //������Ҫ��ʵ��
};

int CarControl::direct(const int d) //-5~5�ֱ�����ҽǶȣ������Զ���0
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
void CarControl::direct(int n) //��nΪת�����������תn>0,��תn<0
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

/*
void CarControl::steer(int n) //��nΪת�����������תn>0,��תn<0
{
	if(n==0)return;
	int abs_n = abs(n);
	if(n<0)
	{
		for(int i=0; i<abs_n; i++)
		{
			if(dir<=0)SendData("x",1); //��ת�Ƕ�����
			else SendData("n",1); //��ת�Ƕȼ�С 
			dir--;
		}
	}
	if(n>0)
	{
		for(int i=0; i<abs_n; i++)
		{
			if(dir>=0) SendData("m",1); //��ת�Ƕ�����
			else SendData("z",1); //��ת�Ƕȼ�С
			dir++;
		}
	}
	return;
}
*/

void CarControl::steer(int angle) //��תangle>0,��תangle<0,�Զ�����
{

    int abs_a=abs(angle);
    if(angle<0)
    {
        while(abs(left_angle-abs_a)>2)  //������Ҫע��angle�ǶȲ����ʵĻ���������ѭ��,�޸ķ���Ϊabs(left_angle-abs_a)>5
        {
            if(left_angle < abs_a)
            {
                SendData("x",1); //��ת�Ƕ�����
                left_angle+=5;  //5�����д�ģ�������ֵ����
            }
            else
            {
                SendData("z",1); //��ת�Ƕȼ�С
                left_angle-=5;
            }
        }
        //SendData("g",1); //��תһ��ֱ��
    }
    else if(angle>0)
    {
        while(abs(right_angle-abs_a)>2)
        {
            if(right_angle < abs_a)
            {
                SendData("m",1); //��ת�Ƕ�����
                right_angle+=5; //5�����д�ģ�������ֵ����
            }
            else
            {
                SendData("n",1); //��ת�Ƕȼ�С
                right_angle-=5;
            }
        }
        //SendData("h",1); //��תһ��ֱ��
    }
}


//#endif // _CarControl_h