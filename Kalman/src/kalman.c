#include <REG52.H>	
#include <math.h>     
#include <stdio.h>   
#include <INTRINS.H>

typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned int   uint;

//******����ģ��ͷ�ļ�*******

#include "MPU6050.H"		//MPU6050ͷ�ļ�
#include "LCD1602.H"

uchar dis[4];
int	Temperature,Temp_h,Temp_l;	//�¶ȼ��ߵ�λ����
//******�ǶȲ���************
    float Gyro_y;        //Y�������������ݴ�
	float Angle_gy;      //�ɽ��ٶȼ������б�Ƕ�
	float Accel_x;	     //X����ٶ�ֵ�ݴ�
	float Angle_ax;      //�ɼ��ٶȼ������б�Ƕ�
	float Angle;         //С��������б�Ƕ�
	uchar value;		 //�Ƕ��������Ա��	


//******����������************
		
float code Q_angle=0.001;  
float code Q_gyro=0.003;
float code R_angle=0.5;
float code dt=0.01;	                  //dtΪkalman�˲�������ʱ��;
char  code C_0 = 1;
float xdata Q_bias, Angle_err;
float xdata PCt_0, PCt_1, E;
float xdata K_0, K_1, t_0, t_1;
float xdata Pdot[4] ={0,0,0,0};
float xdata PP[2][2] = { { 1, 0 },{ 0, 1 } };	

 void delay1ms()
 {
   uchar i,j;	
   for(i=0;i<10;i++)
   for(j=0;j<33;j++)
	   ;		 
  }
void delaynms(uchar n)
 {
   uchar i;
   for(i=0;i<n;i++) delay1ms();

 }


void display(uchar x,uchar y,uchar *seg)	//xΪ�к�,yΪ�к�
{
	uchar i;
	uint t=0;
	switch(x)   					//ȷ���к�
	{
		case 1: i=0x80; break;				//��һ��
		case 2: i=0x90; break; 				//�ڶ���
		case 3: i=0x88; break; 				//������
		case 4: i=0x98; break; 				//������
		default : break;
	}
	i=i+y-1; 				//ȷ���к�
	write_com(i);
	while(seg[t]!='\0')
	{
		write_data(seg[t]);	//д����Ҫ��ʾ�ַ�����ʾ��
		t++;
	}
}


void lcd_printf(uchar *s,int temp_data)
{
	if(temp_data<0)
	{
		temp_data=-temp_data;
		*s='-';
	}
	else *s=' ';
	*++s =temp_data/100+0x30;
	temp_data=temp_data%100;     //ȡ������
	*++s =temp_data/10+0x30;
	temp_data=temp_data%10;      //ȡ������
	*++s =temp_data+0x30; 	
}

void DisplayOneChar(uchar X,uchar Y,uchar DData)
{						
	Y&=1;						
	X&=15;						
	if(Y)X|=0x40;					
	X|=0x80;			
	write_com(X);		
	write_data(DData);		
}

void DisplayListChar(uchar X,uchar Y,uchar *DData,L)
{
	uchar ListLength=0; 
	Y&=0x1;                
	X&=0xF;                
	while(L--)             
	{                       
		DisplayOneChar(X,Y,DData[ListLength]);
		ListLength++;  
		X++;                        
	}    
}

void Display10BitData(int value,uchar x,uchar y)
{
    
	lcd_printf(dis, value);			//ת��������ʾ
	DisplayListChar(x,y,dis,4);	//��ʼ�У��У���ʾ���飬��ʾ����
}


//*********************************************************
// �������˲�
//*********************************************************

//Kalman�˲���20MHz�Ĵ���ʱ��Լ0.77ms��

float Kalman_Filter(float Accel,float Gyro)		
{
	Angle+=(Gyro - Q_bias) * dt; //�������

	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	Gyro_y   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
	 return  Gyro_y;
}	 







//*********************************************************
// ��Ǽ��㣨�������ںϣ�
//*********************************************************

void Angle_Calcu(void)	 
{
	float Gyro_y;        //Y�������������ݴ�
	//float Angle_gy;      //�ɽ��ٶȼ������б�Ƕ�
	float Accel_x;	     //X����ٶ�ֵ�ݴ�
	float Angle_ax;      //�ɼ��ٶȼ������б�Ƕ�
	//float Angle;         //С��������б�Ƕ�
	//uchar value;		 //�Ƕ��������Ա��	


	//------���ٶ�--------------------------

	//��ΧΪ2gʱ�������ϵ��16384 LSB/g
	//�ǶȽ�Сʱ��x=sinx�õ��Ƕȣ����ȣ�, deg = rad*180/3.14
	//��Ϊx>=sinx,�ʳ���1.3�ʵ��Ŵ�

	Accel_x  = GetData(ACCEL_XOUT_H);	  //��ȡX����ٶ�
	Angle_ax = (Accel_x - 1100) /16384;   //ȥ�����ƫ��,����õ��Ƕȣ����ȣ�
	Angle_ax = Angle_ax*1.4*180/3.14;     //����ת��Ϊ��,

	//Display10BitData(Angle_ax,2,1);

    //-------���ٶ�-------------------------

	//��ΧΪ2000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
  
	Gyro_y = GetData(GYRO_YOUT_H);	      //��ֹʱ���ٶ�Y�����Ϊ-30����
	Gyro_y = -(Gyro_y + 30)/16.4;         //ȥ�����ƫ�ƣ�������ٶ�ֵ,����Ϊ������ 
	//Angle_gy = Angle_gy + Gyro_y*0.01;  //���ٶȻ��ֵõ���б�Ƕ�.	  
	
	//Display10BitData(Gyro_y,8,1);		

	//-------�������˲��ں�-----------------------

	//Kalman_Filter(Angle_ax,Gyro_y);       //�������˲��������
		  
	Display10BitData(Kalman_Filter(Angle_ax,Gyro_y),2,1);
	/*//-------�����˲�-----------------------

	//����ԭ����ȡ��ǰ��Ǻͼ��ٶȻ����ǲ�ֵ���зŴ�Ȼ����
    //�����ǽ��ٶȵ��Ӻ��ٻ��֣��Ӷ�ʹ��������Ϊ���ٶȻ�õĽǶ�
	//0.5Ϊ�Ŵ������ɵ��ڲ����ȣ�0.01Ϊϵͳ����10ms	
		
	Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);*/
															  
} 


void display_temp()
{ 
	Temp_h=Single_ReadI2C(TEMP_OUT_H); //��ȡ�¶�
	Temp_l=Single_ReadI2C(TEMP_OUT_L); //��ȡ�¶�
	Temperature=Temp_h<<8|Temp_l;     //�ϳ��¶�
	Temperature = 36.53+ ((double)Temperature) / 340; // ������¶�
	lcd_printf(dis,Temperature);     //ת��������ʾ
	display(1,1,dis);     //��ʼ�У��У���ʾ���飬��ʾλ��
}
 

//*********************************************************
//main
//*********************************************************
void main()
{ 
    delaynms(500);
	init_1602();
	delaynms(500);	   //�ϵ���ʱ
	InitMPU6050();     //��ʼ��MPU6050
	delaynms(500);   
	while(1)
	{
	   
	 Angle_Calcu(); 
	 display_temp();	 

	}
}


