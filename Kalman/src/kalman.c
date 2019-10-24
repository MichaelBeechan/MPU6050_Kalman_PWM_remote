#include <REG52.H>	
#include <math.h>     
#include <stdio.h>   
#include <INTRINS.H>

typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned int   uint;

//******功能模块头文件*******

#include "MPU6050.H"		//MPU6050头文件
#include "LCD1602.H"

uchar dis[4];
int	Temperature,Temp_h,Temp_l;	//温度及高低位数据
//******角度参数************
    float Gyro_y;        //Y轴陀螺仪数据暂存
	float Angle_gy;      //由角速度计算的倾斜角度
	float Accel_x;	     //X轴加速度值暂存
	float Angle_ax;      //由加速度计算的倾斜角度
	float Angle;         //小车最终倾斜角度
	uchar value;		 //角度正负极性标记	


//******卡尔曼参数************
		
float code Q_angle=0.001;  
float code Q_gyro=0.003;
float code R_angle=0.5;
float code dt=0.01;	                  //dt为kalman滤波器采样时间;
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


void display(uchar x,uchar y,uchar *seg)	//x为行号,y为列号
{
	uchar i;
	uint t=0;
	switch(x)   					//确定行号
	{
		case 1: i=0x80; break;				//第一行
		case 2: i=0x90; break; 				//第二行
		case 3: i=0x88; break; 				//第三行
		case 4: i=0x98; break; 				//第四行
		default : break;
	}
	i=i+y-1; 				//确定列号
	write_com(i);
	while(seg[t]!='\0')
	{
		write_data(seg[t]);	//写入需要显示字符的显示码
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
	temp_data=temp_data%100;     //取余运算
	*++s =temp_data/10+0x30;
	temp_data=temp_data%10;      //取余运算
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
    
	lcd_printf(dis, value);			//转换数据显示
	DisplayListChar(x,y,dis,4);	//启始列，行，显示数组，显示长度
}


//*********************************************************
// 卡尔曼滤波
//*********************************************************

//Kalman滤波，20MHz的处理时间约0.77ms；

float Kalman_Filter(float Accel,float Gyro)		
{
	Angle+=(Gyro - Q_bias) * dt; //先验估计

	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	Gyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
	 return  Gyro_y;
}	 







//*********************************************************
// 倾角计算（卡尔曼融合）
//*********************************************************

void Angle_Calcu(void)	 
{
	float Gyro_y;        //Y轴陀螺仪数据暂存
	//float Angle_gy;      //由角速度计算的倾斜角度
	float Accel_x;	     //X轴加速度值暂存
	float Angle_ax;      //由加速度计算的倾斜角度
	//float Angle;         //小车最终倾斜角度
	//uchar value;		 //角度正负极性标记	


	//------加速度--------------------------

	//范围为2g时，换算关系：16384 LSB/g
	//角度较小时，x=sinx得到角度（弧度）, deg = rad*180/3.14
	//因为x>=sinx,故乘以1.3适当放大

	Accel_x  = GetData(ACCEL_XOUT_H);	  //读取X轴加速度
	Angle_ax = (Accel_x - 1100) /16384;   //去除零点偏移,计算得到角度（弧度）
	Angle_ax = Angle_ax*1.4*180/3.14;     //弧度转换为度,

	//Display10BitData(Angle_ax,2,1);

    //-------角速度-------------------------

	//范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
  
	Gyro_y = GetData(GYRO_YOUT_H);	      //静止时角速度Y轴输出为-30左右
	Gyro_y = -(Gyro_y + 30)/16.4;         //去除零点偏移，计算角速度值,负号为方向处理 
	//Angle_gy = Angle_gy + Gyro_y*0.01;  //角速度积分得到倾斜角度.	  
	
	//Display10BitData(Gyro_y,8,1);		

	//-------卡尔曼滤波融合-----------------------

	//Kalman_Filter(Angle_ax,Gyro_y);       //卡尔曼滤波计算倾角
		  
	Display10BitData(Kalman_Filter(Angle_ax,Gyro_y),2,1);
	/*//-------互补滤波-----------------------

	//补偿原理是取当前倾角和加速度获得倾角差值进行放大，然后与
    //陀螺仪角速度叠加后再积分，从而使倾角最跟踪为加速度获得的角度
	//0.5为放大倍数，可调节补偿度；0.01为系统周期10ms	
		
	Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);*/
															  
} 


void display_temp()
{ 
	Temp_h=Single_ReadI2C(TEMP_OUT_H); //读取温度
	Temp_l=Single_ReadI2C(TEMP_OUT_L); //读取温度
	Temperature=Temp_h<<8|Temp_l;     //合成温度
	Temperature = 36.53+ ((double)Temperature) / 340; // 计算出温度
	lcd_printf(dis,Temperature);     //转换数据显示
	display(1,1,dis);     //启始列，行，显示数组，显示位数
}
 

//*********************************************************
//main
//*********************************************************
void main()
{ 
    delaynms(500);
	init_1602();
	delaynms(500);	   //上电延时
	InitMPU6050();     //初始化MPU6050
	delaynms(500);   
	while(1)
	{
	   
	 Angle_Calcu(); 
	 display_temp();	 

	}
}


