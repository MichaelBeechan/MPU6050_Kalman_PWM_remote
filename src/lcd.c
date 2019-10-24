#include "LCD1602.h"

void delay_50us(uint t)     //ºÁÃë¼¶ÑÓÊ±º¯Êý
{
	unsigned int j;
	for(;t>0;t--)
	for(j=19;j>0;j--);
}
void write_com(uchar com)
{
	EN=0;
	RS=0;
	RW=0;
	P0=com;
	delay_50us(10);
	EN=1;
	delay_50us(20);
	EN=0;
}

void write_data(uchar dat)
{
	EN=0;
	RS=1;
	RW=0;
	P0=dat;
	delay_50us(10);
	EN=0x01;
	delay_50us(20);
	EN=0;
}

void init_1602(void)
{
	delay_50us(300);
	write_com(0x38);
	delay_50us(100);
	write_com(0x38);
	delay_50us(100);
	write_com(0x38);
	
	write_com(0x08);
	write_com(0x01);
	write_com(0x06);
	write_com(0x0c);
}
 /*void disp_ascill(uchar *dat,uchar loc,uchar len)
 {
     uchar ii;
	 write_com(loc);
	 for(ii=0;ii<len;ii++)
	 {
	     write_data(dat[ii]);
	 }
 } */

