#ifndef _LCD1602_H_
#define _LCD1602_H_
#include<reg52.h>
#include<intrins.h>
#define uchar unsigned char
#define uint unsigned int
  sbit RS=P2^4;
  sbit RW=P2^5;
  sbit EN=P2^6;	  
#define DATPORT P0
void write_com(uchar com);
void write_data(uchar dat);
void init_1602(void);
//void disp_ascill(uchar *dat,uchar loc,uchar len);
#endif 
