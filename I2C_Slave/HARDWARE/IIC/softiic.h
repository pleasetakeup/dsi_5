#ifndef __SOFT_IIC_H
#define __SOFT_IIC_H
#include "sys.h"	    




  	   		   
//IO��������
#define Soft_SDA_IN()  {GPIOA->CRH&=0XFFFFF0FF;GPIOA->CRH|=8<<4*2;}
#define Soft_SDA_OUT() {GPIOA->CRH&=0XFFFFF0FF;GPIOA->CRH|=3<<4*2;}

//IO��������	 
#define Soft_IIC_SCL    PAout(11) 			//SCL     
#define Soft_IIC_SDA    PAout(10) 			//SDA	 
#define Soft_READ_SDA   PAin(10)  			//����SDA 

//IIC���в�������
void Soft_IIC_Init(void);                	//��ʼ��IIC��IO��				 
void Soft_IIC_Start(void);				//����IIC��ʼ�ź�
void Soft_IIC_Stop(void);	  				//����IICֹͣ�ź�
void Soft_IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 Soft_IIC_Read_Byte(unsigned char ack);	//IIC��ȡһ���ֽ�
u8 Soft_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void Soft_IIC_Ack(void);					//IIC����ACK�ź�
void Soft_IIC_NAck(void);					//IIC������ACK�ź�

#endif







