#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
#include "MyGlobal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
typedef enum{
  PUTOUT,
  //LIGHT,
  PWM,
	PUTTOPWM,
	PWMTOPUT,
}sBackState_t;



extern volatile uint8_t I2C1_ReceviceDone;

//IIC���в�������
void I2C1_Init(void);                //��ʼ��IIC��IO��				 
void I2C2_Init(void);
#endif
















