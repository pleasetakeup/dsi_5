#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define LED1 PBout(15)// PB15
#define LED2 PBout(14)// PB14	
#define LED3 PBout(13)// PB13
#define LED4 PBout(12)// PB12
#define KV1 PAout(1)  // PA1
#define BACKLIGHT PAout(3) //PA3
#define RST PBout(0) //PA3
#define INT PBout(1) //PA3
void LED_Init(void);//��ʼ��

		 				    
#endif
