#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"


//#define KEY0 PEin(4)   	//PE4
//#define KEY1 PEin(3)	//PE3 
//#define KEY2 PEin(2)	//PE2
//#define KEY3 PAin(0)	//PA0  WK_UP

#define KEY1 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)//��ȡ����0
#define KEY2 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)//��ȡ����1
#define SW1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)//��ȡ����2 
#define SW2  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)//��ȡ����3(WK_UP) 



void KEY_Init(void);//IO��ʼ��
u8 KEY_Scan(void);  	//����ɨ�躯��	

#endif
