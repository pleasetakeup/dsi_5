#include "softiic.h"
#include "delay.h"	 
#include "usart.h"	

//����I2C�ٶȵ���ʱ
void Soft_Delay(void)
{
	delay_us(4);
} 
//���ݴ���оƬIIC�ӿڳ�ʼ��
#define Soft_SDA_Pin    GPIO_Pin_10
#define Soft_SCL_Pin    GPIO_Pin_11
void Soft_IIC_Init(void)
{		
/*	
 	RCC->APB2ENR|=1<<3;		//��ʹ������IO PORTBʱ��    
	GPIOB->CRH&=0XFFFF00FF;	//PB10,PB11 �������
	GPIOB->CRH|=0X00003300;
 GPIOB->ODR|=0<<11;	    //PB11 �����	 	
	GPIOB->ODR|=0<<10;	    //PB10 �����	*/ 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = Soft_SDA_Pin | Soft_SCL_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,Soft_SDA_Pin|Soft_SCL_Pin);
}
//����IIC��ʼ�ź�
void Soft_IIC_Start(void)                                             
{
	Soft_SDA_OUT();     //sda�����
	Soft_IIC_SDA=1;	  	  
	Soft_IIC_SCL=1;
	Soft_Delay();
 	Soft_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	Soft_Delay();
	Soft_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void Soft_IIC_Stop(void)
{ 
	Soft_SDA_OUT();//sda�����
	Soft_IIC_SCL=0;
	Soft_IIC_SDA=0;
	Soft_Delay();
	Soft_IIC_SCL=1;
	Soft_Delay();
	Soft_IIC_SDA=1;//STOP:when CLK is high DATA change form low to high 
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 Soft_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	//printf("%d\r\n",89);
	
	Soft_SDA_IN();      //SDA����Ϊ����  
	Soft_IIC_SDA=1;delay_us(1);	   
	Soft_IIC_SCL=1;delay_us(1);	 
	while(Soft_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>100)
		{
			Soft_IIC_Stop();
			return 1;
		} 
	}
	Soft_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void Soft_IIC_Ack(void)
{
	Soft_IIC_SCL=0;
	Soft_SDA_OUT();
	Soft_IIC_SDA=0;
	Soft_Delay();
	Soft_IIC_SCL=1;
	Soft_Delay();
	Soft_IIC_SCL=0;
}
//������ACKӦ��		    
void Soft_IIC_NAck(void)
{
	Soft_IIC_SCL=0;
	Soft_SDA_OUT();
	Soft_IIC_SDA=1;
	Soft_Delay();
	Soft_IIC_SCL=1;
	Soft_Delay();
	Soft_IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void Soft_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	Soft_SDA_OUT(); 	    
    Soft_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        Soft_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	      
		Soft_IIC_SCL=1;
		Soft_Delay();
		Soft_IIC_SCL=0;	
		Soft_Delay();
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 Soft_IIC_Read_Byte(unsigned char ack)
{
	u8 i,receive=0;
 	Soft_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        Soft_IIC_SCL=0; 	    	   
		delay_us(10);
		Soft_IIC_SCL=1;  
		receive<<=1;
		if(Soft_READ_SDA)receive++;   
	}	  				 
	if (!ack)Soft_IIC_NAck();//����nACK
	else Soft_IIC_Ack(); //����ACK   
 	return receive;
}




























