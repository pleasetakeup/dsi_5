#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"	 
#include "stmflash.h"
#include "myiic.h"
#include "softiic.h"
#include "MyGlobal.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_iwdg.h"
#include "adc.h"
#include "led.h"
#include "GT911.h"
u8 buf[43];
u8 i,j;
#define REG_LEN  24
extern sBackState_t light;
extern  uint32_t lastVolt;
u8 ICN6211_REG_ADD[REG_LEN]={0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x34,0x36,0x86,0xB5,0x5C,0x2A,0x56,0x6B,0x69,0x10,0x11,0xB6,0x51,0x09};
u8 ICN6211_REG_DATA[REG_LEN]={0x20,0xE0,0x13,0x28,0x30,0x58,0x00,0x0D,0x03,0x20,0x80,0x28,0x28,0xA0,0xFF,0x01,0x90,0x71,0x2A,0x40,0x88,0x20,0x20,0x10};
TouchData MyTouchData;
static void InitShow(){
	for(i = 0; i<REG_LEN;i++){
			Soft_IIC_Start();
			Soft_IIC_Send_Byte(0x58);
			while(Soft_IIC_Wait_Ack()){
				Soft_IIC_Send_Byte(0x58);
			}
			Soft_IIC_Send_Byte(ICN6211_REG_ADD[i]);
			while(Soft_IIC_Wait_Ack()){
				Soft_IIC_Send_Byte(ICN6211_REG_ADD[i]);
			}
			Soft_IIC_Send_Byte(ICN6211_REG_DATA[i]);
			while(Soft_IIC_Wait_Ack()){
				Soft_IIC_Send_Byte(ICN6211_REG_DATA[i]);
			}
			Soft_IIC_Stop();
	}
}
static void InitTouchData(){
	MyTouchData.point = 0xFF;
	MyTouchData.event = 0xFF;
	//RST = 0;
	//delay_ms(10);
	//RST = 1;

	for(i = 0; i < 5; i++){
		MyTouchData.xLow[i] = 0xFF;
		MyTouchData.xHigh[j] = 0xFF;
		MyTouchData.yHigh[j] = 0xFF;
		MyTouchData.yHigh[j] = 0xFF;
	}
}
void RCC_Configuration(void){
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);     
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
}
void GPIO_Configuration(void)    
{   
	
  GPIO_InitTypeDef GPIO_InitStructure;    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
  GPIO_Init(GPIOC, &GPIO_InitStructure);   
} 
void TIM3_Configuration(void)    
{  
	
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
    
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  
    
  TIM_TimeBaseStructure.TIM_Period = 500;//72MHz  72000000/36000=2000  
  TIM_TimeBaseStructure.TIM_Prescaler = 35999;//36000-1=35999  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  
    
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE );  
  TIM_Cmd(TIM3,ENABLE);  
}
void RCC1_Configuration(void)
{
  //ErrorStatus HSEStartUpStatus;
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();												//??????????????
  /* Enable HSE */
  //RCC_HSEConfig(RCC_HSE_ON);								//?????????(8M??)
  /* Wait till HSE is ready */
  //HSEStartUpStatus = RCC_WaitForHSEStartUp();				//????????
	RCC_HSICmd(ENABLE);
  while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET){}
 // if(HSEStartUpStatus == SUCCESS)							//????????
  {
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);						//??AHB?????????1??
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);							//??APB2?????HCLK??1??
    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);							//??APB1?????HCLK??2??
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);						//????FLASH???????2??
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);	//??FLASH??????
    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);	//4MHz * 14 = 56 MHz
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);										//??PLL??
    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)		//??PLL??????????
    {
    }
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);				//
    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)				
    {
    }
  }
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | 
												 RCC_APB2Periph_GPIOB | 
												 RCC_APB2Periph_GPIOC , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
}


//IWDG_Prescaler_256,3125
//256/40*3125 = 20000ms
static void IWDG_Config(uint8_t prv, uint16_t rlv){
	IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );
	IWDG_SetPrescaler( prv );
	IWDG_SetReload( rlv );
	IWDG_ReloadCounter();
	IWDG_Enable();
}
static void IWDG_Feed(void){
	IWDG_ReloadCounter();
}

void dealBackLight(){

   switch(light){
	   case PUTTOPWM :{
		      Backlight_init();
		      light = PWM;
			 	  lastVolt = 3000;
		 }
		 break;	
		 case  PWMTOPUT :{
		       LED_Init();
		       BACKLIGHT = 0;
		       light = PUTOUT;
		 }
		 break;	
	   case  PUTOUT   :{
		      
		 
		 }
		 break;	
	   case  PWM   :{
		      readVoltage();
		 
		 
		 }
    break;  
	 }




}
 int main(void)
 {	 
	uint8_t status = 0; 
	uint8_t temp[4]={0};
	uint8_t flag = 0;
		GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC1_Configuration();
	 RCC_Configuration();
	 uart_init(115200);	
	printf("DFR0678 V1.0_20210303\r\n");
	 if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET){
		RCC_ClearFlag();	
	   printf("WDT RST%d\r\n",4);		 
	} else{
		//Backlight_init();
		light = PUTTOPWM;
		//dealBackLight();
	}
	ADC_Configuration();
	IWDG_Config(IWDG_Prescaler_256,1000);
	 	delay_init();	    	 //延时函数初始化	 
	//GPIO_Configuration();
	LED_Init();
	
	//TIM3_Configuration();
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	 //串口初始化为9600


	I2C2_Init();
	I2C1_Init();
	ADC_Configuration();
	Backlight_init();
	delay_ms(50);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能PB端口时钟
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1;// PB0和PB1端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_Pin_0);						//输出1
	GPIO_SetBits(GPIOB,GPIO_Pin_1);					//输出1
	
	Soft_IIC_Init();


	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;			//PC1端口配置
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;		//下拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);			//PC1下拉输入
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);				//下拉	
	InitTouchData();
	InitShow();

	
	BACKLIGHT =1;
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	delay_ms(10);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	delay_ms(10); 

	GT911_RD_Reg(GT_PID_REG, temp, 4);

	temp[0] = 0x02;
	GT911_WR_Reg(GT_CTRL_REG, temp, 1);

	GT911_RD_Reg(GT_CFGS_REG, temp, 1);

  if(temp[0] != 0x50){
		GT911_Send_Cfg(1);
	}

	delay_ms(10);
	temp[0] = 0x00;
	GT911_WR_Reg(GT_CTRL_REG, temp, 1);

		while(1){
		  i = 0,j=0;		
		//	readVoltage();
			dealBackLight();
			GT911_RD_Reg(0x814E, buf, 1);
			if(buf[0] == 0x00 || buf[0] == 0xFF){
			  status++;
				if(status == 1){
					delay_ms(1);
					GT911_RD_Reg(0x814E, buf, 1);
					goto test;
				}else{
				}
				if(status > 5){
					status = 5;
				}
				MyTouchData.point = buf[0];
			}else{
				status = 0;
test:
				if((buf[0] & 0x80) && ((buf[0] & 0x0f) > 0)){
					MyTouchData.point = buf[0] & 0x0f;
					if(MyTouchData.point > 0x05){
						MyTouchData.point = 0x05;
					}
					GT911_RD_Reg(0x814E, buf, 1+8*MyTouchData.point);
					for(j = 0; j < MyTouchData.point; j++){
						uint8_t id = buf[1+8*j];
						uint16_t x = (uint16_t)(buf[3+8*j] & 0xFF) << 8 | (uint16_t) buf[2+8*j];
						uint16_t y = (uint16_t)(buf[5+8*j] & 0xFF) << 8 | (uint16_t) buf[4+8*j];
						//x = (uint16_t)(x/10) * 10;
						//y = (uint16_t)(y/8) * 8;
						if(x > 800){
							x =800;
						}
						if(y > 480){
							y = 480;
						}
					//	x=800-x;
					//	y=480-y;
						MyTouchData.xLow[j] = x & 0xFF;
						MyTouchData.xHigh[j] = (x >> 8) & 0xFF;
						MyTouchData.yLow[j] = y & 0xFF;
						MyTouchData.yHigh[j] = (y >> 8) & 0xFF;
						MyTouchData.finger[j] = id;
						//printf("id =%d, x=%d, y=%d\r\n",id,x,y);
					}
				}
			}
			GT911_WR_Reg(0x814E, &flag, 1);
			delay_ms(7);
			IWDG_Feed();

	 }
	}



