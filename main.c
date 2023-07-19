/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ------------------------*/
/* --- Web: www.STCMCU.com --------------------------------------------*/
/* --- Web: www.STCMCUDATA.com  ---------------------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序            */
/*---------------------------------------------------------------------*/

#include	"config.h"
#include	"GPIO.h"
#include	"UART.h"
#include	"Exti.h"
#include	"delay.h"
#include	"timer.h"
#include	"adc.h"

/*************	功能说明	**************

本例程基于STC8H8K64U为主控芯片的实验箱8进行编写测试，STC8G、STC8H系列芯片可通用参考.

演示INT0~INT1 2个唤醒源将MCU从休眠唤醒.

从串口输出唤醒源跟唤醒次数，115200,N,8,1.

ADC说明:
    ADC的第15通道是用来测试内部BandGap参考电压的,由于内部BandGap参考电
压很稳定,不会随芯片的工作电压的改变而变化,所以可以通过测量内部BandGap
参考电压,然后通过ADC的值便可反推出VCC的电压,从而用户可以实现自己的低
压检测功能.
    ADC的第15通道的测量方法:通过ADC的第15通道读取当前内部BandGap参考电压值.
    用户实现自己的低压检测功能的实现方法:首先用户需要在VCC很精准的情况下
(比如5.0V),测量出内部BandGap参考电压的ADC转换值(比如为BGV5),并这个值保存
到EEPROM中,然后在低压检测的代码中,在实际VCC变化后,所测量出的内部BandGap
参考电压的ADC转换值(比如为BGVx),通过计算公式: 实际VCC = 5.0V * BGV5 / BGVx,
即可计算出实际的VCC电压值,需要注意的是,第一步的BGV5的基准测量一定要精确.

标定内部基准Vref, 提供一个稳定的工作电压Vcc, 读内部基准Nref. 
计算内部基准(10位AD) Vref = Nref * Vcc / 1024.
计算内部基准(12位AD) Vref = Nref * Vcc / 4096.

测量某个电压, 读ADC值Nx, 则电压 Ux = Vref * Nx / Nref. 一般Vref = 1190mV.

下载时, 选择时钟 22.1184MHz (用户可在"config.h"修改频率).

******************************************/

/*************	本地预定义	**************/
#define LED1_ALARM  P12
#define LED1_SIGNAL P14
#define LED_LOW_BATT P16
#define LED2_ALARM P35
#define LED2_SIGNAL P36
/*************	本地常量声明	**************/


/*************	本地变量声明	**************/
u8 WakeUpSource;
u8 WakeUpCnt;
u16	Vdc,Nref;
u8 Flag_LED1_ALARM_step,Flag_LED2_ALARM_step,Flag_power_Low;
/*************	本地函数声明	**************/



/*************  外部函数和变量声明 *****************/



/******************** IO口配置 ********************/
void	GPIO_config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;				//结构定义

	GPIO_InitStructure.Pin  = GPIO_Pin_All;			//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7, 或操作
	GPIO_InitStructure.Mode = GPIO_PullUp;			//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	//初始化
	
	GPIO_InitStructure.Pin  = GPIO_Pin_All;			//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7, 或操作
	GPIO_InitStructure.Mode = GPIO_PullUp;			//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	//初始化	
}

/************************ 定时器配置 ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;						//结构定义
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_Priority    = Priority_0;			//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	TIM_InitStructure.TIM_Interrupt = ENABLE;					//中断是否允许,   ENABLE或DISABLE
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / 100UL);		//初值,  1000Hz=1000；100Hz=100
	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);					//初始化Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
}

/******************* AD配置函数 *******************/
void	ADC_config(void)
{
	ADC_InitTypeDef		ADC_InitStructure;		//结构定义
	ADC_InitStructure.ADC_SMPduty   = 31;		//ADC 模拟信号采样时间控制, 0~31（注意： SMPDUTY 一定不能设置小于 10）
	ADC_InitStructure.ADC_CsSetup   = 0;		//ADC 通道选择时间控制 0(默认),1
	ADC_InitStructure.ADC_CsHold    = 1;		//ADC 通道选择保持时间控制 0,1(默认),2,3
	ADC_InitStructure.ADC_Speed     = ADC_SPEED_2X1T;		//设置 ADC 工作时钟频率	ADC_SPEED_2X1T~ADC_SPEED_2X16T
	ADC_InitStructure.ADC_Power     = ENABLE;				//ADC功率允许/关闭	ENABLE,DISABLE
	ADC_InitStructure.ADC_AdjResult = ADC_RIGHT_JUSTIFIED;	//ADC结果调整,	ADC_LEFT_JUSTIFIED,ADC_RIGHT_JUSTIFIED
	ADC_InitStructure.ADC_Priority    = Priority_0;			//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	ADC_InitStructure.ADC_Interrupt = DISABLE;			//中断允许	ENABLE,DISABLE
	ADC_Inilize(&ADC_InitStructure);					//初始化
	ADC_PowerControl(ENABLE);							//单独的ADC电源操作函数, ENABLE或DISABLE
}

/******************** INT配置 ********************/
void	Exti_config(void)
{
	EXTI_InitTypeDef	Exti_InitStructure;							//结构定义

	Exti_InitStructure.EXTI_Interrupt = ENABLE;				//中断使能,   ENABLE或DISABLE
	Exti_InitStructure.EXTI_Mode      = EXT_MODE_Fall;//中断模式,   EXT_MODE_RiseFall,EXT_MODE_Fall
	Exti_InitStructure.EXTI_Priority  = Priority_0;		//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	Ext_Inilize(EXT_INT0,&Exti_InitStructure);				//初始化

	Exti_InitStructure.EXTI_Interrupt = ENABLE;				//中断使能,   ENABLE或DISABLE
	Exti_InitStructure.EXTI_Mode      = EXT_MODE_Fall;//中断模式,   EXT_MODE_RiseFall,EXT_MODE_Fall
	Exti_InitStructure.EXTI_Priority  = Priority_0;		//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	Ext_Inilize(EXT_INT1,&Exti_InitStructure);				//初始化
}

/****************  串口初始化函数 *****************/
void	UART_config(void)
{
	COMx_InitDefine		COMx_InitStructure;					//结构定义
	COMx_InitStructure.UART_Mode      = UART_8bit_BRTx;	//模式, UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
	COMx_InitStructure.UART_BRT_Use   = BRT_Timer2;			//使用波特率, BRT_Timer1, BRT_Timer2 (注意: 串口2固定使用BRT_Timer2)
	COMx_InitStructure.UART_BaudRate  = 115200ul;			//波特率, 一般 110 ~ 115200
	COMx_InitStructure.UART_RxEnable  = ENABLE;				//接收允许,   ENABLE或DISABLE
	COMx_InitStructure.BaudRateDouble = DISABLE;			//波特率加倍, ENABLE或DISABLE
	COMx_InitStructure.UART_Interrupt = ENABLE;				//中断允许,   ENABLE或DISABLE
	COMx_InitStructure.UART_Priority  = Priority_0;			//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	COMx_InitStructure.UART_P_SW      = UART1_SW_P30_P31;	//切换端口,   UART1_SW_P30_P31,UART1_SW_P36_P37,UART1_SW_P16_P17,UART1_SW_P43_P44
	UART_Configuration(UART1, &COMx_InitStructure);		//初始化串口1 UART1,UART2,UART3,UART4

	PrintString1("STC8H8K64U EXINT Wakeup Test Programme!\r\n");	//UART1发送一个字符串
}

/****************  ALL LED熄灭 *****************/
void	ALL_LED_off(void)
{
	LED1_ALARM=0;   //低电平LED熄灭
	LED2_ALARM=0;   //低电平LED熄灭
	LED1_SIGNAL=1;  //高电平LED熄灭
	LED2_SIGNAL=1;  //高电平LED熄灭
	LED_LOW_BATT=1;	//高电平LED熄灭
	Flag_LED1_ALARM_step=FALSE;
	Flag_LED2_ALARM_step=FALSE;
	Timer_LED1_ALARM=0;
	Timer_LED2_ALARM=0;
}

/****************  ALL LED熄灭 *****************/
void	ADC_Vpower(void)
{
	Nref = Get_ADCResult(15);	//读内部基准电压, 读15通道
	Vdc = (u32)1190 * (u32)1024 / Nref;			//Ux = Vref * Nx / Nref. Vref = 1190mV
	if(Vdc>3200)
	{
		//LED_LOW_BATT=1;
		Flag_power_Low=FALSE;
	}
	else 
	{
		//LED_LOW_BATT=0;
		Flag_power_Low=TRUE;
	}
	TX1_write2buff(Vdc/1000 + '0');
	TX1_write2buff('.');
	TX1_write2buff(Vdc%1000/100 + '0');
	TX1_write2buff(Vdc%100/10 + '0');
	TX1_write2buff(Vdc%10 + '0');
	PrintString1("Vdc\r\n");
}

/******************** 主函数***********************/
void main(void)
{
	GPIO_config();
	ALL_LED_off();	
	UART_config();
	Timer_config();
	ADC_config();
//	Exti_config();
	EA  = 1;		//Enable all interrupt
	LED2_SIGNAL=0;  //开机附板信号LED亮1秒
	delay_ms(250);
	delay_ms(250);
	delay_ms(250);
	delay_ms(250);
	LED2_SIGNAL=1;
    ADC_Vpower();		
	while(1)
	{
		if(Flag_power_Low==FALSE)
		{
			if(!INT0)
			{
				switch(Flag_LED2_ALARM_step)
				{
					case 0:
							PrintString1("slave step0 ");
							Timer_LED2_ALARM=200;
							Flag_LED2_ALARM_step=3;  //=3或=1
							break;
					case 1:
							if(Timer_LED2_ALARM==0){PrintString1("slave step1 ");Timer_LED2_ALARM=100;Flag_LED2_ALARM_step=2;}
							break;
					case 2:
							if(Timer_LED2_ALARM==0){PrintString1("slave step2 ");Timer_LED2_ALARM=100;Flag_LED2_ALARM_step=3;}
					case 3:
							if(Timer_LED2_ALARM==0){PrintString1("slave step3 ");Timer_LED2_ALARM=100;}							
							break;							
					default:
							break;					
				}
			}
			else if((Flag_LED2_ALARM_step==2)||(Flag_LED2_ALARM_step==1)&&(Timer_LED2_ALARM<50))
				Timer_LED2_ALARM=0;

			if(!INT1)
			{
				switch(Flag_LED1_ALARM_step)
				{
					case 0:
							PrintString1("master step0 ");
							Timer_LED1_ALARM=200;
							Flag_LED1_ALARM_step=3;  //=3或=1
							break;
					case 1:
							if(Timer_LED1_ALARM==0){PrintString1("master step1 ");Timer_LED1_ALARM=100;Flag_LED1_ALARM_step=2;}
							break;
					case 2:
							if(Timer_LED1_ALARM==0){PrintString1("master step2 ");Timer_LED1_ALARM=100;Flag_LED1_ALARM_step=3;}
					case 3:
							if(Timer_LED1_ALARM==0){PrintString1("master step3 ");Timer_LED1_ALARM=100;}							
							break;							
					default:
							break;					
				}
			}
			else if((Flag_LED1_ALARM_step==2)||(Flag_LED1_ALARM_step==1)&&(Timer_LED1_ALARM<50))
				Timer_LED1_ALARM=0;			

			if(Flag_LED1_ALARM_step!=2)
			{
				if(Timer_LED1_ALARM>150){LED1_ALARM=1;LED2_SIGNAL=0;}
				else if(Timer_LED1_ALARM>100){LED1_ALARM=0;LED2_SIGNAL=0;}
				else if(Timer_LED1_ALARM>50){LED1_ALARM=1;LED2_SIGNAL=0;}
				else {LED1_ALARM=0;if(INT1)LED2_SIGNAL=1;}
			}
			else {LED1_ALARM=0;LED2_SIGNAL=1;}	
			if(Flag_LED2_ALARM_step!=2)
			{
				if(Timer_LED2_ALARM>150){LED2_ALARM=1;LED1_SIGNAL=0;}
				else if(Timer_LED2_ALARM>100){LED2_ALARM=0;LED1_SIGNAL=0;}
				else if(Timer_LED2_ALARM>50){LED2_ALARM=1;LED1_SIGNAL=0;}
				else {LED2_ALARM=0;if(INT0)LED1_SIGNAL=1;}
			}
			else {LED2_ALARM=0;LED1_SIGNAL=1;}
		}
		else 
		{
			if(Timer_ADC==0)
			{
				Timer_ADC=100;
				ADC_Vpower();
			}
			if(Timer_LOW_BATT==0)
			{
				Timer_LOW_BATT=50;
				LED1_SIGNAL=~LED1_SIGNAL;
			}
		}
		
		if((INT0==1)&&(INT1==1)&&(Flag_power_Low==FALSE)&&(Timer_LED2_ALARM==0)&&(Timer_LED1_ALARM==0)) //等待外中断为高电平
		{
			delay_ms(10);	//delay 10ms
			if((INT0==1)&&(INT1==1)) //等待外中断为高电平
			{
				PrintString1("SLEEP  ");
				delay_ms(10);	//delay 10ms
				Exti_config();
				WakeUpSource = 0;
			
				ALL_LED_off();
				MCU_POWER_DOWN();	//Sleep
				_nop_();_nop_();_nop_();
				delay_ms(1);	//delay 1ms
				
				PrintString1("外中断INT");
				TX1_write2buff(WakeUpSource%10+'0');
				PrintString1("  ");
				
				WakeUpCnt++;
				TX1_write2buff(WakeUpCnt/100+'0');
				TX1_write2buff(WakeUpCnt%100/10+'0');
				TX1_write2buff(WakeUpCnt%10+'0');
				PrintString1("次唤醒\r\n");

				ADC_Vpower();
			}
		}

	}

}

