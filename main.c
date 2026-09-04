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
#include	"WDT.h"
#include	"PWM.h"

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
//#define	DEF_ADC_log
//#define	DEF_PWMout_log
/*************	本地预定义	**************/
#define LED_ALARM  P34
#define LED_SIGNAL P36
#define BEEP_ALARM P17
#define POWER_CE   P13
/*************	本地常量声明	**************/


/*************	本地变量声明	**************/
PWMx_Duty PWMA_Duty;
u8 WakeUpSource;
u16 WakeUpCnt;
u16	Vdc,Vadc_led;
u8 Flag_LED1_ALARM_step,Flag_LED2_ALARM_step,Flag_power_Low;
u8 Flag_PWM_LED_ENABLE=0,Flag_PWM_BEEP_ENABLE=0;
u8 Flag_PWM_LED_Outputting=0,Flag_PWM_BEEP_Outputting=0;
/*************	本地函数声明	**************/



/*************  外部函数和变量声明 *****************/


void	WDT_config(void)
{
	WDT_InitTypeDef	WDT_InitStructure;					//结构定义

	WDT_InitStructure.WDT_Enable     = ENABLE;					//中断使能   ENABLE或DISABLE
	WDT_InitStructure.WDT_IDLE_Mode  = WDT_IDLE_STOP;		//IDLE模式是否停止计数		WDT_IDLE_STOP,WDT_IDLE_RUN
	WDT_InitStructure.WDT_PS         = WDT_SCALE_16;		//看门狗定时器时钟分频系数		WDT_SCALE_2,WDT_SCALE_4,WDT_SCALE_8,WDT_SCALE_16,WDT_SCALE_32,WDT_SCALE_64,WDT_SCALE_128,WDT_SCALE_256
	WDT_Inilize(&WDT_InitStructure);					//初始化
}
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
	
	GPIO_InitStructure.Pin  = GPIO_Pin_All;			//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7, 或操作
	GPIO_InitStructure.Mode = GPIO_PullUp;			//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P5,&GPIO_InitStructure);	//初始化
	
	//AD口设置为输入口
	GPIO_InitStructure.Pin  = GPIO_Pin_1;		//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7
	GPIO_InitStructure.Mode = GPIO_HighZ;	//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
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

	// Exti_InitStructure.EXTI_Interrupt = ENABLE;				//中断使能,   ENABLE或DISABLE
	// Exti_InitStructure.EXTI_Mode      = EXT_MODE_Fall;//中断模式,   EXT_MODE_RiseFall,EXT_MODE_Fall
	// Exti_InitStructure.EXTI_Priority  = Priority_0;		//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	// Ext_Inilize(EXT_INT1,&Exti_InitStructure);				//初始化
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

	PrintString1("STC8H8K64U EXINT Wakeup KingJim!\r\n");	//UART1发送一个字符串
}

void	PWM_LED_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;
	
	PWMx_InitStructure.PWM4_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2

	PWMx_InitStructure.PWM4_SetPriority  = Priority_0;			//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	
	PWMx_InitStructure.PWM_Period   = MAIN_Fosc/2000;			//周期时间,   2KHz=2000,1KHz=1000,
	PWMx_InitStructure.PWM4_Duty    = PWMx_InitStructure.PWM_Period/2;	  //PWM4占空比时间, 0~Period
	PWMx_InitStructure.PWM_DeadTime = 0;								//死区发生器设置, 0~255
	
	PWMx_InitStructure.PWM_EnoSelect   = ENO4P;	//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWMx_InitStructure.PWM_PS_SW       = PWM4_SW_P34_P33;	//切换端口,		PWM1_SW_P10_P11,PWM1_SW_P20_P21,PWM1_SW_P60_P61
															//			   PWM2_SW_P12_P13,PWM2_SW_P22_P23,PWM2_SW_P62_P63
															//			   PWM3_SW_P14_P15,PWM3_SW_P24_P25,PWM3_SW_P64_P65
															//			   PWM4_SW_P16_P17,PWM4_SW_P26_P27,PWM4_SW_P66_P67,PWM4_SW_P34_P33

	PWMx_InitStructure.PWM_CC4Enable   = ENABLE;				//开启PWM4P输入捕获/比较输出,  ENABLE,DISABLE
	//PWMx_InitStructure.PWM_CC4NEnable  = ENABLE;				//开启PWM4N输入捕获/比较输出,  ENABLE,DISABLE
	
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;				//主输出使能, ENABLE,DISABLE
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;				//使能计数器, ENABLE,DISABLE
	PWM_Configuration(PWMA, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB
	
	PWMA_Duty.PWM4_Duty=PWMx_InitStructure.PWM_Period/2;
	UpdatePwm(PWMA, &PWMA_Duty);
}
 void PWM_LED_OFF(void)
 {
	PWMx_InitDefine		PWMx_InitStructure;
	
	PWMx_InitStructure.PWM4_Mode    =	CCMRn_FREEZE;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2

	PWMx_InitStructure.PWM4_SetPriority  = Priority_0;			//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	
	PWMx_InitStructure.PWM_Period   = 0;							//周期时间,   0~65535
	PWMx_InitStructure.PWM4_Duty    = 0;	//PWM4占空比时间, 0~Period
	PWMx_InitStructure.PWM_DeadTime = 0;								//死区发生器设置, 0~255
	
	PWMx_InitStructure.PWM_EnoSelect   = 0;	//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWMx_InitStructure.PWM_PS_SW       = 0;	//切换端口,		PWM1_SW_P10_P11,PWM1_SW_P20_P21,PWM1_SW_P60_P61
															//			   PWM2_SW_P12_P13,PWM2_SW_P22_P23,PWM2_SW_P62_P63
															//			   PWM3_SW_P14_P15,PWM3_SW_P24_P25,PWM3_SW_P64_P65
															//			   PWM4_SW_P16_P17,PWM4_SW_P26_P27,PWM4_SW_P66_P67,PWM4_SW_P34_P33

	PWMx_InitStructure.PWM_CC4Enable   = DISABLE;				//开启PWM4P输入捕获/比较输出,  ENABLE,DISABLE
	//PWMx_InitStructure.PWM_CC4NEnable  = ENABLE;				//开启PWM4N输入捕获/比较输出,  ENABLE,DISABLE
	
	PWMx_InitStructure.PWM_MainOutEnable= DISABLE;				//主输出使能, ENABLE,DISABLE
	PWMx_InitStructure.PWM_CEN_Enable   = DISABLE;				//使能计数器, ENABLE,DISABLE
	PWM_Configuration(PWMA, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB
 }
 void	PWM_BEEP_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;
	
	PWMx_InitStructure.PWM5_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2

	PWMx_InitStructure.PWM5_SetPriority  = Priority_0;			//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	
	PWMx_InitStructure.PWM_Period   = MAIN_Fosc/2000;			//周期时间,   2KHz=2000,1KHz=1000,
	PWMx_InitStructure.PWM5_Duty    = PWMx_InitStructure.PWM_Period/2;	  //PWM5占空比时间, 0~Period
	PWMx_InitStructure.PWM_DeadTime = 0;								//死区发生器设置, 0~255
	
	PWMx_InitStructure.PWM_EnoSelect   = ENO5P;	//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWMx_InitStructure.PWM_PS_SW       = PWM5_SW_P17;	//切换端口,		PWM1_SW_P10_P11,PWM1_SW_P20_P21,PWM1_SW_P60_P61
															//			   PWM2_SW_P12_P13,PWM2_SW_P22_P23,PWM2_SW_P62_P63
															//			   PWM3_SW_P14_P15,PWM3_SW_P24_P25,PWM3_SW_P64_P65
															//			   PWM4_SW_P16_P17,PWM4_SW_P26_P27,PWM4_SW_P66_P67,PWM4_SW_P34_P33

	PWMx_InitStructure.PWM_CC5Enable   = ENABLE;				//开启PWM5P输入捕获/比较输出,  ENABLE,DISABLE
	
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;				//主输出使能, ENABLE,DISABLE
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;				//使能计数器, ENABLE,DISABLE
	PWM_Configuration(PWMB, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB
	
	PWMA_Duty.PWM5_Duty=PWMx_InitStructure.PWM_Period/2;
	UpdatePwm(PWMB, &PWMA_Duty); 
}
 void PWM_BEEP_OFF(void)
 {
	PWMx_InitDefine		PWMx_InitStructure;
	
	PWMx_InitStructure.PWM5_Mode    =	CCMRn_FREEZE;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2

	PWMx_InitStructure.PWM5_SetPriority  = Priority_0;			//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	
	PWMx_InitStructure.PWM_Period   = 0;							//周期时间,   0~65535
	PWMx_InitStructure.PWM5_Duty    = 0;	//PWM5占空比时间, 0~Period
	PWMx_InitStructure.PWM_DeadTime = 0;								//死区发生器设置, 0~255
	
	PWMx_InitStructure.PWM_EnoSelect   = 0;	//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWMx_InitStructure.PWM_PS_SW       = 0;	//切换端口,		PWM1_SW_P10_P11,PWM1_SW_P20_P21,PWM1_SW_P60_P61
															//			   PWM2_SW_P12_P13,PWM2_SW_P22_P23,PWM2_SW_P62_P63
															//			   PWM3_SW_P14_P15,PWM3_SW_P24_P25,PWM3_SW_P64_P65
															//			   PWM4_SW_P16_P17,PWM4_SW_P26_P27,PWM4_SW_P66_P67,PWM4_SW_P34_P33

	PWMx_InitStructure.PWM_CC5Enable   = DISABLE;				//开启PWM5P输入捕获/比较输出,  ENABLE,DISABLE
	
	PWMx_InitStructure.PWM_MainOutEnable= DISABLE;				//主输出使能, ENABLE,DISABLE
	PWMx_InitStructure.PWM_CEN_Enable   = DISABLE;				//使能计数器, ENABLE,DISABLE
	PWM_Configuration(PWMB, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB
 }
/****************  ALL LED熄灭 *****************/
void	ALL_LED_off(void)
{
	LED_ALARM=0;   //低电平LED熄灭
	LED_SIGNAL=1;  //高电平LED熄灭
	BEEP_ALARM=0;   //低电平蜂鸣器熄灭
	POWER_CE=0;  //低电平关闭电源
	Flag_LED1_ALARM_step=FALSE;
	Flag_LED2_ALARM_step=FALSE;
	Timer_LED1_ALARM=0;
	Timer_LED2_ALARM=0;
}
/****************  ALL LED熄灭 *****************/

void    PrintNumber(u16 number)
{
	u8 i;
	u8 str[6];
	str[5] = '\0';
	for(i=5;i>0;i--)
	{
		str[i-1]=number%10+'0';
		number=number/10;
	}

	/* 跳过前导的 '0' */
	for(i = 0; i < 4; i++)
	{
		if(str[i] != '0') break;
	}

	/* 打印从第一个非零字符开始的字符串 */
	PrintString1((char *)&str[i]);
}
void	ADC_Vpower(void)
{
	u8 i;
	u16	Nref,Sum_Nx,Nadc_led;

	Sum_Nx=0;
	for(i=0;i<5;i++)
	{
		Sum_Nx += Get_ADCResult(15);	//读内部基准电压, 读15通道
	}
	Nref = Sum_Nx/5;
	Vdc = (u32)1190 * (u32)1024 / Nref;			//Ux = Vref * Nx / Nref;    Vref = 1190mV
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
	#ifdef DEF_ADC_log
	PrintString1("Vdc:");
	PrintNumber(Vdc);
	PrintString1("mV\r\n");
	#endif	

	Sum_Nx=0;
	for(i=0;i<5;i++)
	{
		Sum_Nx += Get_ADCResult(1);	//读LED的温度, 读1通道
	}
	Nadc_led = Sum_Nx/5;
	Vadc_led = (u32)Vdc * (u32)Nadc_led / 1024;	//Ux = Vdc * Nx / 1024;
	#ifdef DEF_ADC_log
	PrintString1("Vadc_led:");
	PrintNumber(Vadc_led);
	PrintString1("mV\r\n");	
	#endif
}

void PWM_output(void)
{
	if(Flag_PWM_LED_ENABLE)
	{
		if(Flag_PWM_LED_Outputting==0)
		{
			Flag_PWM_LED_Outputting=1;
			PWM_LED_config();
			#ifdef DEF_PWMout_log
			PrintString1("LED ON\r\n");
			#endif
		}
	}
	else 
	{
		if(Flag_PWM_LED_Outputting==1)
		{
			Flag_PWM_LED_Outputting=0;
			PWM_LED_OFF();
			#ifdef DEF_PWMout_log
			PrintString1("LED OFF\r\n");
			#endif
			LED_ALARM=0;
		}
	}
	if(Flag_PWM_BEEP_ENABLE)
	{
		if(Flag_PWM_BEEP_Outputting==0)
		{
			Flag_PWM_BEEP_Outputting=1;
			PWM_BEEP_config();
			#ifdef DEF_PWMout_log
			PrintString1("BEEP ON\r\n");
			#endif
		}
	}
	else 
	{
		if(Flag_PWM_BEEP_Outputting==1)
		{
			Flag_PWM_BEEP_Outputting=0;
			PWM_BEEP_OFF();
			#ifdef DEF_PWMout_log
			PrintString1("BEEP OFF\r\n");
			#endif
			BEEP_ALARM=0;
		}
	}
}
/******************** 主函数***********************/
void main(void)
{
	Timer_PowerON=1200;
	GPIO_config();
	ALL_LED_off();	
	UART_config();
	Timer_config();
	ADC_config();
//	Exti_config();
	EA  = 1;		//Enable all interrupt
	POWER_CE=1;
	delay_ms(100);	
    ADC_Vpower();
	WDT_config();		
	while(1)
	{
		WDT_Clear();    // 喂狗
		if(Flag_power_Low==FALSE)
		{
			if(Timer_PowerON==0)
			{
					if(!INT0)
					{
						if(Timer_LED2_ALARM==0)PrintString1("INT0\r\n");
						switch(Flag_LED2_ALARM_step)
						{
							case 0:
									PrintString1("step0 ");
									Timer_LED2_ALARM=200;
									Flag_LED2_ALARM_step=3;  //=3或=1
									break;
							case 1:
									if(Timer_LED2_ALARM==0){PrintString1("step1 ");Timer_LED2_ALARM=100;Flag_LED2_ALARM_step=2;}
									break;
							case 2:
									if(Timer_LED2_ALARM==0){PrintString1("step2 ");Timer_LED2_ALARM=100;Flag_LED2_ALARM_step=3;}
							case 3:
									if(Timer_LED2_ALARM==0){PrintString1("step3 ");Timer_LED2_ALARM=100;}							
									break;							
							default:
									break;					
						}
					}
					else if(Timer_LED2_ALARM<5)
						Timer_LED2_ALARM=0;
	
					if(Flag_LED2_ALARM_step!=2)
					{
						if((Timer_LED2_ALARM<200)&&(Timer_LED2_ALARM>155)||(Timer_LED2_ALARM<100)&&(Timer_LED2_ALARM>55))
						{
							if(Vadc_led<1294)  //LED温度>80℃  80℃时Rt=1.669K，Rp=2.2K
								Flag_PWM_LED_ENABLE=0;
							else
							{
								Flag_PWM_LED_ENABLE=1;
								Flag_PWM_BEEP_ENABLE=0;
							}
						}
						else if((Timer_LED2_ALARM<150)&&(Timer_LED2_ALARM>105)||(Timer_LED2_ALARM<50)&&(Timer_LED2_ALARM>5))
						{   
							Flag_PWM_LED_ENABLE=0;
							Flag_PWM_BEEP_ENABLE=1;							
						}
						else 
						{   
							Flag_PWM_LED_ENABLE=0;
							Flag_PWM_BEEP_ENABLE=0;							
						}
						if(INT0)LED_SIGNAL=1;
						else LED_SIGNAL=0;
					}
					else 
					{
						LED_SIGNAL=1;
						Flag_PWM_LED_ENABLE=0;
						Flag_PWM_BEEP_ENABLE=0;						
					}
			}
		}
		else 
		{
			if(Timer_LOW_BATT==0)
			{
				Timer_LOW_BATT=50;
				LED_SIGNAL=~LED_SIGNAL;
			}
			Flag_PWM_LED_ENABLE=0;
			Flag_PWM_BEEP_ENABLE=0;			
		}

		PWM_output();

		if(Timer_ADC==0)
		{
			Timer_ADC=10;
			ADC_Vpower();
		}		
		
		if(((INT0==1)||(Timer_PowerON!=0))&&(Flag_power_Low==FALSE)&&(Timer_LED2_ALARM==0)&&(Timer_LED1_ALARM==0)) //等待外中断为高电平
		{
			delay_ms(10);	//delay 10ms
			if((INT0==1)||(Timer_PowerON!=0)) //等待外中断为高电平			
			{
				Timer_PowerON=0;
				PrintString1("SLEEP  ");
				delay_ms(10);	//delay 10ms
				Exti_config();
				WakeUpSource = 0;
			
				ALL_LED_off();
				MCU_POWER_DOWN();	//Sleep
				_nop_();_nop_();_nop_();
				delay_ms(1);	//delay 1ms
				
				PrintString1("外中断INT");
				if(WakeUpSource)PrintNumber(WakeUpSource-1);
				PrintString1("  ");
				
				WakeUpCnt++;
				PrintNumber(WakeUpCnt);
				PrintString1("次唤醒\r\n");

				POWER_CE=1;
				delay_ms(20);
				ADC_Vpower();
			}
		}

	}

}

