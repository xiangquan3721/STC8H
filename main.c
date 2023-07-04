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

/*************	功能说明	**************

本例程基于STC8H8K64U为主控芯片的实验箱8进行编写测试，STC8G、STC8H系列芯片可通用参考.

演示INT0~INT4 5个唤醒源将MCU从休眠唤醒.

从串口输出唤醒源跟唤醒次数，115200,N,8,1.

下载时, 选择时钟 22.1184MHz (用户可在"config.h"修改频率).

******************************************/

/*************	本地常量声明	**************/


/*************	本地变量声明	**************/
u8 WakeUpSource;
u8 WakeUpCnt;

/*************	本地函数声明	**************/



/*************  外部函数和变量声明 *****************/



/******************** IO口配置 ********************/
void	GPIO_config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;				//结构定义

	GPIO_InitStructure.Pin  = GPIO_Pin_All;			//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7, 或操作
	GPIO_InitStructure.Mode = GPIO_PullUp;			//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	//初始化
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

	Exti_InitStructure.EXTI_Interrupt = ENABLE;				//中断使能,   ENABLE或DISABLE
	Ext_Inilize(EXT_INT2,&Exti_InitStructure);				//初始化

	Exti_InitStructure.EXTI_Interrupt = ENABLE;				//中断使能,   ENABLE或DISABLE
	Ext_Inilize(EXT_INT3,&Exti_InitStructure);				//初始化

	Exti_InitStructure.EXTI_Interrupt = ENABLE;				//中断使能,   ENABLE或DISABLE
	Ext_Inilize(EXT_INT4,&Exti_InitStructure);				//初始化
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

/******************** 主函数***********************/
void main(void)
{
	GPIO_config();
	UART_config();
//	Exti_config();
	EA  = 1;		//Enable all interrupt
	
	while(1)
	{
		while(!INT0);	//等待外中断为高电平
		while(!INT1);	//等待外中断为高电平
		while(!INT2);	//等待外中断为高电平
		while(!INT3);	//等待外中断为高电平
		while(!INT4);	//等待外中断为高电平
		delay_ms(10);	//delay 10ms

		while(!INT0);	//等待外中断为高电平
		while(!INT1);	//等待外中断为高电平
		while(!INT2);	//等待外中断为高电平
		while(!INT3);	//等待外中断为高电平
		while(!INT4);	//等待外中断为高电平
		delay_ms(10);	//delay 10ms

		Exti_config();
		WakeUpSource = 0;

		MCU_POWER_DOWN();	//Sleep
		_nop_();
		_nop_();
		_nop_();
		delay_ms(1);	//delay 1ms
		
		if(WakeUpSource == 1)	PrintString1("外中断INT0唤醒  ");
		if(WakeUpSource == 2)	PrintString1("外中断INT1唤醒  ");
		if(WakeUpSource == 3)	PrintString1("外中断INT2唤醒  ");
		if(WakeUpSource == 4)	PrintString1("外中断INT3唤醒  ");
		if(WakeUpSource == 5)	PrintString1("外中断INT4唤醒  ");
		WakeUpSource = 0;
		
		WakeUpCnt++;
		TX1_write2buff(WakeUpCnt/100+'0');
		TX1_write2buff(WakeUpCnt%100/10+'0');
		TX1_write2buff(WakeUpCnt%10+'0');
		PrintString1("次唤醒\r\n");
	}

}

