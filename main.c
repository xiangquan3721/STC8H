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
#define	DEF_PWMout_log
/*************	本地预定义	**************/
#define POWER_CTRL   P15
#define LED_CTRL     P34
#define Key_SW1      P32
/*************	本地常量声明	**************/
#define DEF_PWM_Period  MAIN_Fosc/1000			//周期时间,   2KHz=2000,1KHz=1000,
#define LOW_VOLTAGE 1428 //mv  1428mv*2.8+200mv=4000mv,200mv是电池到取样之间的硬件电路损耗
#define LOW_VOLTAGE_MIN 1215 //mv  1215mv*2.8+200mv=3600mv,200mv是电池到取样之间的硬件电路损耗
#define HIGHEST_TEMP 1423  //mv  //LED温度>80℃  80℃时Rt=1.669K，Rp=2.2K
/*************	本地变量声明	**************/
PWMx_Duty PWMA_Duty;
u16	Vdc,Vadc_led,Vadc_Power;
u8 pwm_step_max = 2;
u8 set_pwm = 0;
u8 key_sta = 0;
u8 flag_sw1=0;
u8 flag_temp_thermal=0;
u8 flag_bat_low=0;
u8 flag_power_off=0;
u8 flag_adc_over=0;
u8 flag_low_bat_min=0;

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
	GPIO_InitStructure.Pin  = GPIO_Pin_3;		//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7
	GPIO_InitStructure.Mode = GPIO_HighZ;	//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	//初始化
	//AD口设置为输入口
	GPIO_InitStructure.Pin  = GPIO_Pin_5;		//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7
	GPIO_InitStructure.Mode = GPIO_HighZ;	//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	//初始化	
	
	POWER_CTRL=0;	//电源控制IO口设置为低电平, 1=上电, 0=断电
	LED_CTRL=0;		//LED控制IO口设置为低电平, 1=点亮, 0=熄灭
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
	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / 1000UL);		//初值,  1000Hz=1000；100Hz=100
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

	PrintString1("STC8H1K08 FlashLight_Pro KingJim!\r\n");	//UART1发送一个字符串
}

void	PWM_LED_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;
	
	PWMx_InitStructure.PWM4_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2

	PWMx_InitStructure.PWM4_SetPriority  = Priority_0;			//指定中断优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	
	PWMx_InitStructure.PWM_Period   = DEF_PWM_Period;			//周期时间,   2KHz=2000,1KHz=1000,
	PWMx_InitStructure.PWM4_Duty    = DEF_PWM_Period/10*8;	  //PWM4占空比时间, 0~Period
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
	
	PWMA_Duty.PWM4_Duty=DEF_PWM_Period/10*8;
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
	#ifdef DEF_ADC_log
	PrintString1("Vdc:");
	PrintNumber(Vdc);
	PrintString1("mV\r\n");
	#endif	

	Sum_Nx=0;
	for(i=0;i<5;i++)
	{
		Sum_Nx += Get_ADCResult(13);	//读LED的温度, 读1通道
	}
	Nadc_led = Sum_Nx/5;
	Vadc_led = (u32)Vdc * (u32)Nadc_led / 1024;	//Ux = Vdc * Nx / 1024;
	#ifdef DEF_ADC_log
	PrintString1("Vadc_led:");
	PrintNumber(Vadc_led);
	PrintString1("mV\r\n");	
	#endif

	Sum_Nx=0;
	for(i=0;i<5;i++)
	{
		Sum_Nx += Get_ADCResult(11);	//读电池电压, 读1通道
	}
	Nadc_led = Sum_Nx/5;
	Vadc_Power = (u32)Vdc * (u32)Nadc_led / 1024;	//Ux = Vdc * Nx / 1024;
	#ifdef DEF_ADC_log
	PrintString1("Vadc_Power:");
	PrintNumber(Vadc_Power);
	PrintString1("mV\r\n");	
	#endif	

	flag_adc_over = 1;
	if(Vadc_Power < LOW_VOLTAGE) 
	{
		low_bat_val = 1;
		if(Vadc_Power < LOW_VOLTAGE_MIN) flag_low_bat_min = 1;
	}
    else if(Vadc_Power > LOW_VOLTAGE+150) low_bat_val = 0;
	if(Vadc_led < HIGHEST_TEMP) flag_temp_thermal = 1;
	else flag_temp_thermal = 0;		
}

u8 PWM_log=0xff;
void LED_PWM_SET(u8 pwm)
{
    switch(pwm)
    {
        case 0: //85%
			PWMA_Duty.PWM4_Duty=DEF_PWM_Period/10*8;
			UpdatePwm(PWMA, &PWMA_Duty);
			break;
		case 1: //50%
			PWMA_Duty.PWM4_Duty=DEF_PWM_Period/2;
			UpdatePwm(PWMA, &PWMA_Duty);
            break;
        case 3: //30%
			PWMA_Duty.PWM4_Duty=DEF_PWM_Period/10*3;
			UpdatePwm(PWMA, &PWMA_Duty);
            break;
        default:
			PWMA_Duty.PWM4_Duty=DEF_PWM_Period/2;
			UpdatePwm(PWMA, &PWMA_Duty);
            break;
    }
	#ifdef DEF_PWMout_log
	if(PWM_log != pwm)
	{
		PWM_log = pwm;
		PrintString1("PWM:");
		PrintNumber(pwm);
		PrintString1("\r\n");
	}
	#endif	
}

void LED_PWM_ON(void)
{
	PWM_LED_config();
	#ifdef DEF_PWMout_log
	PrintString1("LED ON\r\n");
	#endif
}

void LED_PWM_OFF(void)
{
	PWM_LED_OFF();
	#ifdef DEF_PWMout_log
	PrintString1("LED OFF\r\n");
	#endif
    LED_CTRL=0;
}

u8 Key_Sw1_Scan(void)
{
    if(Key_SW1 == 0 && key_sta == 0)
    {
        key_sta = 1;
        delay_ms(20);
        if(Key_SW1 == 0)       return 1;
    }
    else if(Key_SW1 == 1)
    {
        key_sta = 0;
    }
    return 0;
}
u8 sw_cnt = 0;
void key_sw1_dete(void)
{
    if(Key_SW1 == 0 && time_sw1_dete == 0)
    {
        time_sw1_dete = 100;
        if(Key_SW1 == 0)
        {
            sw_cnt++;
            if(sw_cnt >= 7)
            {
                sw_cnt = 0;
                flag_sw1 = 1;
                LED_PWM_OFF();
                POWER_CTRL=0;
				#ifdef DEF_PWMout_log
				PrintString1("Power OFF\r\n");
				#endif				
                flag_power_off = 1;
            }
            else flag_sw1 = 0;
        }
    }
    else if(Key_SW1 == 1)
    {
        delay_ms(20);
        if(Key_SW1 == 1)
        {
            sw_cnt = 0;
            flag_sw1 = 0;
        }
    }
}
u16 is = 0;
u8 ix = 0;
u8 iy = 0;
void protect_deal(void)
{
    if(flag_adc_over)
    {
        flag_adc_over = 0;
        if(flag_temp_thermal == 1)
        {
            LED_PWM_SET(3);
            is++;
            if(is >= 60)
            {
                is = 0;
                LED_PWM_OFF();
                POWER_CTRL=0;
            }
        }else is = 0;
    }
    if(low_bat_val == 1 && time_bat_prot >= 500) //500ms闪灯
    {
        time_bat_prot = 0;
        ix++;
        if(ix%2 == 1)
        {
			PWM_LED_OFF();
			LED_CTRL=0;
			#ifdef DEF_PWMout_log
			if(flag_bat_low==0)
			   PrintString1("low voltage\r\n");
			#endif			
            flag_bat_low = 1;
        }
        else
        {
            PWM_LED_config();			
            LED_PWM_SET(3);
        }
        if(flag_low_bat_min==1)//if(ix >= 120)
        {
            LED_PWM_OFF();
            POWER_CTRL=0;
        }
    }

    if(flag_temp_thermal == 0 && low_bat_val == 0)
    {
        if(low_bat_val == 0) pwm_step_max = 2;
        if(flag_bat_low == 1)
        {
            is = 0; ix = 0; iy = 0;
            flag_bat_low = 0;
            LED_PWM_ON();
            if(low_bat_val == 0) LED_PWM_SET(set_pwm);
			#ifdef DEF_PWMout_log
			PrintString1("return to normal\r\n");
			#endif						
        }
    }
}
/******************** 主函数***********************/
u8 sw1 = 0;
u8 m_run = 0;
void main(void)
{
	WDT_config();
	GPIO_config();	
	UART_config();
	Timer_config();
	ADC_config();
//	Exti_config();
	EA  = 1;		//Enable all interrupt
    if(Key_Sw1_Scan() == 1)
    {
		time_run = 700;
		while(time_run) WDT_Clear();
		POWER_CTRL=1;		
		LED_PWM_ON();
		ADC_Vpower();
		TIME_Power_jion=50;
		while(TIME_Power_jion)
		{
			WDT_Clear();
			if(Key_SW1 == 0)TIME_Power_jion=50;
		}
		m_run = 1;
		#ifdef DEF_PWMout_log
		PrintString1("Power ON\r\n");
		#endif			
    }
    while (m_run)
    {
        WDT_Clear();    // 喂狗
        sw1 = Key_Sw1_Scan();
        if(sw1 == 1 && low_bat_val == 0)
        {		
			set_pwm++;
			if(set_pwm >= pwm_step_max) set_pwm = 0;
			LED_PWM_SET(set_pwm);
        }
        key_sw1_dete();
        if(flag_power_off == 0) protect_deal();
        if(flag_power_off == 0 && time_run == 0)
        {
            time_run = 1000;
            ADC_Vpower();
        }
    }
}

