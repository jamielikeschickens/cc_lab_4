/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_gpio.h"

/* Private variables ---------------------------------------------------------*/

static TSL_tTick_ms_T last_tick_tsl;  /* Hold the last tsl time value */
extern uint8_t t_bar[2];              /* LCD bar graph: used for displaying active function */
uint16_t Int_CurrentSTBY;             /* */
void RCC_Configuration();
void Init_GPIOs();
extern void Delay_Asm();

void _init() { }

////////////////////////////////////
// Declare three simple processes //
///////////////////////////////////


void p1_simple()
{
	uint8_t pstring[7] = "P1" ;
	BAR0_OFF ;
	BAR1_ON ;
	BAR2_OFF ;
	BAR3_OFF ;
	LCD_GLASS_Clear() ;
	LCD_GLASS_DisplayString(pstring) ;
}


void p1_better() {
	uint8_t pstring[7] = "P1" ;
	unsigned bars ; // store bars as bit pattern: {BAR3, BAR2, BAR1, BAR0}
	unsigned waitTime = 0xffff ;
	bars = 0x2 ; // BAR1 ON, other off
	OS_Call(OS_CALL_DISPLAY, bars, (unsigned) pstring, 0) ;
	OS_Call(OS_CALL_WAIT, waitTime, 0, 0 ) ;
	OS_Call(OS_CALL_KILL, 0, 0, 0) ;
}


void p2_simple()
{
	uint8_t pstring[7] = "P2" ;
	BAR0_OFF ;
	BAR1_OFF ;
	BAR2_ON ;
	BAR3_OFF ;
	LCD_GLASS_Clear() ;
	LCD_GLASS_DisplayString(pstring) ;
}

void p3_simple()
{
	uint8_t pstring[8] = "P3" ;
	BAR0_OFF ;
	BAR1_OFF ;
	BAR2_OFF ;
	BAR3_ON ;
	LCD_GLASS_Clear() ;
	LCD_GLASS_DisplayString(pstring) ;
}


void OS_Call(osCall callType, unsigned arg1, unsigned arg2, unsigned arg3)
{
	/* TASK2: Complete this function. It should
	   /
	   / 1) Call assembly code which executes a SWI with the same code as supplied in "callType"
	   / 2) Pass the three arguments to the SWI using the APCS standard.
	   / 3) Return when the interrupt handler returns
	   /
	   / You should support all four OS_CALLS described in main.h.
	   */
	BAR0_ON ;
	BAR1_OFF ;
	BAR2_OFF ;
	BAR3_OFF ;
}




int main(void)
{
	uint8_t mystring[7] = "*hhhh*\n" ;
	void (*newProcess)() ; // Declare a 'function pointer'. More on this later on

	////////////////////////////////
	// Next three functions
	// Needed for system to work
	// Leave as they are and at top of main()
	//////////////////////////////

	/* Configure Clocks for Application need */
	RCC_Configuration();

	/* Init I/O ports */
	Init_GPIOs();

	/* Initializes the LCD glass */
	LCD_GLASS_Init();

	/* Drop to process mode and privilege */
	Mode_Switch();


	//////////////////////////
	// Start User Code      //
	/////////////////////////


	/* Switch the leds on and off */
	GPIO_HIGH(LD_GPIO_PORT,LD_GREEN_GPIO_PIN);	 // green on
	GPIO_LOW(LD_GPIO_PORT,LD_BLUE_GPIO_PIN);	// blue off


	/* Set LCD bars */
	/* Changes only take effect when an "LCD_GLASS..." call is made */
	BAR0_ON;
	BAR1_OFF;
	BAR2_OFF;
	BAR3_OFF;


	/* Display Welcome message */
	LCD_GLASS_Clear() ;
	LCD_GLASS_DisplayString(mystring) ;

	// Dumb use of the processes
	p1_simple() ;
	// Task1: Insert a delay by calling existing assembly code
	Delay_Asm();
	p2_simple() ;
	// Task1: Insert a delay by calling existing assembly code
	Delay_Asm();
	p3_simple() ;


	// TASK3: Below is the call we want to support by the end of the lab
	newProcess = &p1_simple ; // Get the address of p1_simple function. This is a "Function pointer"
	OS_Call(OS_CALL_CREATE, (unsigned) newProcess, 0, 0) ;

	newProcess = &p2_simple ; // Get the address of p1_simple function. This is a "Function pointer"

	newProcess = &p3_simple ; // Get the address of p1_simple function. This is a "Function pointer"




	while(1){} ;
}



/*
 *  All below are library functions.
 *  Leave as they are
 *
 */



/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
	/* Enable HSI Clock */
	RCC_HSICmd(ENABLE);

	/*!< Wait till HSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	/* Set HSI as sys clock*/
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

	/* Set MSI clock range to ~4.194MHz*/
	RCC_MSIRangeConfig(RCC_MSIRange_6);

	/* Enable the GPIOs clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC| RCC_AHBPeriph_GPIOD| RCC_AHBPeriph_GPIOE| RCC_AHBPeriph_GPIOH, ENABLE);

	/* Enable comparator, LCD and PWR mngt clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP | RCC_APB1Periph_LCD | RCC_APB1Periph_PWR,ENABLE);

	/* Enable ADC & SYSCFG clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG , ENABLE);

	/* Allow access to the RTC */
	PWR_RTCAccessCmd(ENABLE);

	/* Reset RTC Backup Domain */
	RCC_RTCResetCmd(ENABLE);
	RCC_RTCResetCmd(DISABLE);

	/* LSE Enable */
	RCC_LSEConfig(RCC_LSE_ON);

	/* Wait until LSE is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

	/* RTC Clock Source Selection */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	/* Enable the RTC */
	RCC_RTCCLKCmd(ENABLE);

	/*Disable HSE*/
	RCC_HSEConfig(RCC_HSE_OFF);
	if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET )
	{
		/* Stay in infinite loop if HSE is not disabled*/
		while(1);
	}
}

/**
 * @brief  To initialize the I/O ports
 * @caller main
 * @param None
 * @retval None
 */
void Init_GPIOs (void)
{
	/* GPIO, EXTI and NVIC Init structure declaration */
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure User Button pin as input */
	GPIO_InitStructure.GPIO_Pin = USERBUTTON_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(USERBUTTON_GPIO_PORT, &GPIO_InitStructure);

	/* Select User Button pin as input source for EXTI Line */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);

	/* Configure EXT1 Line 0 in interrupt mode trigged on Rising edge */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 ;  // PA0 for User button AND IDD_WakeUP
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure the LED_pin as output push-pull for LD3 & LD4 usage*/
	GPIO_InitStructure.GPIO_Pin = LD_GREEN_GPIO_PIN | LD_BLUE_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LD_GPIO_PORT, &GPIO_InitStructure);

	/* Force a low level on LEDs*/
	GPIO_LOW(LD_GPIO_PORT,LD_GREEN_GPIO_PIN);
	GPIO_LOW(LD_GPIO_PORT,LD_BLUE_GPIO_PIN);

	/* Counter enable: GPIO set in output for enable the counter */
	GPIO_InitStructure.GPIO_Pin = CTN_CNTEN_GPIO_PIN;
	GPIO_Init( CTN_GPIO_PORT, &GPIO_InitStructure);

	/* To prepare to start counter */
	GPIO_HIGH(CTN_GPIO_PORT,CTN_CNTEN_GPIO_PIN);

	/* Configure Port A LCD Output pins as alternate function */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_9 |GPIO_Pin_10 |GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init( GPIOA, &GPIO_InitStructure);

	/* Select LCD alternate function for Port A LCD Output pins */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15,GPIO_AF_LCD) ;

	/* Configure Port B LCD Output pins as alternate function */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 \
								  | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init( GPIOB, &GPIO_InitStructure);

	/* Select LCD alternate function for Port B LCD Output pins */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15,GPIO_AF_LCD) ;

	/* Configure Port C LCD Output pins as alternate function */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 \
								  | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init( GPIOC, &GPIO_InitStructure);

	/* Select LCD alternate function for Port B LCD Output pins */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource0,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource1,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource2,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10,GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,GPIO_AF_LCD) ;

	/* Configure ADC (IDD_MEASURE) pin as Analogue */
	GPIO_InitStructure.GPIO_Pin = IDD_MEASURE  ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Init( IDD_MEASURE_PORT, &GPIO_InitStructure);
}


///**
//  * @brief  Executed when a sensor is in Error state
//  * @param  None
//  * @retval None
//  */
//void MyLinRots_ErrorStateProcess(void)
//{
//  // Add here your own processing when a sensor is in Error state
//  TSL_linrot_SetStateOff();
//}


///**
//  * @brief  Executed when a sensor is in Off state
//  * @param  None
//  * @retval None
//  */
//void MyLinRots_OffStateProcess(void)
//{
//  // Add here your own processing when a sensor is in Off state
//}



/**
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length, in 1 ms.
 * @retval None
 */
void Delay(uint32_t nTime)
{
	while (TSL_tim_CheckDelay_ms((TSL_tTick_ms_T) nTime, &last_tick_tsl) != TSL_STATUS_OK);
}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* Infinite loop */
	while (1);
}

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
