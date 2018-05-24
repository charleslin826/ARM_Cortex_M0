/**   Project name : LCD_Screen_Control
*   									I2C2 control
--------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "stm32f0xx.h"          // File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void stdout_init (void);
extern void LCD_Home(void); // import from LCD_control.c
extern void LCD_Clear(void);
extern void LCD_Init(void);
extern void LCD_printstring( char *, uint8_t ); // we can ignore the variable "string_ptr" & "string_length"
extern void LCD_printdata(char );// we can ignore the variable "data"
extern void LCD_noCursor(void);
extern void LCD_Cursor(void);
extern void LCD_noBlinkCursor(void);
extern void LCD_BlinkCursor(void);
extern void LCD_DisplayOff(void);
extern void LCD_DisplayOn(void);
extern void LCD_scrollDisplayLeft(void);
extern void LCD_scrollDisplayRight(void);
extern void LCD_setCursor(uint8_t col, uint8_t row);
extern void Set_CGRAM(void);
extern void LCD_BacklightOff(void);
extern void LCD_BacklightOn(void);

volatile uint32_t msTicks;         // Counter for millisecond Interval

#define NUM_KEYS  1                     /* Number of available keys           */

/* Keys for NUCLEO Board */
#define USER    1

/*--------------------------------------------------------------------------------*/
// SysTick Interrupt Handler
/*--------------------------------------------------------------------------------*/
void SysTick_Handler (void) 
{       
    msTicks++;           // Increment Counter
}

/*--------------------------------------------------------------------------------*/
// Delay: delay a number of Systicks
/*--------------------------------------------------------------------------------*/
void Delay(uint32_t dlyTicks){
	uint32_t currentTicks;
	
	currentTicks = msTicks;
	while( (msTicks - currentTicks) < dlyTicks ){
		  __NOP();
	}
}

/**-----------------------------------------------------------------------------
  * @brief  Configures the System clock frequency, AHB/APBx prescalers and Flash
  *         settings.
  * @note   This function should be called only once the RCC clock configuration
  *         is reset to the default reset state (done in SystemInit() function).
  * @param  None
  * @retval None
--------------------------------------------------------------------------------  */
static void SetSysClock(void)
{
	SystemCoreClock = 48000000;
  /* SYSCLK, HCLK, PCLK configuration ----------------------------------------*/

  /* At this stage the HSI is already enabled */

  /* Enable Prefetch Buffer and set Flash Latency */
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
 
  /* HCLK = SYSCLK */
  RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
  /* PCLK = HCLK */
  RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;

  /* PLL configuration = (HSI/2) * 12 = ~48 MHz */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL));
  RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL12);
            
  /* Enable PLL */
  RCC->CR |= RCC_CR_PLLON;

  /* Wait till PLL is ready */
  while((RCC->CR & RCC_CR_PLLRDY) == 0)
  {
  }

  /* Select PLL as system clock source */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

  /* Wait till PLL is used as system clock source */
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
  {
  }
}
		


/*--------------------------------------------------------------------------------*/
// Button_Init(void)        ;Initialize button
// PC.13 to control User botton, set PC.13 is input pin 
/*--------------------------------------------------------------------------------*/
void Button_Init(void) {

  RCC->AHBENR |=  (1ul << 19);                  // Enable GPIOC clock       
  GPIOC->MODER &= ~(3ul << 2*13);               // Set PC.13 is input  
   
}

/*------------------------------------------------------------------------------*/
//uint32_t Button_GetState(void)
// Get USER button (PC.13) state
// return: 1 means USER key pressed
/*------------------------------------------------------------------------------*/
uint32_t Button_GetState (void) {

  uint32_t val = 0;

  if ((GPIOC->IDR & (1ul << 13)) == 0) {         //When USER button pressed PC.13=0
    val |= USER;                                 // set USER button pressed
  }
  return (val);

}
/*--------------------------------------------------------------------------------*/
// setup I2C_2 PB.13 & 14  @ 0423
/*--------------------------------------------------------------------------------*/
void Configure_GPIOB1314_I2C2(void)
{
	RCC->AHBENR |= 0x1ul << 18;   //enable Port B
	GPIOB->MODER &= ~(0x3ul<<13*2 | 0x3ul << 14*2 );  // Clean up PB.13 14
	GPIOB->MODER |= (0x2ul<<13*2 | 0x2ul << 14*2 );	 // Setup PB.13 14
	GPIOB->AFR[1] &= ~(0xful ); // clean up AFR13 . 14
	GPIOB->AFR[1] |= 5<< 5*4 | 5 << 6*4; // setup AF5 for AFR13 . 14
	
	GPIOB->OTYPER |= 0x1ul << 13 | 0x1ul << 14;// Setup Output Open_drain for PB.13 14
	
	
}
/*--------------------------------------------------------------------------------*/
// Setup I2C2  @ 0423
/*--------------------------------------------------------------------------------*/
void Configure_I2C2_Master(void)
{
	RCC->APB1ENR |= 0x1ul<<22; //  I2C2 clock enabled
	
	I2C2->TIMINGR = 0xB0211113ul; // this is so difficult
	I2C2->CR1 |= 0x1ul;
	I2C2->CR2 |= 0x1ul << 25 | 2 << 16 | 0x3f << 1 ;
//I2C2->CR2 |= 0x0ul << 10 // Bit 10 RD_WRN: Transfer direction (master mode) 0: Master requests a write transfer. //Due to Default is 0 so we don't need to change



}
/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	
	char string1[] = "This is LCD~~~~~~~~~";
	char string2[]="Welcome !!!";
	char string_40char[]="012345678901234567890123456789It'sFourty";
	//char data[]="";
	uint8_t char_len,i,data;
	
	
		/* Configure the System clock frequency, AHB/APBx prescalers and Flash settings */
  SetSysClock();
	
	stdout_init();              // Initialize USART 2(PA3 to USART2_RX,PA2 to USART2_TX)  
		                                            
	SysTick_Config(SystemCoreClock/1000);      // System Tick Initializes,set SysTick 1ms interrupt
	
	Button_Init();
	
	Configure_GPIOB1314_I2C2();
	Configure_I2C2_Master();
	LCD_Init();
	

	for(;;){
	printf("LCD display demo.\n\r");
	char_len = strlen(string1);      // need to include string.h
	LCD_printstring(string1, char_len);
	
	while(Button_GetState() == 0);
		
	LCD_setCursor(5,1); //set cursor position = no.6  & at first line	
	char_len = strlen(string2);
	LCD_printstring(string2, char_len);
	Delay(1000);
		
	
	while(Button_GetState() == 0);
	LCD_DisplayOff();
	Delay(100);
	
	while(Button_GetState() == 0);
	LCD_DisplayOn();
	Delay(100);
	
	while(Button_GetState() == 0);
	LCD_Home();
	Delay(100);
	
	while(Button_GetState() == 0);
	LCD_noCursor();
	Delay(100);
	
	while(Button_GetState() == 0);
	LCD_Cursor();
	Delay(100);
	
	while(Button_GetState() == 0);
	LCD_BlinkCursor();
	Delay(100);
	
	while(Button_GetState() == 0);
	LCD_noBlinkCursor();
	Delay(100);
	
	while(Button_GetState() == 0);
	Set_CGRAM();
	Delay(100);
	
	LCD_setCursor(0,1);
	LCD_printdata(0);
	LCD_printdata(1);
	LCD_printdata(2);
	LCD_printdata(3);
	
	while(Button_GetState() == 0);
	LCD_Clear();
	Delay(100);
	
	while(Button_GetState() == 0);
	char_len = strlen(string_40char);
	LCD_printstring(string_40char, char_len);
	Delay(100);
	LCD_printstring(string_40char, char_len);
	Delay(100);
	
	for(i=0;i<40;i++){
		LCD_scrollDisplayLeft();
		Delay(10);
	}
	while(Button_GetState() == 0);
	LCD_Clear();
	data=0xb1;
	for(i=0;i<16;i++){
		LCD_printdata(data);
		data++;
	}
	
	Delay(500);
	
	LCD_setCursor(0,1);
	for(i=0;i<16;i++){
		LCD_printdata(data);
		data++;
	
	}
	
	Delay(1000);
	
	LCD_setCursor(0,0);
	for(i=0;i<16;i++){
		LCD_printdata(data);
		data++;
	}
	
	Delay(500);
	
	LCD_setCursor(0,1);
	for(i=0;i<16;i++){
		LCD_printdata(data);
		data++;
	
	}
	while(Button_GetState() == 0);
	LCD_Clear();
	Delay(500);
	
	while(Button_GetState() == 0);
	LCD_BacklightOff();
	Delay(500);
	
	while(Button_GetState() == 0);
	LCD_BacklightOn();
	Delay(500);
	
	
	}

}







