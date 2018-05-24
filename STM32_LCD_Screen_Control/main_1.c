/**   Project name : 
*   
--------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>

#include "stm32f0xx.h"          // File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void stdout_init (void);
extern void LCD_Home(void);
extern void LCD_Clear(void);
extern void LCD_Init(void);
extern void LCD_printstring( char * , uint8_t );
extern void LCD_printdata(char );


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

/*----------------------
  setup I2C PB.13 , PB.14
----------------------------*/
void Configure_GPIOB1314_I2C2(void)
{
	RCC->AHBENR |= 0x1ul <<18;
	
	GPIOB->MODER &= ~ (0x3ul << 13*2 | 0x3ul << 14*2);
	GPIOB->MODER |= 0x2ul << 13*2 | 0x2ul << 14*2;
	
	GPIOB->AFR[1] &= ~( 0xful << 5*4 | 0xful << 6*4);
	GPIOB->AFR[1] |= 5<< 5*4 | 5 << 6*4;
	
	GPIOB->OTYPER |= 0x1ul << 13 | 0x1ul << 14;
	
	
}

/*-------------------------
  setup I2C2 
---------------------------*/
void Configure_I2C2_Master(void)
{
	RCC->APB1ENR |= 0x1ul <<22;
	
	I2C2->TIMINGR = 0xB0211113ul;
	I2C2->CR1 |= 0x1ul;
	I2C2->CR2 |= 0x1ul << 25 | 2 << 16 | 0x3f <<1;
	
	
}

/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	
	char string1[]="This is LCD";
	uint8_t char_len;
	
	
	
		/* Configure the System clock frequency, AHB/APBx prescalers and Flash settings */
  SetSysClock();
	
	stdout_init();              // Initialize USART 2(PA3 to USART2_RX,PA2 to USART2_TX)  
		                                            
	SysTick_Config(SystemCoreClock/1000);      // System Tick Initializes,set SysTick 1ms interrupt
	
	Button_Init();
	
	Configure_GPIOB1314_I2C2();
	Configure_I2C2_Master();
	LCD_Init();
	
	printf("LCD display demo.\n\r");
	char_len = strlen(string1); 
	LCD_printstring(string1 , char_len);
	
	
	
	
}