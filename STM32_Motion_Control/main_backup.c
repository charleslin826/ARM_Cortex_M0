/**   Project name : 
*   
--------------------------------------------------------------------------------------*/

#include <stdio.h>

#include "stm32f0xx.h"          // File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void stdout_init (void);

volatile uint32_t msTicks;         // Counter for millisecond Interval
volatile uint32_t TIM6_Ticks;
volatile uint32_t Current_Tick;
volatile uint8_t TIM6_flag;
volatile uint32_t Total_Time;

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
/*-----------------------------------------
         set up timer 6
------------------------------------------*/
void Configure_TIM6(void)
{
  RCC->APB1ENR |= 0x1ul <<4;
	
	TIM6->PSC=4799;
	TIM6->ARR=999;
	TIM6->CNT=0;
	TIM6->CR1 |= 0x1ul;
	TIM6->DIER |=0x1ul;
	
	NVIC_ClearPendingIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn,0);
	NVIC_EnableIRQ(TIM6_IRQn);
	
}

void TIM6_IRQHandler(void)
{
	TIM6_Ticks++;
	TIM6->SR &=~0x1ul;
}
/*---------------------
    Config PB.2
-----------------------*/
void Configure_PB2(void)
{
   RCC->AHBENR |= 0x1ul<<18;
	 GPIOB->MODER &=~(0x3ul<<2*2);
	
	 RCC->APB2ENR|=0x1ul;
	 SYSCFG->EXTICR[0] &= ~(0xful<<2*4);
	 SYSCFG->EXTICR[0] |= 0x1ul<<2*4;
	
	 EXTI->IMR|= 0x1ul <<2;
	 
	 EXTI->RTSR |= 0x1ul <<2;
	 EXTI->FTSR |= 0x1ul <<2;
	
	 EXTI->PR |=0x1ul<<2;
	
	 NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
	 NVIC_SetPriority(EXTI2_3_IRQn,3);
	 NVIC_EnableIRQ(EXTI2_3_IRQn);

}

void EXTI2_3_IRQHandler(void)
{
	if(EXTI->PR & (0x1ul<<2)){
	  
		if((GPIOB->IDR&(0x1ul<<2))==(0x1ul<<2)){
			
			  Current_Tick = TIM6_Ticks;
			  TIM6_flag =1;
			  printf("PB.2 detects motion . \n\r");
			
		}else{
			  if(TIM6_flag==1){
				    Total_Time = TIM6_Ticks - Current_Tick;
					  printf("The detection time is %d\n\r",Total_Time);
					  TIM6_flag = 0;
					  printf("End of motion detection by PB.2\n\r");
				}
			
	  }
	}else{
	   printf("Error !!!\n\r");
	}
	   EXTI->PR |= 0x1ul <<2;
	
}

//init GPIOC
void GPIOC_init(void)
{
    RCC->AHBENR |= 0x1ul <<19;
	  
	  GPIOC->MODER &=~0x0000fffful;
	  GPIOC->MODER |= 0x00005555ul;
	
	  GPIOC->OTYPER &=~0x000000fful;
	
	  GPIOC->OSPEEDR &=~0x0000fffful;
	  GPIOC->OSPEEDR &= 0x00005555ul;
	
	  GPIOC->PUPDR & ~0x0000fffful;
}


#define ON_ALL_LED  1
#define OFF_ALL_LED 2
#define ON_ODD_LED  3
#define ON_EVEN_LED 4
#define ON_Bit_LED  5

void LED8_on(int displayMode,int bit)
{
    uint32_t activeBit;
     switch(displayMode){
			 case ON_ALL_LED :
				 GPIOC->ODR |=0x000000fful;
			    break;
			 
			 case OFF_ALL_LED :
				 GPIOC->ODR &=~0x000000fful;
				 break;
			 
			 case ON_ODD_LED :
				 GPIOC->ODR &=~0x000000fful;
			   GPIOC->ODR |=0x00000055ul;			 
				 break;
			 
			 case ON_EVEN_LED :
				 GPIOC->ODR &=~0x000000fful;
			   GPIOC->ODR |=0x000000aaul;
				 break;
			 
			 case ON_Bit_LED :
				 GPIOC ->ODR &=~0x000000fful;
			   activeBit = 0x1ul;
			   activeBit =activeBit<<bit;
			   GPIOC ->ODR |=activeBit;
				 break;
		   default:
				 break;		 
		 }
}





/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	
		/* Configure the System clock frequency, AHB/APBx prescalers and Flash settings */
  SetSysClock();
	
	stdout_init();              // Initialize USART 2(PA3 to USART2_RX,PA2 to USART2_TX)  
		                                            
	SysTick_Config(SystemCoreClock/1000);      // System Tick Initializes,set SysTick 1ms interrupt
	
	Configure_TIM6();
	Configure_PB2();
	GPIOC_init();
	
	TIM6_flag = 0;
	Total_Time = 0;
	
	 printf(" motion sensor begine\n\r");
	
	for(;;){
	   
		while(TIM6_flag){
		   LED8_on(ON_ALL_LED,0);
		}			
		 LED8_on(OFF_ALL_LED,0);		
		}
	
	
	}
	
	
	
	
