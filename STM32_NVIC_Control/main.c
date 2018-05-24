/**   Project name : 
*   
--------------------------------------------------------------------------------------*/

#include <stdio.h>

#include "stm32f0xx.h"          // File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void stdout_init (void);

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
// EXTI(External Interrupt,EXTI0_1_IRQHandler) 0,1  @override
/*--------------------------------------------------------------------------------*/
void EXTI0_1_IRQHandler(void){   //function name need to exactly the same to override it
int i;
for(i=0;i<100;i++){
	__NOP();
}	

printf("This is IRQ5 Interrupt handler\n\r");

}
/*--------------------------------------------------------------------------------*/
// EXTI 2,3
/*--------------------------------------------------------------------------------*/
void EXTI2_3_IRQHandler(void){
int i;
for(i=0;i<100;i++){
	__NOP();
}	

printf("This is IRQ6 Interrupt handler\n\r");

}
/*--------------------------------------------------------------------------------*/
// EXTI 4-15
/*--------------------------------------------------------------------------------*/
void EXTI4_15_IRQHandler(void){
int i;
for(i=0;i<100;i++){
	__NOP();
}	

printf("This is IRQ7 Interrupt handler\n\r");

}

/*--------------------------------------------------------------------------------*/
// PendSV_Handler
/*--------------------------------------------------------------------------------*/
void PendSV_Handler(void){
	printf("This is PendSV exception.\n\r");
}

void NMI_Handler(void){
	printf("This is NMI exception.\n\r");
}
/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	
		/* Configure the System clock frequency, AHB/APBx prescalers and Flash settings */
  SetSysClock();
	
	stdout_init();              // Initialize USART 2(PA3 to USART2_RX,PA2 to USART2_TX)  
		                                            
	SysTick_Config(SystemCoreClock/1000);      // System Tick Initializes,set SysTick 1ms interrupt
	
	SCB->SHP[1] &= ~(0x3ul <<22);  //clear it at first 
	SCB->SHP[1] |= 0x2ul << 22;  //set PendSV Priority 
	
	
	NVIC_SetPriority(EXTI0_1_IRQn , 1); //which is configurable, it can set 0~3
	NVIC_SetPriority(EXTI2_3_IRQn , 3);
	NVIC_SetPriority(EXTI4_15_IRQn , 1);
	
	NVIC_EnableIRQ(EXTI0_1_IRQn); 
	NVIC_EnableIRQ(EXTI2_3_IRQn); 
	NVIC_EnableIRQ(EXTI4_15_IRQn); 
	
	printf(" \n\r NVIC test starts. \n\r");
	
	__disable_irq();
	
	SCB->ICSR |= 0x1ul <<28;  // set PendSV pending bit   please refer PDF(M0_3) p.22
	SCB->ICSR |= 0x1ul <<31;  // set NMI pending bit
	
	NVIC_SetPendingIRQ(EXTI0_1_IRQn); 
	NVIC_SetPendingIRQ(EXTI2_3_IRQn); 
	NVIC_SetPendingIRQ(EXTI4_15_IRQn); 
	
	NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
	__enable_irq();
	
	NVIC_SetPendingIRQ(EXTI2_3_IRQn);	
	NVIC_SetPendingIRQ(EXTI0_1_IRQn); 
	NVIC_SetPendingIRQ(EXTI4_15_IRQn); 
	
	SCB->ICSR |= 0x1ul <<28;  // set PendSV pending bit
	SCB->ICSR |= 0x1ul <<31;  // set NMI pending bit
	
	
	for(;;){
		
	}
}