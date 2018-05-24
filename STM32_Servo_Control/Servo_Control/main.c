/**   Project name : Servo Control 20180416
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

volatile uint32_t button_press;  //== static variable
#define USER_1  1
#define USER_2  2
#define USER_3  3
#define USER_4  4
#define USER_default  10

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
// Config PB.4 for Timer 3 @0416
/*--------------------------------------------------------------------------------*/
void Configure_Timer3_PB4(void){
	RCC->AHBENR |= 0x1ul <<18;        //set Port B
	GPIOB->MODER &= ~(0x3ul << 4*2);  //set PB.4
	GPIOB->MODER |= 0x2ul << 4*2;     // set Alternate mode to use PWM
	GPIOB->AFR[0] &= ~(0xful << 4*4); // clean Alternate function low register on Port 4
	GPIOB->AFR[0] |= 0x1ul << 4*4;    //set Port 4 to AF1 
	
	RCC->APB1ENR |= 0x1ul << 1;       //APB peripheral clock enable register 1 (RCC_APB1ENR) set TIM3 enable
	
	TIM3->PSC =4799;  // 4800    ;  48M/4800=10^4    
	TIM3->ARR =200;		// 200	;  200*(1/10^4)=1/10 = 0.02sec = 20ms (pls refer https://www.servocity.com/how-does-a-servo-work)
	TIM3->CNT =0;
	TIM3->CCMR1 &= ~ (0x7ul << 4);  // clean capture/compare mode register 1
	TIM3->CCMR1 |= 0x6ul << 4;      //0110
	TIM3->CR1 |= 0x1ul;  //set  control register 1  >> 1: Counter enabled
	
}
/*--------------------------------------------------------------------------------*/
// Disable PWM
/*--------------------------------------------------------------------------------*/
void DisablePWM(){
	TIM3->CCER &= ~0x1ul;  //clean capture/compare enable register
	TIM3->CR1 &= ~0x1ul;
}

void EnablePWM(){
	TIM3->CNT = 0;
	TIM3->CR1 |= 0x1ul;
	TIM3->CCER |= 0x1ul;
}

void ChangePWMValue(uint32_t value){
	TIM3->CNT = 0;
	TIM3->CCR1 = value; // capture/compare register 1 

}
/*--------------------------------------------------------------------------------*/
// Set up PA0, PA1 Interrupt   // == void ConfigureExternalIT(void){  //default 
/*--------------------------------------------------------------------------------*/
void ConfigurePA0PA1(void){  
	RCC->AHBENR |= 0x1ul << 17;
	GPIOA -> MODER &= ~(0x3ul | 0x3ul << 1*2);

	/*config SYSCFG EXTI*/
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;    //set 0x1ul enable SYSCFG clock
	SYSCFG -> EXTICR[0] &= ~(0xful | 0xful << 1*4);  //SYSCFG external interrupt configuration register 1
	
	EXTI -> FTSR |= (0x1ul | 0x1ul << 1); //Falling trigger selection refister(FTSR)
	EXTI -> IMR  |= (0x1ul | 0x1ul << 1); //Interrupt mask register(IMR)
	EXTI -> PR   |= (0x1ul | 0x1ul << 1); //Pending register(PR)
	
	NVIC_ClearPendingIRQ(EXTI0_1_IRQn); // == 5 (IRQ5)
	NVIC_SetPriority(EXTI0_1_IRQn, 0);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	
}
/*--------------------------------------------------------------------------------*/
// Set up EXTI0_1_IRQHandler (IRQ5 for PA0 PA1)
/*--------------------------------------------------------------------------------*/
void EXTI0_1_IRQHandler(void){
	uint32_t PR_value;
	
	if(EXTI -> PR & 0x3ul){  // PA0 PA1 
		
		PR_value = EXTI->PR;  ///pending register(EXTI_PR) need to be clear, otherwise will occur interrupt request again and again
		
		if(PR_value & 0x1ul){ // PA.0 was pressed
				if( (GPIOA->IDR & 0x1ul) ==0 ){
					button_press = USER_1;
					printf("PA0 was pressed. \n\r");
				}else{
					printf("NOISE!! PA0 didn't be pressed.\n\r");
				}
		}else{ // PA.1 was pressed
		
		if( (GPIOA->IDR & 0x1ul <<1) == 0 ){
			
					button_press = USER_2;
					printf("PA1 was pressed. \n\r");
				}else{
					printf("NOISE!! PA1 didn't be pressed.\n\r");
				}
	}	
	
	EXTI -> PR |= (0x1ul|0x1ul<<1);
	
	}else{
		printf("ERROR!!!\n\r");
	}
}


/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	int32_t pwm_value;
		/* Configure the System clock frequency, AHB/APBx prescalers and Flash settings */
  SetSysClock();
	
	stdout_init();              // Initialize USART 2(PA3 to USART2_RX,PA2 to USART2_TX)  
		                                            
	SysTick_Config(SystemCoreClock/1000);      // System Tick Initializes,set SysTick 1ms interrupt
	
	Configure_Timer3_PB4();
	ConfigurePA0PA1();
	
	EnablePWM();
	
char str[12];

	for ( pwm_value =1 ; pwm_value <30 ; pwm_value +=1){   // 0~180 degree
		ChangePWMValue(pwm_value);
		printf("%d \n\r", pwm_value);
		Delay(300);
	}
	
	
	button_press = USER_default;
	pwm_value=1;
	
	
	for (;;){
		switch(button_press){
			
			case USER_1:
				printf("PA0 - Change PWM Value PWM value..\n\r");
			if(pwm_value <30){
				pwm_value+=5;
				EnablePWM();
				printf("%d \n\r", pwm_value);
				Delay(1000);
			}else{
				pwm_value=1;
				EnablePWM();
				printf("%d \n\r", pwm_value);
				Delay(1000);
				button_press = USER_default;
			}
			break;
			
			case USER_2:
			printf("PA1 - Set PWM = 15.\n\r");
			pwm_value=15;
			ChangePWMValue(pwm_value);
			EnablePWM();
			printf("%d \n\r", pwm_value);
			Delay(1000);
			button_press = USER_default;
			
			case USER_default:
			default:
						DisablePWM();
			
			
		}
	}
	
	
}