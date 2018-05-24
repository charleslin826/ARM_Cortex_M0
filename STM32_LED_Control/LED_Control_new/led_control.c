/*   Project name : LED_control
*    Julia's project 1
*    Control LED display */
/*---------------------------------------------------------------------------------------*/

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
		

/*----------------------------------------------------------------------------------------------*/
// GPIOA_Init
// PA.5 to control green LED(LD2)
// set PA.5 is output pin with push-pull output, medium speed, no internal pull-up & pull down
/*----------------------------------------------------------------------------------------------*/

void GPIOC_Init (void) {

  RCC->AHBENR |=  (1ul << 19);                  /* Enable GPIOA>17 clock    GPIOC>19      */  

  /* Configure LED (PA.5) pins as push-pull output, No pull-up, pull-down */
  GPIOC->MODER   &= ~((3ul << 2*5));             // set GPIOA pin 5 as output pin
  GPIOC->MODER   |=  ((1ul << 2*5));
  
	GPIOC->OTYPER  &= ~((1ul <<   5));             // set pin 5 as O/P push-pull 
  
	GPIOC->OSPEEDR &= ~((3ul << 2*5));             // set pin 5 as medium speed O/P 
  GPIOC->OSPEEDR |=  ((1ul << 2*5));
  
	GPIOC->PUPDR   &= ~((3ul << 2*5));             // set pin 5 as no pull-up & pull-down
  
}

/*--------------------------------------------------------------------------------*/
// LED_on();    Turn on LED
/*--------------------------------------------------------------------------------*/
void LED_on(void){                               // set pin 5 = 1 to turn on LED
	
	GPIOC->ODR |= ((1ul << 5));	

}

/*--------------------------------------------------------------------------------*/
// LED_off();    Turn off LED
/*--------------------------------------------------------------------------------*/
void LED_off(void){
	GPIOC->ODR &= ~(1ul << 5);              // set pin 5 = 0 to turn off LED
	
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
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	
	GPIOC_Init();               // Initialize PA.5 to control LED
	Button_Init();              // Initialize PC.13 to detect USER button 
	
	stdout_init();              // Initialize USART 2(PA3 to USART2_RX,PA2 to USART2_TX)  
			
	SysTick_Config(SystemCoreClock/1000);       // System Tick Initializes,set up SysTick 1ms interrupt	
	
	
	for(;;){
				
		printf("This is LED on/off display. \n\r");
		printf("Turn on LED for 3 sec. \n\r");
		LED_on();
		Delay(2000);
		
		printf("Turn off LED for 2 sec. \n\r\n\r");
		LED_off();
		Delay(2000);
		
		printf("Please press blue USER button to re-start. \n\r");              
		do{                                    // Wait while holding USER button
			LED_on();
		  Delay(300);
		  LED_off();
		  Delay(300);
	  
	  }while (Button_GetState()== 0); 
		
		printf("Blue USER button was pressed. \n\r\n\r");
	}
	
}