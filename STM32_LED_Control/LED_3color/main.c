/**   Project name : 
*   
--------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
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
//GPIOA_PA.6 - PA.8 init
void GPIOA_init(void)
{
	RCC->AHBENR |= 0x1ul << 17;
	GPIOA->MODER &= ~(0x3ul << 6*2) | (0x3ul << 7*2) | (0x3ul << 8*2);
	GPIOA->MODER |=  (0x1ul << 6*2) | (0x1ul << 7*2) | (0x1ul << 8*2);
	
	GPIOA->OTYPER &= ~(0x7ul << 6);
	
	GPIOA->OSPEEDR &= ~(0x3ul << 6*2) | (0x3ul << 7*2) | (0x3ul << 8*2);
	GPIOA->OSPEEDR |=  (0x1ul << 6*2) | (0x1ul << 7*2) | (0x1ul << 8*2);
	
	GPIOA->PUPDR &= ~(0x3ul << 6*2) | (0x3ul << 7*2) | (0x3ul << 8*2);
	
}

//set up color LED
void colorLED_on(int display_num){
	int color_array[10]={0,1,2,3,4,5,6,7};
	int LED_value;
	
	LED_value = color_array[display_num];
	GPIOA->ODR &= ~(0x7ul << 6);
	GPIOA->ODR |= LED_value <<6;
	
}
/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	uint32_t j;
	stdout_init();
	SysTick_Config(SystemCoreClock/1000);
	GPIOA_init();
	
	for(;;){
		for(int i=0;i<8;i++){
			colorLED_on(i);
			Delay(300);
		}
		
		for(int i=0;i<8;i++){
			j = rand();
			j=j%8;
			colorLED_on(j);		
			Delay(300);
		}
	}
	
	
	
}