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
// SysTick Interrupt Handler    @Override 
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

  RCC->AHBENR |=  (1ul << 19);                  // Enable GPIOC clock      //RCC = Reset and clock control  
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
// Init GPIOA PA.5
/*--------------------------------------------------------------------------------*/
void GPIOA_init(void){ 	//Declaration
	RCC->AHBENR |= 0x1ul <<17;//0x1ul(0x1 unsign long)//if you want to control, need to open clock  (PDF:F072_reference_menu ,P.127)
	
	GPIOA ->MODER &= ~(0x3ul << 5*2); //clear up first   // shift to where it is :"mode 0 > 5 *2bit = 10" (PDF:F072_reference_menu ,P.165)
	GPIOA ->MODER |= 0x1ul << 5*2;// set value =0x1  ==01: General purpose output mode
	
	GPIOA ->OTYPER &= ~(0x1ul << 5*1); // set 1: Output open-drain (type),  (PDF:F072_reference_menu ,P.165)
	
	GPIOA ->OSPEEDR &= ~(0x3ul <<5*2); //set 01: Medium speed, (PDF:F072_reference_menu ,P.166)
	GPIOA ->OSPEEDR |= 0x1ul << 5*2;
	
	GPIOA ->PUPDR &= ~(0x3ul <<5*2 );// set 00: No pull-up, pull-down, (PDF:F072_reference_menu ,P.166)
	
	
}
/*--------------------------------------------------------------------------------*/
// Turn on/off LED
/*--------------------------------------------------------------------------------*/
void LED_on(void){  
	GPIOA->ODR |= 0x1ul << 5*1;  // ODR=  output data register  (PDF:F072_reference_menu ,P.167)
}

void LED_off(void){  
	GPIOA->ODR &= ~(0x1ul << 5*1); 
}
/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	GPIOA_init();
	
	stdout_init();
	
	SysTick_Config(SystemCoreClock/1000);  // set up system timer
	
	for(;;){   // infinite loop  == while
		printf("This is LED on/off test\n\r");
	
	
	LED_on();
	
	Delay(1000);
	
	LED_off();
	
	Delay(1000);
	}
}