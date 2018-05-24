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

void GPIOC_init(void){
	RCC->AHBENR |= 0x1ul <<19; 
	GPIOC->MODER &= ~(0x0000fffful); //00000000 00000000 11111111 11111111
	GPIOC->MODER |= 0x00005555ul;    //0000*4 01010101 01010101
	
	GPIOC->OTYPER &= ~0x000000fful;//0000*6 1111*2
	
	GPIOC->OSPEEDR &=~0x0000fffful;
	GPIOC->OSPEEDR |=0x00005555ul;
	GPIOC->PUPDR &= ~0x0000fffful;
	
}
	#define ON_ALL_LED 1
	#define OFF_ALL_LED 2
	#define ON_ODD_LED 3
	#define ON_EVEN_LED 4
	#define ON_Bit_LED 5

void LED8_on( int displayMode , int bit){
	uint32_t activeBit;
	
	switch(displayMode){
		case ON_ALL_LED:
			GPIOC->ODR |= 0x000000fful;
		break;
		case OFF_ALL_LED:
			GPIOC->ODR &= ~(0x000000fful);
		break;
		case ON_ODD_LED:
			GPIOC->ODR &= ~(0x000000fful);
			GPIOC->ODR |= ~(0x00000055ul);
		
			break;
		case ON_EVEN_LED:
			GPIOC->ODR &= ~(0x000000fful);
			GPIOC->ODR |= ~(0x000000aaul);
			break;
		case ON_Bit_LED:
			GPIOC->ODR &= ~(0x000000fful);
			activeBit = 0x1ul;
			activeBit = activeBit<<bit;
			GPIOC->ODR |= activeBit;
			break;
	}
}

/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	
	stdout_init();
	
	SysTick_Config(SystemCoreClock/1000);
	
	Button_Init();
	
	GPIOC_init();
	
	for(;;){ //endless loop
	
	printf("LED ON/OFF switch \n\r");
	
	for(int i =0; i<2;i++){
		LED8_on(ON_ALL_LED, 0);
		Delay(1000);
		LED8_on(OFF_ALL_LED,0);
		Delay(1000);
	}
	for(int i =0; i<2;i++){
		LED8_on(ON_ODD_LED, 0);
		Delay(1000);
		LED8_on(ON_EVEN_LED, 0);
		Delay(1000);
	}
	for(int i =0; i<8;i++){
		LED8_on(ON_Bit_LED, i);
		Delay(300);
	}
	for(int i =7; i>=0;i--){
		LED8_on(ON_Bit_LED, i);
		Delay(300);
	}
	
}
	
	
	
	
	
}