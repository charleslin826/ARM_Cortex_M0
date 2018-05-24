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

volatile uint32_t button_press;  //== static variable
#define USER_1  1
#define USER_2  2
#define USER_3  3
#define USER_4  4
#define USER_default  10

void GPIOC_init(void){
	RCC->AHBENR |= 0x1ul <<19; 
	GPIOC->MODER &= ~(0x0000fffful); //~(00000000 00000000 11111111 11111111) // clean
	GPIOC->MODER |= 0x00005555ul;    //0000*4 01010101 01010101 //set to output mode
	
	GPIOC->OTYPER &= ~0x000000fful;// set 1111*2 to pin0~7
	
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
// Init GPIOA PA.5   "the LED on the CHIP"
/*--------------------------------------------------------------------------------
void GPIOA_init(void){ 	//Declaration
	RCC->AHBENR |= 0x1ul <<17;//0x1ul(0x1 unsign long)//if you want to control, need to open clock  (PDF:F072_reference_menu ,P.127)
	
	GPIOA ->MODER &= ~(0x3ul << 5*2); //clear up first   // shift to where it is :"mode 0 > 5 *2bit = 10" (PDF:F072_reference_menu ,P.165)
	GPIOA ->MODER |= 0x1ul << 5*2;// set value =0x1  ==01: General purpose output mode
	
	GPIOA ->OTYPER &= ~(0x1ul << 5*1); // set 1: Output open-drain (type),  (PDF:F072_reference_menu ,P.165)
	
	GPIOA ->OSPEEDR &= ~(0x3ul <<5*2); //set 01: Medium speed, (PDF:F072_reference_menu ,P.166)
	GPIOA ->OSPEEDR |= 0x1ul << 5*2;
	
	GPIOA ->PUPDR &= ~(0x3ul <<5*2 );// set 00: No pull-up, pull-down, (PDF:F072_reference_menu ,P.166)
	
	
}
*/
//GPIOA_PA.6 - PA.8 init   for 3 color LED
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
// Turn on/off LED
/*--------------------------------------------------------------------------------*/
void LED_on(void){  
	GPIOA->ODR |= 0x1ul << 5;  // ODR=  output data register  (PDF:F072_reference_menu ,P.167)
}

void LED_off(void){  
	GPIOA->ODR &= ~(0x1ul << 5); 
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
// Set up PB3, PB5 Interrupt(for EXTI)
/*--------------------------------------------------------------------------------*/
void ConfigurePB3PB5(void){
	RCC->AHBENR |= 0x1ul << 18; //Advanced High Performance Bus (AHB) peripheral clock enable register (RCC_AHBENR)
	GPIOB -> MODER &= ~(0x3ul <<3*2| 0x3ul << 5*2);

	/*config SYSCFG EXTI*/
	RCC->APB2ENR |= 0x4ul;    //set 0x1ul enable SYSCFG clock
	SYSCFG -> EXTICR[0] &= ~(0xful << 3*4);  //SYSCFG external interrupt configuration register 1  //use EXTI3  for PB3
	SYSCFG -> EXTICR[0] |= 0x1ul << 3*4; // coz x001: PB[x] pin (F072-ref_menu p.177)
	
	SYSCFG -> EXTICR[1] &= ~(0xful << 1*4);  //SYSCFG external interrupt configuration register 2?  //use EXTI5  for PB5
	SYSCFG -> EXTICR[1] |= 0x1ul << 1*4; // coz x001: PB[x] pin (F072-ref_menu p.177)
	
	EXTI -> FTSR |= (0x1ul<< 3 | 0x1ul << 5);
	EXTI -> IMR  |= (0x1ul<< 3 | 0x1ul << 5);
	EXTI -> PR   |= (0x1ul<< 3 | 0x1ul << 5);
	
	/*Configure NVIC for external Interrupt */
	NVIC_ClearPendingIRQ(EXTI2_3_IRQn); // EXTI2_3_IRQn == 6
	NVIC_SetPriority(EXTI2_3_IRQn, 0);  // set IRQ and set priority 00
	NVIC_EnableIRQ(EXTI2_3_IRQn);				// enable IRQ
	
	/*Configure NVIC for external Interrupt */
	NVIC_ClearPendingIRQ(EXTI4_15_IRQn); // EXTI4_15_IRQn == 7
	NVIC_SetPriority(EXTI4_15_IRQn, 0);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
	
}


/*--------------------------------------------------------------------------------*/
// Set up EXTI2_3_IRQHandler  (IRQ6 for PB3)
/*--------------------------------------------------------------------------------*/
void EXTI2_3_IRQHandler(void){
	uint32_t PR_value;
	
	if(EXTI -> PR & 0x1ul<<3){ //PB3
		
		PR_value = EXTI->PR;  ///pending register(EXTI_PR) need to be clear, otherwise will occur interrupt request again and again
		
		if(PR_value & 0x1ul<<3){ // PB3 was pressed
				if( (GPIOB->IDR & 0x1ul <<3) ==0 ){
					button_press = USER_3;
					printf("PB3 was pressed. \n\r");
				}else{
					printf("NOISE!! PB3 didn't be pressed.\n\r");
				}
		}else{
		printf("ERROR!!!\n\r");
	}
			EXTI -> PR |= (0x1ul<<3);
	
}
}
/*--------------------------------------------------------------------------------*/
// Set up EXTI4_15_IRQHandler  (IRQ7 for PB5)
/*--------------------------------------------------------------------------------*/
void EXTI4_15_IRQHandler(void){
	uint32_t PR_value;
	
	if(EXTI -> PR & (0x1ul<<5)){ //PB5
	
		if( (GPIOB->IDR & 0x1ul <<5) ==0 ){
					button_press = USER_4;
					printf("PB5 was pressed. \n\r");
				}else{
					printf("NOISE!! PB5 didn't be pressed.\n\r");
				}
		}else{
		printf("ERROR!!!\n\r");
	}
	EXTI -> PR |= (0x1ul<<5);
}
	
/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	uint32_t i; // int i  is also acceptable
		/* Configure the System clock frequency, AHB/APBx prescalers and Flash settings */
  SetSysClock();
	
	stdout_init();              // Initialize USART 2(PA3 to USART2_RX,PA2 to USART2_TX)  
		                                            
	SysTick_Config(SystemCoreClock/1000);      // System Tick Initializes,set SysTick 1ms interrupt
	
	GPIOA_init(); //for 3color LED
	GPIOC_init(); //for button *4


	ConfigurePA0PA1();
	ConfigurePB3PB5();
	button_press = USER_default;  // we set USER_default = 0x0A = 10
	printf("Control LED starting. \n\r");
	
	for(;;){
		switch(button_press){
			case USER_1:
				for(i=0 ; i<30; i++){
					if(button_press != USER_1) 
						//goto case_end;
						break;
					LED8_on(ON_ALL_LED,0);
					Delay(100);
					LED8_on(OFF_ALL_LED,0);
					Delay(100);
				}
				break;
			case USER_2:
				for(i=0 ; i<10; i++){
					if(button_press != USER_2) 
						break;
					LED8_on(ON_ODD_LED, 0);
					Delay(500);
					LED8_on(ON_EVEN_LED, 0);
					Delay(500);
				}
				break;
			case USER_3:
				for(int i=0;i<8;i++){
					if(button_press != USER_3) 
						break;
					colorLED_on(i);
					Delay(300);
				}
				printf("333");
				button_press = USER_default;
				break;
			case USER_4:
				for(int i=0;i<8;i++){
					if(button_press != USER_4) 
						break;
					colorLED_on(i);
					Delay(300);
				}
				uint32_t j;
				for(int i=0;i<8;i++){
					if(button_press != USER_4) 
						break;
					j = rand(); // need to #include <stdlib.h>
					j=j%8;
					colorLED_on(j);		
					Delay(300);
				}				
				printf("444");
				button_press = USER_default;
				break;
			case USER_default:
			default:
				LED_off();
				break;	
		}
	}
	
}