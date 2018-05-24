/**   Project name : Car_control
*   
--------------------------------------------------------------------------------------*/

#include <stdio.h>

#include "stm32f0xx.h"          // File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void stdout_init (void);

volatile uint32_t msTicks;         // Counter for millisecond Interval
uint8_t car_start_flag;
#define NUM_KEYS  1                     /* Number of available keys           */

/* Keys for NUCLEO Board */
#define USER    1
/*
Car_Direction(dir_cmd)
dir_cmd = 0 Stop
dir_cmd = 1 Forward
dir_cmd = 2 Turn R
dir_cmd = 3 Turn left
dir_cmd = 3 Backward
*/
#define CarDirection_Stop     0
#define CarDirection_Forward  1
#define CarDirection_Right    2
#define CarDirection_Left     3
#define CarDirection_Backward 4
#define CarDirection_forward_right 5
#define CarDirection_forward_left 6
#define CarDirection_backward_right 7
#define CarDirection_backward_left 8
#define CarDirection_cycle_right 9
#define CarDirection_cycle_left 10

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
//GPIOC_Init_Car @0420
//PC.0-3 = IN.1-4
//set PC.0-3 as output pin with push-pull output, medium speedm no internal pull-up & pull-down
/*--------------------------------------------------------------------------------*/
void GPIOC_Init_Car(void){
  RCC->AHBENR |= 0x1ul << 19; // Enable GPIOC clock

	//set PC.0-3 as output pin 
  GPIOC->MODER &= ~0xfful;
  GPIOC->MODER |= 0x55ul;

	//set as push-pull output
  GPIOC->OTYPER &= ~0xful;

	//set as medium speedm
  GPIOC->OSPEEDR &= ~0xfful;
  GPIOC->OSPEEDR |= 0x55ul;
	//set as no pull-up & pull-down
  GPIOC->PUPDR &= ~0xfful;
}
/*
Car_Direction(dir_cmd)
dir_cmd = 0 Stop
dir_cmd = 1 Forward
dir_cmd = 2 Turn R
dir_cmd = 3 Turn left
dir_cmd = 3 Backward

#define CarDirection_Stop     0
#define CarDirection_Forward  1
#define CarDirection_Right    2
#define CarDirection_Left     3
#define CarDirection_Backward 4
*/
void Car_Direction(uint8_t dir_cmd)
{
	switch(dir_cmd){
	case CarDirection_Stop:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
		break;
	case CarDirection_Forward:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
	GPIOC->ODR|=0x1ul|0x1ul<<2;      //1010
	//car_start_flag=1;
		break;
	case CarDirection_Left:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);  
	GPIOC->ODR|=0x1ul<<2;            //0010
		break;
	case CarDirection_Right:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
	GPIOC->ODR|=0x1ul;               //1000
		break;
	case CarDirection_Backward:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
	GPIOC->ODR|=0x1ul<<1|0x1ul<<3;   //0101
	  break;
	case CarDirection_forward_right:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
	GPIOC->ODR|=0x1ul;               //1000
	Delay(500);
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
	GPIOC->ODR|=0x1ul|0x1ul<<2;      //1010
	Delay(1000);
	  break;
	case CarDirection_forward_left:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);  
	GPIOC->ODR|=0x1ul<<2;            //0010
	Delay(500);
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
	GPIOC->ODR|=0x1ul|0x1ul<<2;      //1010
	Delay(1000);
	  break;
	case CarDirection_backward_right:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);  
	GPIOC->ODR|=0x1ul<<2;            //0010
	Delay(500);
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
	GPIOC->ODR|=0x1ul<<1|0x1ul<<3;   //0101
	Delay(1000);
	  break;
	case CarDirection_backward_left:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);  
	GPIOC->ODR|=0x1ul<<2;            //0010
	Delay(500);
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
	GPIOC->ODR|=0x1ul<<1|0x1ul<<3;   //0101
	Delay(1000);
	  break;
	case CarDirection_cycle_right:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
	GPIOC->ODR|=0x1ul|0x1ul<<3;      //1001
	Delay(1000);
	  break;
	case CarDirection_cycle_left:
	GPIOC->ODR&=~(0x3ul|0x3ul<<2);
	GPIOC->ODR|=0x1ul<<1|0x1ul<<2;   //0110
	Delay(1000);
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
	
	Button_Init();
	GPIOC_Init_Car();
	
	for(;;){
	
	printf("Car Start.\r\n\r\n");
	
	while(Button_GetState()==0); //wait user command
	printf("Forward.\r\n\r\n");
	Car_Direction(CarDirection_Forward);
	Delay(1000);
	
	
	while(Button_GetState()==0); //wait user command
	printf("Right.\r\n\r\n");
	Car_Direction(CarDirection_Right);
	Delay(1000);
	
	while(Button_GetState()==0); //wait user command
	printf("Left.\r\n\r\n");
	Car_Direction(CarDirection_Left);
	Delay(1000);
	
	while(Button_GetState()==0); //wait user command
	printf("Backward.\r\n\r\n");
	Car_Direction(CarDirection_Backward);
	Delay(1000);
	
	while(Button_GetState()==0); //wait user command
	printf("Stop.\r\n\r\n");
	Car_Direction(CarDirection_Stop);
	Delay(1000);

	while(Button_GetState()==0); //wait user command
	printf("CarDirection_forward_right.\r\n\r\n");
	Car_Direction(CarDirection_forward_right);
	Delay(1000);
		
	while(Button_GetState()==0); //wait user command
	printf("CarDirection_forward_left.\r\n\r\n");
	Car_Direction(CarDirection_forward_left);
	Delay(1000);
	
	while(Button_GetState()==0); //wait user command
	printf("CarDirection_backward_right.\r\n\r\n");
	Car_Direction(CarDirection_backward_right);
	Delay(1000);
		
	while(Button_GetState()==0); //wait user command
	printf("CarDirection_backward_left.\r\n\r\n");
	Car_Direction(CarDirection_backward_left);
	Delay(1000);

	while(Button_GetState()==0); //wait user command
	printf("Stop.\r\n\r\n");
	Car_Direction(CarDirection_Stop);
	Delay(1000);
	
	while(Button_GetState()==0); //wait user command
	printf("CarDirection_cycle_right.\r\n\r\n");
	Car_Direction(CarDirection_cycle_right);
	Delay(1000);
		
	while(Button_GetState()==0); //wait user command
	printf("CarDirection_cycle_left.\r\n\r\n");
	Car_Direction(CarDirection_cycle_left);
	Delay(1000);
		
}
	
	
}