/**   Project name : Car_control
*   
--------------------------------------------------------------------------------------*/

#include <stdio.h>

#include "stm32f0xx.h"          // File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void stdout_init (void);
extern void Delay(uint32_t dlyTicks);

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

/*--------------------------------------------------------------------------------

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
	
	
*/