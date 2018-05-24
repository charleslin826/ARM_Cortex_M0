/**   Project name : Melody _20180413 by charles
use Timer 2 to count rather than "msTicks"(system default counter) _ generate "beat"(temper)
use Timer 3 to generate different "frequency"
we set duration all equel so the volumn will equal (same "big small sound"in chinese )
*   
--------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"          // File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void stdout_init (void);

volatile uint32_t msTicks;         // Counter for millisecond Interval
volatile uint32_t TIM2_Ticks;      // Counter for Timer 2 

#define NUM_KEYS  1                     /* Number of available keys           */

/* Keys for NUCLEO Board */
#define USER    1

#define Melody_basic 1
#define Melody_happy_song 2
#define Melody_little_star 3
#define Melody_little_bee 4

/* Keys for NUCLEO Board */
#define USER    1

volatile uint32_t button_press;  //== static variable
#define USER_1  1
#define USER_2  2
#define USER_3  3
#define USER_4  4
#define USER_default  10

	#define ON_ALL_LED 1
	#define OFF_ALL_LED 2
	#define ON_ODD_LED 3
	#define ON_EVEN_LED 4
	#define ON_Bit_LED 5


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
// Set up PA0 only Interrupt    @0416 test
/*--------------------------------------------------------------------------------
void ConfigurePA0(void){  
	RCC->AHBENR |= 0x1ul << 17;
	GPIOA -> MODER &= ~(0x3ul );

	/*config SYSCFG EXTI
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;    //set 0x1ul enable SYSCFG clock
	SYSCFG -> EXTICR[0] &= ~(0xful);  //SYSCFG external interrupt configuration register 1
	
	EXTI -> FTSR |= (0x1ul ); //Falling trigger selection refister(FTSR)
	EXTI -> IMR  |= (0x1ul ); //Interrupt mask register(IMR)
	EXTI -> PR   |= (0x1ul ); //Pending register(PR)
	
	NVIC_ClearPendingIRQ(EXTI0_1_IRQn); // == 5 (IRQ5)
	NVIC_SetPriority(EXTI0_1_IRQn, 3);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	}*/
/*--------------------------------------------------------------------------------*/
// Set up EXTI0_1_IRQHandler (IRQ5 for PA0 only) 0416 test
/*--------------------------------------------------------------------------------
void EXTI0_1_IRQHandler(void){
	uint32_t PR_value;
	
	if(EXTI -> PR & 0x1ul){  // PA0
		
		PR_value = EXTI->PR;  ///pending register(EXTI_PR) need to be clear, otherwise will occur interrupt request again and again
		
		if(PR_value & 0x1ul){ // PA.0 was pressed
				if( (GPIOA->IDR & 0x1ul) ==0 ){
					button_press = USER_1;
					printf("This is IRQ5 \n\r");
				}else{
					printf("NOISE!! PA0 didn't be pressed.\n\r");
				}
		}else{
		printf("ERROR!!!\n\r");
	}
	EXTI -> PR |= (0x1ul|0x1ul<<1);	
	
}
	
	}*/
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

/*-----------------------------------------*/
//Configure Timer 2    @0413
/*-----------------------------------------*/
void Configure_Timer2(void){
	RCC -> APB1ENR |= 0x1ul;  //(ref_menu PDF p.130)
	
	TIM2->PSC =47;  //prescaler  (use this to divide the SYS_CLK(48HMz))
	TIM2->ARR =999; // set value to 1000 (but need to -1) 
	TIM2->CNT = 0;  //set counter to 0 (default value)
	TIM2->CR1 |= 0x1ul;   //set bit 0 . at same time,  'bit4' no need change coz we are use normal upcounter
	TIM2->DIER |= 0x1ul;  // (ref_menu PDF p.426) 

	NVIC_ClearPendingIRQ(TIM2_IRQn); // refer "stm32f072xb.h" Line 111
	NVIC_SetPriority(TIM2_IRQn,1);
	NVIC_EnableIRQ(TIM2_IRQn);
	
}
/*-----------------------------------------*/
//Timer 2 int  @0413
/*-----------------------------------------*/
void TIM2_IRQHandler(void){  //refer "startup_stm32f072xb.s" Line 109
	TIM2_Ticks++;
	TIM2->SR &= ~ 0x1ul;  //  Update interrupt flag  (ref_menu PDF p.427 _status register )
	
}

/*--------------------------------------------------------------------------------*/
// TIM2Delay: delay a number of Systicks  @0413 (copy and rewrite from up one "Delay")
/*--------------------------------------------------------------------------------*/
void TIM2Delay(uint32_t dlyTicks){
	uint32_t currentTicks;
	
	currentTicks = TIM2_Ticks;
	while( (TIM2_Ticks - currentTicks) < dlyTicks ){
		  __NOP();
	}
}
/*--------------------------------------------------------------------------------*/
// Init GPIOA PA.5   "the LED on the CHIP"
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
// Turn on/off LED @0413
/*--------------------------------------------------------------------------------*/
void LED_on(void){  
	GPIOA->ODR |= 0x1ul << 5;  // ODR=  output data register  (PDF:F072_reference_menu ,P.167)
}

void LED_off(void){  
	GPIOA->ODR &= ~(0x1ul << 5); 
}

/*--------------------------------------------------------------------------------*/
// Configure PB.4 for Timer 3  @0413
/*--------------------------------------------------------------------------------*/
void Configure_PB4_TIMer3(void){
	RCC->AHBENR |= 0x1ul << 18;   //Enable PortB 
	GPIOB->MODER &= ~ (0x3ul << 4*2);// clear PB.4 setting
	GPIOB->MODER |= 0x2ul << 4*2;// Enable PB.4 with Alternate function mode
	
	GPIOB->AFR[0] &= ~ (0xful << 4*4);// clear Alternate function low register
	GPIOB->AFR[0] |= 0x1ul << 4*4;// Enable Alternate function low register
	
	RCC->APB1ENR |= 0x2ul;
	
	TIM3->PSC = 479; //prescaler  (use this to divide the SYS_CLK(48HMz))
	TIM3->ARR = 99; // set value to 100 (but need to -1) 
	TIM3->CCR1= 50; //set to ARR's half   this is another comparator   //CCR1="capture/compare register 1 "
	TIM3->CNT = 0;  //set counter to 0 (default value)
	TIM3->CCMR1 |= 0x6ul << 4 ; //control PWM with _ mode 
	TIM3->CR1 |= 0x1ul;//set  control register 1's "bit 0" .
	
	
}

/*--------------------------------------------------------------------------------*/
// Select tone  @0413
/*--------------------------------------------------------------------------------*/
void SelectTone(uint32_t tone_num){
	uint32_t ARR_value;
	//program ARR register
	// program ARR register
	uint32_t ToneArr[]={0,764,681,607,573,510,454,405,382,340,303,286,255,227,202,191,170,152,143,128,114,101};
											//C   D   E   F   G   H   A   B   C^  D^...
	uint32_t ToneHz[]={0,131,147,165,175,196,220,247,261,294,329,349,392,440,493,523,587,659,698,784,880,988};

	ARR_value = ToneArr[tone_num];
	TIM3->ARR = ARR_value;
	TIM3->CCR1= ARR_value/2;
	TIM3->CNT = 0;
	
}
/*--------------------------------------------------------------------------------*/
// Enable PWM  @0413
/*--------------------------------------------------------------------------------*/
void EnablePWM(void){
	TIM3->CNT = 0;
	TIM3->CR1 |= 0x1ul;
	TIM3->CCER |= 0x1ul;
}
/*--------------------------------------------------------------------------------*/
// Disable PWM  @0413
/*--------------------------------------------------------------------------------*/
void DisablePWM(void){
	TIM3->CCER &= ~0x1ul; //turn off
	TIM3->CR1 &= ~0x1ul;  //turn off
	
}

/*--------------------------------------------------------------------------------*/
// Play Melody @0413
/*--------------------------------------------------------------------------------*/
#define beatTime 100
void PlayMelody ( uint32_t song_num){ 
	
	//Basic tone
uint8_t note_basic[]={1,2,3,4,5,6,7,1+7,2+7,3+7,4+7,5+7,6+7,7+7,1+14,2+14,3+14,4+14,5+14,6+14,7+14}; 			
	// Happy Birthday Song
uint8_t note_happy_song[]={5,5,6,5,1+7,7,5,5,6,5,2+7,1+7,5,5,5+7,3+7,1+7,7,6,4+7,4+7,3+7,1+7,2+7,1+7 };
		
uint16_t beat_happy_song[]={250,250,500,500,500,1000,250,250,500,500,500,1000,250,250,500,500,500,500,1500,
														250,250,500,500,500,1500};

	// Little Star Song	
uint8_t note_little_star[]={1,1,5,5,6,6,5,4,4,3,3,2,2,1,5,5,4,4,3,3,2,5,5,4,4,3,3,2,1,1,5,5,6,6,5,4,4,3,3,2,2,1};
		
uint16_t beat_little_star[]={500,500,500,500,500,500,1000,500,500,500,500,500,500,1000,500,500,500,500,500,500,
														1000,500,500,500,500,500,500,1000,500,500,500,500,500,500,1000,500,500,500,500,500,500,1000};
	
	//Little Bee Song	
uint8_t  note_little_bee[]={5,3,3,4,2,2,1,2,3,4,5,5,5,5,3,3,4,2,2,1,3,5,5,3,
														2,2,2,2,2,3,4,3,3,3,3,3,4,5,5,3,3,4,2,2,1,3,5,5,1};	
uint16_t beat_little_bee[]={250,250,500,250,250,500,250,250,250,250,250,250,500,250,250,500,250,250,500,250,250,250,250,1000,
														250,250,250,250,250,250,500,250,250,250,250,250,250,500,250,250,500,250,250,500,250,250,250,250,1000};	

	uint32_t i, tone, beat, soundStop;
	
	soundStop = beatTime *1.2; // =110
	EnablePWM();
	
	switch(song_num){
	
		case Melody_basic:
		beat=500;
		for(i=0;i< sizeof(note_basic); i++){
		tone = note_basic[i];
		SelectTone(tone);
		TIM2Delay(beat);
		DisablePWM();
		TIM2Delay(soundStop);
		EnablePWM();		
	}
		break;
	
		case Melody_happy_song:
		for(i=0;i< sizeof(note_happy_song); i++){
		tone = note_happy_song[i];
		tone = tone+7;
		SelectTone(tone);
		beat = beat_happy_song[i];
		TIM2Delay(beat);
		DisablePWM();
		TIM2Delay(soundStop);
		EnablePWM();		
	}
		break;
	
		case Melody_little_star:
		for(i=0;i< sizeof(note_little_star); i++){
		tone = note_little_star[i];
		tone = tone+7;
		SelectTone(tone);
		beat = beat_little_star[i];
		TIM2Delay(beat);
		DisablePWM();
		TIM2Delay(soundStop);
		EnablePWM();		
			
	}
		break;
	
		case Melody_little_bee:
		for(i=0;i< sizeof(note_little_bee); i++){
		tone = note_little_bee[i];
		tone = tone+7;
		SelectTone(tone);
		beat = beat_little_bee[i];
		TIM2Delay(beat);
		DisablePWM();
		TIM2Delay(soundStop);
		EnablePWM();		
	}
	
		break;
}
	DisablePWM();
	
}	
/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	
		/* Configure the System clock frequency, AHB/APBx prescalers and Flash settings */
  SetSysClock();
	
	stdout_init();              // Initialize USART 2(PA3 to USART2_RX,PA2 to USART2_TX)  
		                                            
	SysTick_Config(SystemCoreClock/1000);      // System Tick Initializes,set SysTick 1ms interrupt
	
	GPIOA_init();
	Configure_Timer2();
	
	Configure_PB4_TIMer3();
	
/*	PlayMelody(Melody_basic);
	TIM2Delay(1500);
	
	PlayMelody(Melody_happy_song);
	TIM2Delay(1500);
	
	PlayMelody(Melody_little_star);
	TIM2Delay(1500);
	
	PlayMelody(Melody_little_bee);
	TIM2Delay(1500);*/
	
	ConfigurePA0PA1();
	//ConfigurePA0();
	ConfigurePB3PB5();
	button_press = USER_default;  // we set USER_default = 0x0A = 10
	printf("Melody Control starting. \n\r");
	
	uint32_t i; // int i  is also acceptable
	
	
	for(;;){
		switch(button_press){
			case USER_1:
				printf("Melody_basic");
				PlayMelody(Melody_basic);
				button_press = USER_default; 
				break;
			case USER_2:
				printf("Melody_happy_song");
				PlayMelody(Melody_happy_song);
				button_press = USER_default; 
				break;
			case USER_3:
				printf("Melody_little_star");
				PlayMelody(Melody_little_star);			
				button_press = USER_default;
				break;
			case USER_4:
				printf("Melody_little_bee");
				PlayMelody(Melody_little_bee);
				button_press = USER_default;
				break;
			default:
				LED_on();
				Delay(1000);
				LED_off();
				Delay(1000);
				break;	
		}
	}
	
}