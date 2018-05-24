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
		printf("This is LED on/off test");
	}
	
	LED_on();
	
	Delay(1000);
	
	LED_off();
	
	Delay(1000);
	
}