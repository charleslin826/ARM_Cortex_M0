/**   Project name : 
*   
--------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "stm32f0xx.h"          // File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void stdout_init (void);
extern void GPIOC_Init_Car(void);
extern void Car_Direction(uint8_t dir_cmd);
extern void LCD_main();
extern void LCD_Clear();
extern void LCD_printstring( char *, uint8_t );
extern void melody_main (void);
extern void PlayMelody ( uint32_t song_num);

volatile uint32_t msTicks;         // Counter for millisecond Interval

#define NUM_KEYS  1                     /* Number of available keys           */

/* Keys for NUCLEO Board */
#define USER    1

//BT data definition @0425

volatile uint8_t CMD_mode, Tx_complete;
volatile char	AT_CMD_char;

#define Melody_basic        1
#define Melody_happy_song   2
#define Melody_little_star  3
#define Melody_little_bee   4

volatile uint8_t send , CMD_length , String_length;

const char nAT_1[]="AT\r\n";
const char nATReset_2[]="AT+RESET\r\n";
const char nATVersion_3[]="AT+VERSION?\r\n";
const char nATAddr_5[]="AT+ADDR?\r\n";
const char nATName_6[]="AT+NAME?\r\n";
char nATNameSET_6[]="AT+NAME=";
const char nATUart_13[]="AT+UART?\r\n";
const char nATUartSET_13[]="AT+UART=19200,0,0\r\n";

char BTName_ORI[]="HC-05\r\n";
char BTName[]="Charles006\r\n";

char string1[] = "This is USART1 Test.\n\r";
char string2[] = "Hi this is M0 test.\n\r";
#define buffer_max 5
volatile char receive_buffer[buffer_max];
volatile uint8_t receive_ptr;
volatile char BT_receive_cmd;

const char msg_LED_ON[] = "LED ON"; 
const char msg_LED_OFF[] = "LED OFF"; 
const char msg_Basic_Song[] = "1.Basic Melody"; 
const char msg_Happy_Song[] = "2.Happy Song"; 
const char msg_Star_Song[] = "3.Little Star Song"; 
const char msg_Bee_Song[] = "4.Little Bee Song"; 
const char msg_ERROR[] = "Wrong Command"; 
const char msg_Button[] = "Press Button"; 
const char msg_CAR_Forward[] ="Car Forward";
const char msg_CAR_Left[] ="Car Left";
const char msg_CAR_Right[] ="Car Right";
const char msg_CAR_Backward[] ="Car Backward";
const char msg_CAR_Stop[] ="Car Stop";
const char msg_START[]="BT START !"; 

#define LED_ON       'o'
#define LED_OFF      'n'
#define CAR_FORWARD  'f'
#define CAR_LEFT     'l'
#define CAR_RIGHT    'r'
#define CAR_BACK     'b'
#define CAR_STOP     'p'
#define CAR_FRIGHT    '5'
#define CAR_FLEFT     '6'
#define CAR_BACKR			'7'
#define CAR_BACKL     '8'
#define CAR_CYCLER 		'9'
#define CAR_CYCLEL 		'T'
#define Melody_1     '1'
#define Melody_2     '2'
#define Melody_3     '3'
#define Melody_4     '4'
#define END_CMD      'e'



/*
#define CarDirection_Stop       0
#define CarDirection_Forward    1
#define CarDirection_Right      2
#define CarDirection_Left       3
#define CarDirection_Backward   4			
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
// Init GPIOA PA.5 to control green LED(LD2)     @0425
// set PA.5 is output pin with push-pull output, medium speed , no pull-up & pull-down
// PA.4 control Wakeup pin (data(0)/CMD(1) switch)
/*--------------------------------------------------------------------------------*/
void GPIOA_init(void){ 	//Declaration
	RCC->AHBENR |= 0x1ul <<17;//0x1ul(0x1 unsign long)//if you want to control, need to open clock  (PDF:F072_reference_menu ,P.127)
	
	GPIOA ->MODER &= ~((0x3ul << 5*2)|(3ul << 2*4 )); //clear up first   // shift to where it is :"mode 0 > 5 *2bit = 10" (PDF:F072_reference_menu ,P.165)
	GPIOA ->MODER |= ((0x1ul << 5*2)|(1ul << 2*4 ));// set value =0x1  ==01: General purpose output mode
	
	GPIOA ->OTYPER &= ~((0x1ul << 5)|(1ul << 4 )); // set 1: Output open-drain (type),  (PDF:F072_reference_menu ,P.165)
	
	GPIOA ->OSPEEDR &= ~((0x3ul << 5*2)|(3ul << 2*4 )); //set 01: Medium speed, (PDF:F072_reference_menu ,P.166)
	GPIOA ->OSPEEDR |= ((0x1ul << 5*2)|(1ul << 2*4 ));
	
	GPIOA ->PUPDR &= ~((0x3ul <<5*2 )|(3ul << 2*4 ));// set 00: No pull-up, pull-down, (PDF:F072_reference_menu ,P.166)
	
	
}
/*--------------------------------------------------------------------------------*/
// Turn on/off LED @0425
/*--------------------------------------------------------------------------------*/
void LED_on(void){  
	GPIOA->ODR |= 0x1ul << 5*1;  // ODR=  output data register  (PDF:F072_reference_menu ,P.167)
}

void LED_off(void){  
	GPIOA->ODR &= ~(0x1ul << 5*1); 

}

/*--------------------------------------------------------------------------------*/
// CMD on/off LED for PA.4 @0425  set pin4 =1 to turn on/off Command mode
/*--------------------------------------------------------------------------------*/
void CMD_on(void){  
	GPIOA->ODR |= 0x1ul << 4;  // ODR=  output data register  (PDF:F072_reference_menu ,P.167)
	CMD_mode=1;
}

void CMD_off(void){  
	GPIOA->ODR &= ~(0x1ul << 4); 
	CMD_mode=0;
}
/*--------------------------------------------------------------------------------*/
// Display string on LCD
/*--------------------------------------------------------------------------------*/
void Display_string( char * ptr){
	uint8_t char_num;
	
	LCD_Clear();
	char_num = strlen(ptr);
	LCD_printstring(ptr, char_num);


}
/*--------------------------------------------------------------------------------*/
// Config PA.9 10 for usart 1 @0423
/*--------------------------------------------------------------------------------*/
void Configure_PA9PA10_USART1(void){

	RCC->AHBENR |= (0x1ul << 17);
	GPIOA->MODER &= ~(0x3ul << 9*2 | 0x3ul << 10*2);
	GPIOA->MODER |= 0x2ul << 9*2 | 0x2ul << 10*2;

	GPIOA->AFR[1] &= ~(0xful << 1*4 | 0xful << 2*4);// 9-8 = 1 so PA9.10 is AFR's no.1 & no.2
	GPIOA->AFR[1] |= (0x1ul << 1*4 | 0x1ul << 2*4); //set AF1 for USART1_ TX & RX
	
}

/*--------------------------------------------------------------------------------*/
// Config usart 1 @0423
/*--------------------------------------------------------------------------------*/
void Configure_usart1(void){
		
	RCC->APB2ENR |= 0x1ul << 14;
	USART1 -> BRR = 48000000/19200; //0x9C4 = 19200  // 0x1388 = 9600  [baud rate] 48000000/9600

	USART1 -> CR3 |= 0x1ul << 12;// shundown / ignore the error of overrun   
	//Bit 12 OVRDIS: Overrun Disable (PDF p.731)

	USART1 -> CR1 |= 0x1ul << 5 | 0x1ul << 2 | 0x1ul; //5_1: A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USARTx_ISR 
//2_1: Receiver is enabled and begins searching for a start bit // 0_1: USART enabled

	USART1 -> ICR |= 0x1ul << 6;
	
	USART1 -> CR1 |= 0x1ul << 6 | 0x1ul << 3; //6_1: A USART interrupt is generated whenever TC=1 in the USARTx_ISR register 
	//3_1: Transmitter is enabled
 	
	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn,3);
	NVIC_EnableIRQ(USART1_IRQn);
	
	 
}
/*--------------------------------------------------------------------------------*/
// USART 1 interrupt
/*--------------------------------------------------------------------------------*/
void USART1_IRQHandler(void){

	uint8_t chartoreceive;
	
	if(CMD_mode ==1){
	
		if ( (USART1->ISR & USART_ISR_TC) == USART_ISR_TC) { // 傳輸結束(End of transmission flags, TC) Enable Bit(TE、RE)
			USART1->ICR |= USART_ICR_TCCF;
			if( send == CMD_length)
			{
				Tx_complete =1;
			}
		}
		
		if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE ){  // 接收緩衝區滿(Receive buffer full, RXNE)
			
			chartoreceive = (uint8_t) USART1->RDR;
			USART2->TDR = chartoreceive;  // transmitte
			receive_buffer[receive_ptr] = chartoreceive;
			receive_ptr++;
			if(receive_ptr >= buffer_max)
				receive_ptr=0;
			
		}
	}else{
			
		if ( (USART1->ISR & USART_ISR_TC) == USART_ISR_TC) { // 傳輸結束(End of transmission flags, TC) Enable Bit(TE、RE)
			USART1->ICR |= USART_ICR_TCCF;
			if( send == CMD_length)
			{
				Tx_complete =1;
			}
		}
		
		if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE ){  // 接收緩衝區滿(Receive buffer full, RXNE)
			
			chartoreceive = (uint8_t) USART1->RDR;
			USART2->TDR = chartoreceive;  // transmitte
			
			switch(chartoreceive){
				
				case LED_ON:				
				case 'O':
						BT_receive_cmd = LED_ON;
						LED_on();
						Display_string( (char * ) msg_LED_ON);
						break;
				case LED_OFF:				
				case 'N':
						BT_receive_cmd = LED_OFF;
						LED_off();
						Display_string( (char * ) msg_LED_OFF);
						break;
				case CAR_FORWARD:
				case 'F':
						BT_receive_cmd = CAR_FORWARD;
						Car_Direction(CarDirection_Forward);
						Display_string( (char * ) msg_CAR_Forward);
						break;
				case CAR_LEFT:
				case 'L':
						BT_receive_cmd = CAR_LEFT;
						Car_Direction(CarDirection_Left);
						Display_string( (char * ) msg_CAR_Left);
						Delay(1000);
						BT_receive_cmd = CAR_FORWARD;
						Car_Direction(CarDirection_Forward);
						Display_string( (char * ) msg_CAR_Forward);
						break;
				case CAR_RIGHT:
				case 'R':
						BT_receive_cmd = CAR_RIGHT;
						Car_Direction(CarDirection_Right);
						Display_string( (char * ) msg_CAR_Right);
						Delay(1000);
						BT_receive_cmd = CAR_FORWARD;
						Car_Direction(CarDirection_Forward);
						Display_string( (char * ) msg_CAR_Forward);
						break;
				case CAR_BACK:
				case 'B':
						BT_receive_cmd = CAR_BACK;
						Car_Direction(CarDirection_Backward);
						Display_string( (char * ) msg_CAR_Backward);
						break;
				case CarDirection_forward_right:
				case '5':
						BT_receive_cmd = CAR_BACKR;
						Car_Direction(CarDirection_forward_right);
						Delay(1000);
						break;
				case CarDirection_forward_left:
				case '6':
						BT_receive_cmd = CAR_FLEFT;
						Car_Direction(CarDirection_forward_left);
						break;
				case CarDirection_backward_right:
				case '7':
						BT_receive_cmd = CAR_BACKR;
						Car_Direction(CarDirection_backward_right);
						break;
				case CarDirection_backward_left:
				case '8':
						BT_receive_cmd = CAR_BACKL;
						Car_Direction(CarDirection_backward_left);
						break;
				case CarDirection_cycle_right:
				case '9':
						BT_receive_cmd = CAR_CYCLER;
						Car_Direction(CarDirection_cycle_right);
						break;
				case CarDirection_cycle_left:
				case 'T':
						BT_receive_cmd = CarDirection_cycle_left;
						Car_Direction(CarDirection_cycle_left);
						break;
				case CAR_STOP:
				case 'P'|'S'|'s':
						BT_receive_cmd = CAR_STOP;
						Car_Direction(CarDirection_Stop);
						Display_string( (char * ) msg_CAR_Stop);
						break;
				case Melody_1:
					
						BT_receive_cmd = Melody_1;
						Display_string( (char * ) msg_Basic_Song);
						break;
				case Melody_2:
					
						BT_receive_cmd = Melody_2;
						Display_string( (char * ) msg_Happy_Song);
						break;
				case Melody_3:
						BT_receive_cmd = Melody_3;
						Display_string( (char * ) msg_Star_Song);
						break;
				case Melody_4:
						BT_receive_cmd = Melody_4;
						Display_string( (char * ) msg_Bee_Song);
						break;
				default:
						BT_receive_cmd = END_CMD;
						break;
			}
		}
	}

}
/*--------------------------------------------------------------------------------*/
// Send AT CMD to BT   @0425 [HC-05 BT module munu PDF p.9~10]
/*--------------------------------------------------------------------------------*/
void Send_ATCMD(char * ATCMD_ptr){
	Tx_complete=0;
	send=0;
	CMD_length;
	CMD_length = strlen(ATCMD_ptr);
	
	do{
		if(send == CMD_length){
			
			Tx_complete = 1 ;
		
		}else if(USART1 -> ISR & USART_ISR_TXE){  //check if tranfer buffer is empty yet   傳送緩衝區空(Transmit buffer empty, TXE).
			USART1->TDR = * (ATCMD_ptr);
			ATCMD_ptr++;
			send++;
		}

	}while(Tx_complete == 0);  // transfer until finished

}

/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
	
		/* Configure the System clock frequency, AHB/APBx prescalers and Flash settings */
  SetSysClock();
	
	stdout_init();              // Initialize USART 2(PA3 to USART2_RX,PA2 to USART2_TX)  
		                                            
	SysTick_Config(SystemCoreClock/1000);      // System Tick Initializes,set SysTick 1ms interrupt
	//level up the interrupt's priority otherwise it will wait forever..
	NVIC_SetPriority(SysTick_IRQn, 1);
	
	Button_Init(); // Init user button (blue)
	GPIOA_init();
	
	Configure_PA9PA10_USART1();
	Configure_usart1();
	
	GPIOC_Init_Car();
	LCD_main();
	melody_main ();
	
	//CMD_on();	
	//enter command mode
	printf("enter command mode\n\r"); // for debug purpose
	
	
//	CMD_off();
//	BT_receive_cmd = END_CMD;
//	LED_off();
	
	printf("Send data to handset \n\r");
//	while(Button_GetState() == 0);	
	receive_ptr =0;
	Send_ATCMD( (char *) string1 ); 
	Delay(1000);
	Send_ATCMD( (char *) string2 ); 


	for(;;){
	
		switch(BT_receive_cmd){
			
			case LED_ON:
					printf("LED on\n\r");
					Delay(1000);
					break;
			case LED_OFF:
					printf("LED off\n\r");
					Delay(1000);
					break;
			case Melody_1:
				PlayMelody(Melody_basic); 
				break;
			case Melody_2:
				PlayMelody(Melody_happy_song); 
				break;
			case Melody_3:
				PlayMelody(Melody_little_star); 
				break;
			case Melody_4:
				PlayMelody(Melody_little_bee); 
				break;			
			case END_CMD:
					printf("END CMD\n\r");
					Delay(1000);
					break;
			case CAR_FORWARD:					
				printf("Forward.\r\n\r\n");
				Car_Direction(CarDirection_Forward);
				Delay(1000);
			default:
					Delay(1000);
					break;
		}
		
		
	}

}



	/*
	printf("1. AT CMD\n\r");
	while(Button_GetState() == 0);	
	receive_ptr =0;
	Send_ATCMD( (char *) nAT_1 ); //use (char *) to casting (轉型) nAT_1(due to it is a const)
	Delay(1000);
	
	printf("2. GET BT version\n\r");
	while(Button_GetState() == 0);	
	receive_ptr =0;
	Send_ATCMD( (char *) nATVersion_3 ); // lookup version
	Delay(1000);
	
	printf("3. GET BT MAC address\n\r");
	while(Button_GetState() == 0);	
	receive_ptr =0;
	Send_ATCMD( (char *) nATAddr_5 ); 
	Delay(1000);
	
	printf("4. GET BT name\n\r");
	while(Button_GetState() == 0);	
	receive_ptr =0;
	Send_ATCMD( (char *) nATName_6 ); 
	Delay(1000);

	printf("5. GET USART \n\r");
	while(Button_GetState() == 0);	
	receive_ptr =0;
	Send_ATCMD( (char *) nATUart_13 ); 
	Delay(1000);
	
	printf("6. SET BT name\n\r");
	while(Button_GetState() == 0);	
	receive_ptr =0;
	strcat(nATNameSET_6, BTName);
	Send_ATCMD( (char *) nATNameSET_6 );
	Delay(1000);
	
	printf("7. GET BT name\n\r");
	while(Button_GetState() == 0);	
	receive_ptr =0;
	Send_ATCMD( (char *) nATName_6 ); 
	Delay(1000);
	
	printf("8. SET USART\n\r");
	while(Button_GetState() == 0);	
	receive_ptr =0;
	Send_ATCMD( (char *) nATUartSET_13 ); 
	Delay(1000);
	
		printf("9. GET USART \n\r");
	while(Button_GetState() == 0);	
	receive_ptr =0;
	Send_ATCMD( (char *) nATUart_13 ); 
	Delay(1000);
	
	*/