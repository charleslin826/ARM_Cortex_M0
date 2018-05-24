/**   Project name : LCD_Screen_Control
*   									I2C2 control
--------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "stm32f0xx.h"          // File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void stdout_init (void);
extern void LCD_Home(void); // import from LCD_control.c
extern void LCD_Clear(void);
extern void LCD_Init(void);
extern void LCD_printstring( char *, uint8_t ); // we can ignore the variable "string_ptr" & "string_length"
extern void LCD_printdata(char );// we can ignore the variable "data"
extern void LCD_noCursor(void);
extern void LCD_Cursor(void);
extern void LCD_noBlinkCursor(void);
extern void LCD_BlinkCursor(void);
extern void LCD_DisplayOff(void);
extern void LCD_DisplayOn(void);
extern void LCD_scrollDisplayLeft(void);
extern void LCD_scrollDisplayRight(void);
extern void LCD_setCursor(uint8_t col, uint8_t row);
extern void Set_CGRAM(void);
extern void LCD_BacklightOff(void);
extern void LCD_BacklightOn(void);

#define NUM_KEYS  1                     /* Number of available keys           */

/* Keys for NUCLEO Board */
#define USER    1

/*--------------------------------------------------------------------------------*/
// setup I2C_2 PB.13 & 14  @ 0423
/*--------------------------------------------------------------------------------*/
void Configure_GPIOB1314_I2C2(void)
{
	RCC->AHBENR |= 0x1ul << 18;   //enable Port B
	GPIOB->MODER &= ~(0x3ul<<13*2 | 0x3ul << 14*2 );  // Clean up PB.13 14
	GPIOB->MODER |= (0x2ul<<13*2 | 0x2ul << 14*2 );	 // Setup PB.13 14
	GPIOB->AFR[1] &= ~(0xful ); // clean up AFR13 . 14
	GPIOB->AFR[1] |= 5<< 5*4 | 5 << 6*4; // setup AF5 for AFR13 . 14
	
	GPIOB->OTYPER |= 0x1ul << 13 | 0x1ul << 14;// Setup Output Open_drain for PB.13 14
	
	
}
/*--------------------------------------------------------------------------------*/
// Setup I2C2  @ 0423
/*--------------------------------------------------------------------------------*/
void Configure_I2C2_Master(void)
{
	RCC->APB1ENR |= 0x1ul<<22; //  I2C2 clock enabled
	
	I2C2->TIMINGR = 0xB0211113ul; // this is so difficult
	I2C2->CR1 |= 0x1ul;
	I2C2->CR2 |= 0x1ul << 25 | 2 << 16 | 0x3f << 1 ;
//I2C2->CR2 |= 0x0ul << 10 // Bit 10 RD_WRN: Transfer direction (master mode) 0: Master requests a write transfer. //Due to Default is 0 so we don't need to change



}
/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int LCD_main (void) {        // User application starts here

	uint8_t char_len,i;
	char data;
	
	Configure_GPIOB1314_I2C2();
	Configure_I2C2_Master();
	LCD_Init();
	


}







