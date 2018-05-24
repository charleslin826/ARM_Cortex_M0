
#include <stdio.h>
#include "stm32f0xx.h"
#include "RTE_components.h"

extern void Delay(uint32_t);

// LCD commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ShiftOFF  0x00
#define LCD_ShiftON   0x01
#define LCD_ENTRYSHIFTINCREMENT 0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00
#define Line_MaxCol  0xF

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0x04  // Enable bit
#define Rw 0x02  // Read/Write bit
#define Rs 0x01  // Data Register select bit

volatile uint8_t BackLight_flag , DisplayFunction, LCD_line , DisplayControl, DisplayMode;

/*--------------------
 I2C2 send data
 ---------------------*/
 void I2C2_SendData(uint8_t data)
 {
	    while ( (I2C2->ISR & I2C_ISR_TXE) != I2C_ISR_TXE);    // if TXE = 1 , TX register is empty
	    I2C2->TXDR = data;
	  
 }
 
 
 
/*---------------------------------------
   write data to I2C2 and make ES H->L
 ----------------------------------------*/
void I2C_write4bits(uint8_t value)
{

   I2C2_SendData(value |En );
	 I2C2->CR2 |= I2C_CR2_START;
	 I2C2_SendData(value & ~En);

}	

 
 
/*---------------------------------
  write command to LCD module, RS=0
------------------------------------*/
void LCD_command(uint8_t cmd)
{
	uint8_t highnib , lownib;
	
	highnib = cmd & 0xf0;
	highnib |= BackLight_flag;
	
	lownib= (cmd << 4) & 0xf0;
	lownib |= BackLight_flag;
	
	I2C_write4bits(highnib);
	I2C_write4bits(lownib);
	Delay(3);
	
} 

 
/*---------------------------------
  write data to LCD module, RS=1
------------------------------------*/
void LCD_data(uint8_t data)
{
	uint8_t highnib , lownib;
	
	highnib = data & 0xf0;
	highnib |= Rs|BackLight_flag;
	
	lownib= (data << 4) & 0xf0;
	lownib |= Rs |BackLight_flag;
	
	I2C_write4bits(highnib);
	I2C_write4bits(lownib);
	Delay(3);
	
} 



/*-----------------------
   write a char to LCD
 ------------------------*/
void LCD_printdata(char data)
{
	 LCD_data(data);
		
}
 
/*--------------------------
  write a string to LCD
--------------------------*/
void LCD_printstring( char * string_ptr, uint8_t string_length)
{
	
	uint8_t i; 
	char ch;
	
	for(i=0; i< string_length ; i++)
	{
		ch = *string_ptr;
		LCD_data(ch);
		string_ptr++;
	}
		
} 

 
/*-----------------------------------------
   LCD initialization
1. Function set : DL =0 ;4bit interface
                  N=1, 2 line display
                  F=0 , 5x8 font
2. Display control: D= 1 ; display on
                    C =1 ; cursor on
                    B = 1 ; Blinking on
--------------------------------------------*/ 
void LCD_Init(void)
{

   uint8_t value;
	
	DisplayFunction= LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
	LCD_line =1;
	BackLight_flag = LCD_NOBACKLIGHT;

	I2C2_SendData(0);
	I2C2->CR2 |= I2C_CR2_START;
	I2C2_SendData(0);
	
	I2C_write4bits(0x30);
	Delay(5);
	
	I2C_write4bits(0x30);
	Delay(5);
	
	I2C_write4bits(0x30);
	
  I2C_write4bits(0x20);
	
	value = LCD_FUNCTIONSET | DisplayFunction;
	LCD_command(value);
	
	DisplayControl = LCD_DISPLAYOFF | LCD_CURSOROFF |LCD_BLINKOFF;
	value = LCD_DISPLAYCONTROL | DisplayControl;
	LCD_command(value);
	 
  value= LCD_CLEARDISPLAY;
	LCD_command(value);
	
	DisplayMode = LCD_ShiftOFF | LCD_ENTRYSHIFTINCREMENT;
	value = LCD_ENTRYMODESET | DisplayMode;
	LCD_command(value);

  DisplayControl = LCD_DISPLAYON | LCD_CURSORON;
	BackLight_flag = LCD_BACKLIGHT;
	value = LCD_DISPLAYCONTROL | DisplayControl;
	LCD_command(value);


}	


/*-------------------------
  Clear LCD display
---------------------------*/
void LCD_Clear(void)
{
	LCD_command(LCD_CLEARDISPLAY);
	Delay(3);
	
}	
	
/*-----------------------
   Set cursor at Home
-------------------------*/
void LCD_Home(void)
{
	LCD_command(LCD_RETURNHOME);
	Delay(3);
	
}

	
	
	
	
	
	
	
	
	
	











 
 
 
 
 