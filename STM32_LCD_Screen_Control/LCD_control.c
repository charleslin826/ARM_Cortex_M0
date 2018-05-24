
#include <stdio.h>
#include "stm32f0xx.h"						// File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void Delay(uint32_t);    //import from main.c

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

volatile uint8_t BackLight_flag , DisplayFunction , LCD_line , DisplayControl , DisplayMode ;

/*--------------------------------------------------------------------------------*/
// I2C2 send data  @ 0423
/*--------------------------------------------------------------------------------*/
void I2C2_SendData(uint8_t data){
 //ISR(Interrupt and status register)
	while ( (I2C2->ISR & I2C_ISR_TXE) != I2C_ISR_TXE ); //if TXE =1 , TX register is empty  
	I2C2->TXDR = data; // TXDR(Transmit data register )
	
}

/*--------------------------------------------------------------------------------*/
// write data to I2C2 and make ES H->L  @ 0423
/*--------------------------------------------------------------------------------*/
void I2C_write4bits(uint8_t value){
	
	I2C2_SendData(value |En); //send 2 bytes for whole I2C cycle
	I2C2->CR2 |= I2C_CR2_START;
	I2C2_SendData(value & ~En);

}


/*--------------------------------------------------------------------------------*/
// write command to LCD module, RS=0  @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_command(uint8_t cmd){

uint8_t highnib, lownib;

	highnib = cmd & 0xf0;
	highnib |= BackLight_flag;

	lownib = (cmd << 4) & 0xf0;
	lownib |= BackLight_flag;

	I2C_write4bits(highnib);
	I2C_write4bits(lownib);
	Delay(3);
	
}

/*--------------------------------------------------------------------------------*/
// write command to LCD module, RS=1  @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_data(uint8_t data){

uint8_t highnib, lownib;

	highnib = data & 0xf0;
	highnib |= Rs|BackLight_flag;

	lownib = (data << 4) & 0xf0;
	lownib |= Rs|BackLight_flag;

	I2C_write4bits(highnib);
	I2C_write4bits(lownib);
	Delay(3);
	
}

/*--------------------------------------------------------------------------------*/
// write a char to LCD  @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_printdata(char data){

	LCD_data(data);

}



/*--------------------------------------------------------------------------------*/
// write a string to LCD  @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_printstring( char * string_ptr, uint8_t string_length){
		
	uint8_t i;
	char ch;
	
	for(i=0; i<string_length; i++){ // put every string array into character for printing out
		ch = *string_ptr; //point to every char
		LCD_data(ch);   	//print it
		string_ptr++;	
	}
}

/*--------------------------------------------------------------------------------*/
/* LCD initialization  @ 0423
	1. Function set : DL=0, 4bit interface
										N=1, 2 line display
										F=0, 5*8 font
2. Display control: D=1, display on
										C=1, cursor on
										B=1, Blinking on
*/
/*--------------------------------------------------------------------------------*/
void LCD_Init(void){
	
	uint8_t value;
	DisplayFunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
	LCD_line	=1;
	BackLight_flag = LCD_NOBACKLIGHT;
	
	I2C2_SendData(0);
	I2C2->CR2 |= I2C_CR2_START; //start transmit first data
	I2C2_SendData(0);
	
	I2C_write4bits(0x30); //pls refer PDF LCD_HITACHI p.46
	Delay(5);
	
	I2C_write4bits(0x30);
	Delay(5);
	
	I2C_write4bits(0x30);
	I2C_write4bits(0x20);
	
	value = LCD_FUNCTIONSET | DisplayFunction;
	LCD_command(value);
	
	DisplayControl = LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF;   // turn off all fist
	value = LCD_DISPLAYCONTROL | DisplayControl;
	LCD_command(value);
	
	value= LCD_CLEARDISPLAY;
	LCD_command(value);
	
	DisplayMode = LCD_ShiftOFF | LCD_ENTRYSHIFTINCREMENT; // every time we write it will plus 1 automatically
	value = LCD_ENTRYMODESET | DisplayMode;
	LCD_command(value);
	
	DisplayControl = LCD_DISPLAYON | LCD_CURSORON;			// turn on
	BackLight_flag = LCD_BACKLIGHT;											// turn on
	value = LCD_DISPLAYCONTROL | DisplayControl;
	LCD_command(value);
	
	
}
/*--------------------------------------------------------------------------------*/
// Clear LCD display @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_Clear(void){
	LCD_command(LCD_CLEARDISPLAY);
	Delay(3); // delay 3ms for clean up 
	
	
}

/*--------------------------------------------------------------------------------*/
// Set cursor at Home  @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_Home(void){

	LCD_command(LCD_RETURNHOME);
	Delay(3);


}
/*--------------------------------------------------------------------------------*/
// Set cursor position @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_setCursor(uint8_t col, uint8_t row){
	
	uint8_t row_offset[] = {0,0x40};

		if(row > LCD_line ){  //LCD_line =1
				row = LCD_line;
		}
		
		if(col > Line_MaxCol ){  // Line_MaxCol = 0xF  (total 16 column per row)
				col = Line_MaxCol;
		}
		
		LCD_command(LCD_SETDDRAMADDR | (col+row_offset[row]));  // LCD_SETDDRAMADDR = 0x80
			
}


/*--------------------------------------------------------------------------------*/
// Turn off Display @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_DisplayOff(void){
		DisplayControl &= ~ LCD_DISPLAYON;
		LCD_command( LCD_DISPLAYCONTROL | DisplayControl);

}

/*--------------------------------------------------------------------------------*/
// Turn on Display @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_DisplayOn(void){
		DisplayControl |= LCD_DISPLAYON; //#define LCD_DISPLAYON 0x04
		LCD_command( LCD_DISPLAYCONTROL | DisplayControl);

}


/*--------------------------------------------------------------------------------*/
// Turn off Cursor @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_noCursor(void){
		DisplayControl &= ~ LCD_CURSORON;
		LCD_command( LCD_DISPLAYCONTROL | DisplayControl);

}

/*--------------------------------------------------------------------------------*/
// Turn on Cursor @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_Cursor(void){
		DisplayControl |=  LCD_CURSORON;
		LCD_command( LCD_DISPLAYCONTROL | DisplayControl);

}


/*--------------------------------------------------------------------------------*/
// Turn off Cursor Blink @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_noBlinkCursor(void){
		DisplayControl &= ~ LCD_BLINKON;
		LCD_command( LCD_DISPLAYCONTROL | DisplayControl);

}

/*--------------------------------------------------------------------------------*/
// Turn on Cursor Blink @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_BlinkCursor(void){
		DisplayControl |=  LCD_BLINKON;
		LCD_command( LCD_DISPLAYCONTROL | DisplayControl);

}

/*--------------------------------------------------------------------------------*/
// Scroll to left @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_scrollDisplayLeft(void){

	LCD_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT); // #define LCD_CURSORSHIFT 0x10   //  LCD_DISPLAYMOVE & LCD_MOVELEFT 0x00


}

/*--------------------------------------------------------------------------------*/
// Scroll to right @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_scrollDisplayRight(void){

	LCD_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT); // #define LCD_CURSORSHIFT 0x10  /LCD_DISPLAYMOVE 0x00 / LCD_MOVERIGHT 0x04


}

/*--------------------------------------------------------------------------------*/
// create font @ 0423
/*--------------------------------------------------------------------------------*/
void LCD_createChar(uint8_t location, uint8_t charmap[]){

	uint8_t i;
	location &= 0x7;
	LCD_command(LCD_SETCGRAMADDR | (location <<3));  // #define LCD_SETCGRAMADDR 0x40

	for(i=0; i<8 ;i++){
		LCD_data(charmap[i]);
	}

}

/*--------------------------------------------------------------------------------*/
// font data@ 0423
/*--------------------------------------------------------------------------------*/
void Set_CGRAM(void){

	uint8_t char_array0[8]={0x00, 0x00, 0x11,0x0a, 0x00, 0x40, 0x0a};
	uint8_t char_array1[8]={0x00, 0x00, 0x1b,0x00, 0x04, 0x00, 0x0e};
	uint8_t char_array2[8]={0x1f, 0x11, 0x15,0x1b, 0x15, 0x1f, 0x00};
	uint8_t char_array3[8]={0x1f, 0x11, 0x15,0x1b, 0x11, 0x1f, 0x00};
	uint8_t char_array4[8]={0x1f, 0x15, 0x1b,0x11, 0x11, 0x1f, 0x00};
	
	LCD_createChar(0, char_array0);
	LCD_createChar(1, char_array1);
	LCD_createChar(2, char_array2);
	LCD_createChar(3, char_array3);
	LCD_createChar(4, char_array4);
}


/*--------------------------------------------------------------------------------*/
// Turn off backlight@ 0423
/*--------------------------------------------------------------------------------*/
void LCD_BacklightOff(void){
	BackLight_flag = LCD_NOBACKLIGHT;
	I2C2_SendData(LCD_NOBACKLIGHT); 
	I2C2->CR2 |= I2C_CR2_START;
	I2C2_SendData(LCD_NOBACKLIGHT); //send 2 bytes for whole I2C cycle
	
}

/*--------------------------------------------------------------------------------*/
// Turn on backlight@ 0423
/*--------------------------------------------------------------------------------*/
void LCD_BacklightOn(void){
	BackLight_flag = LCD_BACKLIGHT;
	I2C2_SendData(LCD_BACKLIGHT); //send 2 bytes for whole I2C cycle
	I2C2->CR2 |= I2C_CR2_START;
	I2C2_SendData(LCD_BACKLIGHT);
	
}
