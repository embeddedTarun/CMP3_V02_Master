/*
 * lcd1604.c
 *
 *  Created on: Feb 6, 2024
 *  Author: Intellisense
 */

#include <lcd1604.h>
#include "stm32f1xx_hal.h"

/*********** Define the LCD PINS below ****************/

#define RS_Pin GPIO_PIN_4
#define RS_GPIO_Port GPIOB

//#define RW_Pin GPIO_PIN_2
//#define RW_GPIO_Port GPIOA

#define EN_Pin GPIO_PIN_5
#define EN_GPIO_Port GPIOB

#define LCD_D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOB

#define LCD_D5_Pin  GPIO_PIN_7
#define D5_GPIO_Port GPIOB

#define LCD_D6_Pin GPIO_PIN_8
#define D6_GPIO_Port GPIOB

#define LCD_D7_Pin GPIO_PIN_9
#define D7_GPIO_Port GPIOB

#define joy_btn_Pin GPIO_PIN_1
#define jbtn_GPIO_Port  GPIOB



/****************** define the timer handler below  **************/



void delay (int us)
{

	HAL_Delay(1);

}

/*******************************************************************/

void send_to_lcd ( char data, int rs)
{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, rs);  // rs = 1 for data, rs=0 for command

	/* write the data to the respective pin */
	//HAL_GPIO_WritePin(D7_GPIO_Port, LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin, data<<4);
	HAL_GPIO_WritePin(D7_GPIO_Port, LCD_D7_Pin, (data&0x08));
	HAL_GPIO_WritePin(D6_GPIO_Port, LCD_D6_Pin, (data&0x04));
	HAL_GPIO_WritePin(D5_GPIO_Port, LCD_D5_Pin, (data&0x02));
	HAL_GPIO_WritePin(D4_GPIO_Port, LCD_D4_Pin, (data&0x01));


	 /*
	 * Toggle EN PIN to send the data
	 * if the HCLK > 100 MHz, use the  20 us delay
	 * if the LCD still doesn't work, increase the delay to 50, 80 or 100..
	 */
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);
	delay(1);  //    //delay (20);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 0);
	delay(1);     //delay (20);
}

void lcd_send_cmd( char cmd)
{
	 char datatosend;

    /* send upper nibble first */
    datatosend = ((cmd>>4)&0x0f);

    send_to_lcd(datatosend,0);  // RS must be 0 while sending command

    /* send Lower Nibble */
    datatosend = ((cmd)&0x0f);   // 0x00 replace with 0x00
	send_to_lcd(datatosend, 0);
}

void lcd_send_data( char data)
{
	 char datatosend;
	/* send higher nibble */

	datatosend = ((data>>4)&0x0f);
	send_to_lcd(datatosend, 1);  // rs =1 for sending data
	/* send Lower nibble */

	datatosend = ((data)&0x0f);
	send_to_lcd(datatosend, 1);

}

void lcd_clear(void)
{
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}

void lcd_put_cur(int row, int col)
{
    switch(row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
	  case 2:
	  col |= 0x90;
	 break;
	  case 3:
        col |= 0xD0;
	break;
    }

 //   lcd_send_cmd(row);
    lcd_send_cmd(col);

}


void lcd_init(void)
{
	lcd_send_cmd (0x02);
	HAL_Delay(1);
	lcd_send_cmd (0x28);HAL_Delay(1);
	lcd_send_cmd (0x0c);HAL_Delay(1);
	lcd_send_cmd (0x06);HAL_Delay(1);
	lcd_send_cmd (0x01);HAL_Delay(1);



}

void lcd_send_string(char* str)
{
	//str[5]=0;
	while (*str) lcd_send_data (*str++);


}

void lcd_string_new (char *str)		// Send string to LCD function */
{
	int i;
	for(i=0; str[i]!=0;i++)		// Send each char of string till the NULL /
	{
		lcd_send_data (str[i]);
	}
}


