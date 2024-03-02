/*
 * lcd1604.h
 *
 *  Created on: Feb 6, 2024
 *      Author: Intellisense
 */

#ifndef INC_LCD1604_H_
#define INC_LCD1604_H_

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);

//void delay (int us);

void lcd_string_new (char *str);
void SysTick_Handler(void);
//void delayMicroseconds(uint32_t microseconds);




#endif /* INC_LCD1604_H_ */
