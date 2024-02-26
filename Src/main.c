/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lcd1604.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef uint8_t crc_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WIDTH  (8 * sizeof(crc_t))
#define TOPBIT (1 << (WIDTH - 1))


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */



#define joy_btn_Pin GPIO_PIN_10
#define jbtn_GPIO_Port  GPIOB

//#define x_channel ADC_Select_4
//#define y_channel ADC_Select_5
uint32_t adc_value = 0;
uint32_t raw_value = 0;
uint8_t button = 1;          // joy stic button status
uint32_t x_value = 0;  		 // joy stic x value from ADC
uint32_t y_value = 0;   	 // joy stic y value from ADC
uint32_t spd_value = 0;      // speed value
uint32_t damp_value = 0;     // damping value
uint8_t  home = 1;        	 // hold the exact position of the cursor in home screen
uint8_t  hold = 0;   		 // hold the function in while loop
uint8_t  jpos = 0;           // cursor (< >) detection
uint8_t  row_pos = 0;  		 // row position
uint8_t  arrow = 0;   	 	 //  arrow ( > ) position in the UNL mode  function
float limit_1 = 0.0;   	 // motor limit_1 value
float limit_2 = 0.0;   	 // motor limit_2 value
float motor_step = 0.0;      // final motor steps after setting limit_1 and limit_2
float motor_limit = 0.0;     // differnec of 1st limit and 2nd limt in the man mode
float motor_count = 0.0;
uint8_t motor_dir = 0;        // motor direction status 1 = Left /reverse and 2 = RIGHT / Forward

uint8_t motor_speed_pot = 0;
uint8_t motor_damp_pot = 0;
uint16_t speed_send = 0;
char direction;


uint8_t man_mode = 0;
uint8_t unl_mode = 0;
uint8_t man_flag = 0;
uint8_t unl_flag = 0;
uint8_t fr_flag = 0;
uint8_t frpv_flag = 0;
uint8_t exit_flag = 0;
uint8_t start_flag;

unsigned int  home_pos = 0;  // home position in animation mode
float  old_home_pos = 0.0;   // old position in animation mode
float new_home_pos = 0.0;    // new position in animation mode
float current_pos = 0.0;     // current position in animation mode
float step = 0.0;            // motor will move step to left or right in animation mode
uint8_t steps = 0;          // total steps of the shots taken in animation mode
uint8_t shots = 0;          // No of shorts to be taken in animation
uint8_t  delay_val = 0;     // delay between the each short
uint8_t  motor_lr =  0;     // motor direction Left or Right

int ac = 0;

crc_t POLYNOMIAL = 0xcb;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */


void home_scr(void);
void auto_unl(void);
void read_joystic();
//uint32_t get_adc_value(uint32_t channel);
uint32_t get_adc_value(uint32_t channel, uint8_t rank);
void read_joystic();
void float_to_string(float value);
void back_dis(void);
void live_fun(void);
void free_ride_fun(void);
void re_calibration(void);
void start_fun(void);
void data_transmit(void);
void get_pot_value(void);
void rx_data(void);
void data_display(void);

////////// 485 //////////

//void USART1_IRQHandler(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void send_on_rs485(uint8_t* buffer);
void recieve_on_rs485(uint8_t buffer[],size_t buffer_length);

crc_t CRC_generate(crc_t* message, crc_t polynomial, int nBytes );
uint8_t* message_packet_with_crc(crc_t* message2, crc_t polynomial, int nBytes );
crc_t CRC_CHECK_decode(crc_t* message, crc_t polynomial, int nBytes );


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///////  485     ////

//uint8_t* send_buffer="h---Lcd\r\n";
//uint8_t send_buffer[10]={0x55,'-','-','-','L','c','d','\r','\n',0x01};



uint8_t  rx_motor_dir;
uint32_t rx_motor_step;

uint8_t send_buffer[11] = {0x55,'F',0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x01,0x55};

uint8_t Previous_buffer[11];
uint8_t RECIEVE_VALID_DATA[11];
uint8_t recieve_buffer[11];

uint8_t RS_485_Data_validate=0;
int BUFFER_LENGTH=11;


uint8_t state_of_rs485 = 1;

//char buffer1[9]="Driver2\r\n";
//uint8_t buffer[14];
int counter;
uint8_t cnt =0;


/// home screen message /////


long map(long x, long in_min, long in_max, long out_min, long out_max)
	{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}


  void home_scr(void)
  {
	  lcd_put_cur(1, 1);
	  lcd_string_new("RE-CALIBRATE?");
	  HAL_Delay(2);
	  lcd_put_cur(2, 2);
	  lcd_string_new("<NO>");
	  HAL_Delay(1);
	  lcd_put_cur(2, 9);
	  lcd_string_new(" YES ");
	  HAL_Delay(50);
  }

////  ADC value read function


  uint32_t get_adc_value(uint32_t channel, uint8_t rank)
  	{
  		ADC_ChannelConfTypeDef sConfig = {0};
  		uint32_t g_ADCValue = 0;
  		sConfig.Channel = channel;
  		sConfig.Rank = rank;
  		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;


  		//add to channel select
  		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  			{
  				Error_Handler();
  			}
  		HAL_ADC_Start(&hadc1);

  		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  		 {
  			g_ADCValue = HAL_ADC_GetValue(&hadc1);
  		 }
  		else
  			{
  				Error_Handler();
  			}

  	//  remove from channel select

  	//  sConfig.Rank = 0 ;

  	//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  	//	Error_Handler();

  			HAL_ADC_Stop(&hadc1);
  			return (g_ADCValue);

  	}




///// joystic value read function

  void read_joystic()
  {
	y_value = get_adc_value(ADC_CHANNEL_4,1);
	HAL_Delay(1);
    x_value = get_adc_value(ADC_CHANNEL_3,1);
    HAL_Delay(1);

  }
  void read_speed()
   {
 	spd_value = get_adc_value(ADC_CHANNEL_1,1);
     HAL_Delay(1);
   }
  void read_damping()
  {
 	damp_value = get_adc_value(ADC_CHANNEL_2,1);
     HAL_Delay(1);
   }



  void lcd_int_to_str(int raw_data)  //////// int to string convert function
  {
  	char str1[12];
  	sprintf(str1, "%d", raw_data);
  //	lcd_put_cur(1, 10);
    lcd_string_new(str1);
  	//return str;

  }

  void float_to_string(float value)
  {
    char str[128];
    sprintf(str, "%.1f", value);
    lcd_string_new(str);
  }


 void rx_data()
 {
		rx_motor_dir  =   RECIEVE_VALID_DATA[1];          // exact direction of the motor
		rx_motor_step   |=     RECIEVE_VALID_DATA[2] << 24;
		rx_motor_step   |=     RECIEVE_VALID_DATA[3] << 16;
		rx_motor_step   |=     RECIEVE_VALID_DATA[4] << 8;
		rx_motor_step   |=     RECIEVE_VALID_DATA[5];
        HAL_Delay(1);
	////////////////
		motor_step = rx_motor_step / 2560;       // convert the motor micro steps into float numbers
        HAL_Delay(1);
 }

 //************ free Ride, recording playback  video back function and their subfunctions


  void  Recording_fun()
  {
	lcd_clear();
	HAL_Delay(1500);
	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	read_joystic(); HAL_Delay(5);

	while ( button != 0  )
	{
		button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
		read_joystic(); HAL_Delay(5);
		lcd_clear(); lcd_put_cur(0, 1); lcd_string_new("[press ok]");
	}

    live_fun(); /// call again to the live function

  }




 void Playback_fun()
  { lcd_clear();
	HAL_Delay(1500);
	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	read_joystic(); HAL_Delay(5);

	while ( button != 0  )
	{
		button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
		read_joystic(); HAL_Delay(5);
		lcd_clear(); lcd_put_cur(0, 1); lcd_string_new("[press ok]");
	}

  live_fun(); /// call again to the live function
}


 void Videoloop_fun()
  { lcd_clear();
	HAL_Delay(1500);
	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	read_joystic(); HAL_Delay(5);

	while ( button != 0  )
	{
		button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
		read_joystic(); HAL_Delay(5);
		lcd_clear(); lcd_put_cur(0, 1); lcd_string_new("[press ok]");
	}

  live_fun(); /// call again to the live function
  }


void back_dis()
{
  //  int b = 1;
	lcd_clear(); //lcd_put_cur(0, 1); lcd_string_new("[Back]");
//	lcd_put_cur(0,0); lcd_string_new(">");
	 HAL_Delay(200);
	 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	 read_joystic(); HAL_Delay(5);

 while ( button != 0 &&  y_value  <= 3500 )
   {
	   button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
       read_joystic(); HAL_Delay(10);
       lcd_put_cur(0, 1); lcd_string_new("[Back]");
       lcd_put_cur(0,0); lcd_string_new(">");
	}


  if (button == 0)
	   {
		lcd_clear();
		exit_flag = 1;
		HAL_Delay(1);
		auto_unl();  /// call back to auto unlimted limt function

	   }

   lcd_put_cur(0, 1);lcd_string_new("[Free Ride]");lcd_put_cur(1, 1); lcd_string_new("[Recording]");
   lcd_put_cur(2, 1);lcd_string_new("[Playback]  0s");lcd_put_cur(3, 1); lcd_string_new("[VideoLoop]");
   lcd_put_cur(0,0);lcd_string_new(">");
   row_pos = 0;

}









void free_ride_fun()
{

    fr_flag = 0;
    lcd_clear();
	lcd_put_cur(1, 0); lcd_string_new("FreeRide  ");
	get_pot_value();
	//data_transmit();
	data_display();
	HAL_Delay(1000); // debounce dealy
	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
    read_joystic(); //HAL_Delay(5);
  //  HAL_Delay(1000);
  //  lcd_int_to_str(speed_send);



 /// *************** when limits are set in Man mode  *************

  if (man_flag == 1)
  {
	 motor_count = motor_limit;
     button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	 while (button != 0 )
	 	      {
	 	            button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	 	        	read_joystic(); HAL_Delay(1);

	 	         if ((x_value <= 2098 ) && ( x_value >= 1902))
	 	           {
	 	        	 data_display();
	 	        	 speed_send = 15;
	 	        	 direction = 'S';
	 	        	 get_pot_value();
	 	        	 data_transmit();
	 	          	 rx_data();
	 	        	 HAL_Delay(5);
	 	           }

	 	    else if ((x_value <= 1900) && ( x_value >= 0 ))  //// 1st limit increment
	 	      	   {

	 	         	data_display();
	 	    	     direction = 'F';
	 	            get_pot_value();
	 	      		speed_send = map( x_value,  1900, 0, 15, 1550) * motor_speed_pot / 100;
	 	      		data_transmit();
	 	      		rx_data();
	 	      		if (motor_count < motor_limit)
	 	      		{
	 	      			motor_count = motor_count + motor_step;
	 	      		}

	 	      		HAL_Delay(5);
	 	     		}

	 	     else if ((x_value >= 2100) && (x_value < 4095)) // 1st limit decrement
	 	      		{
	 	    	    data_display();
	 	      		direction = 'R';
	 	      		get_pot_value();
	 	      		speed_send = map( x_value,  2100, 4000, 15, 1550) * motor_speed_pot / 100;
	 	      		data_transmit();
	 	      		rx_data();
	 	      		if (motor_count > 0.1)
	 	      		{
	 	      		  motor_count = motor_count - motor_step;
	 	      		}

	 	      		HAL_Delay(5);
	 	      		}
	 	        }

	}



 // **********  when limits are unlimited       ***********************

  else if (man_flag == 0)

  {
      motor_count = motor_step;
	  button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	  read_joystic(); HAL_Delay(5);
	  while (button != 0 )
	      {
	            button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	        	read_joystic(); HAL_Delay(1);

	         if ((x_value <= 2098 ) && ( x_value >= 1902))
	           {
	        	 speed_send = 15;
	        	 direction = 'S';
	        	 get_pot_value();
	        	 data_transmit();
	        	 motor_count = motor_step;
	        	 data_display();
	        	 rx_data();
	        	 HAL_Delay(5);
	           }

	    else if ((x_value <= 1900) && ( x_value >= 0 ))  //// 1st limit increment
	      	   {
	      		direction = 'F';
	            get_pot_value();
	      		speed_send = map( x_value,  1900, 0, 15, 1550) * motor_speed_pot / 100;
	      		data_transmit();
	      		motor_count = motor_step;
	      		data_display();
	      		rx_data();
	      		HAL_Delay(5);
	     		}

	     else if ((x_value >= 2100) && (x_value < 4095)) // 1st limit decrement
	      		{
	      		direction = 'R';
	      		get_pot_value();
	      		speed_send = map( x_value,  2100, 4000, 15, 1550) * motor_speed_pot / 100;
	      		data_transmit();
	      		motor_count = motor_step;
	      		data_display();
	      		rx_data();
	      		HAL_Delay(5);
	      		}
	        }

      }


///  back to the live function  /////////

	  lcd_clear();
      live_fun();

}




void data_display()
{
  	 lcd_put_cur(1, 10);  float_to_string(motor_count);    // motor steps display on lcd
	 lcd_put_cur(2, 0);   lcd_int_to_str(motor_speed_pot);
	 lcd_put_cur(2, 14);  lcd_int_to_str(motor_damp_pot);
	 lcd_put_cur(2,  5);  lcd_string_new("       ");
	 lcd_put_cur(2, 7);   lcd_int_to_str(speed_send);
	 lcd_put_cur(1, 14);  lcd_send_data(direction);
	 HAL_Delay(1);
}

//********* transmit the data over RS484 module **************

void data_transmit()
{
	 send_buffer[1] = direction;
	 send_buffer[2] = motor_speed_pot;
	 send_buffer[3] = motor_damp_pot;
 	 send_buffer[4] = speed_send>>8;
	 send_buffer[5] = speed_send & 0x00FF;
	// send_buffer[6] =  u steps to run;
	// send_buffer[7] =  u steps to run;

	 while(state_of_rs485 != 1);
	 send_on_rs485(send_buffer);
	 HAL_Delay(1);
}







  //********** function to get speed and damp pot value and then  mapped it  **********

void get_pot_value()
{
	spd_value = get_adc_value(ADC_CHANNEL_1);  HAL_Delay(5);
	motor_speed_pot = map( spd_value,  0, 4090, 0, 100);
	damp_value = get_adc_value(ADC_CHANNEL_2);  HAL_Delay(5);
	motor_damp_pot = map( damp_value,  0, 4090, 0, 100);
	x_value = get_adc_value(ADC_CHANNEL_3);  HAL_Delay(1);
}




//////  Automatic UNL sub functions  //////////

  void live_fun()                  //FRPV_fun()
{
/////
      lcd_clear();
      hold = 1; row_pos = 0;
      unl_flag = 1;
	  lcd_put_cur(0, 1);lcd_string_new("[Free Ride]");lcd_put_cur(1, 1); lcd_string_new("[Recording]");
	  lcd_put_cur(2, 1);lcd_string_new("[Playback]  0s");lcd_put_cur(3, 1); lcd_string_new("[VideoLoop]");
      lcd_put_cur(0,0);lcd_string_new(">");
      button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
      read_joystic(); HAL_Delay(5);
	  HAL_Delay(800);

    while( button != 0 && unl_flag ==1 )
   {

	      button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	      read_joystic(); HAL_Delay(5);
	  	  	  /// arrow movement
      if ((y_value >= 3000) && (y_value < 4095))
	     {
	        lcd_put_cur(row_pos,0); lcd_string_new(" ");
	        row_pos++;
	     if (row_pos >= 4)
	       {
	    	 row_pos = 0;
	    	 back_dis();

	       }

            lcd_put_cur(row_pos,0);lcd_string_new(">");
	        HAL_Delay(250);
	      }


     }
	   switch(row_pos)
	   {
	   	 case 0:
	   	 free_ride_fun(); break;

	   	 case 1:
	   	 Recording_fun(); break;

	   	 case 2:
	   	 Playback_fun();  break;

	   	 case 3:
	   	 Videoloop_fun(); break;

	   	 case 4:

	 //  	 back_dis();

	   	 break;

	    }


	   if (exit_flag == 1)  { HAL_Delay(100); auto_unl(); }  // call back to auto unl function for loop

 }



void timelapse_fun()
{ // hold = 0;
	HAL_Delay(50);
}


//************ Animation mode function ************************//


void animation_fun()
{  // hold = 0;
	int cursor_pos = 0;
 //   int j_toggle = 0;
    int j_toggle1 = 0;
	row_pos = 0;
	hold = 1;
	lcd_clear();
//	lcd_put_cur(0, 1); lcd_string_new("Direction:Right");
//	lcd_put_cur(1, 1); lcd_string_new("HomePos:<     >"); lcd_put_cur(1, 11); lcd_int_to_str(home_pos);
//	lcd_put_cur(2, 1); lcd_string_new("Step:   <     >"); lcd_put_cur(2, 11); float_to_string(step);
//	lcd_put_cur(3, 1); lcd_string_new("Steps:  <     >"); lcd_put_cur(3, 11); lcd_int_to_str(steps);
//	lcd_put_cur(0,0);lcd_string_new(">");
//	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	read_joystic(); HAL_Delay(5);
    HAL_Delay(50);

while ( hold == 1 )
{

	lcd_put_cur(0, 1); lcd_string_new("Direction:    ");
	lcd_put_cur(1, 1); lcd_string_new("HomePos:<     >"); lcd_put_cur(1, 11); lcd_int_to_str(home_pos);
	lcd_put_cur(2, 1); lcd_string_new("Step:   <     >"); lcd_put_cur(2, 11); float_to_string(step);
	lcd_put_cur(3, 1); lcd_string_new("Steps:  <     >"); lcd_put_cur(3, 11); lcd_int_to_str(steps);
	lcd_put_cur(0,0);lcd_string_new(">");
	cursor_pos =1; row_pos = 0;

////////// First screen of the animaion ///////

	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	read_joystic(); //HAL_Delay(5);

     while( cursor_pos >= 1 &&  cursor_pos <= 4  )
   {

      read_joystic(); HAL_Delay(5);
      if ((y_value <= 2900) && (y_value > 1000)) {   j_toggle1 = 1;  HAL_Delay(2); }

      else if ((y_value >= 3000 ) && (y_value <= 4095) && ( j_toggle1 == 1) )
       {  cursor_pos++; j_toggle1 = 0;
          lcd_put_cur(row_pos,0); lcd_string_new(" ");  if ( row_pos <= 3) { row_pos++; }     // row_pos++;  if (row_pos >= 4) { row_pos = 0; }
          lcd_put_cur(row_pos,0); lcd_string_new(">"); // HAL_Delay(100);
       }      //back_dis();

       else if ((y_value <= 800) && (y_value >= 0 ) && ( j_toggle1 == 1))
         { cursor_pos--; j_toggle1 = 0;
    	   lcd_put_cur(row_pos,0); lcd_string_new(" ");
    	   row_pos--; if(row_pos < 0) { row_pos = 0; }
    	   lcd_put_cur(row_pos,0);lcd_string_new(">"); //HAL_Delay(100);
         }


	 switch(row_pos)
	   {
	    case 0:   //  read_joystic();  HAL_Delay(3);
	    	 	 while( y_value <= 2500 && y_value > 1000 )
	    	  {   read_joystic(); HAL_Delay(3);
                if (x_value <= 900) { motor_lr = 1;  }
            	else if (x_value >= 3000) { motor_lr = 0;  }
                if (motor_lr == 1) {lcd_put_cur(0,11);lcd_string_new("Right");   }    //HAL_Delay(150);
                else if (motor_lr == 0 ) { lcd_put_cur(0,11); lcd_string_new(" Left"); } //HAL_Delay(150);

	    	  }   HAL_Delay(100);

	    	 	 break;

	   	 case 1: //   read_joystic(); HAL_Delay(3);
	   		     while( y_value < 2500 && y_value > 1000 ) { read_joystic(); HAL_Delay(3);
	   		     if (x_value <= 900) { home_pos++; lcd_put_cur(1,11); lcd_int_to_str(home_pos);  HAL_Delay(75); }
	   		  	 else if (x_value >= 3000) { home_pos--; lcd_put_cur(1,11);lcd_int_to_str(home_pos);  HAL_Delay(75); }

	   		   }   HAL_Delay(100);
	   		     break;

	   	 case 2: //read_joystic(); HAL_Delay(3);
	   	 	 	 while( y_value < 2500 && y_value > 1000  )  { read_joystic(); HAL_Delay(3);
	   	 	 	 if (x_value <= 900) { step = step + 0.1; lcd_put_cur(2,11); float_to_string(step);  HAL_Delay(75); }
	   	 	 	 else if (x_value >= 3000) { step = step - 0.1; if( step <= 0.0) { step = 0.0; }
	   	 	 	 lcd_put_cur(2,11);float_to_string(step); HAL_Delay(75); 	 }

		       }  HAL_Delay(100);	break;

	   	 case 3:  // read_joystic(); HAL_Delay(3);
 		        while( y_value < 2500 && y_value > 1000 )
 		        { read_joystic(); HAL_Delay(3);
 		        if (x_value <= 900) { steps++; lcd_put_cur(3,11); lcd_int_to_str(steps); HAL_Delay(75); }
 		    	else if (x_value >= 3000) { steps--; if(steps >= 255) { steps = 0; }}
 		  		lcd_put_cur(3,11);lcd_int_to_str(steps);  HAL_Delay(75);

 		        }  HAL_Delay(100);
 		        break;
	    }
     // }
   }
	 HAL_Delay(100);
	 cursor_pos = 1;

          //////////////  Animation second screen display ////////////
        lcd_clear(); row_pos = 0;
        lcd_put_cur(0, 1); lcd_string_new("Shots:      < >");  lcd_put_cur(1,14); lcd_int_to_str(shots);
        lcd_put_cur(1, 1); lcd_string_new("Delay:     < s>"); lcd_put_cur(1,13); lcd_int_to_str(delay_val);
        lcd_put_cur(2, 1); lcd_string_new("[Start]");
        lcd_put_cur(3, 1); lcd_string_new("<BACK>"); lcd_put_cur(0,0);lcd_string_new(">");
        read_joystic(); HAL_Delay(5);

//      while ( cursor_pos >= 1 && cursor_pos <= 4  )
//
//      {

//        	     	read_joystic(); HAL_Delay(3);
//        	        if ((y_value >= 3000 ) && (y_value <= 4095))
//        	        {
//        	         cursor_pos++;
//        	         lcd_put_cur(row_pos,0); lcd_string_new(" "); if ( row_pos <= 3) { row_pos++; }     //row_pos++;  if (row_pos >= 4) { row_pos = 0; }
//        	         lcd_put_cur(row_pos,0);lcd_string_new(">");   //HAL_Delay(100);
//        	        }      //back_dis();
//        	        else if ((y_value <= 800) && (y_value >= 0 ))
//        	        {
//        	     	 cursor_pos--;
//        	     	 lcd_put_cur(row_pos,0); lcd_string_new(" ");
//        	     	  row_pos--; if(row_pos < 0) { row_pos = 0; }
//        	     	 lcd_put_cur(row_pos,0);lcd_string_new(">");   //HAL_Delay(100);
//        	        }

    	        read_joystic(); HAL_Delay(3);
    	    while( cursor_pos >= 1 &&  cursor_pos <= 4  )
    	   {

    	      read_joystic(); HAL_Delay(5);
    	      if ((y_value <= 2900) && (y_value > 1000)) {   j_toggle1 = 1;  HAL_Delay(2); }

    	      else if ((y_value >= 3000 ) && (y_value <= 4095) && ( j_toggle1 == 1) )
    	       {  cursor_pos++; j_toggle1 = 0;
    	          lcd_put_cur(row_pos,0); lcd_string_new(" ");  if ( row_pos <= 3) { row_pos++; }     // row_pos++;  if (row_pos >= 4) { row_pos = 0; }
    	          lcd_put_cur(row_pos,0); lcd_string_new(">"); // HAL_Delay(100);
    	       }      //back_dis();

    	       else if ((y_value <= 800) && (y_value >= 0 ) && ( j_toggle1 == 1))
    	         { cursor_pos--; j_toggle1 = 0;
    	    	   lcd_put_cur(row_pos,0); lcd_string_new(" ");
    	    	   row_pos--; if(row_pos < 0) { row_pos = 0; }
    	    	   lcd_put_cur(row_pos,0);lcd_string_new(">"); //HAL_Delay(100);
    	         }


        		 switch(row_pos)
        		   {
        		  	 case 0:    //read_joystic(); HAL_Delay(2);
        		  	 	 	 while( y_value <= 2500 && y_value > 1000 )
        		  	 	 	 	{
        		  	 	 		   read_joystic(); HAL_Delay(3);
         		  	 	 	 	if (x_value <= 900)
        		  	 	 	 	 {   shots++;  if (shots > 9) { shots = 0; }
        		  	 	 	 	     lcd_put_cur(0,14); lcd_int_to_str(shots); HAL_Delay(75); }
        		  	 	 	 	else if (x_value >= 3000) { shots--; if(shots >= 255) { shots = 0; } }
        		  	 	 	 	lcd_put_cur(0,14);lcd_int_to_str(shots);  HAL_Delay(75);

        		  	 	 	   } 	HAL_Delay(100);

        		  		 break;

        		   	 case 1:  //read_joystic(); HAL_Delay(1);
        		   	 	 	 while ( y_value < 2500 && y_value > 1000 )
        		   	 	 {
        		   	 	 		 read_joystic(); HAL_Delay(3);
        		   	 	 	 if (x_value <= 900)
        		   	 	 	 {
        		   	 	 	 delay_val++; if (delay_val > 9) { delay_val = 0; }
        		   	 	 	 lcd_put_cur(1,13); lcd_int_to_str(delay_val);
        		   	 	 	 HAL_Delay(75); }

        		   	 	 	 else if (x_value >= 3000)
        		   	 	 	 { delay_val--; if(delay_val >= 255) { delay_val = 0; }
        		   	 	 	 }
        		   	 	 	 lcd_put_cur(1,13); lcd_int_to_str(delay_val);   HAL_Delay(75);

        		   	 	 }  HAL_Delay(100);

        		   		 break;

        		   	 case 2:    ////////// call to the start function
                           while( y_value <= 2500 && y_value > 1000 )
                         {
                            read_joystic(); HAL_Delay(2);
                            button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
                            if (button == 0) { lcd_clear(); start_flag = 1; start_fun();}
                         }
        		   		 break;

        		   	 case 3:  HAL_Delay(75);
        		   		      //read_joystic(); HAL_Delay(2);
           		  	 	  while( y_value <= 2500 && y_value > 1000 )
        		            {
        		   		       read_joystic(); HAL_Delay(2);
        		   		       button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
        		   		      if (button == 0) { auto_unl(); }

        		   		    } HAL_Delay(100);


        		   		  break;

        		    }
        	  //  }
               }

         lcd_clear();
         HAL_Delay(150);

  }
}

void start_fun(void)
{
       int steps_count = 0;
       int shots_count = 0;
       int wait = 0;
       int toggle = 0;
       new_home_pos = home_pos;
       wait = delay_val*1000;
       current_pos = current_pos + home_pos;

  	while (start_flag == 1)
  	{
  	   toggle = 1;
       lcd_put_cur(1, 1); lcd_string_new("ANIM  dir"); lcd_put_cur(1,10);
	   if (motor_lr == 1) { lcd_string_new("R"); } else { lcd_string_new("L");}

//	   if (old_home_pos != new_home_pos)
//	   {
//		   lcd_put_cur(1,12); lcd_int_to_str( new_home_pos);
//		   lcd_put_cur(2, 0); lcd_string_new("Go Home?  N<-->Y");
//		   read_joystic(); HAL_Delay(2);
//	    while( x_value <= 2500 && x_value > 1000 )
//		   {    read_joystic(); HAL_Delay(2);
//		       if (x_value <= 900) // Yes
//		          {  lcd_put_cur(1,12); float_to_string( new_home_pos); }
//
//		     else if (x_value >= 3000)  // No
//		     	  {	lcd_put_cur(1,12); float_to_string( old_home_pos ); }
//		   }
//         }
//
//	   else {  lcd_put_cur(1,12); lcd_int_to_str(old_home_pos );  }


	   lcd_put_cur(1,12); float_to_string(current_pos);
	   lcd_put_cur(2,0); lcd_string_new(" /            / ");
	   lcd_put_cur(2,0); lcd_int_to_str(steps_count);
	   lcd_put_cur(2,2); lcd_int_to_str(steps);
	   lcd_put_cur(2,8); float_to_string(step);
	   lcd_put_cur(2,13); lcd_int_to_str(shots_count);
	   lcd_put_cur(2,15); lcd_int_to_str(shots);

	   button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	   HAL_Delay(1);

	   while(button != 0 && toggle == 0 )	 /////// pause and exit scan
	{

		button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
		read_joystic(); HAL_Delay(2);

		if (steps_count <= steps )
		 {
		   for ( shots_count = 0; shots_count < shots; shots_count++ )
			{
			    lcd_put_cur(2,13); lcd_int_to_str(shots_count);
			    button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
			    HAL_Delay(wait);

			}
		  }

             read_joystic(); HAL_Delay(2);
        while( x_value <= 2500 && x_value > 1000 && toggle == 0)
         {
        	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
        	read_joystic(); HAL_Delay(2);
	       if (x_value <= 900) // Yes
	          {
	    	   if(steps_count < steps) { steps_count++; lcd_put_cur(2,0); lcd_int_to_str(steps_count); }
	    	   current_pos = current_pos + step;
	    	   HAL_Delay(50);
	          }

	        else if (x_value >= 3000)  // No
	     	  {
	    	    if(steps_count > 0){ steps_count--; }
	    	    lcd_put_cur(2,0); lcd_int_to_str(steps_count);
	    	    current_pos = current_pos - step;
	    	    HAL_Delay(50);
	     	  }

	            if (button == 0) { toggle = 1;}


         }
	                          HAL_Delay(1);
                              if (button == 0)
	    	    	 	{

	    	    			lcd_clear(); lcd_put_cur(1, 5); lcd_string_new("PAUSE");  lcd_put_cur(1,14); lcd_int_to_str(shots);
	    	    			lcd_put_cur(1, 0); lcd_string_new("Press OK To Exit");
	    	    			button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	    	    			HAL_Delay(250);
	    	    			button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	    	    	/////   button press second time      /////
	    	    			if(button == 0) { animation_fun(); }
	    	             }

         //HAL_Delay(150);
          lcd_put_cur(1,12); lcd_int_to_str(current_pos );

      }



	    lcd_clear();
	 //   HAL_Delay(100);

  	}
	///// call back to the animation mode


  	HAL_Delay(2);
  	animation_fun();
}






//*********** config from the function *************///

void config_fun(void)
{

  man_flag = 0;
  exit_flag = 1;
  unl_flag = 0;
  lcd_clear();
  // home_scr();
  HAL_Delay(5);
  re_calibration();
 }








//******************* limits are unlimited function *******************
///// when presson on the UNL button

void auto_unl()
{
	exit_flag = 0;
//	unl_flag = 1;
//	hold = 1;
	row_pos = 0;
	//lcd_put_cur(0, 0); lcd_string_new("** limits are **");
	//lcd_put_cur(1, 0); lcd_string_new("** unlimited  **");
	//HAL_Delay(500);
	lcd_clear();
	lcd_put_cur(0, 1);lcd_string_new("LIVE");lcd_put_cur(1, 1); lcd_string_new("TIMELAPSE");
	lcd_put_cur(2, 1);lcd_string_new("ANIMATION");lcd_put_cur(3, 1); lcd_string_new("CONFIG");
	lcd_put_cur(0,0);lcd_string_new(">");

	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	read_joystic(); HAL_Delay(5);
	HAL_Delay(500); // switch de bounce dealy

  while( button != 0 )
 {
    button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
    read_joystic(); HAL_Delay(5);

   if (y_value >= 3000)
    {
	   lcd_put_cur(row_pos,0); lcd_string_new(" ");
        row_pos++;

        if (row_pos >= 4) { row_pos = 0; }

      lcd_put_cur(row_pos,0);  lcd_string_new(">");
      HAL_Delay(200);
    }

  }

  HAL_Delay(5);

 switch(row_pos)
  {
 	 case 0:   	// int raw = 0; // live_fun(raw);
 		 live_fun();
 	 break;

 	 case 1:
 		 timelapse_fun();
 	 break;

 	 case 2:
 		 animation_fun();
 	 break;

 	 case 3:
 		 config_fun();
 	 break;
 	 }

    unl_flag = 0;

  }


//////// work range function and their sub functions


void man_fun()
{
	man_mode = 0;
//	char current_pos;
	//HAL_Delay(500);
	lcd_put_cur(1, 0); lcd_string_new("1st Limit "); lcd_put_cur(1, 10); float_to_string(limit_1);
	lcd_put_cur(2, 0); lcd_string_new("<Move> & PressOK");
    HAL_Delay(1000);  // button de bounce delay
	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
    read_joystic(); HAL_Delay(1);
    motor_limit = motor_step;

	while ( button != 0 )
 {

	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	read_joystic(); HAL_Delay(2);

	if ((x_value <= 700) &&  (x_value >= 0 ))  //// 1st limit increment
	   {

	    	 limit_1 = limit_1 + 0.1; lcd_put_cur(1, 10); float_to_string(limit_1);
	    	 HAL_Delay(150);  // joy stic movement dealy

	    }

	 else if ((x_value >= 3000)  &&  (x_value <= 4095)) // 1st limit decrement
	    {
	       limit_1 = limit_1 - 0.1; lcd_put_cur(1, 10); float_to_string(limit_1);
	       HAL_Delay(150); // joy stic movement delay
	    }
    }

   ///////////// second limit setting

	     limit_2 = limit_1;
	     lcd_put_cur(1, 0);lcd_string_new("2nd Limit "); lcd_put_cur(1, 10); float_to_string(limit_2);
	     HAL_Delay(1000);
	     button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	     read_joystic(); HAL_Delay(2);
	     // button de bounce dealy for second limit

    while ( button != 0 )    /// setting the value of second limit
   {
         button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
         read_joystic(); HAL_Delay(2);

    if ((x_value <= 700) &&  (x_value >= 0))//// 2nd limit increment
        {

		     limit_2 = limit_2 + 0.1; lcd_put_cur(1, 10); float_to_string(limit_2);
		     HAL_Delay(150);

        }
    else if ((x_value >= 3000) && (x_value <= 4095))
    {
    	    limit_2 = limit_2 - 0.1; lcd_put_cur(1, 10); float_to_string(limit_2);
	        HAL_Delay(150);
    }

   }

 // ****** calculation of the motor steps from limit_1 to limit_2
         motor_limit = limit_2 - limit_1;
         lcd_clear();
         lcd_put_cur(1, 0);  lcd_string_new("** limits are **");
         lcd_put_cur(2, 2);  lcd_string_new("0  to");
         lcd_put_cur(2, 10); float_to_string(motor_limit);

 //*********  motor direction change function string ************ //

     if(motor_step > 0.0) {
        	 motor_dir = 2; lcd_put_cur(2, 13); lcd_string_new("M L");
         }
     if (motor_step < 0.0){
        	 motor_dir = 2; lcd_put_cur(2, 13); lcd_string_new("M R");
         }
             HAL_Delay(1000);
             lcd_clear();

// ********  call to free_ride_fun function
            man_flag = 1;
            auto_unl();    // unl_flag = 1;  // set the unl flag 1 to call the unlimited limit function


}





//********* UNL Function ************

void unl_fun()
{
	hold = 0;
	lcd_clear();
	HAL_Delay(1);
	auto_unl();   // call to the aut_unl function again
//	HAL_Delay(1);

}

 void work_range()
 {
  //  hold = 1;
    int cur_pos = 1;
    lcd_put_cur(1, 3);lcd_string_new("WORK");lcd_put_cur(1, 8); lcd_string_new("RANGE");
    lcd_put_cur(2, 3);lcd_string_new("<MAN>");lcd_put_cur(2, 10); lcd_string_new(" UNL ");
    button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);

    HAL_Delay(100);

while (  button != 0 )
  {
     button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
     read_joystic(); HAL_Delay(1);

     if ((x_value <= 700)  && (x_value >= 0))
        {  cur_pos = 2;
  	       lcd_put_cur(2, 3);lcd_string_new(" MAN ");lcd_put_cur(2, 10); lcd_string_new("<UNL>");
        }

     else if ((x_value >=3000) && (x_value <= 4095))
        { cur_pos = 1;
  		  lcd_put_cur(2, 3); lcd_string_new("<MAN>"); lcd_put_cur(2, 10); lcd_string_new(" UNL ");
        }

    }

      switch (cur_pos)
      {
        case 1:
    	   man_fun();    //auto_unl();  //man_flag = 1; unl_flag = 0;  break;

    	   break;

        case 2:
          lcd_clear(); lcd_put_cur(0, 0); lcd_string_new("** limits are **");
          lcd_put_cur(1, 0); lcd_string_new("** unlimited  **");
          HAL_Delay(500);  auto_unl(); HAL_Delay(5);
    	  //unl_flag = 1; man_flag = 0;  break;
         break ;
      }

//        if ( cur_pos == 1 )   // curson on the MAN
//           {   man_flag = 1; unl_flag = 0;  //  man_fun();
//    	   }  ///////// call to the man function
//
//        if (cur_pos  == 2)
//           { unl_flag = 1; man_flag = 0;  unl_fun();
//           }   //////   call to the unl function


  }


 void re_calibration()
 {

	  home_scr();
	 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	 read_joystic(); HAL_Delay(2);

	  while ( button != 0 )
	  {     button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin); read_joystic(); HAL_Delay(2);

    	  if ((x_value <=700) && (x_value >= 0))
	 		  {  home = 2; lcd_put_cur(2, 2); lcd_string_new(" NO "); lcd_put_cur(2, 9); lcd_string_new("<YES>");
	 		     HAL_Delay(1);  }

	 	  else if ((x_value >= 3500) && (x_value < 4095))
	 		  {  home = 1; lcd_put_cur(2, 2);lcd_string_new("<NO>");lcd_put_cur(2, 9); lcd_string_new(" YES ");
	 		     HAL_Delay(1);
	 		  }
	   }

	 		switch (home)
	 	  {
	 		case 1:
	 			lcd_clear(); //jpos = 1;
	 			 lcd_put_cur(0, 0); lcd_string_new("** limits are **");
	 			 lcd_put_cur(1, 0); lcd_string_new("** unlimited  **");
	 			 HAL_Delay(500);  auto_unl(); HAL_Delay(5);
	 			break;

	 		case 2:
	 			  lcd_clear(); jpos = 1;  work_range(); HAL_Delay(1); // callling to the work range function

	 		  if ( man_flag == 1 ) { lcd_clear(); man_flag = 0;   man_fun(); } /// calling to the man function

	 		  if ( unl_flag == 0 )  { lcd_clear(); auto_unl();     }

   			 break; 	HAL_Delay(5);
	 	 }
	       HAL_Delay(50);

 }


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  lcd_init();
  lcd_clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(jbtn_GPIO_Port, joy_btn_Pin, 1);
  lcd_put_cur(1, 0); lcd_string_new("CAMERA SLIDER 03");
  HAL_Delay(1); lcd_put_cur(2, 0); lcd_string_new("ROUND SHAPE SLIDE");
  HAL_Delay(2000); lcd_clear();




  while (1)
  {
//       	send_buffer[02] = speed_send;
//	    while(state_of_rs485 != 1);
//	    send_on_rs485(send_buffer);
//	    HAL_Delay(100);




//	  while(1)
//	  {
//		 // data_transmit();
//		//  rx_data();
//		  rx_motor_step = 0x1F400;
//		  motor_step = rx_motor_step /2560;
//		  lcd_put_cur(1, 10);  float_to_string(motor_step);
//
//	  }

	 // ******** joy stic and button readin g***********


	      home_scr();
	      button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	  	  read_joystic(); HAL_Delay(2);

	  //********** joy stic push button scanning ************

	  	 while ( button != 0 )
	     {
	  		button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	  		read_joystic(); HAL_Delay(2);

	  	  if ((x_value <=700) && (x_value >= 0))
	  		  {
	  		    home = 2; lcd_put_cur(2, 2); lcd_string_new(" NO "); lcd_put_cur(2, 9); lcd_string_new("<YES>");
	  		    HAL_Delay(1);
	  		  }

	  	  else if ((x_value >= 3500) && (x_value < 4095))
	  		  {
	  		     home = 1; lcd_put_cur(2, 2);lcd_string_new("<NO>");lcd_put_cur(2, 9); lcd_string_new(" YES ");
	  		     HAL_Delay(1);
	  		  }
	      }

	  		switch (home)
	  	  {
	  		case 1:
	  			lcd_clear(); //jpos = 1;
	  			lcd_put_cur(0, 0); lcd_string_new("** limits are **");
	  			lcd_put_cur(1, 0); lcd_string_new("** unlimited  **");
	  			HAL_Delay(500);  auto_unl(); HAL_Delay(5);
	  			break;

	  		case 2:
	  			  lcd_clear(); jpos = 1;  work_range();  // callling to the work range function
	  			  HAL_Delay(1);
	  			  auto_unl();

	              break;
	         }


	        HAL_Delay(50);




	  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	        ////////////  485   ///////////

	  	  //if (buffer[0]=='U')
	  	  //{HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, RJ45_LED_Y_Pin);HAL_Delay(500);
	  	  //HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, RJ45_LED_Y_Pin);HAL_Delay(500);
	  	  //HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, RJ45_LED_Y_Pin);HAL_Delay(500);
	  	  //
	  	  //}
	  	  HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, RJ45_LED_Y_Pin);HAL_Delay(500);


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|JS_SW_Pin|RJ45_LED_G_Pin|RJ45_LED_Y_Pin
                          |LCD_BRIGHTNESS_Pin|LCD_RS_Pin|LCD_E_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin JS_SW_Pin RJ45_LED_G_Pin RJ45_LED_Y_Pin
                           LCD_BRIGHTNESS_Pin LCD_RS_Pin LCD_E_Pin LCD_D4_Pin
                           LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|JS_SW_Pin|RJ45_LED_G_Pin|RJ45_LED_Y_Pin
                          |LCD_BRIGHTNESS_Pin|LCD_RS_Pin|LCD_E_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX_EN_Pin */
  GPIO_InitStruct.Pin = MAX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAX_EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void recieve_on_rs485(uint8_t buffer[],size_t buffer_length)
{
	HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, 0);
	HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, 0);
	HAL_UART_Receive_IT(&huart1, buffer, buffer_length);

}
void send_on_rs485(uint8_t* buffer)
{

     	HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, 1);
		HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, 1);
		message_packet_with_crc(buffer,  POLYNOMIAL,(BUFFER_LENGTH-1));
		strcpy( Previous_buffer,buffer);
//		if(cnt<1){
//		buffer[2]=0x0d;cnt++;}
//		else{buffer[2]=0x2d;}
		HAL_UART_Transmit(&huart1, buffer,BUFFER_LENGTH,100);
		//HAL_UART_Transmit(&huart1, "driver1\r\n", 9,100);
	//	HAL_Delay(1);
		recieve_on_rs485(recieve_buffer,BUFFER_LENGTH);

}


crc_t CRC_generate(crc_t* message1, crc_t polynomial, int nBytes )
{
	crc_t  remainder = 0;
	for (int byte = 0; byte < nBytes; ++byte)
	{
		remainder ^= ((*(message1+byte)) << (WIDTH - 8));/* Bring the next byte into the remainder.   */
		//printf("Hello World %x \n" ,remainder);
		/* Perform modulo-2 division, a bit at a time. */
		for (uint8_t bit = 8; bit > 0; --bit )
		{
			/*Try to divide the current data bit.*/
			if (remainder & TOPBIT){remainder = (remainder ^ POLYNOMIAL)<<1;}
			else{remainder = (remainder << 1);}
		}
	}
	return (remainder);/* The final remainder is the CRC result. */
}

uint8_t* message_packet_with_crc(crc_t* message2, crc_t polynomial, int nBytes )
{
	uint8_t crc_value=0;
	crc_value = CRC_generate( message2,  polynomial,  nBytes );
	message2+=nBytes;
	*message2=crc_value;
	return message2-=nBytes;
}


crc_t CRC_CHECK_decode(crc_t* message, crc_t polynomial, int nBytes )
{
	crc_t  remainder = 0;
	for (int byte = 0; byte < nBytes; ++byte)
	{
		remainder ^= ((*(message+byte)) << (WIDTH - 8));/* Bring the next byte into the remainder.   */
		//printf("Hello World %x \n" ,remainder);
		/* Perform modulo-2 division, a bit at a time. */
		for (uint8_t bit = 8; bit > 0; --bit )
		{
			/*Try to divide the current data bit.*/
			if (remainder & TOPBIT){remainder = (remainder ^ POLYNOMIAL)<<1;}
			else{remainder = (remainder << 1);}
		}
	}
	//printf(" xcv %x \n" ,remainder);
	if(remainder!=0){return 1;}
	else {return 0;}
//	return (remainder);/* The final remainder is the CRC result. */
}



//void USART1_IRQHandler(void)
//{
//counter++;
//	HAL_UART_Receive_IT(&huart1, buffer, 16);
//	//HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, LED_Pin);
////	if (buffer == 0x55)
////	{
////		HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, LED_Pin);
////
////	}
//
//
//
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

		counter++;
		RS_485_Data_validate = CRC_CHECK_decode(recieve_buffer,  POLYNOMIAL, BUFFER_LENGTH); //returns 0 if valid ,1 if invalid.

			if(RS_485_Data_validate==0)
			{
				if(recieve_buffer[0]== 0x55)// if acknowledgment is valid
				{
					strcpy( RECIEVE_VALID_DATA,recieve_buffer);//store data into actual data buffer
					send_buffer[0]=0x55;//0x55 is acknowledgment for a valid data
					//send_on_rs485(send_buffer);

					//  Seprate  the buffer byte according to the their value
					state_of_rs485=1;
				}
				else if(recieve_buffer[0]== 0x65)// if acknowledgment is 0x65 means data is not received properly
				{

							//send previous data
					send_buffer[0]=0x55;
					Previous_buffer[0]=0x55;
					send_on_rs485(Previous_buffer);
					state_of_rs485=2;
				}

				// send acknowledgment of valid data
		//		send_buffer[0]=0x55;//0x55 is acknowledgment for a valid data


			}
			else
			{
				//neglect the data
				// send acknowledgment of invalid data
				send_buffer[0]=0x65;
				send_on_rs485(send_buffer);
				state_of_rs485=3;
			}
		//	send_on_rs485(send_buffer);


}
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
//	while(1){
//	HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, LED_Pin);HAL_Delay(100);
//
//
//	}
//
//
//}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
