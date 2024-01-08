/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <icons.h>
#include <ssd1306.h>
#include <ssd1306_conf.h>
#include <ssd1306_fonts.h>
#include<stdio.h>
#include <stdbool.h>
#include<string.h>
#include<stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Kp 4.33 // 2.05 for 300
//4.05 4 3.8 3.75 3.65 3.5  for 500.
// 				for 700
// 4.20				for 400
// 4.33 for 450
#define Ki 0	//0for every
#define Kd 50 //23 for 300
//53.5 45.5 40.5 for 500
// 60 for 400
// 50 for 450
#define Default_speed 450
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;


/* USER CODE BEGIN PV */

float Present_Voltage = 0.00;
float Battery_voltage = 0;
uint16_t ADC_Data = 0;
float D_Drop_Voltage = 0.09 ;
float Max_Bat_Voltage = 12.6;
float Weak_Bat_Voltage = 10.8;
float Number_of_Resister = 4;
uint8_t bluetooth_status = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//bluetooth variables
UART_HandleTypeDef huart1;
uint8_t u8_RxBuff[50]; // buffer luu chuoi nhan duoc
uint8_t u8_RxData; // luu byte nhan duoc
uint8_t u8_TxBuff[50] = "Hello PA\n"; // buffer truyen di
uint8_t _rxIndex; // con tro cua rxbuff
uint16_t Tx_Flag = 0;
uint8_t u8_condition;
    char result_str[50];
    char *arg_list[50];
uint8_t u8_start = 0;
float Ki_value , Kp_value , Kd_value ;
int Speed_value = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if (huart->Instance == USART1)
    {
      if (u8_RxData != 13) // NULL ASCII
      {
        u8_RxBuff[_rxIndex++] = u8_RxData; // Them data vao rxbuffer
      }
      else if(u8_RxData == 13)
      {
        _rxIndex = 0;// reset lai con tro
        Tx_Flag = 1;
      }
      HAL_UART_Receive_IT(&huart1, &u8_RxData, 1);
    }
}
void UART_Handle_Menu()
{
	// Cut string

	    char *temp_token = strtok((char *)u8_RxBuff, " ");
	    uint8_t _Index_arg = 0;

	    while(temp_token != NULL)
	    {
	      arg_list[_Index_arg] = temp_token;

	      _Index_arg = _Index_arg+1;
	      temp_token = temp_token+1;
	      temp_token = strtok(NULL, " ");
	    }
	    // Handle
	    if(strstr(arg_list[0], "M")!= NULL)
	          {
	            if(strstr(arg_list[1], "START" )!= NULL)
	            {
	              u8_start = 1;
	              sprintf(result_str, "OK = %d\r\n", u8_start);
	    		  HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);

	             } else if (strstr(arg_list[1], "STOP" )!= NULL){
	              u8_start = 0;
	              sprintf(result_str, "OK = %d\r\n", u8_start);
	    		  HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);
	             }
	          }

	    if(strstr(arg_list[0], "KI") != NULL)
	    {
	      u8_condition = 1;
	      sprintf(result_str, "OK KI = ?\r\n");
	      HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);
	    } else if (strstr(arg_list[0], "KP") != NULL)
	    {
	      u8_condition = 2;
	      sprintf(result_str, "OK KP = ?\r\n");
	      HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);
	    } else if (strstr(arg_list[0], "KD") != NULL)
	    {
	      u8_condition = 3;
	      sprintf(result_str, "OK KD = ?\r\n");
	      HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);
	    } else if (strstr(arg_list[0], "Speed") != NULL)
	    {
	      u8_condition = 4;
	      sprintf(result_str, "OK Speed = ?\r\n");
	      HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);
	    } else {
	      sprintf(result_str, "Invalid \r\n");
	      HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);
	    }


	    sprintf(result_str, "Condition:%d\r\n", u8_condition);
	    HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);


	    switch (u8_condition)
	  {
	   case 1:
			Ki_value = atof(arg_list[1]);
			sprintf(result_str, "OK KI = %0.5f\r\n", Ki_value);
			HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);
			u8_condition = 0;
			break;
	  case 2:
		  Kp_value = atof(arg_list[1]);
		  sprintf(result_str, "OK KP = %0.5f\r\n", Kp_value);
		  HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);
		  u8_condition = 0;
		break;
	  case 3:
		Kd_value = atof(arg_list[1]);
		sprintf(result_str, "OK KD = %0.5f\r\n", Kd_value);
		HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);
		u8_condition = 0;
		break;
	  case 4:
		  Speed_value = atoi(arg_list[1]);
		  sprintf(result_str, "OK Speed = %d\r\n", Speed_value);
		  HAL_UART_Transmit(&huart1, (uint8_t *)result_str, strlen(result_str), HAL_MAX_DELAY);
		  u8_condition = 0;
		break;
	  }
}

/* Control Motor using timer */
void My_Motor_Control(int32_t left_vel ,int32_t right_vel)
{
	// Left motor control
	if(left_vel < 0)
	{
		TIM4->CCR1 = left_vel;
		TIM4->CCR2 = 0;
	}
	else
	{
		TIM4->CCR1 = 0;
		TIM4->CCR2 = left_vel;
	}

	//Right motor control
	if(right_vel < 0)
	{
//		TIM4->CCR4 = right_vel;
//		TIM4->CCR3 = 0;
		TIM4->CCR4 = 0;
		TIM4->CCR3 = right_vel;
	}
	else
	{
//		TIM4->CCR4 = 0;
//		TIM4->CCR3 = right_vel;
		TIM4->CCR4 = right_vel;
		TIM4->CCR3 = 0;
	}
}

void my_Oled_Display(float Kp_val , float Ki_val , float Kd_val , uint16_t M_Speed_val );

/* Oled part */
//void my_Low_Power_Alert()
//{
//	  Battery_voltage =  (float)HAL_ADC_GetValue(&hadc1)*3.3*4 /( 4096*(Max_Bat_Voltage - D_Drop_Voltage) ) *100;
//	 if(Battery_voltage < 25.0 )
//	 {
//		 HAL_Delay(10);
//		 my_Oled_Display(Kp , Ki , Kd , Default_speed);
//	  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
//
//	 }else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
//}

void Display_PID(float Kp_val , float Ki_val , float Kd_val , uint16_t M_Speed_val )
{
	  char Kp_Val_Str[6] ;
	  char Ki_Val_Str[6] ;
	  char Kd_Val_Str[6] ;
	  char M_Speed_Val_Str[6];

	  /* Display PID value */
	  sprintf(Kp_Val_Str , "%.3f" , Kp_val);
	  sprintf(Ki_Val_Str , "%.3f" , Ki_val);
	  sprintf(Kd_Val_Str , "%.3f" , Kd_val);
	  sprintf(M_Speed_Val_Str , "%d" , M_Speed_val);

/* Display Kp */
	  ssd1306_SetCursor(75 , 20);
	  ssd1306_WriteString("P: ",Font_6x8,White);
	  ssd1306_WriteString(Kp_Val_Str,Font_6x8,White);

/* Display Ki */
	  ssd1306_SetCursor(75 , 30);
	  ssd1306_WriteString("I: ",Font_6x8,White);
	  ssd1306_WriteString(Ki_Val_Str,Font_6x8,White);

/* Display Kd */
	  ssd1306_SetCursor(75 , 40);
	  ssd1306_WriteString("D: ",Font_6x8,White);
	  ssd1306_WriteString(Kd_Val_Str,Font_6x8,White);

	  /* Display M_Speed Value */
	  ssd1306_SetCursor(75, 50);
	  ssd1306_WriteString("Sp:",Font_6x8,White);
	  ssd1306_WriteString(M_Speed_Val_Str,Font_6x8,White);
}

void Display_Battery()
{

	Battery_voltage =  ((float)HAL_ADC_GetValue(&hadc1)*3.3*4 /4096);

	char BatStr[9] ;
	/* Display battery status */
		  sprintf(BatStr, "%.0f", Battery_voltage);
		  if (Battery_voltage <= 10.8)
		  {
			  ssd1306_DrawBitmap(2, 17, battery_5 , 25 ,25 ,White); //Bat [0 - 5%]
			  ssd1306_DrawBitmap(115, 1, plug , 16, 16, White);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
		  }
		  else if (Battery_voltage <= 11.25 && Battery_voltage >= 10.8)
		  {
			  ssd1306_DrawBitmap(2, 17, battery_4 , 25, 25, White); //Bat [5 - 25%]
			  ssd1306_DrawBitmap(115, 1, plug , 16, 16, White);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);

		  }
		  else if (Battery_voltage <= 11.7 && Battery_voltage >=11.25)
		  {
			  ssd1306_DrawBitmap(2, 17, battery_3 , 25, 25, White); //Bat [25 - 50%]
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
		  }
		  else if (Battery_voltage <= 12.15 && Battery_voltage >= 11.7)
		  {
			  ssd1306_DrawBitmap(2, 17, battery_2 , 25, 25, White); //Bat [50 - 75%]
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
		  }
		  else
		  {
			  ssd1306_DrawBitmap(2, 17, battery_1 , 25, 25, White); //Bat [75 - 100%]
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
		  }

		  ssd1306_SetCursor(30 , 21);
		  ssd1306_WriteString(BatStr,Font_11x18 , White);
		  ssd1306_WriteString("v",Font_7x10 , White);

}

void Display_Frame()
{
	/* Draw Frames */
		  ssd1306_Line(1, 17, 128, 17, White);
		  ssd1306_Line(70, 18, 70, 64, White);
		  ssd1306_Line(1, 41, 69, 41,  White);

}

void Display_Logo()
{
	/* Display Logo */
		  ssd1306_DrawBitmap(1, 1, LogoPIFAVENGER , 90, 16, White); //Logo

}

void Display_Bluetooth_status()
{
	  /* Display Bluetooth Status*/
	  if(1)
	  {
	  	  ssd1306_DrawBitmap(107, 4, bluetooth , 10, 10, White);
	  }

}


void my_Oled_Display(float Kp_val , float Ki_val , float Kd_val , uint16_t M_Speed_val )
{

	  bluetooth_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	  ssd1306_Fill(Black);
	  Display_Logo();
	  Display_Frame();
	  Display_Bluetooth_status(bluetooth_status);
	  Display_Battery();
	  ssd1306_DrawBitmap(1 , 40 , Bug , 59 , 26 ,White);
	  Display_PID(Kp_val,Ki_val,Kd_val,M_Speed_val);

/* Initialize Screen */
	  ssd1306_UpdateScreen();


}

uint16_t sum = 0;
uint16_t s[5];

void sensorRead() {
	  s[0] = (GPIOA->IDR >> 0) & 0x01;
	  s[1] = (GPIOA->IDR >> 1) & 0x01;
	  s[2] = (GPIOA->IDR >> 2) & 0x01;
	  s[3] = (GPIOA->IDR >> 3) & 0x01;
	  s[4] = (GPIOA->IDR >> 4) & 0x01;
}

uint16_t old_d;
uint16_t Distance(uint16_t sensor_value []) {
	uint16_t d = 0;
	char i = 0;
	//calibration step
	for(char k = 1; k < 6; k++) {
		if(!sensor_value[k-1])	// black line
		{
			i++;
			d += k*100;
		}
	}
	if(i>0 && i<5) {
		old_d = d/i;
		return old_d;
	}
	else if (i==5) //when all sensor detect black line --- very special case
	{
		return 300;	// keep moving forward
	}
	else	//no line? -- khi ma xe no chay lo qua duong vach vi du vo cua gap
	{
		if(old_d < 200 && old_d < 400) {
			return 300;
		}
		else
			return old_d;	// thi nho lai truong hop truoc do ma tiep tuc di nhu vay
		// vi du cua gap ma queo phai ma bi lo thi cu tiep tuc chay nhu trang thai truoc do
	}
	//06-12-23 -- chua giai quyet duoc truong hop goc nhon or goc vuong.
}

void PID_control(float _Kp, float _Ki, float _Kd, uint16_t base_speed, uint16_t distance) {
		int distance_error;
		static int old_distance_error = 0;
		static float I = 0;
		float P = 0, D = 0, Total = 0;
		int16_t increased_speed = 0, decreased_speed = 0;

		distance_error = 300 - distance;
		P = _Kp * distance_error ;

//		D = _Kd * (distance_error - old_distance_error)/5;
		D = _Kd * (distance_error - old_distance_error);

		old_distance_error = distance_error;

		if((-100 < distance_error) && ( distance_error<100))
			I = I + (_Ki * distance_error);
		else
			_Ki = 0;

		Total = P + I + D;

		////************************
		increased_speed = base_speed + ((Total > 0) ? Total : -Total);
		decreased_speed = base_speed - ((Total > 0) ? Total : -Total);

		// Ensure the motors do not go below 0 or above their set max
//		if(increased_speed < 0)
//			increased_speed = 0;
//		if(increased_speed > 900)
//			increased_speed = 900;
//		if(decreased_speed < 0)
//			decreased_speed = 0;
//		if(decreased_speed > 900)
//			decreased_speed = 900;

		// Motor control
		if (distance_error > 0)
		{
			My_Motor_Control(decreased_speed, increased_speed);
		}
		else if (distance_error < 0)
		{
			My_Motor_Control(increased_speed, decreased_speed);
		}
		else
		{
			My_Motor_Control(base_speed , base_speed);
		}
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
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
//  HAL_UART_Receive_IT(&huart1,&rxData,1);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  ssd1306_Init();
  my_Oled_Display(Kp , Ki , Kd , Default_speed);

  //for bluetooth mode
//   my_Oled_Display(Kp_value, Ki_value, Kd_value, Speed_value);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart1, u8_TxBuff, sizeof(u8_TxBuff), 100);
  while (1)
  {
	  //for bluetooth
	  		 HAL_UART_Receive_IT(&huart1, &u8_RxData, 1);

			 if (Tx_Flag)
			  {
				for(int i = 0; i < 50; i++) // copy data tu Rx sang Tx
			  {
				u8_TxBuff[i] = u8_RxBuff[i];
				}
				//UART_Handle_Condition();
				UART_Handle_Menu();
				Tx_Flag = 0;
			  }

			 while (u8_start == 1) {
				 sensorRead();
				 sum = Distance(s);
				 // Bluetooth tuning
				 //	PID_control(Kp_value, 0, 0, 700, sum);
				 // normal tuning.
				 PID_control(Kp, Ki, Kd, Default_speed, sum);

		  		 HAL_UART_Receive_IT(&huart1, &u8_RxData, 1);

				 if(Tx_Flag)
				  {
					for(int i = 0; i < 50; i++) // copy data tu Rx sang Tx
				  {
					u8_TxBuff[i] = u8_RxBuff[i];
					}
					//UART_Handle_Condition();
				 UART_Handle_Menu();
		  		 Tx_Flag = 0;
				  }
			 }

			 PID_control(0, 0, 0, 0, 0);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	  }
  }
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 159;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//ADC Interrupt
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	  if(hadc->Instance == hadc1.Instance)
	  {
		  ADC_Data =  HAL_ADC_GetValue(&hadc1);

	  }
}

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
