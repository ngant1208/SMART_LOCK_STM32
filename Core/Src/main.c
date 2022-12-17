/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void LCD_init(void); // thiet lap lcd

void LCD_command(uint8_t value); // dua lenh vao lcd

void LCD_data(uint8_t value); // dua du lieu vao lcd

void LCD_string(uint8_t *str); // hien thi chuoi tren lcd
 void Keys_Detect(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	uint16_t data_pins[8]; // bien toan cuc
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
  /* USER CODE BEGIN 2 */
	data_pins[0] = D0_Pin;
  data_pins[1] = D1_Pin;
  data_pins[2] = D2_Pin;
  data_pins[3] = D3_Pin; 
  data_pins[4] = D4_Pin;
  data_pins[5] = D5_Pin;
  data_pins[6] = D6_Pin;
  data_pins[7] = D7_Pin;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LCD_init();
	
	HAL_Delay(1000);
	
	LCD_string((uint8_t *)"you press key");

                      
  while (1)
  {
    /* USER CODE END WHILE */
     Keys_Detect();
		 HAL_Delay(100);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
void Keys_Detect(void)
{
	// set hang 1 len muc cao
	HAL_GPIO_WritePin(GPIOB, R1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, R2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, R3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, R4_Pin, GPIO_PIN_RESET);
	
	if(HAL_GPIO_ReadPin(GPIOB, C1_Pin) == GPIO_PIN_SET)
	{
		/* 7 */
		
		  LCD_command(0x38);         // set lcd o che do 8 bit             
	    LCD_command(0XC0);              // dua con tro tro ve vi tri dau tien cua hang 2       
	    LCD_string((uint8_t *)"Key 7");  //hien key an
		while (HAL_GPIO_ReadPin (GPIOB, C1_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C2_Pin) == GPIO_PIN_SET)
	{
		/* 8 */

		  LCD_command(0x38);                       
	    LCD_command(0XC0);                      
	    LCD_string((uint8_t *)"Key 8");
		while (HAL_GPIO_ReadPin (GPIOB, C2_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C3_Pin) == GPIO_PIN_SET)
	{
		/* 9 */
		  LCD_command(0x38);                      
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key 9");
		while (HAL_GPIO_ReadPin (GPIOB, C3_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C4_Pin) == GPIO_PIN_SET)
	{
		/* A */
		  LCD_command(0x38);                                           
	    LCD_command(0XC0);                     
	    LCD_string((uint8_t *)"Key /");
		while (HAL_GPIO_ReadPin (GPIOB, C4_Pin));
	}
	
	
	HAL_GPIO_WritePin(GPIOB, R1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, R2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, R3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, R4_Pin, GPIO_PIN_RESET);
	
	if(HAL_GPIO_ReadPin(GPIOB, C1_Pin) == GPIO_PIN_SET)
	{
		/* 4 */
		  LCD_command(0x38);                       
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key 4");
		while (HAL_GPIO_ReadPin (GPIOB, C1_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C2_Pin) == GPIO_PIN_SET)
	{
		/* 5 */
		  
		  LCD_command(0x38);                       
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key 5");
		while (HAL_GPIO_ReadPin (GPIOB, C2_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C3_Pin) == GPIO_PIN_SET)
	{
		/* 6 */
		 

		  LCD_command(0x38);                     
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key 6");
		while (HAL_GPIO_ReadPin (GPIOB, C3_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C4_Pin) == GPIO_PIN_SET)
	{
		/* B */
		  

		  LCD_command(0x38);                      
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key x");
		while (HAL_GPIO_ReadPin (GPIOB, C4_Pin));
	}
	

	HAL_GPIO_WritePin(GPIOB, R1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, R2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, R3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, R4_Pin, GPIO_PIN_RESET);
	
	if(HAL_GPIO_ReadPin(GPIOB, C1_Pin) == GPIO_PIN_SET)
	{
		/* 1 */
		  

		  LCD_command(0x38);                       
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key 1");
		while (HAL_GPIO_ReadPin (GPIOB, C1_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C2_Pin) == GPIO_PIN_SET)
	{
		/* 2 */
		  

		  LCD_command(0x38);                       
	    LCD_command(0XC0);                      
	    LCD_string((uint8_t *)"Key 2");
		while (HAL_GPIO_ReadPin (GPIOB, C2_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C3_Pin) == GPIO_PIN_SET)
	{
		/* 3 */
		  

		  LCD_command(0x38);                       
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key 3");
		while (HAL_GPIO_ReadPin (GPIOB, C3_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C4_Pin) == GPIO_PIN_SET)
	{
		/* - */
		 
		  LCD_command(0x38);                       
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key -");
		while (HAL_GPIO_ReadPin (GPIOB, C4_Pin));
	}


	HAL_GPIO_WritePin(GPIOB, R1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, R2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, R3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, R4_Pin, GPIO_PIN_SET);
	
	if(HAL_GPIO_ReadPin(GPIOB, C1_Pin) == GPIO_PIN_SET)
	{
		/* ON */
		  

		  LCD_command(0x38);                       
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key C");
		while (HAL_GPIO_ReadPin (GPIOB, C1_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C2_Pin) == GPIO_PIN_SET)
	{
		/* 0 */
		  
		  LCD_command(0x38);                      
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key 0");
		while (HAL_GPIO_ReadPin (GPIOB, C2_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C3_Pin) == GPIO_PIN_SET)
	{
		/* = */
		 
		  LCD_command(0x38);                      
	    LCD_command(0XC0);                       
	    LCD_string((uint8_t *)"Key =");
		while (HAL_GPIO_ReadPin (GPIOB, C3_Pin));
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, C4_Pin) == GPIO_PIN_SET)
	{
		/* + */
		  
		  LCD_command(0x38);                       
	    LCD_command(0XC0);                     
	    LCD_string((uint8_t *)"Key +");
		while (HAL_GPIO_ReadPin (GPIOB, C4_Pin));
	}
}
/*****************************************************************************************************************************/

void LCD_init(void)
{
	 HAL_Delay(100);     
   LCD_command(0x38);  // hàm set lcd lam viec o che do 8 bit


	 LCD_command(0x0C);  // set con tro xuong dòng 2


	 LCD_command(0x06);  //tang vi trí con tro.


	 LCD_command(0x01);  // xoa man hinh
	
	
	 LCD_command(0x80);  // tat man hinh hien thi va con tro 
}

/*****************************************************************************************************************************/


void LCD_command(uint8_t value)
{
   HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
	
	
	
	  for (int i = 0; i < 8; i++)
	{
    HAL_GPIO_WritePin(GPIOA, data_pins[i], ((value >> i) & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
  }


	
	HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);    
  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1);    
  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);   
	
}
/*****************************************************************************************************************************/


void LCD_data(uint8_t value)
{
	 HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);

	  for (int i = 0; i < 8; i++)
	{
    HAL_GPIO_WritePin(GPIOA, data_pins[i], ((value >> i) & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
  }


	
  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);    
  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1);   
  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);   
}
/*****************************************************************************************************************************/


void LCD_string(uint8_t *str)
{
	int i = 0;
	
	while(str[i] != '\0')
	{
		 LCD_data(str[i]);
		 i++;
	}
   
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D0_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |RS_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R1_Pin|R2_Pin|R3_Pin|R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D0_Pin D1_Pin D2_Pin D3_Pin
                           D4_Pin D5_Pin D6_Pin D7_Pin
                           RS_Pin EN_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |RS_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin C4_Pin */
  GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin|C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
