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
#include "adc.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "pid.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SPWM_NUM 400
#define DATA_LEN 3
#define DATA_CH_NUM 6
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t adc_data[DATA_LEN][DATA_CH_NUM]={0};
uint16_t adc_data_fin[DATA_CH_NUM]={0};

pid_type_def pfc_pid;
const fp32 pfc_p_i_d[3]={0.0001,0.0001,0};

uint16_t vol_to_spwm;
float vol_set=10;

int32_t vol_in;
int32_t curr_in;
uint16_t vol_in_polar=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void set_PWM(uint16_t dutyA,uint16_t dutyB)
{
	TIM1->CCR2=dutyA;
}
void set_slow_H(void)
{
	HAL_GPIO_WritePin(SLOW_L_GPIO_Port,SLOW_L_Pin,GPIO_PIN_RESET);
	{
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
	HAL_GPIO_WritePin(SLOW_H_GPIO_Port,SLOW_H_Pin,GPIO_PIN_SET);
}

void set_slow_L(void)
{
	HAL_GPIO_WritePin(SLOW_H_GPIO_Port,SLOW_H_Pin,GPIO_PIN_RESET);
	{
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
	HAL_GPIO_WritePin(SLOW_L_GPIO_Port,SLOW_L_Pin,GPIO_PIN_SET);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	uint8_t i,j;
	uint16_t max,min;
	uint32_t sum;
	static float scale=1.0f;//510k/0.75k
	
	
	if(hadc==&hadc1)
	{
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
		max=0,min=4096;
		for(i=0;i<DATA_CH_NUM;i++)
		{
			sum=0;
			for(j=0;j<DATA_LEN;j++)
			{
				if(max<adc_data[j][i])
					max=adc_data[j][i];
				if(min>adc_data[j][i])
					min=adc_data[j][i];
				sum+=adc_data[j][i];
			}
			adc_data_fin[i]=(sum-max-min)/(DATA_LEN-2);
		}
		
		
		vol_in=1895-adc_data_fin[0];
		curr_in=adc_data_fin[1]-1937;
		
		if(vol_in<40&&vol_in>-40)
		{
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3|GPIO_PIN_4,GPIO_PIN_RESET);
		}
		else 
		{
			if(vol_in_polar==0&&vol_in>10)
			{
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);

				vol_in_polar=1;
				//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
				set_slow_L();
			}
			
			if(vol_in_polar==1&&vol_in<-10)
			{
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
				
				vol_in_polar=0;
				//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
				set_slow_H();
			}
		}
		
		if(vol_in_polar==1)//pwm cannot too small
		{
			PID_calc(&pfc_pid,curr_in,-vol_in);
			if(pfc_pid.out<0)
				pfc_pid.out=0;
			pfc_pid.out=3600;
			set_PWM(7199-pfc_pid.out,0);
		}

		if(vol_in_polar==0)//pwm cannot too big
		{
			PID_calc(&pfc_pid,-curr_in,-vol_in);
			if(pfc_pid.out<0)
				pfc_pid.out=0;
			pfc_pid.out=3600;
			set_PWM(pfc_pid.out,0);
		}
		
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM17_Init();
  MX_ADC1_Init();
  MX_COMP2_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  
	PID_init(&pfc_pid,PID_DELTA,pfc_p_i_d,7200-1,7200-1);
	
	HAL_GPIO_WritePin(SLOW_L_GPIO_Port,SLOW_L_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SLOW_H_GPIO_Port,SLOW_H_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);
	
//	OLED_Init();
//	OLED_Display_On();
//	OLED_printf(0,0,16,"Hello World.");
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
