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
#include "dma.h"
#include "hrtim.h"
#include "hrtim.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "pid.h"
#include "arm_math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DATA_LEN 20
#define DATA_CH_NUM 6
#define MAX_PWM 23040
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t adc_data[DATA_LEN][DATA_CH_NUM]= {0};
uint16_t adc_data_fin[DATA_CH_NUM]= {0};

pid_type_def curr_pid;
pid_type_def vol_pid;
const fp32 curr_p_i_d[3]= {0,0.4,0};
const fp32 vol_p_i_d[3]= {0,0.4,0};

uint16_t scop[DATA_LEN];

float vol_set=10;
float vol_out=0;
float vol_in=0;
float curr_in=0;

uint16_t pwm[4][2]={0};
float modul=0.9;
uint16_t temp[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void set_PWM(void)
{
	for(uint8_t i=0;i<4;i++)
	{
		hhrtim1.Instance->sTimerxRegs[i].CMP1xR = pwm[i][0];
		hhrtim1.Instance->sTimerxRegs[i].CMP2xR = pwm[i][1];
	}
}

void soft_start(void)
{
	uint16_t pwm_tem;
	if(vol_out<2)
	{
		HAL_TIM_Base_Stop_IT(&htim17);
		__HAL_TIM_SET_COUNTER(&htim17,0);
		
		for(pwm_tem=0; pwm_tem<9000; pwm_tem+=200)
		{

			HAL_Delay(10);
		}

		HAL_TIM_Base_Start_IT(&htim17);
	}
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t sector=1;
	static uint16_t step=0;
	uint16_t A,B,C,T0,T7;
	uint16_t T_per,T_nex,T0_7;
	
	
	
	if(htim==&htim17)
	{
		T_per=modul*MAX_PWM*sin(PI/3.0f*(1.0f-step/60.0f));
		T_nex=modul*MAX_PWM*sin(PI/3.0f*step/60.0f);
		T0_7=MAX_PWM-T_nex-T_per;
		T0=T7=T0_7/2;
		
		switch(sector)
		{
			case 1:
			{
				pwm[0][0]=T0/2;
				pwm[0][1]=MAX_PWM-T0/2;
				
				pwm[2][0]=MAX_PWM/2-(T_nex+T7)/2;
				pwm[2][1]=MAX_PWM/2+(T_nex+T7)/2;
				
				pwm[3][0]=MAX_PWM/2-T7/2;
				pwm[3][1]=MAX_PWM/2+T7/2;
				
				break;
			}
			case 2:
			{
				pwm[2][0]=T0/2;
				pwm[2][1]=MAX_PWM-T0/2;
				
				pwm[0][0]=MAX_PWM/2-(T_per+T7)/2;
				pwm[0][1]=MAX_PWM/2+(T_per+T7)/2;
				
				pwm[3][0]=MAX_PWM/2-T7/2;
				pwm[3][1]=MAX_PWM/2+T7/2;

				break;
			}
			case 3:
			{
				pwm[2][0]=T0/2;
				pwm[2][1]=MAX_PWM-T0/2;
				
				pwm[3][0]=MAX_PWM/2-(T_nex+T7)/2;
				pwm[3][1]=MAX_PWM/2+(T_nex+T7)/2;
				
				pwm[0][0]=MAX_PWM/2-T7/2;
				pwm[0][1]=MAX_PWM/2+T7/2;

				break;
			}
			case 4:
			{
				pwm[3][0]=T0/2;
				pwm[3][1]=MAX_PWM-T0/2;
				
				pwm[2][0]=MAX_PWM/2-(T_per+T7)/2;
				pwm[2][1]=MAX_PWM/2+(T_per+T7)/2;
				
				pwm[0][0]=MAX_PWM/2-T7/2;
				pwm[0][1]=MAX_PWM/2+T7/2;

				break;
			}
			case 5:
			{
				pwm[3][0]=T0/2;
				pwm[3][1]=MAX_PWM-T0/2;
				
				pwm[0][0]=MAX_PWM/2-(T_nex+T7)/2;
				pwm[0][1]=MAX_PWM/2+(T_nex+T7)/2;
				
				pwm[2][0]=MAX_PWM/2-T7/2;
				pwm[2][1]=MAX_PWM/2+T7/2;

				break;
			}
			case 6:
			{
				pwm[0][0]=T0/2;
				pwm[0][1]=MAX_PWM-T0/2;
				
				pwm[3][0]=MAX_PWM/2-(T_per+T7)/2;
				pwm[3][1]=MAX_PWM/2+(T_per+T7)/2;
				
				pwm[2][0]=MAX_PWM/2-T7/2;
				pwm[2][1]=MAX_PWM/2+T7/2;

				break;
			}
			
		}
		for(uint8_t i=0;i<4;i++)
			temp[i]=pwm[i][1]-pwm[i][0];
		
		set_PWM();

		step++;
		if(step==60)
		{
			step=0;
			sector++;
		}
		
		if(sector==7)
			sector=1;
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
	}
}

void bubble(uint16_t *a,uint8_t num)
{
	uint16_t bubble_temp = 0;
	for (uint16_t i = 0; i < num; i++)
		for (uint16_t j = 0; j < num-1-i; j++)
			if (a[j]>a[j+1])
			{
				bubble_temp = a[j + 1];
				a[j + 1] = a[j];
				a[j] = bubble_temp;
			}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint16_t i,j;
	uint32_t sum;

	if(hadc==&hadc1)
	{
		for(j=0;j<3;j++)
		{
			for(i=0;i<DATA_LEN;i++)
				scop[i]=adc_data[i][j];
			bubble(scop,DATA_LEN);
			for(i=2,sum=0;i<DATA_LEN-2;i++)
				sum+=scop[i];
			adc_data_fin[j]=(float)sum/(float)(DATA_LEN-4);
		}

		vol_out=adc_data_fin[0]*0.0081f+0.5817f;
		vol_in=adc_data_fin[1]*0.0081f+0.5817f;
		curr_in=adc_data_fin[2]*0.0081f+0.5817f;
		
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
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

	PID_init(&vol_pid,PID_DELTA,vol_p_i_d,10,6);
	PID_init(&curr_pid,PID_DELTA,curr_p_i_d,23040/2,23040/2);

	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);

	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);

	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);

	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_C);

	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);

	//HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);

	OLED_Init();
	OLED_Display_On();
	OLED_printf(0,0,16,"Hello World.");

	HAL_TIM_Base_Start_IT(&htim17);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
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
