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
#define DATA_LEN 5
#define DATA_CH_NUM 6
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t spwm[]=
{
1020,1349,1679,2009,2338,
2667,2996,3324,3651,3978,
4305,4630,4955,5278,5601,
5922,6242,6561,6878,7194,
7509,7822,8133,8442,8750,
9056,9360,9661,9961,10258,
10553,10846,11136,11424,11709,
11992,12272,12549,12823,13095,
13363,13628,13891,14150,14405,
14658,14907,15153,15395,15634,
15869,16100,16328,16552,16772,
16988,17200,17409,17613,17813,
18009,18201,18388,18571,18750,
18925,19095,19261,19422,19579,
19731,19878,20021,20159,20292,
20421,20545,20664,20778,20887,
20992,21091,21186,21275,21360,
21439,21514,21583,21648,21707,
21761,21810,21854,21893,21926,
21955,21978,21996,22009,22017,

22017,22009,21996,21978,21955,
21926,21893,21854,21810,21761,
21707,21648,21583,21514,21439,
21360,21275,21186,21091,20992,
20887,20778,20664,20545,20421,
20292,20159,20021,19878,19731,
19579,19422,19261,19095,18925,
18750,18571,18388,18201,18009,
17813,17613,17409,17200,16988,
16772,16552,16328,16100,15869,
15634,15395,15153,14907,14658,
14405,14150,13891,13628,13363,
13095,12823,12549,12272,11992,
11709,11424,11136,10846,10553,
10258,9961,9661,9360,9056,
8750,8442,8133,7822,7509,
7194,6878,6561,6242,5922,
5601,5278,4955,4630,4305,
3978,3651,3324,2996,2667,
2338,2009,1679,1349,1020
};
//max duty 91.2
//medium 1020
//amplitude ?

uint16_t adc_data[DATA_LEN][DATA_CH_NUM]={0};
uint16_t adc_data_fin[DATA_CH_NUM]={0};

float adc_ch1_amp=0;
uint16_t adc_ch1_max=0;
uint16_t adc_ch1_min=4096;

uint32_t sys_tic=0;

pid_type_def amp_pid;
const fp32 amp_p_i_d[3]={0,0.4,0};

uint16_t vol_to_spwm;
float vol_set=7;
float rms;
float vol_sq_sum=0;
uint16_t vol_sq_times=0;

uint16_t cycle_extern=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void set_PWM(uint16_t duty,uint8_t ch)
{
	hhrtim1.Instance->sTimerxRegs[ch].CMP1xR = duty;
	hhrtim1.Instance->sTimerxRegs[ch].CMP2xR = duty+(23404-duty)/2;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t spwm_index=0;
	
	if(htim==(&htim16))
	{
		sys_tic++;
		if(sys_tic%20==0)
		{
			adc_ch1_max=0;
			adc_ch1_min=4096;
			
			vol_sq_sum=vol_sq_sum/vol_sq_times;
			rms=sqrt(vol_sq_sum);
			vol_sq_times=vol_sq_sum=0;
			
			//PID_calc(&amp_pid,rms,vol_set);

			amp_pid.out=1;
			if(amp_pid.out<0)
				amp_pid.out=1;
		}
	}
	
	if(htim==(&htim17))
	{
		if(spwm_index<200)
		{
			set_PWM(amp_pid.out*spwm[spwm_index],0);
			if(spwm_index==0)
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
		}
		if(spwm_index>=200)
		{
			set_PWM((23040-amp_pid.out*spwm[spwm_index-200]),0);
			if(spwm_index==200)
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
		}
		spwm_index++;
		if(spwm_index==400)
			spwm_index=0;
		
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
	}
	

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	uint16_t i,j;
	uint32_t sum;
	static float scale=510.0f/2.0f;
	
	if(hadc==&hadc1)
	{
		for(i=0;i<DATA_CH_NUM;i++)
		{
			sum=0;
			for(j=0;j<DATA_LEN;j++)
				sum+=adc_data[j][i];
			adc_data_fin[i]=sum/DATA_LEN;
		}
		
		if(adc_data_fin[0]>adc_ch1_max)
		{
			adc_ch1_max=adc_data_fin[0];
			adc_ch1_amp=(adc_ch1_max-adc_ch1_min)/4096.0f*3.3f/20*scale*1.516f+0.1133f;
		}
		if(adc_data_fin[0]<adc_ch1_min)
		{
			adc_ch1_min=adc_data_fin[0];
			adc_ch1_amp=(adc_ch1_max-adc_ch1_min)/4096.0f*3.3f/20*scale*1.516f+0.1133f;
		}		
		
		vol_sq_sum+=pow(((adc_data_fin[0]-1900)/4096.0f*3.3f/20*255.0f*1.516f+0.1133f),2);
		vol_sq_times++;
		
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);
	}
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{	
	//全部都是用来计算外部信号周期的变量，每测出5组就设定一次
	static uint8_t exter_cycle_flag=0;
	static uint16_t last_counter=0;
	static uint16_t now_counter=0;
	static uint16_t half_exter_cycle;
	static uint16_t exter_cycle_buff[5]={0};
	static uint8_t exter_cycle_buff_index=0;
	static uint32_t exter_cycle_buff_sum;
	
	if(hcomp==&hcomp2)
	{
		now_counter=__HAL_TIM_GetCounter(&htim15);
		
		if(now_counter-last_counter>100)
		{
			HAL_TIM_Base_Stop(&htim15);
			
			if(exter_cycle_flag==0)
				half_exter_cycle=now_counter;
			else
				exter_cycle_buff[exter_cycle_buff_index++]=half_exter_cycle+now_counter;
			exter_cycle_flag=!exter_cycle_flag;
			
			if(exter_cycle_buff_index==5)
			{
				exter_cycle_buff_index=0;
				exter_cycle_buff_sum=exter_cycle_buff[0]+exter_cycle_buff[1]+exter_cycle_buff[2]+exter_cycle_buff[3]+exter_cycle_buff[4];
				cycle_extern=exter_cycle_buff_sum/5+5;
				__HAL_TIM_SetAutoreload(&htim17,cycle_extern*72/400-1);
			}
			
			last_counter=now_counter=0;
			
			__HAL_TIM_SetCounter(&htim15,0);
			HAL_TIM_Base_Start(&htim15);
		}
		else
			last_counter=now_counter;
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
  MX_TIM17_Init();
  MX_ADC1_Init();
  MX_COMP2_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  
	PID_init(&amp_pid,PID_DELTA,amp_p_i_d,1,0.4);
	
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
	
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
	
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
//	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
//	
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);
//	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_C);
//	
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2);
//	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);

	HAL_GPIO_WritePin(SLOW_L_GPIO_Port,SLOW_L_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SLOW_H_GPIO_Port,SLOW_H_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);
	
	OLED_Init();
	OLED_Display_On();
	OLED_printf(0,0,16,"Hello World.");
	
	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,100);
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);
	
	HAL_COMP_Start_IT(&hcomp2);
	HAL_TIM_Base_Start(&htim15);
	
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
