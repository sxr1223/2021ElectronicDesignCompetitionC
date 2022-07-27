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
#define DATA_LEN 20
#define DATA_CH_NUM 6
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t spwm[]=
{
11520,11684,11849,12013,12178,12342,12506,12670,12833,12996,
13159,13321,13483,13645,13806,13966,14126,14285,14443,14601,
14758,14914,15069,15224,15377,15530,15682,15832,15982,16130,
16277,16423,16568,16712,16854,16995,17135,17273,17410,17546,
17679,17812,17943,18072,18200,18326,18450,18573,18694,18813,
18930,19045,19159,19271,19381,19489,19594,19698,19800,19900,
19998,20094,20187,20279,20368,20455,20540,20623,20703,20781,
20857,20931,21002,21071,21138,21202,21264,21323,21380,21434,
21487,21536,21583,21628,21670,21710,21747,21782,21814,21843,
21870,21895,21917,21936,21953,21967,21979,21988,21994,21998,
22000,21998,21994,21988,21979,21967,21953,21936,21917,21895,
21870,21843,21814,21782,21747,21710,21670,21628,21583,21536,
21487,21434,21380,21323,21264,21202,21138,21071,21002,20931,
20857,20781,20703,20623,20540,20455,20368,20279,20187,20094,
19998,19900,19800,19698,19594,19489,19381,19271,19159,19045,
18930,18813,18694,18573,18450,18326,18200,18072,17943,17812,
17679,17546,17410,17273,17135,16995,16854,16712,16568,16423,
16277,16130,15982,15832,15682,15530,15377,15224,15069,14914,
14758,14601,14443,14285,14126,13966,13806,13645,13483,13321,
13159,12996,12833,12670,12506,12342,12178,12013,11849,11684,
11520,11355,11190,11026,10861,10697,10533,10369,10206,10043,
9880,9718,9556,9394,9233,9073,8913,8754,8596,8438,
8281,8125,7970,7815,7662,7509,7357,7207,7057,6909,
6762,6616,6471,6327,6185,6044,5904,5766,5629,5493,
5360,5227,5096,4967,4839,4713,4589,4466,4345,4226,
4109,3994,3880,3768,3658,3550,3445,3341,3239,3139,
3041,2945,2852,2760,2671,2584,2499,2416,2336,2258,
2182,2108,2037,1968,1901,1837,1775,1716,1659,1605,
1552,1503,1456,1411,1369,1329,1292,1257,1225,1196,
1169,1144,1122,1103,1086,1072,1060,1051,1045,1041,
1040,1041,1045,1051,1060,1072,1086,1103,1122,1144,
1169,1196,1225,1257,1292,1329,1369,1411,1456,1503,
1552,1605,1659,1716,1775,1837,1901,1968,2037,2108,
2182,2258,2336,2416,2499,2584,2671,2760,2852,2945,
3041,3139,3239,3341,3445,3550,3658,3768,3880,3994,
4109,4226,4345,4466,4589,4713,4839,4967,5096,5227,
5360,5493,5629,5766,5904,6044,6185,6327,6471,6616,
6762,6909,7057,7207,7357,7509,7662,7815,7970,8125,
8281,8438,8596,8754,8913,9073,9233,9394,9556,9718,
9880,10043,10206,10369,10533,10697,10861,11026,11190,11355
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
const fp32 amp_p_i_d[3]={0,0.01,0};

uint16_t vol_to_spwm;
float vol_set=10;
float rms;
float vol_sq_sum=0;
uint16_t vol_sq_times=0;

uint16_t cycle_extern=20000;

static uint16_t spwm_index=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void set_PWM(uint16_t dutyA,uint16_t dutyB)
{
	hhrtim1.Instance->sTimerxRegs[0].CMP1xR = 23040/2-dutyA/2;
	hhrtim1.Instance->sTimerxRegs[0].CMP2xR = 23040/2+dutyA/2;
	hhrtim1.Instance->sTimerxRegs[0].CMP3xR = dutyA+(23404-dutyA)/2;

	hhrtim1.Instance->sTimerxRegs[1].CMP1xR = 23040/2-dutyB/2;
	hhrtim1.Instance->sTimerxRegs[1].CMP2xR = 23040/2+dutyB/2;
}

//#define test
#ifdef test
uint32_t test_pwm=21040;
#endif
//22000
//900
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static float x;
	if(htim==(&htim16))//1k,1ms
	{
		sys_tic++;
		if(sys_tic%(cycle_extern*5/1000)==0)
		{
			adc_ch1_max=0;
			adc_ch1_min=4096;
			
			vol_sq_sum=vol_sq_sum/vol_sq_times;
			x=sqrt(vol_sq_sum);
//			rms=-1.77028f*x*x + 43.33845f*x + 0.05275f ;
			rms=43.35312f*x + 0.17551f ;
//			rms=-28.61181f*x*x*x*x + 31.61418f*x*x*x - 13.79850f*x*x+ 45.18009f*x - 0.04284f;
			vol_sq_times=vol_sq_sum=0;
			
			PID_calc(&amp_pid,rms,vol_set);

			amp_pid.out=1;
			if(amp_pid.out<0)
				amp_pid.out=0;
		}
	}

	if(htim==(&htim17))
	{
		if(spwm_index<200)
		{
#ifdef test
			set_PWM(test_pwm,test_pwm);
#else
			set_PWM(spwm[spwm_index]*amp_pid.out,spwm[200+spwm_index]*amp_pid.out);
#endif
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
#ifdef test
			set_PWM(test_pwm,test_pwm);
#else
			set_PWM(spwm[spwm_index]*amp_pid.out,spwm[spwm_index-200]*amp_pid.out);
#endif
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
	}
	

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
		
		if(adc_data_fin[0]>adc_ch1_max)
			adc_ch1_max=adc_data_fin[0];
		if(adc_data_fin[0]<adc_ch1_min)
			adc_ch1_min=adc_data_fin[0];
		
		adc_ch1_amp=(adc_ch1_max-adc_ch1_min)/4096.0f*3.3f*scale;	
		
		vol_sq_sum+=pow(((adc_data_fin[0]-1864)/4096.0f*3.3f*scale),2);
		vol_sq_times++;
		
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);
	}
}
#define EXTERN_CYCLE_SAMP_TIMES 10	
//us
uint16_t exter_cycle_buff[EXTERN_CYCLE_SAMP_TIMES]={0};
uint8_t exter_cycle_buff_index=0;
uint32_t exter_cycle_buff_sum;
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{	
	//全部都是用来计算外部信号周期的变量

	if(hcomp==&hcomp2)
	{		
		if(__HAL_TIM_GetCounter(&htim15)>2000)
		{
			HAL_TIM_Base_Stop(&htim15);
			
			//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
			if(__HAL_TIM_GetCounter(&htim15)>1*1000)
				exter_cycle_buff[exter_cycle_buff_index++]=__HAL_TIM_GetCounter(&htim15);

			if(exter_cycle_buff_index==EXTERN_CYCLE_SAMP_TIMES)
			{
				for(exter_cycle_buff_index=0;exter_cycle_buff_index<EXTERN_CYCLE_SAMP_TIMES;exter_cycle_buff_index++)
					exter_cycle_buff_sum+=exter_cycle_buff[exter_cycle_buff_index];
				cycle_extern=exter_cycle_buff_sum/EXTERN_CYCLE_SAMP_TIMES*2;
				exter_cycle_buff_sum=0;
				exter_cycle_buff_index=0;
				
				__HAL_TIM_SetAutoreload(&htim17,cycle_extern*72/400-1);
				//spwm_index=0;
			}
			__HAL_TIM_SetCounter(&htim15,0);
			HAL_TIM_Base_Start(&htim15);
		}
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
	
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
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
	
//	OLED_Init();
//	OLED_Display_On();
//	OLED_printf(0,0,16,"Hello World.");
//	
	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,2000);
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);
	
//	HAL_COMP_Start_IT(&hcomp2);
//	HAL_TIM_Base_Start(&htim15);
	
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
