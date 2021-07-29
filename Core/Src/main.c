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

#define SQRT_3_DIV_2 	0.86602540378f
#define SQRT_6_DIV_3 	0.81649658093f
#define SQRT_2			1.41421356237f
#define SQRT_3 			1.73205080757f
#define SQRT_6 			2.44948974278f
#define PI_2_DIV_3		2.09439510239f
#define TWO_DIV_THREE	0.66666666667f

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//ADC buffer
uint16_t adc_data[DATA_LEN][DATA_CH_NUM]= {0};
uint16_t adc_data_fin[DATA_CH_NUM]= {0};

//PID
pid_type_def curr_q_pid;
pid_type_def vol_q_pid;
pid_type_def curr_d_pid;
pid_type_def vol_d_pid;
pid_type_def vol_DC_pid;

const fp32 curr_q_p_i_d[3]= {0,0.4,0};
const fp32 curr_d_p_i_d[3]= {0,0.4,0};
const fp32 vol_d_p_i_d[3]= {0,0.4,0};
const fp32 vol_q_p_i_d[3]= {0,0.4,0};
const fp32 vol_DC_p_i_d[3]={0,0.4,0};

float vol_set=10.0;

//pwm
uint16_t pwm[4][2]={0};
uint16_t pwm_temp[4];

//vol_in_theta
float sin_temp,cos_temp;

//sample data
float abc_curr_samp[3]={1,2,3};
float al_be_curr_samp[2]={2,4};
float dq_curr_samp[2]={1,1};

float abc_vol_in[3]={1,2,3};
float al_be_vol_in[2]={2,4};
float dq_vol_in[2]={1,1};

float vol_DC_samp;

float module_vol_in;
float theta_vol=0.01;

float dq_curr_out[2];
float al_be_curr_out[2];

//SVPWM
uint8_t sector;
float V_DC=10;
int32_t X,Y,Z;
uint16_t T_per,T_nex;
uint16_t T=23040;
uint16_t T0,T7;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void clarke_amp(float abc[3],float alph_beta_res[2])
{
	alph_beta_res[0]=(2.0f*abc[0]-abc[1]-abc[2])/3.0f;
	alph_beta_res[1]=(abc[1]-abc[2])/SQRT_3;
}

void clarke_pow(float abc[3],float alph_beta_res[2])
{
	alph_beta_res[0]=(2.0f*abc[0]-abc[1]-abc[2])/SQRT_6;
	alph_beta_res[1]=(abc[1]-abc[2])/SQRT_2;
}

void clarke_amp_simp(float abc[3],float alph_beta_res[2])
{
	alph_beta_res[0]=abc[0];
	alph_beta_res[1]=(abc[0]+2.0f*abc[1])/SQRT_3;
}

void iclarke_amp(float abc_res[3],float alph_beta[2])
{
	abc_res[0]=SQRT_6_DIV_3*alph_beta[0];
	abc_res[1]=-alph_beta[0]/SQRT_6+alph_beta[1]/SQRT_2;
	abc_res[2]=abc_res[1]-alph_beta[1]*SQRT_2;
}

void park(float alph_beta[2],float dq_res[2])
{
	dq_res[0]=alph_beta[0]*cos_temp+alph_beta[1]*sin_temp;
	dq_res[1]=-alph_beta[0]*sin_temp+alph_beta[1]*cos_temp;
}

void ipark(float alph_beta_res[2],float dq[2])
{
	alph_beta_res[0]=dq[0]*cos_temp-dq[1]*sin_temp;
	alph_beta_res[1]=dq[0]*sin_temp+dq[1]*cos_temp;
}

void clarke_park_amp(float abc[3],float dq_res[2],float theta)
{
	dq_res[0]=TWO_DIV_THREE*(arm_cos_f32(theta)*abc[0]+arm_cos_f32(theta-PI_2_DIV_3)*abc[1]+arm_cos_f32(theta+PI_2_DIV_3));
	dq_res[1]=TWO_DIV_THREE*(-arm_sin_f32(theta)*abc[0]-arm_sin_f32(theta-PI_2_DIV_3)*abc[1]-arm_sin_f32(theta+PI_2_DIV_3));
}

void iclarke_park_amp(float abc_res[3],float dq[2],float theta)
{	
	abc_res[0]=dq[0]*arm_cos_f32(theta)-dq[1]*arm_sin_f32(theta);
	abc_res[1]=dq[0]*arm_cos_f32(theta-PI_2_DIV_3)-dq[1]*arm_sin_f32(theta-PI_2_DIV_3);
	abc_res[2]=dq[0]*arm_cos_f32(theta+PI_2_DIV_3)-dq[1]*arm_sin_f32(theta+PI_2_DIV_3);
}

void set_PWM(void)
{
	for(uint8_t i=0;i<4;i++)
	{
		hhrtim1.Instance->sTimerxRegs[i].CMP1xR = pwm[i][0];
		hhrtim1.Instance->sTimerxRegs[i].CMP1xR = pwm[i][1];
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	
	if(htim==&htim17)
	{

		for(uint8_t i=0;i<4;i++)
			pwm_temp[i]=pwm[i][1]-pwm[i][0];
		
		set_PWM();

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
	static uint16_t i,j;
	static uint32_t sum;
	static uint16_t scop[DATA_LEN];
	
	if(hadc==&hadc1)
	{
		for(j=0;j<DATA_CH_NUM;j++)
		{
			for(i=0;i<DATA_LEN;i++)
				scop[i]=adc_data[i][j];
			bubble(scop,DATA_LEN);
			for(i=2,sum=0;i<DATA_LEN-2;i++)
				sum+=scop[i];
			adc_data_fin[j]=(float)sum/(float)(DATA_LEN-4);
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
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

	PID_init(&vol_d_pid,PID_DELTA,vol_d_p_i_d,10,6);
	PID_init(&vol_q_pid,PID_DELTA,vol_q_p_i_d,23040/2,23040/2);
	PID_init(&curr_d_pid,PID_DELTA,curr_d_p_i_d,1,1);
	PID_init(&curr_q_pid,PID_DELTA,curr_q_p_i_d,1,1);
	PID_init(&vol_DC_pid,PID_DELTA,vol_DC_p_i_d,1,1);
	
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

	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);

	OLED_Init();
	OLED_Display_On();
	OLED_printf(0,0,16,"Hello World.");

	HAL_TIM_Base_Start_IT(&htim17);
	
	for(uint16_t i=0;i<20000;i++)
	{
		clarke_amp(abc_vol_in,al_be_vol_in);
		
		arm_sqrt_f32(al_be_vol_in[0]*al_be_vol_in[0]+al_be_vol_in[1]*al_be_vol_in[1],&module_vol_in);
		sin_temp=al_be_vol_in[1]/module_vol_in;
		cos_temp=al_be_vol_in[0]/module_vol_in;
		
		clarke_amp(abc_curr_samp,al_be_curr_samp);
		park(al_be_curr_samp,dq_curr_samp);

		PID_calc(&vol_DC_pid,vol_DC_samp,vol_set);
		
		PID_calc(&curr_d_pid,dq_curr_samp[0],0);
		PID_calc(&curr_q_pid,dq_curr_samp[1],vol_DC_pid.out*module_vol_in);
		
		dq_curr_out[0]=curr_d_pid.out;
		dq_curr_out[1]=curr_q_pid.out;
		ipark(al_be_curr_out,dq_curr_out);

		sector=(al_be_curr_out[0]>0)|((SQRT_3*al_be_curr_out[0]-al_be_curr_out[1]>0)<<1)|((SQRT_3*al_be_curr_out[0]+al_be_curr_out[1]<0)<<2);
		
		X=SQRT_3*al_be_curr_out[1]*T/V_DC;
		Y=(al_be_curr_out[1]/SQRT_3+al_be_curr_out[0])*T/V_DC/TWO_DIV_THREE;
		Z=(al_be_curr_out[1]/SQRT_3-al_be_curr_out[0])*T/V_DC/TWO_DIV_THREE;
		
		switch(sector)
		{
			case 1:
			{
				T_per=Z;
				T_nex=Y;
				break;
			}
			case 2:
			{
				T_per=Y;
				T_nex=-X;
				break;
			}
			case 3:
			{
				T_per=-Z;
				T_nex=X;
				break;
			}
			case 4:
			{
				T_per=-X;
				T_nex=Z;
				break;
			}
			case 5:
			{
				T_per=X;
				T_nex=-Y;
				break;
			}
			case 6:
			{
				T_per=-Y;
				T_nex=-Z;
				break;
			}
		}
		
		T0=T7=(T-T_per-T_nex)/2;
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
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		clarke_amp(abc_curr_samp,al_be_curr_samp);
		iclarke_amp(abc_curr_samp,al_be_curr_samp);
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
