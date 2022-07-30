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
#define DATA_LEN 5
#define DATA_CH_NUM 6

uint16_t MAX_PWM_DELT=100;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t adc_data[DATA_LEN][DATA_CH_NUM]={0};
uint16_t adc_data_fin[DATA_CH_NUM]={0};

pid_type_def pfc_pid;
const fp32 pfc_p_i_d[3]={0.0000,0.0000,0};

uint16_t vol_to_spwm;
float vol_set=10;

int32_t vol_in;
int32_t curr_in;
int8_t vol_in_polar=0;
int8_t vol_in_polar_last=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void set_PWM(uint16_t dutyA,uint16_t dutyB)
{
//		if((TIM1->CCR2-dutyA)>MAX_PWM_DELT)
//			TIM1->CCR2-=MAX_PWM_DELT;
//		else if((dutyA-TIM1->CCR2)>MAX_PWM_DELT)
//			TIM1->CCR2+=MAX_PWM_DELT;
//		else
//			TIM1->CCR2=dutyA;
	TIM1->CCR2=3600;
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
		
		for(i=0;i<DATA_CH_NUM;i++)
		{
			sum=0;
			max=0,min=4096;
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
		
		
		vol_in=1880-adc_data_fin[0];
		curr_in=adc_data_fin[2]-1838;
		
		if(vol_in<100&&vol_in>-100)
		{
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3|GPIO_PIN_4,GPIO_PIN_RESET);
			
			vol_in_polar_last=vol_in_polar;
			vol_in_polar=0;
			
			pfc_pid.Iout=0;
		}
		else
		{
			if(vol_in_polar==0&&vol_in>10)
			{
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);

				vol_in_polar_last=vol_in_polar;
				vol_in_polar=1;
				//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
				set_slow_L();
			}
			
			if(vol_in_polar==0&&vol_in<-10)
			{
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
				
				vol_in_polar_last=vol_in_polar;
				vol_in_polar=-1;
				//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
				set_slow_H();
			}
		}
		
		
		
		
		if(vol_in_polar==1)//pwm cannot too small
		{
			PID_calc(&pfc_pid,curr_in,vol_in);
			if(pfc_pid.out<0)
				pfc_pid.out=0;
			set_PWM(7199-pfc_pid.out,0);
		}

		if(vol_in_polar==-1)//pwm cannot too big
		{
			PID_calc(&pfc_pid,curr_in,vol_in);
			if(pfc_pid.out<0)
				pfc_pid.out=0;
			set_PWM(pfc_pid.out,0);
		}
		
		if(vol_in_polar==0)
		{

			
		}
		
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);
	}
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float Irms=0;       //电流有效�?
float Vrms=0;       //电压有效�?
float Frequency=0;  //频率
float PowerFactor=1;//功率因数
float PActive=0;    //有功功率
double W_KWH=0;     //累积功�??

uint8_t  Uart2_RxBuf[50]={0};//串口接收缓存
uint32_t Uart2_RxCnt=0;//接收计数
uint8_t data_error=0;
uint8_t CmdTxBuf[]={0x55,0x55,0x01,0x02,0x00,0x00,0xAD};

uint8_t SUI_101A_Get(uint8_t adder){

	
	
	CmdTxBuf[2]=adder;
	Uart2_RxCnt=0;
	CmdTxBuf[6]=CmdTxBuf[0]+CmdTxBuf[1]+CmdTxBuf[2]+CmdTxBuf[3]+CmdTxBuf[4]+CmdTxBuf[5];//重新计算校验�?
	HAL_UART_Transmit_DMA(&huart3, CmdTxBuf,7); 
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)Uart2_RxBuf, 31);

//	printf(" | V:%10.05f | I:%10.05f | P:%10.05f | PF:%10.05f | F:%10.05f | W:%10.05f |\r\n",Vrms,Irms,PActive,PowerFactor,Frequency,W_KWH);
	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t t=20;
	uint8_t rxlen=0;
	uint8_t i=0;
	uint8_t sum=0;
	uint8_t n=0;
	while(t)
	{
		t--;
		rxlen=31;
//		if((rxlen==Uart2_RxCnt)&&(rxlen!=0)){//接收到了数据,且接收完成了
//			if(rxlen==(Uart2_RxBuf[5]+7)){
//				//数据长度正确
//			}
//			else{
//				data_error=3;
//			}
			sum=0;
			rxlen-=1;//除去校验位的长度
			for(i=0;i<rxlen;i++){
				sum+=Uart2_RxBuf[i];
			}
			if(sum==Uart2_RxBuf[rxlen]){//校验和正硿
				Vrms=(double)(((uint32_t)Uart2_RxBuf[6] <<24)|((uint32_t)Uart2_RxBuf[7] <<16)|((uint32_t)Uart2_RxBuf[8] <<8)|((uint32_t)Uart2_RxBuf[9] <<0))/1000.0;
				Irms=(double)(((uint32_t)Uart2_RxBuf[10]<<24)|((uint32_t)Uart2_RxBuf[11]<<16)|((uint32_t)Uart2_RxBuf[12]<<8)|((uint32_t)Uart2_RxBuf[13]<<0))/1000.0;
				PActive=(double)(((uint32_t)Uart2_RxBuf[14]<<24)|((uint32_t)Uart2_RxBuf[15]<<16)|((uint32_t)Uart2_RxBuf[16]<<8)|((uint32_t)Uart2_RxBuf[17]<<0))/1000.0;
				n=18;
				PowerFactor=(double)(int32_t)(((int32_t)Uart2_RxBuf[n++]<<24)|((int32_t)Uart2_RxBuf[n++]<<16)|((int32_t)Uart2_RxBuf[n++]<<8)|((int32_t)Uart2_RxBuf[n++]<<0))/10000.0;
				Frequency=(double)(((uint32_t)Uart2_RxBuf[n++]<<24)|((uint32_t)Uart2_RxBuf[n++]<<16)|((uint32_t)Uart2_RxBuf[n++]<<8)|((uint32_t)Uart2_RxBuf[n++]<<0))/1000.0;
				W_KWH=(double)(((uint32_t)Uart2_RxBuf[n++]<<24)|((uint32_t)Uart2_RxBuf[n++]<<16)|((uint32_t)Uart2_RxBuf[n++]<<8)|((uint32_t)Uart2_RxBuf[n++]<<0))/10000.0;
			}
			else{//数据校验错误
				data_error=1;
			}
			break;
		}
//	}
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
  MX_DMA_Init();
  MX_TIM17_Init();
  MX_ADC1_Init();
  MX_COMP2_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_TIM16_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
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
	SUI_101A_Get(1);
	  HAL_Delay(500);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
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
