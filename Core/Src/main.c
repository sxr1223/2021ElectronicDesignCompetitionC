/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file			 : main.c
	* @brief			: Main program body
	******************************************************************************
	* @attention
	*
	* <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
	* All rights reserved.</center></h2>
	*
	* This software component is licensed by ST under BSD 3-Clause license,
	* the "License"; You may not use this file except in compliance with the
	* License. You may obtain a copy of the License at:
	*						opensource.org/licenses/BSD-3-Clause
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "math.h"
#include "string.h"
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DATA_LEN 10
#define DATA_CH_NUM 7

#define MAX_VOL_NUM 3
#define MAX_FILTER 50
#define MAX_PWM_DUTY 14400
#define MIN_PWM_DUTY 96

#define MAX_CHARGE_VOL 17.8f
#define MIN_CHARGE_VOL 16.5f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef enum
{
	mode_init=0,
	mode_I=1,
	mode_II=2
}C_mode_t;

C_mode_t now_C_mode=mode_init,last_C_mode=mode_init;

uint16_t adc_data[DATA_LEN][DATA_CH_NUM]= {0};
uint16_t adc_data_fin[DATA_CH_NUM]= {0};
float actuall_value[DATA_CH_NUM]={0};
float kb_convert[DATA_CH_NUM][2]=
{
	{0.0156354319,0.8760253401},//0.9999492730
	{0.0018310598,0.0330754579},//0.9998300626
	{0.0094204314,0.1838279771},//0.9999567220
	{0.0004230486,0.0189600747},//0.9996990227
	{0.0005140228,0.0371836402},//0.9999189104
	{0.0056227647,0.2171621009},//0.9999612940
	{0.0004376324,0.0208325824} //0.9980617362 
};

uint16_t scop[DATA_LEN];

uint16_t adc2_data[DATA_LEN][4]= {0};
float buck_curr_out;

float vol_set=30;
float vol_charge=17.5;
float load_curr_ration_mian=0.5;
float mode_convert_vol=50;
float stop_relay_vol=20;

float learning_rate_mode_II=0.001;
float learning_rate_mode_I=0.001;

//mode_I
pid_type_def battery_buck_vol_out_pid;
pid_type_def battery_buck_curr_out_pid;
pid_type_def main_boost_vol_out_pid;

const fp32 battery_buck_vol_in_p_i_d[3]={0.0f,-0.09f,0.0f};
const fp32 battery_buck_curr_in_p_i_d[3]={0.0f,-1.0f,0.0f};
const fp32 main_boost_vol_out_p_i_d[3]={0.0f,1.0f,0.0f};

//mode_II
pid_type_def main_boost_curr_out_pid;
pid_type_def battery_boost_curr_out_pid;
pid_type_def vol_out_pid;
pid_type_def MPPT_mode_II_pid;

const fp32 main_boost_curr_out_p_i_d[3]= {0,10,0};
const fp32 battery_boost_curr_out_p_i_d[3]= {0,10,0};
const fp32 vol_out_p_i_d[3]= {0,0.0005,0};
const fp32 MPPT_mode_II_p_i_d[3]= {0,1.0,0};


//MPPT
float now_pow;
float now_vol_in;

float last_pow=0;
float last_vol_in=0;

float delt_pow;
float delt_vol_in;

float min_delt_pow=0.5;
float min_delt_vol_in=0.5;

float min_step=0.1;

float slope;
float step;
float delt_charge_vol=0.05;

uint8_t MPPT_is_on=0;
int8_t direction=1;
uint16_t MPPT_fre=4000;

float vin_target;
float res_inside=10;
int8_t key_no=-1;
uint8_t mode_I_soft_start_flag=0;
uint8_t mode_II_soft_start_flag=0;

uint16_t charge_over_vol_counter;
uint16_t charge_con_counter=0;
uint16_t charge_con_fre=500;

uint16_t MPPT_mode_II_counter=0;
uint16_t MPPT_mode_II_fre=25;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void change_stopbuckboost(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void change_stopbuck(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void change_stopboost(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14|GPIO_PIN_15,GPIO_PIN_RESET);
}

void change_startbuck(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void change_startboost(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void HAL_HRTIM_RegistersUpdateCallback(HRTIM_HandleTypeDef * hhrtim,uint32_t TimerIdx)
{
	if(TimerIdx==HRTIM_TIMERINDEX_MASTER)
	{		

	}
}

void bubble(uint16_t *a,uint8_t num)
{
	uint16_t temp = 0;
	for (uint16_t i = 0; i < num; i++)
		for (uint16_t j = 0; j < num-1-i; j++)
			if (a[j]>a[j+1])
			{
				temp = a[j + 1];
				a[j + 1] = a[j];
				a[j] = temp;
			}
}



void C_mode_machine(void)
{
	if(last_C_mode!=now_C_mode)
	{
		//mode_I
		if(now_C_mode==mode_I)
		{
			load_curr_ration_mian=1.0;
			mode_I_soft_start_flag=1;
			PID_clear(&main_boost_vol_out_pid);
			PID_clear(&battery_buck_vol_out_pid);
			battery_buck_vol_out_pid.out=4000;
			main_boost_vol_out_pid.max_out=0;
			
			change_stopbuckboost();
			HAL_Delay(1);
			change_startbuck();
			last_C_mode=now_C_mode;
			return;
		}
		
		//mode_II
		if(now_C_mode==mode_II)
		{
			load_curr_ration_mian=0.5;
			mode_II_soft_start_flag=1;
			PID_clear(&main_boost_curr_out_pid);
			PID_clear(&battery_boost_curr_out_pid);
			PID_clear(&vol_out_pid);
			main_boost_curr_out_pid.max_out=0;
			battery_boost_curr_out_pid.max_out=0;
			vol_out_pid.max_out=0;
			
			change_stopbuckboost();
			HAL_Delay(1);
			change_startboost();
			last_C_mode=now_C_mode;
			return;
		}
		
		if(now_C_mode==mode_init)
		{
			change_stopbuckboost();
			last_C_mode=now_C_mode;
			return;
		}
	}
}

void MPPT(void)
{
	static uint16_t i_MPPT=0;
	
	i_MPPT++;
	if(i_MPPT==MPPT_fre)
	{
		i_MPPT=0;
		return;
	}
			
	now_pow=actuall_value[0]*actuall_value[4];
	now_vol_in=actuall_value[0];
	
	delt_vol_in=now_vol_in-last_vol_in;
	delt_pow=now_pow-last_pow;

	last_pow=now_pow;
	last_vol_in=now_vol_in;
	
	slope=delt_pow;
	step=(learning_rate_mode_I*slope);
	
	if(step<0)
	{
		direction=-direction;
	}
	
	if(fabs(delt_vol_in)<min_delt_vol_in||fabs(delt_pow)<min_delt_pow)
		return;
	
	if(MPPT_is_on!=1)
		return;
	
	
	
	if(now_C_mode==mode_I)
	{	
		if(fabs(step)>=min_step)
		{
			if(fabs(battery_buck_vol_out_pid.out)<0.9f*battery_buck_vol_out_pid.max_out)
				vol_charge+=delt_charge_vol*direction;
			else
				battery_buck_vol_out_pid.max_out+=delt_charge_vol*direction*100;
		}
		
		if(vol_charge>MAX_CHARGE_VOL)
			vol_charge=MAX_CHARGE_VOL;
		if(vol_charge<MIN_CHARGE_VOL)
			vol_charge=MIN_CHARGE_VOL;
		
		if(battery_buck_vol_out_pid.max_out<0)
			battery_buck_vol_out_pid.max_out=0.1;
		if(battery_buck_vol_out_pid.max_out>19)
			battery_buck_vol_out_pid.max_out=19;
	}
	
	if(now_C_mode==mode_II)
	{
		load_curr_ration_mian+=(learning_rate_mode_II*slope);
	}
}
uint8_t relay_is_on=0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint16_t i,j;
	uint32_t sum;
	
	
	
	if(hadc==&hadc1)
	{
		
		if(mode_I_soft_start_flag==1)
		{
			main_boost_vol_out_pid.max_out+=100;
			//battery_buck_vol_out_pid.max_out+=100;
			
			if(main_boost_vol_out_pid.max_out>main_boost_vol_out_pid.max_iout)
				main_boost_vol_out_pid.max_out=main_boost_vol_out_pid.max_out;
//			if(battery_buck_vol_out_pid.max_out>battery_buck_vol_out_pid.max_iout)
//				battery_buck_vol_out_pid.max_out=battery_buck_vol_out_pid.max_out;
			
			if(battery_buck_vol_out_pid.max_out>=battery_buck_vol_out_pid.max_iout&&
					main_boost_vol_out_pid.max_out>=main_boost_vol_out_pid.max_iout)
				mode_I_soft_start_flag=0;
		}

		if(mode_II_soft_start_flag==1)
		{
			main_boost_curr_out_pid.max_out+=100;
			battery_boost_curr_out_pid.max_out+=100;
			vol_out_pid.max_out+=100;
			
			if(main_boost_curr_out_pid.max_out>main_boost_curr_out_pid.max_iout)
				main_boost_curr_out_pid.max_out=main_boost_curr_out_pid.max_iout;
			if(battery_boost_curr_out_pid.max_out>battery_boost_curr_out_pid.max_iout)
				battery_boost_curr_out_pid.max_out=battery_boost_curr_out_pid.max_iout;
			if(vol_out_pid.max_out>vol_out_pid.max_iout)
				vol_out_pid.max_out=vol_out_pid.max_iout;
			
			if(main_boost_curr_out_pid.max_out>=main_boost_curr_out_pid.max_iout&&
					battery_boost_curr_out_pid.max_out>=battery_boost_curr_out_pid.max_iout&&
					vol_out_pid.max_out>=vol_out_pid.max_iout)
				mode_II_soft_start_flag=0;
		}
		
		for(uint8_t j=0;j<DATA_CH_NUM;j++)
		{
			for(i=0;i<DATA_LEN;i++)
				scop[i]=adc_data[i][j];
			bubble(scop,DATA_LEN);
			for(i=2,sum=0;i<DATA_LEN-2;i++)
				sum+=scop[i];		
			adc_data_fin[j]=(float)sum/(float)(DATA_LEN-4);
			
			actuall_value[j]=kb_convert[j][0]*adc_data_fin[j]+kb_convert[j][1];
		}
		
		for(i=0;i<DATA_LEN;i++)
			scop[i]=adc2_data[i][0];
		bubble(scop,DATA_LEN);
		for(i=2,sum=0;i<DATA_LEN-2;i++)
			sum+=scop[i];	
		buck_curr_out=sum/(float)(DATA_LEN-2)*0.0012229299f-0.0174449447f;
		
		actuall_value[0]-=0.2f;
		
		vin_target=0.5f*(actuall_value[0]+res_inside*actuall_value[1]);
		
//		if(actuall_value[5]>MAX_CHARGE_VOL)
//		{
//			change_stopbuckboost();
//			return;
//		}
//		
//		if(actuall_value[5]<MAX_CHARGE_VOL)
//		{
//			change_startbuck();
//		}
		
		if(actuall_value[0]>mode_convert_vol)
			now_C_mode=mode_I;
		else
			now_C_mode=mode_II;
		
		C_mode_machine();
		
//		if(now_C_mode==mode_II)
//		{	
//			MPPT_mode_II_counter++;
//			if(MPPT_mode_II_counter>=MPPT_mode_II_fre)
//			{
//				PID_calc(&MPPT_mode_II_pid,actuall_value[0],vin_target);
//				PID_calc(&vol_out_pid,actuall_value[2],vol_set);
//				MPPT_mode_II_counter=0;
//			}
//			load_curr_ration_mian=1.0f-MPPT_mode_II_pid.out;
//			//load_curr_ration_mian=0.5;
//			
//			PID_calc(&main_boost_curr_out_pid,actuall_value[3],vol_out_pid.out*load_curr_ration_mian);
//			PID_calc(&battery_boost_curr_out_pid,actuall_value[6],vol_out_pid.out*(1-load_curr_ration_mian));
//		}
		
		if(now_C_mode==mode_II)
		{	
			MPPT_mode_II_counter++;
			if(MPPT_mode_II_counter>=MPPT_mode_II_fre)
			{
				PID_calc(&MPPT_mode_II_pid,actuall_value[0],vin_target);
				PID_calc(&vol_out_pid,actuall_value[2],vol_set);
				MPPT_mode_II_counter=0;
			}
			load_curr_ration_mian=1.0f-MPPT_mode_II_pid.out;
			//load_curr_ration_mian=0.5;
			
			PID_calc(&main_boost_curr_out_pid,actuall_value[2],vol_set);
			PID_calc(&battery_boost_curr_out_pid,actuall_value[6],vol_out_pid.out*(1-load_curr_ration_mian));
		}
		
		if(now_C_mode==mode_I)
		{
			
			charge_con_counter++;
			if(charge_con_counter==charge_con_fre)
			{
				if(vin_target<28)
				{
					PID_calc(&battery_buck_vol_out_pid,actuall_value[0],vin_target);
					charge_con_counter=0;
					if(actuall_value[5]>MAX_CHARGE_VOL)
					{
						change_stopbuckboost();
						while(1)
						{
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
							HAL_Delay(200);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
						}
					}
				}
			}
			
			PID_calc(&battery_buck_curr_out_pid,actuall_value[0],vin_target);
			
			if(actuall_value[0]<50&&battery_buck_vol_out_pid.error[0]<100.0f)
			{
				PID_calc(&main_boost_vol_out_pid,actuall_value[2],vol_set);
			}
		}
		
		MPPT();
		
		if(main_boost_vol_out_pid.out<0)
			main_boost_vol_out_pid.out=100;
		
		if(battery_buck_curr_out_pid.out<0)
			battery_buck_curr_out_pid.out=100;
		
		if(battery_boost_curr_out_pid.out<0)
			battery_boost_curr_out_pid.out=100;		
		
		if(main_boost_curr_out_pid.out<0)
			main_boost_curr_out_pid.out=100;

		
		if(now_C_mode==mode_II)
			hhrtim1.Instance->sTimerxRegs[0].CMP1xR = MAX_PWM_DUTY - main_boost_curr_out_pid.out;
		if(now_C_mode==mode_I)
			hhrtim1.Instance->sTimerxRegs[0].CMP1xR = MAX_PWM_DUTY - main_boost_vol_out_pid.out;
		
		hhrtim1.Instance->sTimerxRegs[1].CMP1xR = battery_buck_curr_out_pid.out;
		hhrtim1.Instance->sTimerxRegs[3].CMP1xR = MAX_PWM_DUTY - MPPT_mode_II_pid.out;
		
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);
		HAL_ADC_Start_DMA(&hadc2,(uint32_t*)adc2_data,DATA_LEN*4);
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
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

	PID_init(&MPPT_mode_II_pid,PID_DELTA,MPPT_mode_II_p_i_d,13000,13000);
	PID_init(&vol_out_pid,PID_DELTA,vol_out_p_i_d,2,2);
	PID_init(&main_boost_curr_out_pid, PID_DELTA,main_boost_curr_out_p_i_d,MAX_PWM_DUTY-500,MAX_PWM_DUTY-500);
	PID_init(&battery_boost_curr_out_pid, PID_DELTA,battery_boost_curr_out_p_i_d,MAX_PWM_DUTY-500,MAX_PWM_DUTY-500);
	
	PID_init(&battery_buck_vol_out_pid, PID_DELTA, battery_buck_vol_in_p_i_d,2.5,0.1);
	PID_init(&battery_buck_curr_out_pid, PID_DELTA, battery_buck_curr_in_p_i_d,12000,MAX_PWM_DUTY-500);
	PID_init(&main_boost_vol_out_pid,PID_DELTA,main_boost_vol_out_p_i_d,MAX_PWM_DUTY-500,MAX_PWM_DUTY-500);
	
	main_boost_vol_out_pid.out=200;
	battery_buck_curr_out_pid.out=7000;
	MPPT_mode_II_pid.out=0.5;
	
	
	
	HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER);

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
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)adc2_data,DATA_LEN*4);

	now_C_mode=mode_I;
	MPPT_is_on=0;

	change_startbuck();

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5,GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
		if(GPIOB->IDR&0x0040)
		{
			key_no=1;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0080)
		{
			key_no=2;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0100)
		{
			key_no=3;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0200)
		{
			key_no=4;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
		if(GPIOB->IDR&0x0040)
		{
			key_no=1+4;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0080)
		{
			key_no=2+4;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0100)
		{
			key_no=3+4;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0200)
		{
			key_no=4+4;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		if(GPIOB->IDR&0x0040)
		{
			key_no=1+8;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0080)
		{
			key_no=2+8;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0100)
		{
			key_no=3+8;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0200)
		{
			key_no=4+8;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
		if(GPIOB->IDR&0x0040)
		{
			key_no=1+12;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0080)
		{
			key_no=2+12;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0100)
		{
			key_no=3+12;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(GPIOB->IDR&0x0200)
		{
			key_no=4+12;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
		
		if(key_no!=-1)
		{
			if(key_no==1)
				vol_charge-=0.05;
			if(key_no==2)
				vol_charge+=0.05;
			
			key_no=-1;
		}
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//static uint8_t MPPT_is_on=0;
//static uint8_t delt_PWM=10;
//static uint16_t MPPT_out;
//static uint8_t direction=1;

//float now_pow=0;
//float last_pow=0;
//float pow_delt;
//float vol_delt=0;
//float last_vol;

//void MPPT(void)
//{
//	now_pow=actuall_value[0]*actuall_value[1];
//	vol_delt=actuall_value[0]-last_vol;
//	if(vol_delt==0)
//		return;
//	
//	pow_delt=(now_pow-last_pow)*vol_delt;
//	
//	if(actuall_value[5]<MAX_CHARGE_VOL&&actuall_value[5]>MIN_CHARGE_VOL)
//	{
//		if(MPPT_is_on==0)
//		{
//			MPPT_out=battery_buck_vol_out_pid.out;
//			MPPT_is_on=1;
//		}
//		else
//		{
//			if(pow_delt<0)
//				direction=-direction;
//			
//			MPPT_out+=delt_PWM*direction;
//			battery_buck_vol_out_pid.out=MPPT_out;
//		}
//	}
//	else if(MPPT_is_on==1)
//	{
//		PID_clear(&main_boost_curr_out_pid);
//		MPPT_is_on=0;
//	}
//	
//	last_pow=now_pow;
//	last_vol=actuall_value[0];
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
