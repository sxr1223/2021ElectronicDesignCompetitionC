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
#define DATA_LEN 1
#define DATA_CH_NUM 6
#define MAX_PWM_F 23040.0f
#define MAX_PWM_I 23040

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

typedef struct
{
    fp32 out;          //滤波输出的数据
    fp32 num[2];
} first_order_filter_type_t;

#define AVERAGE_FILITER_NUM 10
typedef struct
{
	float data[AVERAGE_FILITER_NUM];
	float sum;
	float out;
	uint8_t index;
} averge_filter_type_t;

first_order_filter_type_t adc_fliter_first_order_LP[6]=
{	{0,{0.0099009901f,0.9900990099f}},
	{0,{0.0099009901f,0.9900990099f}},
	{0,{0.0099009901f,0.9900990099f}},
	{0,{0.0099009901f,0.9900990099f}},
	{0,{0.0099009901f,0.9900990099f}},
	{0,{0.0099009901f,0.9900990099f}}
};

averge_filter_type_t vol_d_samp_ave_filter={0};
averge_filter_type_t vol_q_samp_ave_filter={0};
averge_filter_type_t curr_d_samp_ave_filter={0};
averge_filter_type_t curr_q_samp_ave_filter={0};
//ADC buffer and k(x-b)
//uint16_t adc_data[DATA_LEN][DATA_CH_NUM]= {0};
uint16_t adc_data[DATA_CH_NUM]= {0};
float adc_kb[DATA_CH_NUM][2]={
	{1,0},
	{1,0},
	{1,0},
	{1,0},
	{1,0},
	{1,0}
};

uint16_t temp_vol_samp[3];

//PID
pid_type_def curr_q_pid;
pid_type_def vol_q_pid;
pid_type_def curr_d_pid;
pid_type_def vol_d_pid;
pid_type_def vol_DC_pid;

const fp32 curr_q_p_i_d[3]= {0.0000,0.00005,0};
const fp32 curr_d_p_i_d[3]= {0.0000,0.00005,0};
const fp32 vol_d_p_i_d[3]=  {0,0.00005,0};
const fp32 vol_q_p_i_d[3]=  {0,0.00005,0};
const fp32 vol_DC_p_i_d[3]= {0,0.0001,0};

const fp32 curr_d_max_out=20;
const fp32 curr_q_max_out=20;
const fp32 curr_d_imax_out=18;
const fp32 curr_q_imax_out=18;

const fp32 vol_d_max_out=5000;
const fp32 vol_q_max_out=5000;
const fp32 vol_d_imax_out=4000;
const fp32 vol_q_imax_out=4000;

const fp32 vol_DC_max_out=10;
const fp32 vol_DC_imax_out=10;

float dq_vol_set[2]= {500,0};
float dq_vol_set_open_loop[2]= {7,0};

float curr_set=200;
//pwm
uint16_t pwm[4][2]= {{0,0},{0,0},{0,0},{0,0}};
uint16_t pwm_temp[4];

//vol_in_theta
float sin_temp,cos_temp;
float sin_cos_temp;

//sample data
float abc_curr_out_samp[3]= {1,2,3};
float al_be_curr_out_samp[2]= {2,4};
float dq_curr_out_samp[2]= {1,1};

float abc_vol_out_samp[3]= {1,2,3};
float al_be_vol_out_samp[2]= {2,4};
float dq_vol_out_samp[2]= {1,1};

float vol_DC_samp;

float module_vol_out;
float theta_vol=0.01;

float dq_curr_pid_out[2];
float al_be_curr_pid_out[2];
float abc_curr_pid_out[3];
float neuter=0;
float abc_curr_pid_out_max=0;
float abc_curr_pid_out_min=0;

uint16_t SVPWM_time[3];

float last_dq_vol_out_samp[2];
float last_dq_curr_out_samp[2];

//SVPWM
uint8_t sector=1;
float V_DC=15;
int32_t X,Y,Z;
uint16_t T_per,T_nex;
uint16_t T=23040;
uint16_t T0,T7;
void (*sector_fun[6])(void);

//pwm_cal_flag
uint8_t pwm_need_cal_flag=1;

//cail
uint8_t cail_time;
float cail_res[DATA_CH_NUM]= {0,0,0,0,0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void average_filiter(averge_filter_type_t *temp,float input)
{
	temp->index=(temp->index+1)%AVERAGE_FILITER_NUM;
	temp->sum=temp->sum-temp->data[temp->index]+input;
	temp->data[temp->index]=input;
	temp->out=temp->sum/AVERAGE_FILITER_NUM;
}

void first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->out =input*(first_order_filter_type->num[0])+(first_order_filter_type->out)*(first_order_filter_type->num[1]);
}

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

void reset(void)
{
	PID_init(&vol_d_pid,PID_DELTA,vol_d_p_i_d,10,9);
	PID_init(&vol_q_pid,PID_DELTA,vol_q_p_i_d,10,9);
	PID_init(&curr_d_pid,PID_DELTA,curr_d_p_i_d,8,7.5);
	PID_init(&curr_q_pid,PID_DELTA,curr_q_p_i_d,8,7.5);
	PID_init(&vol_DC_pid,PID_DELTA,vol_DC_p_i_d,11520,11520);
}

void set_PWM(void)
{
	for(uint8_t i=0; i<4; i++)
	{
		hhrtim1.Instance->sTimerxRegs[i].CMP1xR = pwm[i][0];
		hhrtim1.Instance->sTimerxRegs[i].CMP2xR = pwm[i][1];
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim17)
	{
		if(pwm_need_cal_flag==0)
		{
			set_PWM();
			pwm_need_cal_flag=1;
		}
		
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

void sector_1(void)
{
	T_per=Y;
	T_nex=Z;
	T0=T7=(T-T_per-T_nex)/2;
	pwm[1][0]=T0/2;
	pwm[1][1]=MAX_PWM_I-T0/2;

	pwm[0][0]=MAX_PWM_I/2-(T_per+T7)/2;
	pwm[0][1]=MAX_PWM_I/2+(T_per+T7)/2;

	pwm[2][0]=MAX_PWM_I/2-T7/2;
	pwm[2][1]=MAX_PWM_I/2+T7/2;
}
void sector_2(void)
{
	T_per=-X;
	T_nex=Y;
	T0=T7=(T-T_per-T_nex)/2;
	pwm[0][0]=T0/2;
	pwm[0][1]=MAX_PWM_I-T0/2;

	pwm[2][0]=MAX_PWM_I/2-(T_per+T7)/2;
	pwm[2][1]=MAX_PWM_I/2+(T_per+T7)/2;

	pwm[1][0]=MAX_PWM_I/2-T7/2;
	pwm[1][1]=MAX_PWM_I/2+T7/2;
}



void sector_3(void)
{
	T_per=-Z;
	T_nex=X;
	T0=T7=(T-T_per-T_nex)/2;
	pwm[0][0]=T0/2;
	pwm[0][1]=MAX_PWM_I-T0/2;

	pwm[1][0]=MAX_PWM_I/2-(T_nex+T7)/2;
	pwm[1][1]=MAX_PWM_I/2+(T_nex+T7)/2;

	pwm[2][0]=MAX_PWM_I/2-T7/2;
	pwm[2][1]=MAX_PWM_I/2+T7/2;
}
void sector_4(void)
{
	T_per=Z;
	T_nex=-X;

	T0=T7=(T-T_per-T_nex)/2;

	pwm[2][0]=T0/2;
	pwm[2][1]=MAX_PWM_I-T0/2;

	pwm[1][0]=MAX_PWM_I/2-(T_per+T7)/2;
	pwm[1][1]=MAX_PWM_I/2+(T_per+T7)/2;

	pwm[0][0]=MAX_PWM_I/2-T7/2;
	pwm[0][1]=MAX_PWM_I/2+T7/2;
}
void sector_5(void)
{
	T_per=X;
	T_nex=-Y;
	T0=T7=(T-T_per-T_nex)/2;
	pwm[1][0]=T0/2;
	pwm[1][1]=MAX_PWM_I-T0/2;

	pwm[2][0]=MAX_PWM_I/2-(T_nex+T7)/2;
	pwm[2][1]=MAX_PWM_I/2+(T_nex+T7)/2;

	pwm[0][0]=MAX_PWM_I/2-T7/2;
	pwm[0][1]=MAX_PWM_I/2+T7/2;
}
void sector_6(void)
{
	T_per=-Y;
	T_nex=-Z;
	T0=T7=(T-T_per-T_nex)/2;
	pwm[2][0]=T0/2;
	pwm[2][1]=MAX_PWM_I-T0/2;

	pwm[0][0]=MAX_PWM_I/2-(T_nex+T7)/2;
	pwm[0][1]=MAX_PWM_I/2+(T_nex+T7)/2;

	pwm[1][0]=MAX_PWM_I/2-T7/2;
	pwm[1][1]=MAX_PWM_I/2+T7/2;
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
  MX_ADC2_Init();
  MX_COMP2_Init();
  MX_COMP6_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

	PID_init(&vol_d_pid,PID_DELTA,vol_d_p_i_d,vol_d_max_out,vol_d_imax_out);
	PID_init(&vol_q_pid,PID_DELTA,vol_q_p_i_d,vol_q_max_out,vol_q_imax_out);
	PID_init(&curr_d_pid,PID_DELTA,curr_d_p_i_d,curr_q_max_out,curr_q_imax_out);
	PID_init(&curr_q_pid,PID_DELTA,curr_q_p_i_d,curr_d_max_out,curr_d_imax_out);
	PID_init(&vol_DC_pid,PID_DELTA,vol_DC_p_i_d,vol_DC_max_out,vol_DC_imax_out);

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

//	OLED_Init();
//	OLED_Display_On();
//	OLED_printf(0,0,16,"Hello World.");

	HAL_TIM_Base_Start_IT(&htim17);

	sector_fun[0]=sector_1;
	sector_fun[1]=sector_2;
	sector_fun[2]=sector_3;
	sector_fun[3]=sector_4;
	sector_fun[4]=sector_5;
	sector_fun[5]=sector_6;
	
	pwm[3][1]=32000/2;
	set_PWM();

	pwm_need_cal_flag=0;
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);
	
	
	for(cail_time=0;cail_time<20;)
	{
		if(pwm_need_cal_flag==1)
		{
			for(uint8_t i=0;i<DATA_CH_NUM;i++)
				cail_res[i]=cail_res[i]+adc_data[i];
			cail_time++;
			pwm_need_cal_flag=0;
		}
	}
	
	for(uint8_t i=0;i<DATA_CH_NUM;i++)
		cail_res[i]=cail_res[i]/20.0f;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(pwm_need_cal_flag==1)
		{
			for(uint8_t i=0;i<3;i++)
			{
				abc_vol_out_samp[i]= -(adc_data[i]  -cail_res[i]  )/2048.0/SQRT_3;
				abc_curr_out_samp[i]=-(adc_data[i+3]-cail_res[i+3])/2048.0f/SQRT_3;
			}
			
			HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data,DATA_LEN*DATA_CH_NUM);
			
			clarke_amp(abc_vol_out_samp,al_be_vol_out_samp);
			clarke_amp(abc_curr_out_samp,al_be_curr_out_samp);

			sin_temp=arm_sin_f32(theta);
			cos_temp=arm_cos_f32(theta);
			sin_cos_temp=al_be_vol_out_samp[0]*al_be_vol_out_samp[0]+al_be_vol_out_samp[1]*al_be_vol_out_samp[1];
			arm_sqrt_f32(sin_cos_temp,&sin_cos_temp);

			sin_temp=al_be_vol_out_samp[1]/sin_cos_temp;
			cos_temp=al_be_vol_out_samp[0]/sin_cos_temp;
			
			park(al_be_curr_out_samp,dq_curr_out_samp);
//			park(al_be_vol_out_samp,dq_vol_out_samp);
			
//			dq_curr_out_samp[0]=-dq_curr_out_samp[0];
//			dq_curr_out_samp[1]=-dq_curr_out_samp[1];
//			dq_vol_out_samp[0]=-dq_vol_out_samp[0];
//			dq_vol_out_samp[1]=-dq_vol_out_samp[1];
			
//			average_filiter(&vol_d_samp_ave_filter,-dq_vol_out_samp[0]);
//			average_filiter(&vol_q_samp_ave_filter,-dq_vol_out_samp[1]);
//			average_filiter(&curr_d_samp_ave_filter,-dq_curr_out_samp[0]);
//			average_filiter(&curr_q_samp_ave_filter,-dq_curr_out_samp[1]);
			
//			if(fabs(dq_curr_out_samp[0]-last_dq_curr_out_samp[0])>150)
//				dq_curr_out_samp[0]=last_dq_curr_out_samp[0];
//			else
//				last_dq_curr_out_samp[0]=dq_curr_out_samp[0];
//			
//			if(fabs(dq_curr_out_samp[1]-last_dq_curr_out_samp[1])>150)
//				dq_curr_out_samp[1]=last_dq_curr_out_samp[1];
//			else
//				last_dq_curr_out_samp[1]=dq_curr_out_samp[1];
			
//			PID_calc(&vol_d_pid,vol_d_samp_ave_filter.out,dq_vol_set[0]);
//			PID_calc(&vol_q_pid,vol_q_samp_ave_filter.out,dq_vol_set[1]);

//			PID_calc(&curr_d_pid,dq_curr_out_samp[0],curr_set);
//			PID_calc(&curr_q_pid,dq_curr_out_samp[1],0);
//			
//			PID_calc(&curr_d_pid,dq_curr_out_samp[0],dq_vol_set[0]);
//			PID_calc(&curr_q_pid,dq_curr_out_samp[1],dq_vol_set[1]);

//			dq_curr_pid_out[0]=fabs(curr_d_pid.out);
//			dq_curr_pid_out[1]=fabs(curr_q_pid.out);
			
			dq_curr_pid_out[0]=dq_vol_set_open_loop[0];
			dq_curr_pid_out[1]=dq_vol_set_open_loop[1];
			
			ipark(al_be_curr_pid_out,dq_curr_pid_out);

//			sector=(al_be_curr_pid_out[1]>0)|((SQRT_3*al_be_curr_pid_out[0]-al_be_curr_pid_out[1]>0)<<1)|((SQRT_3*al_be_curr_pid_out[0]+al_be_curr_pid_out[1]<0)<<2);

//			X=SQRT_3*al_be_curr_pid_out[1]*T/V_DC;
//			Y=(al_be_curr_pid_out[1]/SQRT_3+al_be_curr_pid_out[0])*23040.0f/V_DC*1.5f;
//			Z=(al_be_curr_pid_out[1]/SQRT_3-al_be_curr_pid_out[0])*23040.0f/V_DC*1.5f;

//			sector_fun[sector-1]();
			
			iclarke_amp(abc_curr_pid_out,al_be_curr_pid_out);
			
			abc_curr_pid_out_max=abc_curr_pid_out[0];
			abc_curr_pid_out_min=abc_curr_pid_out[0];
			
			if(abc_curr_pid_out[1]>abc_curr_pid_out_max)
				abc_curr_pid_out_max=abc_curr_pid_out[1];
			if(abc_curr_pid_out[2]>abc_curr_pid_out_max)
				abc_curr_pid_out_max=abc_curr_pid_out[2];
			
			if(abc_curr_pid_out[1]<abc_curr_pid_out_min)
				abc_curr_pid_out_min=abc_curr_pid_out[1];
			if(abc_curr_pid_out[2]<abc_curr_pid_out_min)
				abc_curr_pid_out_min=abc_curr_pid_out[2];
			
			neuter=0.5f-0.5f*(abc_curr_pid_out_min+abc_curr_pid_out_max);
			
			SVPWM_time[0]=(abc_curr_pid_out[0]+neuter)*MAX_PWM_F;
			SVPWM_time[1]=(abc_curr_pid_out[1]+neuter)*MAX_PWM_F;
			SVPWM_time[2]=(abc_curr_pid_out[2]+neuter)*MAX_PWM_F;
			
			pwm[0][0]=(MAX_PWM_I-SVPWM_time[0])/2;
			pwm[0][1]=pwm[0][0]+SVPWM_time[0];
			pwm[1][0]=(MAX_PWM_I-SVPWM_time[1])/2;
			pwm[1][1]=pwm[1][0]+SVPWM_time[1];
			pwm[2][0]=(MAX_PWM_I-SVPWM_time[2])/2;
			pwm[2][1]=pwm[2][0]+SVPWM_time[2];
			
			pwm_need_cal_flag=0;
			
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
		}

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
