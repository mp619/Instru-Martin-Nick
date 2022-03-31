/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#define ARM_MATH_CM4

#include "main.h"
#include "arm_math.h"


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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//Temp Char buffer
char str[150] = { ' ' };

//Define ADC Buffer size, remember first 3 samples are often junk!
#define ADC_BUFFER_SIZE 256
//Define the various LUT period values(determine samplying rate thus Fout) and LUT sizes
#define SINE_10 32500
#define SINE_100 3250
#define SINE_300 1083
#define SINE_1K 325
#define SINE_10K 33
#define SINE_30K 11
#define SINE_100K 3
#define SINE_1K_LUT_SIZE 256

//This flag is used to indicate when the ADC Buffer has been filled by the DMA
volatile int buffer_is_full = 0;

//Define all the needed ADC buffers
uint32_t adc_buffer_A[ADC_BUFFER_SIZE];

const uint16_t sine_wave_array_1k[SINE_1K_LUT_SIZE] = { 2048, 2098, 2148, 2198,
		2248, 2298, 2348, 2398, 2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784,
		2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143, 3185, 3226, 3267, 3307,
		3346, 3385, 3423, 3459, 3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722,
		3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919, 3939, 3958, 3975, 3992,
		4007, 4021, 4034, 4045, 4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
		4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065, 4056, 4045, 4034, 4021,
		4007, 3992, 3975, 3958, 3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777,
		3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530, 3495, 3459, 3423, 3385,
		3346, 3307, 3267, 3226, 3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877,
		2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496, 2447, 2398, 2348, 2298,
		2248, 2198, 2148, 2098, 2048, 1997, 1947, 1897, 1847, 1797, 1747, 1697,
		1648, 1599, 1550, 1501, 1453, 1405, 1358, 1311, 1264, 1218, 1172, 1127,
		1082, 1038, 995, 952, 910, 869, 828, 788, 749, 710, 672, 636, 600, 565,
		530, 497, 465, 433, 403, 373, 345, 318, 291, 266, 242, 219, 197, 176,
		156, 137, 120, 103, 88, 74, 61, 50, 39, 30, 22, 15, 10, 6, 2, 1, 0, 1,
		2, 6, 10, 15, 22, 30, 39, 50, 61, 74, 88, 103, 120, 137, 156, 176, 197,
		219, 242, 266, 291, 318, 345, 373, 403, 433, 465, 497, 530, 565, 600,
		636, 672, 710, 749, 788, 828, 869, 910, 952, 995, 1038, 1082, 1127,
		1172, 1218, 1264, 1311, 1358, 1405, 1453, 1501, 1550, 1599, 1648, 1697,
		1747, 1797, 1847, 1897, 1947, 1997};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

static void My_TIM2_Init(int period) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = period;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

static void My_TIM4_Init(int period) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = period;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

//This functions initiates the ADC via DMA and waits for the sample buffer to fill up before leaving control

struct voltage_measurement
{
	double magnitude;
	double phase;
};

struct RLC
{
	float R;
	float L;
	float C;
};

void output_number(int a)
{
	  int message_size = 40;
	  char str[message_size];

	  sprintf(str, "(output value = %i)", a);

	  HAL_UART_Transmit(&huart2,str,sizeof(str),10);// Send string
	  HAL_UART_Transmit(&huart2,"\r\n",2,10);		// Send new line command
	  //HAL_UART_Transmit(&huart2,",",2,10);		// Send comma
}

void output_RLC(struct RLC a)
{
	  int message_size = 40;
	  char str[message_size];

	  sprintf(str, "(R = %i) (L = %i) (C = %i)", (int)a.R, (int)a.L, (int)a.C);

	  HAL_UART_Transmit(&huart2,str,sizeof(str),10);// Send string
	  HAL_UART_Transmit(&huart2,"\r\n",2,10);		// Send new line command
	  //HAL_UART_Transmit(&huart2,",",2,10);		// Send comma
}

int get_best_resistor(int frequency)
{
	int resistor = 0;
	int is_clipping = 0;

	select_measurement_mode(2);

	while (is_clipping == 0){

		resistor++;
		select_resistor(resistor);

		if(resistor >= 6){ //if none clip, just use the highest (1 Meg) resistor
			resistor = 5;
			is_clipping = 1; //not clipping but we now want to exit the while loop
		}

		sync_sample_blocking(&adc_buffer_A[0], ADC_BUFFER_SIZE, frequency); //make samples at freq selected

		for(int i = 0; i < ADC_BUFFER_SIZE/2; i++){
			if((uint16_t)(adc_buffer_A[i] & 0x00000FFF) == 0 || (uint16_t)(adc_buffer_A[i] & 0x00000FFF) == 4096){ //if ADC min/max is reached
				is_clipping = 1;
			}
		}

		/*
		HAL_UART_Transmit(&huart2,"Start\n",6,1); //writes all samples to serial
		for(int i=0;i<ADC_BUFFER_SIZE;i++){
			sprintf(str, "%d",(uint16_t)(adc_buffer_A[i] & 0x00000FFF));
			HAL_UART_Transmit(&huart2,(uint8_t*)str,50,1);
			HAL_UART_Transmit(&huart2,(uint8_t*)"\n",1,1);
		}
		HAL_UART_Transmit(&huart2,(uint8_t*)"End\n",4,1);
		*/


		HAL_Delay(100);

	}
	return resistor-1;
}

struct voltage_measurement make_voltage_measurement(int measurement_mode, int resistor, int frequency)
{
	select_resistor(resistor);
	select_measurement_mode(measurement_mode);

	sync_sample_blocking(&adc_buffer_A[0], ADC_BUFFER_SIZE, frequency); //Make ADC samples

	int timer_error = 2;

	switch(frequency) {
		   case SINE_10 :
			  timer_error = 2;
		      break;
		   case SINE_100:
			  timer_error = 2;
		      break;
		   case SINE_1K:
			  timer_error = 2;
		      break;
		   case SINE_10K:
			  timer_error = 4;
		      break;
		   case SINE_30K:
			  timer_error = 4;
		      break;
		   case SINE_100K:
			  timer_error = 8;
		      break;
		   default :
			  timer_error = 2;
	}

	double k = 0.00390625 * timer_error; // 1/256 * timer error
	double pi = 2 * acos(0.0);

	double total_IP = 0;
	double total_QP = 0;

	for(int i = 0; i < ADC_BUFFER_SIZE/timer_error; i++){
		double IP_signal = cos(-2*pi*k*i);
		double QP_signal = sin(-2*pi*k*i);

		total_IP = total_IP + (IP_signal * (uint16_t)(adc_buffer_A[i] & 0x00000FFF));
		total_QP = total_QP + (QP_signal * (uint16_t)(adc_buffer_A[i] & 0x00000FFF));
	}


//	HAL_UART_Transmit(&huart2,"Start\n",6,1); //writes all samples to serial
//	for(int i=0;i<ADC_BUFFER_SIZE/timer_error;i++){
//		sprintf(str, "%d",(uint16_t)(adc_buffer_A[i] & 0x00000FFF));
//		HAL_UART_Transmit(&huart2,(uint8_t*)str,50,1);
//		HAL_UART_Transmit(&huart2,(uint8_t*)"\n",1,1);
//	}
//	HAL_UART_Transmit(&huart2,(uint8_t*)"End\n",4,1);


	struct voltage_measurement vm = {0,0};

	vm.magnitude = sqrt(pow(total_IP, 2) + pow(total_QP, 2))*(2*k);
	vm.phase = atan(total_QP/total_IP) - (pi/2);

	return vm;
}


void select_resistor(int resistor){
	switch(resistor) {
	   case 1  ://100 ohm resistor (Not working on breadboard)
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	      break;
	   case 2  ://1k resistor
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	      break;
	   case 3  ://10k resistor
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	      break;
	   case 4  ://100k resistor
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	      break;
	   case 5  ://1Meg resistor
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	      break;
	   default :
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}

double get_resistor(int resistor){
	switch(resistor) {
	   case 1  ://100 ohm resistor (Not working on breadboard)
		  return 100;
	      break;
	   case 2  ://1k resistor
		  return 1000;
	      break;
	   case 3  ://10k resistor
		  return 10000;
	      break;
	   case 4  ://100k resistor
		  return 100000;
	      break;
	   case 5  ://1Meg resistor
		   return 1000000;
	      break;
	   default :
		   return 100;
	}
}

void select_measurement_mode(int measurement_mode){
	switch(measurement_mode) {
	   case 1  : //measure between V_vg and V_1
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); //A
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); //B
	      break;
	   case 2  ://measure between V_vg and V_2
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	      break;
	   default ://default to V_vg and V_1 measurement
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	}
}

void sync_sample_blocking(uint32_t *buffer, int buffer_size, int frequency) {

//Set the correct samplying rate for DAC and ADC
	My_TIM2_Init(frequency);
	My_TIM4_Init(frequency);
//Start the DAC via DMA
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) sine_wave_array_1k,
	SINE_1K_LUT_SIZE, DAC_ALIGN_12B_R);

//Setup ADC
//Must disable the DMA first otherwise, it will not let you change the destination pointer as you intend!
	HAL_ADC_Stop_DMA(&hadc1);
	buffer_is_full = 0;
	HAL_ADC_Start_DMA(&hadc1, buffer, buffer_size);

//Setup common sync timer
	HAL_TIM_Base_Stop(&htim2);
	HAL_TIM_Base_Stop(&htim4);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim4);
	while (buffer_is_full == 0);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Stop(&htim2);
	HAL_TIM_Base_Stop(&htim4);
}

//This is the call back the DMA calls after ADC buffer is full.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//Disable the sampling timer for both the ADC and DAC
	HAL_TIM_Base_Stop(&htim2);
	HAL_TIM_Base_Stop(&htim4);
//Set the buffer full flag
	buffer_is_full = 1;
}


//double measure_DUT_capacitance(){
//	double r_dut = measure_DUT_impedance(SINE_10K);
//	double pi = 2 * acos(0.0);
//	return 1/(2*pi*100*r_dut);
//
//}

//Matrix initilization and functions

/* Transpose of A Buffer */
float32_t At_f32[4];
/* (Transpose of A * A) Buffer */
float32_t AtA_f32[4];
/* Inverse(Transpose of A * A)  Buffer */
float32_t IAtA_f32[4];
/* Inverse(Transpose of A * A)*AT  Buffer */
//float32_t IAtAAt_f32[10];
float32_t IAtAAt_f32[4];
/* Test Output Buffer */
float32_t X_f32[2];
/* Reactance Buffer */
//float32_t X_f32[5];
uint32_t Rows, Columns;

const float32_t A_f32[4] = {
	/* Omega 	1/Omega */
	6283.185307, -0.00015915494,
	62831.85307, -0.000015915494
	//628318.5307, -0.0000015915494,
};

arm_matrix_instance_f32 calculate_LCseries(float32_t *Reactance, arm_matrix_instance_f32 A){	// Returns {L; 1/C}
	arm_status status;
	// Create B matrix
	int row = 2;
	int column = 1;
	arm_matrix_instance_f32 B;	//B matrix of reactances
	arm_mat_init_f32(&B, row, column, Reactance);

	//Work out calc and return [L;1/C]
	row = 2;
	column = 1;
	arm_matrix_instance_f32 X;
	arm_mat_init_f32(&X, row, column, X_f32);
	status = arm_mat_mult_f32(&A, &B, &X);
	return X;
}

arm_matrix_instance_f32 calculate_LCparallel(float32_t *Admittance, arm_matrix_instance_f32 A){	// Return {C; 1/L}
	arm_status status;
	// Create B matrix
	int row = 2;
	int column = 1;
	arm_matrix_instance_f32 B;	//B matrix of reactances
	arm_mat_init_f32(&B, row, column, Admittance);

	//Work out calc and return [L;1/C]
	row = 2;
	column = 1;
	arm_matrix_instance_f32 X;
	arm_mat_init_f32(&X, row, column, X_f32);
	status = arm_mat_mult_f32(&A, &B, &X);
	return X;
}

struct RLC calculate_LC(float32_t *Reactance, arm_matrix_instance_f32 A, uint8_t type){	// Returns LC struct, [type == 0](Series); [type == 1](parallel)
	arm_matrix_instance_f32 X;//Ans
	float32_t Admittance[2];
	struct RLC Passive = {0};
	if(type){
		for (int i = 0; i < 2; i++){
			Admittance[i] = 1/Reactance[i];
		}
		X = calculate_LCparallel(Admittance, A);
		Passive.L = *(X.pData);
		Passive.C = 1/(*(X.pData+1));
	} else {
		X = calculate_LCseries(Reactance, A);
		Passive.C = *(X.pData);
		Passive.L = 1/(*(X.pData+1));
	}
	return Passive;
}

struct RLC measure_DUT_impedance(int freq){

	//int best_resistor = get_best_resistor(freq);
	int best_resistor = 3;

	int times_to_average = 10;

	struct voltage_measurement vdut_total = {0};
	struct voltage_measurement vrf_total = {0};
	struct RLC DUT = {0};

	for(int i = 0; i < times_to_average; i++){
		struct voltage_measurement vdut = make_voltage_measurement(1,best_resistor,freq);
		HAL_Delay(10);
		struct voltage_measurement vrf = make_voltage_measurement(2,best_resistor,freq);
		HAL_Delay(10);

		vdut_total.magnitude += vdut.magnitude;
		vdut_total.phase += vdut.phase;
		vrf_total.magnitude += vrf.magnitude;
		vrf_total.phase += vrf.phase;
	}

	//double DUT_resistance = (vm1_total/vm2_total)*get_resistor(best_resistor);
	vdut_total.phase = vdut_total.phase/times_to_average;
	vrf_total.phase = vrf_total.phase/times_to_average;

	DUT.R = (vdut_total.magnitude/vrf_total.magnitude)*(cos(vdut_total.phase)/cos(vrf_total.phase))*get_resistor(best_resistor) - 100;

	return DUT;

	//return DUT_resistance - 100; //due the the 100 Ohms resistor in series with DUT
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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  ////////////////////////INIT MATRIX////////////////////////////
  arm_matrix_instance_f32 A;	//Omega Matrix
  arm_matrix_instance_f32 At;	//Transpose of Omega
  arm_matrix_instance_f32 AtA;	//A transpose * A
  arm_matrix_instance_f32 IAtA;	//Inverse of A transpose *A
  arm_matrix_instance_f32 IAtAAt;//Inverse of (A transpose *A)*A

  arm_status status;

  //Init A
  Rows = 2;
  Columns = 2;
  arm_mat_init_f32(&A, Rows, Columns, (float32_t *)A_f32);

  //Init transpose(A);
  Rows = 2;
  Columns = 2;
  arm_mat_init_f32(&At, Rows, Columns, At_f32);
  status = arm_mat_trans_f32(&A, &At);

  //Init transpose(A)*A
  Rows = 2;
  Columns = 2;
  arm_mat_init_f32(&AtA, Rows, Columns, AtA_f32);
  status = arm_mat_mult_f32(&At, &A, &AtA);

  //Init inverse(transpose(A)*A)
  Rows = 2;
  Columns = 2;
  arm_mat_init_f32(&IAtA, Rows, Columns, IAtA_f32);
  status = arm_mat_inverse_f32(&AtA, &IAtA);

  //Init inverse(transpose(A)*A)*transpose(A)
  Rows = 2;
  Columns = 2;
  arm_mat_init_f32(&IAtAAt, Rows, Columns, IAtAAt_f32);
  status = arm_mat_mult_f32(&IAtA, &At, &IAtAAt);

  //Testing
//  float32_t Reactance[5] = {	-3.1202,
//		    					0.3100,
//								6.2514,
//								62.8287,
//								628.3182};

//  float32_t Reactance[5] = {	1/-15.60084075,
//		  	  	  	  	  	  	  1/1.5500925,
//								  1/31.256775,
//								  1/314.143335,
//								  1/3141.5909085};

//  float32_t Reactance[2] = {-2340.5,
//  	  	  	  	  	  	  	-234.05};

  //struct DUT Ans = calculate_LC(Reactance, IAtAAt, 0);

  //X = calculate_LCparallel(Admittance, IAtAAt);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//double r_dut = measure_DUT_impedance(SINE_10K);
		//struct voltage_measurement vm1 = make_voltage_measurement(1,3,SINE_100);
		//struct voltage_measurement vm2 = make_voltage_measurement(2,3,SINE_100);
		struct RLC DUT = measure_DUT_impedance(SINE_10K);
		//output_number(DUT.R);
		output_RLC(DUT);
		//sprintf(str, "%d",(int)(r_dut));
		//HAL_UART_Transmit(&huart2,(uint8_t*)str,50,1);
		//HAL_UART_Transmit(&huart2,(uint8_t*)"\n",1,1);
		HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 30;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 30;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 PB3
                           PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
	while (1) {
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

