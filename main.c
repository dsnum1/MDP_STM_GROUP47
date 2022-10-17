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
#include "main.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
#include "pid.h"
#include <math.h>
#include <string.h>
#include "ICM20948.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ECHO_Pin GPIO_PIN_13
#define ECHO_GPIO_Port GPIOD
#define TRIG_Pin GPIO_PIN_4
#define TRIG_GPIO_Port GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
ICM20948 imu;
UART_HandleTypeDef huart3;
int forward_motion = 0;
int SPEED = 3000;
float beta, inclination, prevYaw, yawVal[5], yawDiff, yawDir[4], yawThreshold;

int calibration_lock =0;  // 1 means calibrating //0 means reading

sint16 classic_median_data1_result;
sint16 advance_median_data1_result;
float input_test_data1[10];
float ref_angle = 0;
float total_distance = 0;

int angle = 152;

float LEFT = 80;
float LEFT_2= 100;
float STRAIGHT = 150;
float RIGHT = 200;
float RIGHT_2 = 230;
char lumos_dir; // 0 for north, 1 for west 3 for south, 4 is west



uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

double encL, encR;

uint8_t ack[10]="ack\0";

PID_TypeDef PIDA;
PID_TypeDef PIDB;
PID_TypeDef PIDC;
PID_TypeDef PIDD;

uint8_t debug_hello[20];


// moving average queue data structure for angle values
float q_array[20] = {0};
float sum_q = 0;
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t encoderTaskAHandle;
const osThreadAttr_t encoderTaskA_attributes = {
   .name = "encoderTaskA",
  .stack_size = 128*4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t encoderTaskBHandle;
const osThreadAttr_t encoderTaskB_attributes = {
	.name = "encoderTaskB",
	.stack_size = 128*4,
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t encoderTaskMAINHandle;
const osThreadAttr_t encoderTaskMAIN_attributes = {
	.name = "encoderTaskMAIN",
	.stack_size = 128*4,
	.priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
void motor(void *argument);
void imuRead(void *argument);
void encoderA(void *argument);
void encoderB(void *argument);
void move_forward_PID(double period, double setpoint);
void move_backward_PID(double period, double setpoint);
void concatValues(void);


void stopMove();

void moveForward_dist();
void moveForward_time(uint16_t t, uint16_t speed);

void moveBackward_dist();
void moveBackward_time(uint16_t t, uint16_t speed);

void moveForwardLeft(int t );
void moveForwardRight(int t);
void moveBackwardLeft(int t);
void moveBackwardRight(int t);
void turn_backward_PID(uint32_t dist);
void turn_forward_PID(uint32_t dist);
void StartDefaultTask(void *argument);

int getInputMagnitude(char msb , char lsb);

void goLeft();
void goRight();

void clearBuffer();


void enqueue_dequeue(sint16 new_val);

void move_forward_PID_dis(double period, double setpoint);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t aRxBuffer[20];

int encoderAVal, encoderBVal, encoderTarget;
uint8_t startDriving;
float pi, radius;
float encoderGrad, encoderInt;
float adjustDist, changeDist, distA, distB, distBuffer, distOffset, robotDist, tempA, tempB, turnOffset;
float encoderDelay;
float yaw_g;
uint8_t strrr[4] = "TTT\n";


void HCSR04_Read(void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	HAL_Delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC2);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC2);
		}
	}
}




void turn_backward_PID(uint32_t dist)
{

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	float period = (float) dist;
	period = period * 5;
	OLED_Refresh_Gram();
	double PWMa, PIDOuta, PWMSetpointa = 3000;
	double PWMb, PIDOutb, PWMSetpointb = 3000;

	uint32_t tick = HAL_GetTick();

	PID(&PIDA, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

		PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
		PID_SetSampleTime(&PIDA, 500);
		PID_SetOutputLimits(&PIDA, 2500, 3500);
		PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
		PID_SetSampleTime(&PIDB, 500);
		PID_SetOutputLimits(&PIDB, 2500, 3500);

		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 3000);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 3000);
	int cnt1a = __HAL_TIM_GET_COUNTER(&htim2);
	int cnt1b = __HAL_TIM_GET_COUNTER(&htim3);

	while (period>0) {
		//osDelay(100);
		PWMa = encL * 5;
		PWMb = encR * 5;

		OLED_ShowNumber(10, 10, PIDOuta, 5, 12);
		OLED_ShowNumber(50, 10, PIDOutb, 5, 12);

		if(PID_Compute(&PIDA))
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
		if(PID_Compute(&PIDB))
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);

		if (period > 200){
			osDelay(200);
			period -= 200;
		} else {
			osDelay(period);
			period -= period;
		}
	}
	stopMove();
	HAL_Delay(1000);
}




void encoderA(void *argument){
		HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
		int cnt1, cnt2, diff;
		uint32_t tick;
		cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
		tick = HAL_GetTick();
		uint8_t hello[20];
		uint16_t dir;
		uint8_t yawer[10];
		float yaw_g = 0;
		float gz;
		float tmnow,tmold,dt;
		char sbuf[5][100];

		float returnValue =0;
		float old_val = 0;
		int n = 7;
		float p_array[7] = { 0 };
		int key;
		int j;


		  for(;;)
		  {
//			  tmnow = HAL_GetTick();
//			  dt = tmnow -tmold;
//			  tmold = tmnow;
//			  IMU_GyroRead(&imu);
//
//			  gz = imu.gyro[2];
//			  yaw_g = yaw_g + (gz*dt*0.001);
//
//			  p_array[0] = yaw_g;
//
//			  for(int i = 9 ; i >0 ; i--){
//				  p_array[i] = p_array[i-1];
//			  }
//
//			 sprintf(yawer, "%5.2f\r\n", imu.gyro[2]);
//			 HAL_UART_Transmit_IT(&huart3, yawer, 10);
//
//			  for (int i = 1; i < n; ++i) {
//			             key = p_array[i];
//			             j = i - 1;
//
//			             // Move elements of arr[0..i-1],
//			             // that are greater than key,
//			             // to one position ahead of
//			             // their current position
//			             while (j >= 0 && p_array[j] > key) {
//			                 p_array[j + 1] = p_array[j];
//			                 j = j - 1;
//			             }
//			             p_array[j + 1] = key;
//			      }
//
//		     returnValue = p_array[(n-1)/2];
//		     old_val = yaw_g;
//		     sprintf(yawer, "%5.2f\r\n", returnValue);
//		     HAL_UART_Transmit_IT(&huart3, yawer, 10);

//

//			  sprintf(yawer, "%5.2f\r\n", yaw_g);
//			  HAL_UART_Transmit(&huart3, yawer, 10, HAL_MAX_DELAY);

//			  sprintf(sbuf[0], "%5.2d\r\n", yaw_g);
//			  HAL_UART_Transmit(&huart3, yawer, 10, HAL_MAX_DELAY);




//			  sprintf(yawer, "%5.2f\r\n", input_test_data1[2]);
//			  HAL_UART_Transmit(&huart3, yawer, 10, HAL_MAX_DELAY);


//			  classic_median_data1_result = ClassicMedianFilter(input_test_data1, sizeof(input_test_data1)/sizeof(sint16));




		  if(HAL_GetTick()-tick > 1000L){
//			  sprintf(sbuf[0], "Accelerometer[x] : %5.2f Accelerometer[y] :%5.2f Accelerometer[z] : %5.2f", imu.gyro[0], imu.gyro[1], imu.gyro[2]);
//			  HAL_UART_Transmit_IT(&huart3, (uint8_t *)sbuf[0], 73);

		    cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
		    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
		      if(cnt2<=cnt1)
		        diff = cnt1 - cnt2;
		      else
		        diff = (65535 - cnt2) + cnt1;
		    }
		    else{
		      if(cnt2<=cnt1)
		        diff = cnt2 - cnt1;
		      else
		        diff = (65535 - cnt1) + cnt2;
		    }

		    encL = diff;

//		    sprintf(hello,"Speed:%5f\0",input_test_data1[0]);
//		    OLED_ShowString(10,20,hello);
		    dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
		    sprintf(hello,"Dir:%5f\0",classic_median_data1_result);
		    OLED_ShowString(10,30,hello);
		    cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
		    tick = HAL_GetTick();
		  }
		    //osDelay(1);
		  }

}


void encoderB(void *argument){
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
			  int cnt1, cnt2, diff;
			  uint32_t tick;
			  cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
			  tick = HAL_GetTick();
			  uint8_t hello[20];
			  uint16_t dir;
			  for(;;)
			  {
			  if(HAL_GetTick()-tick > 1000L){
			    cnt2 = __HAL_TIM_GET_COUNTER(&htim3);
			    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
			      if(cnt2<=cnt1)
			        diff = cnt1 - cnt2;
			      else
			        diff = (65535 - cnt2) + cnt1;
			    }
			    else{
			      if(cnt2<=cnt1)
			        diff = cnt2 - cnt1;
			      else
			        diff = (65535 - cnt1) + cnt2;
			    }

			    encR = diff;


			    dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
			    sprintf(hello,"Dir:%5d\0",dir);
			    OLED_ShowString(10,50,hello);
			    cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
			    tick = HAL_GetTick();
			  }
			    //osDelay(1);
			  }
}

void move_forward_PID(double period, double setpoint)
{
//	48.5 in 1000ms
	startDriving = 1;
	htim1.Instance->CCR4 = 180; // set direction of the wheel to be forward
	HAL_Delay(500);
	htim1.Instance->CCR4 = 151; // set direction of the wheel to be forward
	HAL_Delay(500);
	OLED_Refresh_Gram();
	period = period *10;
	period = (period * (18.74*1.5/1.33)/1.17)*1.25;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);

	double PIDOuta, PWMSetpointa = setpoint;
	double PIDOutb, PWMSetpointb = setpoint;


	PID(&PIDA, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDA, 500);
	PID_SetOutputLimits(&PIDA, 2500, 3500);
	PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDB, 500);
	PID_SetOutputLimits(&PIDB, 2500, 3500);

	uint32_t tick = HAL_GetTick();

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 3000);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 3000);

	while (period>0) {
		htim1.Instance->CCR4 = 151;
		//osDelay(100);

		OLED_ShowNumber(25,10,PIDOuta,5,12);
		OLED_ShowNumber(65,10,PIDOutb,5,12);


		if(PID_Compute(&PIDA) && PID_Compute(&PIDB))
			if(HAL_GetTick() - tick > 2500L)
			{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
			}

		if (period > 200){
			HAL_Delay(200);
			period -= 200;
		} else {
			HAL_Delay(period);
			period -= period;
		}
	}
		stopMove();
		HAL_Delay(1000);
}


void move_forward_distance(double distance, double speed)
{
		float initial_dist = (distA+distB)/2 +1  ;  //+1 for error
		htim1.Instance->CCR4 = 180; // set direction of the wheel to be forward
		HAL_Delay(500);
		htim1.Instance->CCR4 = 151; // set direction of the wheel to be forward
		HAL_Delay(500);

		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);

		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speed);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, speed);
		while ((((distA+distB)/2)-initial_dist) < distance  )
		{

		}
		stopMove();
}

void move_backward_distance(double distance, double speed)
{
		float initial_dist = (distA+distB)/2 +1  ;  //+1 for error
		htim1.Instance->CCR4 = 180; // set direction of the wheel to be forward
		HAL_Delay(500);
		htim1.Instance->CCR4 = 151; // set direction of the wheel to be forward
		HAL_Delay(500);

		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);

		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speed);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, speed);
		while ((((distA+distB)/2)-initial_dist) < distance  )
		{

		}
		stopMove();
}



void move_forward_PID_dis(double period, double setpoint)
{
    double initial_dist = distA;
    // set wheel direction accurately forward
	htim1.Instance->CCR4 = 180; // set direction of the wheel to be forward
	HAL_Delay(500);
	htim1.Instance->CCR4 = 151; // set direction of the wheel to be forward
	HAL_Delay(500);



    // controlling motors
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);


    //defineing PID variables
	double PIDOuta, PWMSetpointa = setpoint;
	double PIDOutb, PWMSetpointb = setpoint;


    //enabling PID control variables
	PID(&PIDA, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);


    //
	PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDA, 500);
	PID_SetOutputLimits(&PIDA, 1200, 1700);
	PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDB, 500);
	PID_SetOutputLimits(&PIDB, 1200, 1700);

	uint32_t tick = HAL_GetTick();

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1500);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1500);

	while ((distA-initial_dist) < period  ) {
		htim1.Instance->CCR4 = 151;
		//osDelay(100);

		OLED_ShowNumber(25,10,PIDOuta,5,12);
		OLED_ShowNumber(65,10,PIDOutb,5,12);


		if(PID_Compute(&PIDA) && PID_Compute(&PIDB))
			if(HAL_GetTick() - tick > 1000L)
			{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
			}

	}
		stopMove();
		HAL_Delay(1000);
}




void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
  uint32_t cnt1A, cnt1B, cnt2A, cnt2B, diff1A, diff1B, diff2A, diff2B, dur, tick1, tick2;
  uint8_t yawer[10];
  		float gz;
  		float tmnow = 0;
  		float tmold = 0;
		float dt = 0;
  		char sbuf[5][100];
  		float returnValue =0;
  		float old_val = 0;
  		int n = 5;
  		float p_array[5] = { 0 };
  		float c_array[5] = {0};
  		int key;
  		int j;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  cnt1A = __HAL_TIM_GET_COUNTER(&htim2);
  cnt1B = __HAL_TIM_GET_COUNTER(&htim3);

  float check1 = 0;
  float check2 = 0;
  float check3 = 0;
  float check4 = 0;

  char bus[15];
  tick1 = HAL_GetTick();
  tick2 = HAL_GetTick();
  int acceptance = 0;
  while(acceptance==0){
	  Gyro_calibrate(&imu);
	  IMU_GyroRead(&imu);
	  check1 = imu.gyro[2];
	  HAL_Delay(200);
	  HAL_Delay(200);
	  HAL_Delay(200);
	  IMU_GyroRead(&imu);
	  yaw_g = 0;
	  check4 = imu.gyro[12];
	  if (check1 < 0.3 &&  check1 >-0.3 && check4 < 0.3 && check4 >-0.3){
		  acceptance = 1;
//		  sprintf(yawer, "L%5.2f,  %5.2f--  \r\n", check1, check4);
//		  HAL_UART_Transmit(&huart3, yawer, 15, HAL_MAX_DELAY);
	  }
  }
  /* Infinite loop */
  for(;;)
  {
	  			  tmnow = HAL_GetTick();
	  			  dt = tmnow -tmold;
	  			  tmold = tmnow;
	  			  IMU_GyroRead(&imu);
	  			  gz = imu.gyro[2];
	  			  yaw_g = yaw_g + (gz*dt*0.001);
					if(yaw_g>=180){
						  yaw_g= yaw_g -360;
					  }
					  else if(yaw_g<=-180){
						  yaw_g= yaw_g + 360;
					  }


	  			  p_array[0] = yaw_g;
	  			  c_array[0] = yaw_g;

//	  			  sprintf(yawer, "M%5.2f,  %5.2f--  \r\n", yaw_g, imu.gyro[2]);
//	  			  HAL_UART_Transmit(&huart3, yawer, 15, HAL_MAX_DELAY);
	  			  sprintf(yawer, "%d\r\n", (int)yaw_g);
	  			  OLED_ShowString(10,20, yawer);

	  			  for(int i = 4 ; i >0 ; i--){
	  				  p_array[i] = p_array[i-1];
	  				  c_array[i] = p_array[i];
	  			  }


	  			 for (int i = 1; i < n; ++i) {
	  			             key = c_array[i];
	  			             j = i - 1;

	  			             // Move elements of arr[0..i-1],
	  			             // that are greater than key,
	  			             // 0 1 2 3 4 5 6
	  			             // to one position ahead of
	  			             // their current position
	  			             while (j >= 0 && c_array[j] > key) {
	  			                 c_array[j + 1] = c_array[j];
	  			                 j = j - 1;
	  			             }
	  			             c_array[j + 1] = key;
	  			      }
	  			 ref_angle = c_array[2];
//	  		  	 sprintf(yawer, "%5.2f,--  \r\n", c_array[2]);
//	  			 HAL_UART_Transmit(&huart3, yawer, 10, HAL_MAX_DELAY);
	  			 HAL_Delay(10);
	  		     old_val = yaw_g;



		tick2 = HAL_GetTick();
		dur = tick2 >= tick1 ? tick2 - tick1 : 4294967295 - tick1 + tick2;

		if (dur > 50)
		{
		cnt2A = __HAL_TIM_GET_COUNTER(&htim2);
		cnt2B = __HAL_TIM_GET_COUNTER(&htim3);

		diff1A = abs(cnt1A - cnt2A);
		diff1B = abs(cnt1B - cnt2B);
		diff2A = abs(65535 - (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? cnt2A - cnt1A : cnt1A - cnt2A));
		diff2B = abs(65535 - (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3) ? cnt2B - cnt1B : cnt1B - cnt2B));

		encoderAVal = diff1A < diff2A ? diff1A : diff2A;
		encoderBVal = diff1B < diff2B ? diff1B : diff2B;


		tempA += pi * radius * encoderAVal / 165;
		tempB += pi * radius * encoderBVal / 165;

		distA += 1.065* encoderGrad * pi * radius * encoderAVal / 165;
		distB += encoderGrad * pi * radius * encoderBVal / 165;

		sprintf(bus,"Dist:%5.2f\0",distB);
		OLED_ShowString(10,50,bus);

		cnt1A = __HAL_TIM_GET_COUNTER(&htim2);
		cnt1B = __HAL_TIM_GET_COUNTER(&htim3);

		tick1 = HAL_GetTick();
	}
  }
  /* USER CODE END encoder */
}






void move_backward_PID(double period, double setpoint){
	period = period *10;
	period = period * (18.74*1.5/1.33)*1.11;
	htim1.Instance->CCR4 = 180; // set direction of the wheel to be forward
	HAL_Delay(500);
	htim1.Instance->CCR4 = 152; // set direction of the wheel to be forward
	HAL_Delay(500);

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);

	double PIDOuta, PWMSetpointa = setpoint;
	double PIDOutb, PWMSetpointb = setpoint;


	PID(&PIDA, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDA, 500);
	PID_SetOutputLimits(&PIDA, 2500, 3500);
	PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDB, 500);
	PID_SetOutputLimits(&PIDB, 2500, 3500);

	uint32_t tick = HAL_GetTick();

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 3000);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 3000);

	while (period>0) {
		htim1.Instance->CCR4 = 152;
		//osDelay(100);

		OLED_ShowNumber(25,10,PIDOuta,5,12);
		OLED_ShowNumber(65,10,PIDOutb,5,12);

		if(PID_Compute(&PIDA) && PID_Compute(&PIDB))
			if(HAL_GetTick() - tick > 2500L)
			{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
			}

		if (period > 200){
			HAL_Delay(200);
			period -= 200;
		} else {
			HAL_Delay(period);
			period -= period;
		}
	}
		stopMove();
		HAL_Delay(1000);
}



void moveForward_time(uint16_t t, uint16_t speed){
//	htim1.Instance->CCR4 = angle; // set direction of the wheel to be forward

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // left RC motor
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // right RC motor
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	// stepper motor
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);

	HAL_Delay(t);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
}


void moveBackward_time(uint16_t t, uint16_t speed){
//	htim1.Instance->CCR4 = angle; // set direction of the wheel to be forward
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // left RC motor
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // right RC motor
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	// stepper motor
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);

	HAL_Delay(t);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
}

void moveForwardLeft2(uint32_t speed, uint32_t angle)
{
	double PIDOuta, PWMSetpointa = 1000;
	double PIDOutb, PWMSetpointb = 1000;


	PID(&PIDC, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDA, 100);
	PID_SetOutputLimits(&PIDA, speed-300, speed+300);
	PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDB, 100);
	PID_SetOutputLimits(&PIDB, speed-300, speed+300);

	uint32_t tick = HAL_GetTick();

	uint8_t yawer[10];
		htim1.Instance->CCR4 = angle;
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);


	if(lumos_dir == 'N'){
		while(ref_angle <=90){
			if(PID_Compute(&PIDC) && PID_Compute(&PIDD))
				if(HAL_GetTick() - tick > 100L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}

			}
		lumos_dir = 'W';
	}
	else if(lumos_dir=='W'){
		while(ref_angle<=180 && ref_angle > 0){
			if(PID_Compute(&PIDC) && PID_Compute(&PIDD))
				if(HAL_GetTick() - tick > 100L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}

		}
		lumos_dir = 'S';
	}
	else if(lumos_dir=='S'){
		while(ref_angle > 0){
			if(PID_Compute(&PIDC) && PID_Compute(&PIDD))
				if(HAL_GetTick() - tick > 100L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}

		}

		while(ref_angle <= -90){
			if(PID_Compute(&PIDC) && PID_Compute(&PIDD))
				if(HAL_GetTick() - tick > 100L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
		}
		lumos_dir = 'E';
	}
	else if(lumos_dir=='E'){
		while(ref_angle< 0){
			if(PID_Compute(&PIDC) && PID_Compute(&PIDD))
				if(HAL_GetTick() - tick > 100L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
		}
		lumos_dir = 'N';
	}


	stopMove();

}



void turnLeft_x_angles(int speeda, int speedb, int wheel_angle, float required_angle)
{
	float curr_angle = ref_angle;

	double PIDOuta, PWMSetpointa = 1000;
	double PIDOutb, PWMSetpointb = 1000;


	PID(&PIDC, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDA, 100);
	PID_SetOutputLimits(&PIDA, speeda-300, speeda+300);
	PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDB, 100);
	PID_SetOutputLimits(&PIDB, speedb-300, speedb+300);

	uint32_t tick = HAL_GetTick();

	uint8_t yawer[10];
		htim1.Instance->CCR4 = wheel_angle;
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speeda);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speedb);

	while(fabs(ref_angle-curr_angle )< required_angle){
				if(PID_Compute(&PIDC) && PID_Compute(&PIDD)){
					if(HAL_GetTick() - tick > 100L)
					{
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
					}
				}
	}


	yaw_g = 0;
	lumos_dir = 'N';

}

void turnRight_x_angles(int speeda, int speedb, float wheel_angle, float required_angle )
{
	float curr_angle = ref_angle;

	double PIDOuta, PWMSetpointa = 1000;
	double PIDOutb, PWMSetpointb = 1000;


	PID(&PIDC, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDA, 100);
	PID_SetOutputLimits(&PIDA, speeda-300, speeda+300);
	PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDB, 100);
	PID_SetOutputLimits(&PIDB, speedb-300, speedb+300);

	uint32_t tick = HAL_GetTick();

	uint8_t yawer[10];
		htim1.Instance->CCR4 = wheel_angle;
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speeda);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speedb);

	while(fabs(ref_angle-curr_angle )< required_angle){
if(PID_Compute(&PIDC) && PID_Compute(&PIDD)){
	if(HAL_GetTick() - tick > 100L)
		{
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
		}
	}
	}

	yaw_g = 0;
	lumos_dir = 'N';
}



void moveForwardLeft(int t){
	/*
	 * 			"*q12?? => 90* left. A little extra"
	 * 			"*q24??" => 90deg
	 * */
//	uint32_t period = 1125;   // PWM 3000,350 Left 90 deg inside floor, 55
//	htim1.Instance->CCR4 = 95;
//	HAL_Delay(100);
//		// set pins
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1200);
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 3000);
//	HAL_Delay(t);
//	stopMove();
	double PIDOuta, PWMSetpointa = 1500;
	double PIDOutb, PWMSetpointb = 1500;


	PID(&PIDC, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDA, 500);
	PID_SetOutputLimits(&PIDA, 1300, 2000);
	PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDB, 500);
	PID_SetOutputLimits(&PIDB, 1300, 2000);

	uint32_t tick = HAL_GetTick();

	uint8_t yawer[10];
		htim1.Instance->CCR4 = 200;
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2000);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000);


		if(lumos_dir == 'N'){
			while(ref_angle <=85){
				if(PID_Compute(&PIDC) && PID_Compute(&PIDD))
					if(HAL_GetTick() - tick > 100L)
					{
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
					}

				}
			lumos_dir = 'W';
		}
		else if(lumos_dir=='W'){
			while(ref_angle<=180 && ref_angle > 0){
				if(PID_Compute(&PIDC) && PID_Compute(&PIDD))
					if(HAL_GetTick() - tick > 100L)
					{
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
					}
			}
			lumos_dir = 'S';
		}
		else if(lumos_dir=='S'){
			while(ref_angle > 0){
				if(PID_Compute(&PIDC) && PID_Compute(&PIDD))
					if(HAL_GetTick() - tick > 100L)
					{
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
					}

			}

			while(ref_angle <= -90){
				if(PID_Compute(&PIDC) && PID_Compute(&PIDD))
					if(HAL_GetTick() - tick > 100L)
					{
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
					}
			}
			lumos_dir = 'E';
		}
		else if(lumos_dir=='E'){
			while(ref_angle< 0){
				if(PID_Compute(&PIDC) && PID_Compute(&PIDD))
					if(HAL_GetTick() - tick > 100L)
					{
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
					}
			}
			lumos_dir = 'N';
		}


		stopMove();
}


void moveForwardRight2(uint32_t speed, uint32_t angle)
{
	double PIDOuta, PWMSetpointa = speed;
	double PIDOutb, PWMSetpointb = speed;


	PID(&PIDC, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDD, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);


	PID_SetMode(&PIDC, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDC, 100);
	PID_SetOutputLimits(&PIDC, speed-300, speed+300);
	PID_SetMode(&PIDD, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDD, 100);
	PID_SetOutputLimits(&PIDD, speed-300, speed+300);

	uint32_t tick = HAL_GetTick();

	uint8_t yawer[10];
		htim1.Instance->CCR4 = angle;
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);


	if(lumos_dir == 'N'){
		while(ref_angle >=-85){
			if(HAL_GetTick() - tick > 500L)
			{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
			}
			}
		lumos_dir = 'E';
	}
	else if(lumos_dir=='E'){
		while(ref_angle>=-180 && ref_angle < 0){
			if(HAL_GetTick() - tick > 500L)
			{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
			}
		}
		lumos_dir = 'S';
	}
	else if(lumos_dir=='S'){
		while(ref_angle < 0){
			if(HAL_GetTick() - tick > 500L)
			{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
			}
		}

		while(ref_angle >= 90){
			if(HAL_GetTick() - tick > 500L)
			{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
			}
		}
		lumos_dir = 'W';
	}
	else if(lumos_dir=='W'){
		while(ref_angle>0){
			if(HAL_GetTick() - tick > 500L)
			{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
			}
		}
		lumos_dir = 'N';
	}


	stopMove();

}



void moveForwardRight(int t){
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
//	htim1.Instance->CCR4 = 210;    //extreme right
//	HAL_Delay(100);		// set pins
//
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1000);
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 3000);
//	HAL_Delay(t);
//	stopMove();

		double PIDOuta, PWMSetpointa = 1500;
		double PIDOutb, PWMSetpointb = 1500;


		PID(&PIDC, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
		PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

		PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
		PID_SetSampleTime(&PIDA, 500);
		PID_SetOutputLimits(&PIDA, 1300, 2000);
		PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
		PID_SetSampleTime(&PIDB, 500);
		PID_SetOutputLimits(&PIDB, 1300, 2000);

		uint32_t tick = HAL_GetTick();

	uint8_t yawer[10];
		htim1.Instance->CCR4 = 105;
		HAL_Delay(300);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2000);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000);

		if(lumos_dir == 'N'){
			while(ref_angle >=-85){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
				}
			lumos_dir = 'E';
		}
		else if(lumos_dir=='E'){
			while(ref_angle>=-180 && ref_angle < 0){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}
			lumos_dir = 'S';
		}
		else if(lumos_dir=='S'){
			while(ref_angle < 0){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}

			while(ref_angle >= 90){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}
			lumos_dir = 'W';
		}
		else if(lumos_dir=='W'){
			while(ref_angle>0){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}
			lumos_dir = 'N';
		}


		stopMove();
}

void moveBackwardLeft(int t){
	double PIDOuta, PWMSetpointa = 1500;
	double PIDOutb, PWMSetpointb = 1500;


	PID(&PIDC, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDA, 500);
	PID_SetOutputLimits(&PIDA, 1300, 2000);
	PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDB, 500);
	PID_SetOutputLimits(&PIDB, 1300, 2000);

	uint32_t tick = HAL_GetTick();

uint8_t yawer[10];
	htim1.Instance->CCR4 = 105;
	HAL_Delay(300);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000);


		if(lumos_dir == 'N'){
			while(ref_angle >=-90){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
				}
			lumos_dir = 'E';
		}
		else if(lumos_dir=='E'){
			while(ref_angle>=-180 && ref_angle < 0){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}
			lumos_dir = 'S';
		}
		else if(lumos_dir=='S'){
			while(ref_angle < 0){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}

			while(ref_angle >= 90){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}
			lumos_dir = 'W';
		}
		else if(lumos_dir=='W'){
			while(ref_angle>0){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}
			lumos_dir = 'N';
		}


		stopMove();
}

void moveBackwardRight(int t){
	double PIDOuta, PWMSetpointa = 1500;
	double PIDOutb, PWMSetpointb = 1500;


	PID(&PIDC, &encL, &PIDOuta, &PWMSetpointa, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&PIDB, &encR, &PIDOutb, &PWMSetpointb, 0.17, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&PIDA, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDA, 500);
	PID_SetOutputLimits(&PIDA, 1300, 2000);
	PID_SetMode(&PIDB, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&PIDB, 500);
	PID_SetOutputLimits(&PIDB, 1300, 2000);

	uint32_t tick = HAL_GetTick();

	uint8_t yawer[10];
		htim1.Instance->CCR4 = 200;
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);


		if(lumos_dir == 'N'){
			while(ref_angle <=90){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
				}
			lumos_dir = 'W';
		}
		else if(lumos_dir=='W'){
			while(ref_angle<=180 && ref_angle > 0){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}
			lumos_dir = 'S';
		}
		else if(lumos_dir=='S'){
			while(ref_angle > 0){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}

			while(ref_angle <= -90){
				if(HAL_GetTick() - tick > 500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}
			lumos_dir = 'E';
		}
		else if(lumos_dir=='E'){
			while(ref_angle< 0){
				if(HAL_GetTick() - tick > 2500L)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PIDOuta);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PIDOutb);
				}
			}
			lumos_dir = 'N';
		}


		stopMove();
}


void goLeft(){
	htim1.Instance->CCR4 = 100;
	HAL_Delay(1000);
	move(3000);
}
void goRight(){
	htim1.Instance->CCR4 = 200;
	HAL_Delay(1000);
	move(3000);
}

int concatValue(uint8_t aRxBuffer[], uint8_t start, uint8_t end)
{
	int ret = 0, i;
	for(i=start;i<=end;++i) {
		ret = ret*10 + (aRxBuffer[i]-48);
	}
	return ret;
}


void clearBuffer(){
	for(int i = 0 ; i < 20; i++){
		aRxBuffer[i] = '0';
	}
}

int getInputMagnitude(char msb , char lsb){
	int msb_in = (msb-48)*10;
	int lsb_in = (lsb-48);
	return msb_in + lsb_in;
}


void enqueue_dequeue(sint16 new_val){
	for(int i = i ; i < 10; i++){
		input_test_data1[9-i] = input_test_data1[9-i-1];
	}
	input_test_data1[0] = new_val;
}


void move_f_for_time(int t, int speed)
{
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);

	HAL_Delay(t);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
}


void move_b_for_time(int t, int speed)
{
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);

	HAL_Delay(t);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  OLED_Init();
  uint8_t* status = IMU_Initialise(&imu, &hi2c1, &huart3);
  while(status!=0){
	  HAL_Delay(1000);
	  status = IMU_Initialise(&imu, &hi2c1, &huart3);
  }
  for(int i = 0 ; i < 10; i++){
	  input_test_data1[i] = 0;
  }
  pi = 3.1415926536;
  radius = 3.35;
  startDriving = 0;
  yaw_g = 0;
  tempA = 0;
  adjustDist = 5; //adjustDist > distBuffer + encoderInt
  changeDist = 10;
  encoderDelay = 25;
  lumos_dir = 'N';
  encoderGrad = 0.208496572267945;
  encoderInt = 1.08776618891631;

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // left RC motor
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // right RC motor
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	// stepper motor

  for (int counter = 0; counter < 5; ++counter)
  {
	yawVal[counter] = 0;
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(motor, NULL, &motorTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(imuRead, NULL, &imuTask_attributes);

  /* creation of encoderTaskA */
  // encoderTaskAHandle = osThreadNew(encoderA, NULL, &encoderTaskA_attributes);

  /* creation of encoderTaskB */
  //encoderTaskBHandle = osThreadNew(encoderB, NULL, &encoderTaskB_attributes);


  encoderTaskMAINHandle = osThreadNew(encoder, NULL, &encoderTaskMAIN_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin PE1 */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallBack( UART_HandleTypeDef *huart){
	/*Prevent unused argument(s) compilation warning*/
	UNUSED(huart);
	HAL_UART_Transmit(&huart3, (uint8_t *)aRxBuffer, 20, 0xFFFF);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  uint8_t oled_display[20] = "LOSLPOSL";
  float initial_distance = 0;
	//moveForward_time(2000);


  for(;;)
  {
	HCSR04_Read();
	HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 20);
	sprintf(oled_display,"%s" ,aRxBuffer );
	OLED_ShowString(4,5, oled_display);
	OLED_Refresh_Gram();

	if(aRxBuffer[0]=='w')
	{
		goW(concatValue(aRxBuffer, 2,6), concatValue(aRxBuffer,7, 10));
		clearBuffer();
	}
	else if(aRxBuffer[0]=='a')
	{
		angle-=10;
		setWheelAngle(angle);
		OLED_ShowNumber(65,50,angle,5,12);
		clearBuffer();
	}
	else if(aRxBuffer[0]=='s')
	{
		goS(concatValue(aRxBuffer, 2,6), concatValue(aRxBuffer,7, 10));
		clearBuffer();
	}
	else if(aRxBuffer[0]=='d')
	{
		angle+=10;
		setWheelAngle(angle);
		OLED_ShowNumber(65,50,angle,5,12);
		clearBuffer();
	}

	if(aRxBuffer[0] == '*' && aRxBuffer[19]=='?')
	{
			switch(aRxBuffer[1]){
						case 'q':
							//executing left turn
							OLED_ShowString(10,60 , "Moving Left");
							OLED_Refresh_Gram();
//							move_forward_PID(2.9, 3000);
//							moveForwardLeft(getInputMagnitude((int)aRxBuffer[2], (int)aRxBuffer[3]));
							moveForwardLeft(((int)((int)aRxBuffer[2]-48))*1000 + ((int)((int)aRxBuffer[3]-48)) * 100 + ((int)(aRxBuffer[4]-48))*10);
//							move_forward_PID(3.6, 3000);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'w':
							//executing forward
							OLED_ShowString(10,60, "Moving Forward");
							OLED_Refresh_Gram();
							move_forward_PID(getInputMagnitude((int)aRxBuffer[2], (int)aRxBuffer[3]), 3000);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'e':
							//executing right turn
							OLED_ShowString(10,60, "Moving Right");
							OLED_Refresh_Gram();
							//move_forward_PID_dis(((int)((int)aRxBuffer[2]-48))*10 + ((int)((int)aRxBuffer[3]-48)) * 1, 1500);
							move_forward_distance((((int)(aRxBuffer[2]-48))*10 + ((int)((int)aRxBuffer[3]-48)) * 1), 800);
//							moveForwardRight(getInputMagnitude((int)aRxBuffer[2], (int)aRxBuffer[3]));
//							move_forward_PID(3.6, 3000);
//							moveForwardRight(((int)((int)aRxBuffer[2]-48))*1000 + ((int)((int)aRxBuffer[3]-48)) * 100 + ((int)(aRxBuffer[4]-48))*10);
//							move_forward_PID(3, 3000);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'a':
							OLED_ShowString(10,60 , "Rev Left");
							OLED_Refresh_Gram();
//							moveBackwardLeft(getInputMagnitude((int)aRxBuffer[2], (int)aRxBuffer[3]));
							htim1.Instance->CCR4 =	((int)aRxBuffer[2]-48)*100 + ((int)aRxBuffer[3]-48)*10 + (aRxBuffer[4]-48);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 's':
							OLED_ShowString(10,60, "Reversing");
							OLED_Refresh_Gram();
							move_backward_PID(getInputMagnitude((int)aRxBuffer[2], (int)aRxBuffer[3]), 3000);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'd':
							OLED_ShowString(10,60, "Rev Right");
							OLED_Refresh_Gram();
							htim1.Instance->CCR4 = 180;
//							move_backward_PID(3.6, 3000);
//							moveBackwardRight(getInputMagnitude((int)aRxBuffer[2], (int)aRxBuffer[3]));
//							moveBackwardRight(((int)((int)aRxBuffer[2]-48))*1000 + ((int)((int)aRxBuffer[3]-48)) * 100 + ((int)(aRxBuffer[4]-48))*10);
//							move_backward_PID(2.9, 3000);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'x':
							OLED_ShowString(10,60, "Taking Picture");
							OLED_Refresh_Gram();
							stopMove();
							forward_motion = 0;
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case '1':
							sendPWMtoMotor(concatValue(aRxBuffer, 2,5));
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case '2':
							//straighten wheels
							straightenWheel();
							clearBuffer();
							HAL_Delay(300);
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case '3':
							//enable motor forward direction
							motorDirectionForward();
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case '4':
							//enable motor reverse direction
							motorDirectionBackward();
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case '5':
							//send pwm to motorA
							_sendPWMtoA(concatValue(aRxBuffer, 2,5));
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case '6':
							//send pwm to motorB
							_sendPWMtoB(concatValue(aRxBuffer, 2,5));
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case '7':
							//send angle to stepper motor
							setWheelAngle(concatValue(aRxBuffer, 2,4));
							HAL_Delay(300);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case '8':
							//delay motor
							HAL_Delay(concatValue(aRxBuffer, 2,6));
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case '9':
							//run forward for distance
							// it should complete the first x - 10 cm at high pwm
							// it should then complete the next 10 cm at low pwm for accuracy
							move_forward_distance(concatValue(aRxBuffer, 2, 4), concatValue(aRxBuffer, 5, 8));
							HAL_Delay(400);
							move_forward_distance(10, 800);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'p':
							//run backward for distance
							move_backward_distance(concatValue(aRxBuffer, 2, 4), concatValue(aRxBuffer, 5, 8));
							HAL_Delay(400);
							move_backward_distance(10, 800);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'o':
							// run forward for time
							moveForward_time(concatValue(aRxBuffer, 2, 6), concatValue(aRxBuffer, 7, 10));
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'i':
							// run backward for time
							forward_motion = 1;
							moveBackward_time(concatValue(aRxBuffer, 2, 6), concatValue(aRxBuffer, 7,10));
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							forward_motion = 1;
							break;
						case 'u':
							// run left forward until angle changes by 90
							moveForwardLeft2(concatValue(aRxBuffer, 2, 5), concatValue(aRxBuffer, 6,8));
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'y':
							// run right forward until angle changes by 90
							moveForwardRight2(concatValue(aRxBuffer, 2,5), concatValue(aRxBuffer, 6, 8));
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 't':
							HAL_Delay(10000);
							moveForwardLeft2(800, 90);
							HAL_Delay(300);
							moveForwardRight2(1000, 240);
							HAL_Delay(300);
							moveForwardRight2(1000, 240);
							HAL_Delay(300);
							moveForwardLeft2(800, 90);
							HAL_Delay(300);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'r':
							move_till_obstacle(concatValue(aRxBuffer, 2,4)	, concatValue(aRxBuffer, 5 ,6)	, concatValue(aRxBuffer, 7,10)		,	concatValue(aRxBuffer, 11, 14)	);
							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'g':
//							setWheelAngle(90);
//							HAL_Delay(100);
//							moveForward_time();
//							straightenWheel();
//							HAL_Delay(100);
//							moveBackward_time();
//							setWheelAngle(240);
//							moveForward_time();


							clearBuffer();
							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;
						case 'h':
//							setWheelAngle(240);
//							HAL_Delay(100);
//							moveForward_time();
//							straightenWheel();
//							HAL_Delay(100);
//							moveBackward_time();
//							setWheelAngle(90);
//							moveForward_time();
//
//							clearBuffer();
//							HAL_UART_Transmit_IT(&huart3, (uint8_t *)ack, 10);
							break;

			}
	}
	//memset(aRxBuffer, 0, sizeof aRxBuffer);
  }
  /* USER CODE END 5 */
}



void stopMove(){
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
}

void moveForward_dist(int distance){

	osDelay(10000);			// delay for disconnecting robot and moving
	uint32_t period = ((float)distance-0.2) / 0.0324;   // PWM 2000

	htim1.Instance->CCR4 = 150; // set direction of the wheel to be forward
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, SPEED);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, SPEED);

	HAL_Delay(period);

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
		HAL_Delay(4000);
}
//void moveForward_time(uint32_t t){
//	htim1.Instance->CCR4 = 150; // set direction of the wheel to be forward
//	int counter = 1000;
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, SPEED);
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, SPEED);
//
//	HAL_Delay(t);
//
//
//	OLED_ShowString(10,50, "Stopping");
//	OLED_Refresh_Gram();
//
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
//	HAL_Delay(4000);
//
//
//}
void move(uint32_t time){
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, SPEED);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, SPEED);

	HAL_Delay(time);


	OLED_ShowString(10,50, "Stopping");
	OLED_Refresh_Gram();

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
	HAL_Delay(4000);

}




void moveBackward_dist(){

}
//void moveBackward_time(int t){
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, SPEED);
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, SPEED);
//
//	HAL_Delay(t);
//	OLED_ShowString(10,50, "Stopping");
//	OLED_Refresh_Gram();
//
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
//	HAL_Delay(4000);
//}




//
//void turn_left_by_angle(uint32_t angle) {
//	uint32_t period = ((float)angle * 48.5 * M_PI * 1000.0) / (14.5 * 360.0) - (450 * ((float)angle / 90.0));  // PWM 2000,285 Inside floor
////	uint32_t period = 2950;   // PWM 2000,285 Left 90 deg outside floor
//	uint32_t period = 1125;   // PWM 3000,350 Left 90 deg inside floor, 55
////	uint32_t period = 835;   // PWM 5000,1000 Left 90 deg inside floor, 60
////	uint32_t period = 1300;   // PWM 3000,350 Left 90 deg outside floor
//	turn_left(true, period);
//}







/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void *argument)
{
  /* USER CODE BEGIN motor */
  /* Infinite loop */

	for(;;){

	}


  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_imuRead */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imuRead */
void imuRead(void *argument)
{
  /* USER CODE BEGIN imuRead */
  /* Infinite loop */
	//uint8_t ch = 'A';
	//char temp[20];
	char sbuf[9][100];
	for(;;){
		//IMU_AccelRead(&imu);
		//sprintf(sbuf[0], "Accelerometer[x] : %5.2f Accelerometer[y] :%5.2f Accelerometer[z] : %5.2f", imu.acc[0], imu.acc[1], imu.acc[2]);
		//HAL_UART_Transmit_IT(&huart3, (uint8_t *)sbuf[0], 73);
		//IMU_GyroRead(&imu);
		//sprintf(sbuf[1], "Gyroscopemete[x] : %5.2f Gyroscopemete[y] :%5.2f GyroscopeMete[z] : %5.2f", imu.gyro[0], imu.gyro[1], imu.gyro[2]);
		//HAL_UART_Transmit_IT(&huart3, (uint8_t *)sbuf[1], 73);

		HAL_Delay(1000);
	//	HAL_UART_Transmit_IT();
//		 sprintf(sbuf[1], "%5.2f,", imu.acc[1]);
//		 HAL_UART_Transmit(&huart3, (uint8_t *)sbuf[1], 6, HAL_MAX_DELAY);
//		 sprintf(sbuf[2], "%5.2f,", imu.acc[2]);
//		 HAL_UART_Transmit(&huart3, (uint8_t *)sbuf[2], 6, HAL_MAX_DELAY);
//		 sprintf(sbuf[3], "%5.2f,", imu.gyro[0]);
//		 HAL_UART_Transmit(&huart3, (uint8_t *)sbuf[3], 6, HAL_MAX_DELAY);
//		 sprintf(sbuf[4], "%5.2f,", imu.gyro[1]);
//		 HAL_UART_Transmit(&huart3, (uint8_t *)sbuf[4], 6, HAL_MAX_DELAY);
//		 sprintf(sbuf[5], "%5.2f,", imu.gyro[2]);
//		 HAL_UART_Transmit(&huart3, (uint8_t *)sbuf[5], 6, HAL_MAX_DELAY);
//
//		 HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
		//IMU_GyroRead(&imu);
		//sprintf(temp, "%5.2f,%5.2f,%5.2f\r\n",imu.acc[0], imu.acc[1], imu.acc[2]);
		//HAL_UART_Transmit(&huart3, temp, 20, HAL_MAX_DELAY);

	}
  /* USER CODE END imuRead */
}



void motorDirectionForward()
{
		_motorAForward();
		_motorBForward();
}


void _motorAForward()
{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
}


	void _motorBForward()
{
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
}


void motorDirectionBackward()
{
		_motorABackward();
		_motorBBackward();
}


void _motorABackward()
{
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);}


void _motorBBackward()
{
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);}


void setWheelAngle(uint16_t angle)
{
		htim1.Instance->CCR4 = angle; // set direction of the wheel to be forward
//		HAL_Delay(300);
}

void sendPWMtoMotor(uint16_t speed)
{
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // left RC motor
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // right RC motor
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	// stepper motor

	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);
}


void _sendPWMtoA(uint16_t PWM)
{
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM);
}

void _sendPWMtoB(uint16_t PWM)
{
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PWM);
}


void runForwardFor_Time(uint16_t time)
{
		motorDirectionForward();
		osDelay(time);
		stopMotor();
}


void runBackwardFor_Time(uint16_t time)
{
		motorDirectionBackward();
		osDelay(time);
		stopMotor();
}


/*
 * threshold1 is always greater than threshold2*/
void move_till_obstacle(int threshold1, int threshold2, int speed1, int speed2)
{
	HCSR04_Read();
	HAL_Delay(100);
	while(Distance > threshold1)
	{
		HCSR04_Read();
		HAL_Delay(100);
		goF(speed1);
	}
	while(Distance > threshold2 )
	{
		HCSR04_Read();
		HAL_Delay(100);
		goF(speed2);
	}
	stopMove();
}


void goF(int speed)
{
	  	 HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // left RC motor
	  	 HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // right RC motor
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);
}



void goB(int speed)
{
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // left RC motor
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // right RC motor
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);
}


void goW(int time, int speed)
{
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // left RC motor
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // right RC motor
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);
		HAL_Delay(time);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
		HAL_Delay(400);
}

void goS(int time , int speed)
{
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // left RC motor
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // right RC motor
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);
		HAL_Delay(time);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
		HAL_Delay(400);}




void straightenWheel()
{
		htim1.Instance->CCR4 = 180; // set direction of the wheel to be forward
		HAL_Delay(200);
		htim1.Instance->CCR4 = 154	; // set direction of the wheel to be forward
		HAL_Delay(200);
}


void stopMotor()
{
		_stopMotorA();
		_stopMotorB();
}

	void _stopMotorA()
	{
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	}

	void _stopMotorB()
	{
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
	}




/* USER CODE BEGIN Header_encode */
/**
* @brief Function implementing the encoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encode */
void encode(void *argument)
{
  /* USER CODE BEGIN encode */
  /* Infinite loop */
		for(;;)
		{

		}
  /* USER CODE END encode */
}

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
