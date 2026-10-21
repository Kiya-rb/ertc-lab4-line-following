/* USER CODE sections of main.c as submitted for Lab 4 (line following).
 * The course framework and the STM32CubeMX-generated skeleton around these
 * sections belong to the Embedded Real-Time Control course and are not
 * redistributed here. */

/* ---- USER CODE: Header ---- */
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
/* ---- USER CODE: Includes ---- */
#include <math.h>
#include <stdbool.h>
#include <stdlib.h> // For atoi()
#include "ertc-datalogger.h"
#include "SX1509_Registers.h"
/* ---- USER CODE: PTD ---- */

/* ---- USER CODE: PD ---- */
//#define	HAL_TIMEOUT		1
#define TS	0.01

#define PITCH 0.008f     // 8 mm
#define H 0.085         // Distance from sensor to center
#define D 0.165         // Wheelbase
#define WHEEL_RADIUS 0.034f // Wheel radius
#define SensorsNumber 8.0f // number of sensors
#define M_PI 3.1415926535

#define VBATT	7.6
#define V2DUTY	((float)(TIM8_ARR_VALUE+1)/VBATT)
#define DUTY2V	((float)VBATT/(TIM8_ARR_VALUE+1))

#define RPM2RADS	2*M_PI/60

#define P_gain 0.8
#define I_gain 7.5563

#define SX1509_I2C_ADDR1 0x3E //	SX1509 Proxy Sensors I2C address
#define SX1509_I2C_ADDR2 0x3F //	SX1509 Keypad I2C address

#define HAL_TIMEOUT 100
#define I2C_TIMEOUT 200

#define MAX_DIGITS 3  // Maximum 3-digit frequency (e.g., "125#")
/* ---- USER CODE: PM ---- */

/* ---- USER CODE: PV ---- */
struct ertc_dlog logger;
/* ---- USER CODE: PFP ---- */

/* ---- USER CODE: 0 ---- */
int _write(int file, char *ptr, int len)
{
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}

struct datalog {
	float w1, w2;
	float u1, u2, u3;
} data;

float currentSpeedtest = 0;

static uint32_t PreviousCount1 = 0;
static uint32_t PreviousCount2 = 0;
// reads the encoders and calculate the motor speed
float ReadCurrentSpeed(int motorNumber,uint32_t* PreviousCount){

  uint32_t CurrentCount = 0;
  int32_t DiffCount = 0;
  //static uint32_t PreviousCount = 0;

  // get diff count for specified motor htim3 for motor 1 and htim4 for motor 2
  switch (motorNumber){
    case 1:
      CurrentCount = __HAL_TIM_GET_COUNTER(&htim3);

      // calculate diff count of motor 1
      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
        {
          /* check for counter underflow */
          if (CurrentCount <= *PreviousCount) // Case of no underflow
            DiffCount = CurrentCount- *PreviousCount;
          else // Case of underflow
            DiffCount =-((TIM3_ARR_VALUE+1)- CurrentCount)- *PreviousCount;
        }
        else
        {
          /* check for counter overflow */
          if (CurrentCount >= *PreviousCount) // Case of no overflow
            DiffCount = CurrentCount- *PreviousCount;
          else	// Case of overflow
            DiffCount = ((TIM3_ARR_VALUE+1)- *PreviousCount) + CurrentCount;
        }
      *PreviousCount = CurrentCount; // Storing the current value

      break;
    case 2:
      CurrentCount = __HAL_TIM_GET_COUNTER(&htim4);

      // calculate diff count of motor 2
      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
        {
          /* check for counter underflow */
          if (CurrentCount <= *PreviousCount) // Case of no underflow
            DiffCount = CurrentCount- *PreviousCount;
          else // Case of underflow
            DiffCount =-((TIM4_ARR_VALUE+1)- CurrentCount)- *PreviousCount;
        }
        else
        {
          /* check for counter overflow */
          if (CurrentCount >= *PreviousCount) // Case of no overflow
            DiffCount = CurrentCount- *PreviousCount;
          else	// Case of overflow
            DiffCount = ((TIM4_ARR_VALUE+1)- *PreviousCount) + CurrentCount;
        }
      *PreviousCount = CurrentCount; // Storing the current value
      break;
    }

	/*
	// Calculating speed from pulse count
	/* We are using 4x encoder mode this means that Pulse-Per-Round for the wheel is 3840
	 * So the number of revolutions is TIM3_DiffCount/3840. To Calculate speed we should
	 * divide the number of revolutions by the sampling time Ts to get the speed in rps
	 * and then we can convert it to rpm by multiplying by 60
	 *
	 */

	float currentSpeed = 0;
	currentSpeed = ((float)DiffCount /(float)TIM3_ARR_VALUE)/ TS * 60.0; // Current speed in rpm
	currentSpeedtest = currentSpeed;
	return currentSpeed;
}

float clamp(float Output_temp){
	  // saturating the output
	 if(Output_temp > 6.0) Output_temp = 6.0;
	 if(Output_temp <-6.0) Output_temp = -6.0;
	 return Output_temp;
}

static float I_term_1 = 0;
static float I_term_2 = 0;

// calculate the duty based on the current speed and reference speed using a pi controller
float calculateControlSignal(float currentSpeed, float referenceSpeed, float* I_term){
	 // PI Controller
	 float error = (referenceSpeed - currentSpeed)*RPM2RADS; //rad/sec
	 float P_term = P_gain*error;
	 float Output_temp = 0;

	 *I_term = *I_term + I_gain*TS*error;
	 Output_temp = P_term+*I_term; // volt
   	 //float Output_temp = P_term+*I_term; // volt
	 float Output_clamped = clamp(Output_temp);

	 data.u1 = Output_temp;
   // Anti wind-up
	 bool is_saturated = (Output_clamped != Output_temp);
	 bool worsens_saturation = (error * Output_temp > 0);
	 if((is_saturated)&&(worsens_saturation)){
		 *I_term = *I_term - I_gain*TS*error;
	 }
	 data.u3 = *I_term;
   
	 return Output_clamped;
}

void ForwardCoast (int32_t motorNumber, int32_t duty){
	switch (motorNumber){
		case 1:
			if (duty >= 0) { // rotate forward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)duty);
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
			 } else { // rotate backward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)-duty);
			 }
			break;
		case 2:
			if (duty >= 0) { // rotate forward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)duty);
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
			 } else { // rotate backward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)-duty);
			 }
			break;
		}
}
void ForwardBreak (int32_t motorNumber, int32_t duty){
	switch (motorNumber){
		case 1:
			if (duty >= 0) { // rotate forward
			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)TIM8_ARR_VALUE);
			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)(TIM8_ARR_VALUE- duty));
			 } else { // rotate backward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)(TIM8_ARR_VALUE - (-duty)));
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)TIM8_ARR_VALUE);
			 }
			break;
		case 2:
			if (duty >= 0) { // rotate forward
			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)TIM8_ARR_VALUE);
			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)(TIM8_ARR_VALUE- duty));
			 } else { // rotate backward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)(TIM8_ARR_VALUE - (-duty)));
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)TIM8_ARR_VALUE);
			 }
			break;
		}
}

uint8_t LineSensorBinary[8];
void StoreBinary(uint8_t num) {
  for (int i = 7; i >= 0; i--) {
      printf("%d", (num >> i) & 1);  // Extract each bit
      LineSensorBinary[i]= (num >> i) & 1;
      if (i % 4 == 0) printf(" ");  // Add space for readability
  }
  printf("\n");
}

float CalculateEsl(){

	// Remove the impact of noise on the sensors
	for (int i = 0; i <= 7; i++) {
		if(LineSensorBinary[i] == 1 && i != 0 && i != 7){
			if(LineSensorBinary[i-1] != 1 && LineSensorBinary[i+1] != 1)
				LineSensorBinary[i] = 0;
		}
		else if(i == 0){
			if(LineSensorBinary[i+1] != 1)
				LineSensorBinary[i] = 0;
		}
		else if(i == 7){
			if(LineSensorBinary[i-1] != 1)
				LineSensorBinary[i] = 0;
		}
	}

	float LineSensorWn[8];
	//calculate Wn the distance between the sensor at position n and the center of the sensor array in meter
	for (int i = 0; i <= 7; i++)
	{
	// pitch is 8mm or 0.008m
	LineSensorWn[i] = ((float)(SensorsNumber - 1)/2.0f - (float)i) * PITCH;
	}

	float numerator = 0;
	float denominator = 0;
	// Calculate sum of elements of LineSensorWn and LineSensorBinary arrays
	for (int i = 0; i <= 7; i++)
	{
	numerator += (float) LineSensorBinary[i]*LineSensorWn[i];
	denominator += LineSensorBinary[i];
	}

	if (denominator == 0) return 0; // Handle the devide by zero exception
	float esl = (float) numerator / denominator;
	return (esl);
}

float CalculateYawError(float esl){
  // Distance between the line sensor and the vehicle’s center H = 0.085 m
    float yawError = atan2f(esl, H);
    //float yawErrorDegreee = yawError * (180.0f / M_PI);

  return yawError;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	static int kLed = 0;



	/* Speed ctrl routine */
	if(htim->Instance == TIM6)
	{
		uint8_t lineSensorData;
		if (HAL_I2C_Mem_Read(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_DATA_B, 1, &lineSensorData, 1, I2C_TIMEOUT) != HAL_OK) {
		  printf("Error reading REG_DATA_B\n");
		  return 0;
		}

		StoreBinary(lineSensorData);

		// Calculate esl
		float Esl = CalculateEsl();

    // Calculate yaw error
    float yawError = CalculateYawError(Esl); // in radian
    float yawKP = 5.0; // tune this based on the results

    float yawRate = yawKP * yawError;

    // adaptive base speed using yaw error
    float baseSpeed = 0.534; // in V/s (150 rpm)
    if (yawError < 0.087) // if the yaw error is less than 5 degree
    {
      baseSpeed = 0.534;
    }
    else 
    {
      baseSpeed = 0.534 *0.66;
    }
    
    float vr = baseSpeed + (yawRate * D / 2.0f); 
    float vl = baseSpeed - (yawRate * D / 2.0f);

    //float lineFollowingGain = 10000;

		float referenceSignal1 = vr / WHEEL_RADIUS / RPM2RADS; // in rpm
		float referenceSignal2 = vl / WHEEL_RADIUS / RPM2RADS; // in rpm

		// Read the speed from the motors
		float currentSpeed1 = ReadCurrentSpeed(1,&PreviousCount1);
		float currentSpeed2 = ReadCurrentSpeed(2,&PreviousCount2);

		float Output_1 = calculateControlSignal(currentSpeed1, referenceSignal1, &I_term_1);
		float Output_2 = calculateControlSignal(currentSpeed2, referenceSignal2, &I_term_2);

		// Set the correct PWM Value
		int32_t duty1;
		int32_t duty2;
    
		/* calculate duty properly */
		duty1 = Output_1*V2DUTY;
		duty2 = Output_2*V2DUTY;

		ForwardBreak(1, duty1);
		ForwardBreak(2, duty2);


		/*	Prepare data packet */
		data.w1 = currentSpeed1 ;
		data.w2 = currentSpeed2;

		//data.u1 = duty2; 		//Controller Output
		data.u2 = duty2;

		ertc_dlog_send(&logger, &data, sizeof(data));

		// Indicate that the program is running
		if(++kLed >= 10)
		{
			kLed = 0;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
	}
}

/* ---- USER CODE: 1 ---- */
  uint8_t data;
  HAL_StatusTypeDef status;
  /* ---- USER CODE: Init ---- */

  /* ---- USER CODE: SysInit ---- */

  /* ---- USER CODE: 2 ---- */

  logger.uart_handle = huart3; // for serial
  //logger.uart_handle = huart2; // for wifi

  /* Reset LCD */
  HAL_GPIO_WritePin(GPIO_OUT_SPI_CS_LCD_GPIO_Port, GPIO_OUT_SPI_CS_LCD_Pin, GPIO_PIN_SET);

  HAL_Delay(1000);

  /* Disable EXTI4_IRQ during SX1509 initialization */
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);

    ITM_SendChar('h');

    /* Software reset */
    data = 0x12;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    data = 0x34;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    HAL_Delay(100);

    /* Set KeyPad scanning engine */

    /* Set RegClock to 0x40 (enable internal oscillator; 2MHz freq) */
    data = 0x40;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_CLOCK, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set Bank A RegDir to 0xF0 (IO[0:3] as out) */
    data = 0xF0;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DIR_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set Bank B RegDir to 0x0F (IO[8:11] as in) */
    data = 0x0F;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DIR_B, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set Bank A RegOpenDrain to 0x0F (IO[0:3] as open-drain outputs) */
    data = 0x0F;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_OPEN_DRAIN_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set Bank B RegPullup to 0x0F (pull-ups enabled on inputs IO[8:11]) */
    data = 0x0F;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_PULL_UP_B, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set Bank B RegDebounceEnable to 0x0F (enable debouncing on IO[8:11]) */
    data = 0x0F;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DEBOUNCE_ENABLE_B, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegDebounceConfig to 0x05 (16ms debounce time) */
    data = 0x05;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DEBOUNCE_CONFIG, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegKeyConfig1 to 0x7D (8s auto-sleep; 32ms scan time per row) */
    data = 0x7D;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_CONFIG_1, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegKeyConfig2 to 0x1B (4 rows; 4 columns) */
    data = 0x1B;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_CONFIG_2, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Enable EXTI4_IRQ after SX1509 initialization */
    HAL_Delay(100);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    /* Disable EXTI2_IRQ during SX1509 initialization */
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);

  /* Software reset */
    data = 0x12;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    data = 0x34;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    HAL_Delay(100);

    /* Set RegDirA to 0xFF (all IO of Bank A configured as inputs) */
    data = 0xFF; // 0 = out; 1 = in
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_DIR_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegDirB to 0xFF (all IO of Bank B configured as inputs) */
    data = 0xFF; // 0 = out; 1 = in
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_DIR_B, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegInterruptMaskA to 0x00 (all IO of Bank A will trigger an interrupt) */
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_INTERRUPT_MASK_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegSenseHighA to 0xAA (IO[7:4] of Bank A will trigger an interrupt on falling edge) */
    data = 0xAA;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_SENSE_HIGH_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegSenseLowA to 0xAA (IO[3:0] of Bank A will trigger an interrupt on falling edge) */
    data = 0xAA;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_SENSE_LOW_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Enable EXTI2_IRQ after SX1509 initialization */
    HAL_Delay(100);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);




  /* Start encoders timers */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /* Start servomotors PWM (avoid floating inputs to servomotors) */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /* Start motor PWM */
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  /* Start speed ctrl ISR */
  HAL_TIM_Base_Start_IT(&htim6);

  /* ---- USER CODE: WHILE ---- */
  while (1)
  {
    /* ---- USER CODE: 3 ---- */
	  ertc_dlog_update(&logger);

  }
  /* ---- USER CODE: MX_GPIO_Init_1 ---- */
/* ---- USER CODE: MX_GPIO_Init_2 ---- */
/* ---- USER CODE: 4 ---- */

/* ---- USER CODE: Error_Handler_Debug ---- */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  static uint32_t kLed = 0;
	  if(++kLed >= 1000)
	  {
		  kLed = 0;
		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  }
  }
  /* ---- USER CODE: 6 ---- */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  