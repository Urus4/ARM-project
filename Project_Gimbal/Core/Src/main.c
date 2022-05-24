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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "Kalman.h"
#include "math.h"
#include "i2c-lcd.h"
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//=====================================================================================================================
//MPU6050 register value
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define RA_XA_OFFS_L_TC     0x07
#define RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define RA_YA_OFFS_L_TC     0x09
#define RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define RA_ZA_OFFS_L_TC     0x0B
#define RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define RA_XG_OFFS_USRL     0x14
#define RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define RA_YG_OFFS_USRL     0x16
#define RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define RA_ZG_OFFS_USRL     0x18
//=====================================================================================================================
//kalman Filter variable

#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.01745329251994329576923690768489
#define RESTRICT_PITCH

struct Kalman kalmanX, kalmanY, kalmanZ;
int accX, accY, accZ;
int gyroX, gyroY, gyroZ;
int magX, magY, magZ;
double roll, pitch, yaw;
double gyroXangle, gyroYangle, gyroZangle;
double compAngleX, compAngleY, compAngleZ;
double kalAngleX, kalAngleY, kalAngleZ;

void MPU6050_Init (void);
void updateMPU6050();
void  func(void);
void  updatePitchRoll(void);
void  updateYaw(void);
void  InitAll(void);
//=====================================================================================================================
//Time variable
uint32_t t_prev, t_now, time_count;
double t_period,t_dt;
int duration = 300;
int pre_time = 0, cur_time;


int read_us();
void init_time( );
void calc_time();
//=====================================================================================================================
//Step motor variable
#define StepPerRev 4096

uint32_t value[2];
int8_t cur_pitch_angle = 0;
int8_t pre_pitch_angle = 0;
int8_t cur_roll_angle = 0;
int8_t pre_roll_angle = 0;
int state = 0;

uint8_t step_count_pitch_L = 0;
uint8_t step_count_pitch_H = 0;
uint8_t step_count_roll_L = 0;
uint8_t step_count_roll_H = 0;

void stepper_step_angle (double angle, int direction,int rpm,GPIO_TypeDef  *GPIOx, uint16_t num1, uint16_t num2, uint16_t num3, uint16_t num4);
void stepper_half_drive(int step,GPIO_TypeDef  *GPIOx, uint16_t num1, uint16_t num2, uint16_t num3, uint16_t num4);
void delay(uint16_t us);
void Stepper_set_rpm(int rpm);
//=====================================================================================================================
// LCD
char joy_roll_value [30];
char joy_pitch_value [30];
char gyro_roll_value [30];
char gyro_pitch_value [30];
//=====================================================================================================================
//Get Gyro,Accel value

 void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;
	uint8_t offset;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);


	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{

		printf(" Connect Success\n ");
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> �????????????????????????? 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> �????????????????????????? 250 �?????????????????????????/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1 , &Data, 1, 1000);


		offset = 0x02;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_XA_OFFS_H, 1, &offset, 1, 1000);
		offset = 0x32;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_XA_OFFS_L_TC, 1, &offset, 1, 1000);


		offset = 0XFB;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_YA_OFFS_H, 1, &offset, 1, 1000);
		offset = 0xC8;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_YA_OFFS_L_TC, 1, &offset, 1, 1000);


		offset = 0x08;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_ZA_OFFS_H, 1, &offset, 1, 1000);
		offset = 0xA1;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_ZA_OFFS_L_TC, 1, &offset, 1, 1000);


		offset = 0xFF;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_XG_OFFS_USRH, 1, &offset, 1, 1000);
		offset = 0x6B;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_XG_OFFS_USRL, 1, &offset, 1, 1000);


		offset = 0x01;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_YG_OFFS_USRH, 1, &offset, 1, 1000);
		offset = 0x19;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_YG_OFFS_USRL, 1, &offset, 1, 1000);

		offset = 0xFF;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_ZG_OFFS_USRH, 1, &offset, 1, 1000);
		offset = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RA_ZG_OFFS_USRL, 1, &offset, 1, 1000);

	}else
	printf(" Connect Fail\n ");


}



 void InitAll()
 {
     /* Set Kalman and gyro starting angle */
     updateMPU6050();
     updatePitchRoll();

     setAngle(&kalmanX,roll); // First set roll starting angle
     gyroXangle = roll;
     compAngleX = roll;

     setAngle(&kalmanY,pitch); // Then pitch
     gyroYangle = pitch;
     compAngleY = pitch;

     setAngle(&kalmanZ,yaw); // And finally yaw
     gyroZangle = yaw;
     compAngleZ = yaw;

 }



void updateMPU6050()
 {
	uint8_t Rec_Data1[6];
	uint8_t Rec_Data2[6];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data1, 6, 1000);

	gyroX = -(int16_t)(Rec_Data1[0] << 8 | Rec_Data1 [1]);
	gyroY = (int16_t)(Rec_Data1[2] << 8 | Rec_Data1 [3]);
	gyroZ = -(int16_t)(Rec_Data1[4] << 8 | Rec_Data1 [5]);



	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data2, 6, 1000);

	accX = (int16_t)(Rec_Data2[0] << 8 | Rec_Data2 [1]);
	accY = -(int16_t)(Rec_Data2[2] << 8 | Rec_Data2 [3]);
	accZ = (int16_t)(Rec_Data2[4] << 8 | Rec_Data2 [5]);

 }


void updatePitchRoll()
{

    #ifdef RESTRICT_PITCH
    roll = atan(accY/sqrt(accX*accX+accZ*accZ)) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    #else // Eq. 28 and 29
    roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    #endif
}
//=====================================================================================================================
//Step motor
void Stepper_set_rpm(int rpm)
{
	delay(60000000/StepPerRev/rpm);
}

void stepper_half_drive(int step, GPIO_TypeDef  *GPIOx, uint16_t num1, uint16_t num2, uint16_t num3, uint16_t num4)
{
	switch(step){
	case 0:
		HAL_GPIO_WritePin(GPIOx, num1, GPIO_PIN_SET);      //IN1
		HAL_GPIO_WritePin(GPIOx, num2, GPIO_PIN_RESET);    //IN2
		HAL_GPIO_WritePin(GPIOx, num3, GPIO_PIN_RESET);    //IN3
		HAL_GPIO_WritePin(GPIOx, num4, GPIO_PIN_RESET);    //IN4
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOx, num1, GPIO_PIN_SET);      //IN1
		HAL_GPIO_WritePin(GPIOx, num2, GPIO_PIN_SET);      //IN2
		HAL_GPIO_WritePin(GPIOx, num3, GPIO_PIN_RESET);    //IN3
		HAL_GPIO_WritePin(GPIOx, num4, GPIO_PIN_RESET);    //IN4
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOx, num1, GPIO_PIN_RESET);    //IN1
		HAL_GPIO_WritePin(GPIOx, num2, GPIO_PIN_SET);      //IN2
		HAL_GPIO_WritePin(GPIOx, num3, GPIO_PIN_RESET);    //IN3
		HAL_GPIO_WritePin(GPIOx, num4, GPIO_PIN_RESET);    //IN4
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOx, num1, GPIO_PIN_RESET);    //IN1
		HAL_GPIO_WritePin(GPIOx, num2, GPIO_PIN_SET);      //IN2
		HAL_GPIO_WritePin(GPIOx, num3, GPIO_PIN_SET);      //IN3
		HAL_GPIO_WritePin(GPIOx, num4, GPIO_PIN_RESET);    //IN4
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOx, num1, GPIO_PIN_RESET);    //IN1
		HAL_GPIO_WritePin(GPIOx, num2, GPIO_PIN_RESET);    //IN2
		HAL_GPIO_WritePin(GPIOx, num3, GPIO_PIN_SET);      //IN3
		HAL_GPIO_WritePin(GPIOx, num4, GPIO_PIN_RESET);    //IN4
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOx, num1, GPIO_PIN_RESET);    //IN1
		HAL_GPIO_WritePin(GPIOx, num2, GPIO_PIN_RESET);    //IN2
		HAL_GPIO_WritePin(GPIOx, num3, GPIO_PIN_SET);      //IN3
		HAL_GPIO_WritePin(GPIOx, num4, GPIO_PIN_SET);      //IN4
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOx, num1, GPIO_PIN_RESET);    //IN1
		HAL_GPIO_WritePin(GPIOx, num2, GPIO_PIN_RESET);    //IN2
		HAL_GPIO_WritePin(GPIOx, num3, GPIO_PIN_RESET);    //IN3
		HAL_GPIO_WritePin(GPIOx, num4, GPIO_PIN_SET);      //IN4
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOx, num1, GPIO_PIN_SET);      //IN1
		HAL_GPIO_WritePin(GPIOx, num2, GPIO_PIN_RESET);    //IN2
		HAL_GPIO_WritePin(GPIOx, num3, GPIO_PIN_RESET);    //IN3
		HAL_GPIO_WritePin(GPIOx, num4, GPIO_PIN_SET);      //IN4
		break;
	}
}

void stepper_step_angle (double angle, int direction,int rpm, GPIO_TypeDef  *GPIOx, uint16_t num1, uint16_t num2, uint16_t num3, uint16_t num4)
{
	float anglepersequence = 0.703125;
	int numberofsequences = (int) (angle/anglepersequence);

	for(int seq = 0;seq<numberofsequences;seq++)
	{
		if(direction == 0)
		{
			for(int step = 7;step>=0;step--)
			{
				stepper_half_drive(step,GPIOx, num1, num2, num3, num4);
				Stepper_set_rpm(rpm);
			}
		}
		else if(direction == 1)
		{
			for(int step = 0;step<8;step++)
			{
				stepper_half_drive(step,GPIOx, num1, num2, num3, num4);
				Stepper_set_rpm(rpm);
			}
		}
	}
}

//=====================================================================================================================
// Time
int read_us()
{
	int cur_time = (time_count*10000) + __HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3,0);

	return cur_time;
}
void init_time( ) {

	HAL_TIM_Base_Start_IT(&htim3);
    t_prev = read_us();
}
void calc_time() 						// Kalman calc
{
	t_now = read_us();
	t_period = (t_now - t_prev)/1000000.0;
	t_prev = t_now;
	t_dt = t_period;
}
void delay(uint16_t us)						// Stepmotor RPM calc
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<us);
}
//=====================================================================================================================
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	char buf[4];
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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, value, 2);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  MPU6050_Init();
  lcd_init();
  InitAll();
  init_time();
  Init(&kalmanX);
  Init(&kalmanY);
  Init(&kalmanZ);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  func();
	  printf("kalAngleX : %.2lf    kalAngleY : %.2lf\n", kalAngleX,roll);
//==================================================================================================================
// gyro mode - step control
	  if( state == 1)
	  {
		  //Gyro step motor control
		  pre_pitch_angle = cur_pitch_angle;
		  pre_roll_angle = cur_roll_angle;
		  cur_pitch_angle = kalAngleX;
		  cur_roll_angle = kalAngleY;
		  if(cur_pitch_angle > pre_pitch_angle || cur_roll_angle > pre_roll_angle)
		  {
			stepper_step_angle(cur_pitch_angle - pre_pitch_angle , 1,13, GPIOC, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
			stepper_step_angle(cur_roll_angle - pre_roll_angle , 1,13, GPIOB, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15,GPIO_PIN_1);

		  }
		  if(cur_pitch_angle < pre_pitch_angle || cur_roll_angle < pre_roll_angle)
		  {
			stepper_step_angle(pre_pitch_angle - cur_pitch_angle, 0, 13, GPIOC, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
			stepper_step_angle(pre_roll_angle - cur_roll_angle, 0,13, GPIOB, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_1);

		  }

//		  printf("kalAngleX : %.2lf    kalAngleY : %.2lf\n", kalAngleX,kalAngleY);
//=====================================================================================================================
// gyro mode - LCD
		  lcd_clear();
		  lcd_put_cur(0, 0);
		  lcd_send_string("Gyro mode");
		  lcd_put_cur(1, 0);
		  sprintf(gyro_roll_value,"r=%.1lf", kalAngleX);
		  lcd_send_string(gyro_roll_value);
		  lcd_put_cur(1, 9);
		  sprintf(gyro_pitch_value,"p=%.1lf", kalAngleY);
		  lcd_send_string(gyro_pitch_value);
	  }
//=====================================================================================================================
// joystick mode - step control
	  else if(state == 0)
	  {
		  //Joystic step motor control
		  while(value[1] == 4095)
		  {
			  if(step_count_roll_L > 0)
			  {
				  stepper_step_angle(1, 1, 13, GPIOC, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
				  step_count_roll_L--;
			  }
			  else if (step_count_roll_H < 90 )
			  {
				  step_count_roll_H++;
				  stepper_step_angle(1, 1, 13, GPIOC, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
			  }
		  }
		  while(value[1] < 1000)
		  {
			  if(step_count_roll_H > 0)
			  {
				  stepper_step_angle(1, 0, 13, GPIOC, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
				  step_count_roll_H--;
			  }
			  else if(step_count_roll_L < 90 ){
				  step_count_roll_L++ ;
				  stepper_step_angle(1, 0, 13, GPIOC, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
			  }
		  }

		  while(value[0] == 4095)
		  {
				if(step_count_pitch_L > 0)
				{
					stepper_step_angle(1, 1,13, GPIOB, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_1);
					step_count_pitch_L--;
				}
				else if (step_count_pitch_H < 90 )
				{

					step_count_pitch_H++ ;
					stepper_step_angle(1, 1,13, GPIOB, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_1);
				}
		  }
		  while(value[0] < 1000)
		  {
				if(step_count_pitch_H > 0)
				{
					stepper_step_angle(1, 0,13, GPIOB, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_1);
					step_count_pitch_H--;
				}
				else if (step_count_pitch_L < 90 )
				{

					step_count_pitch_L++ ;
					stepper_step_angle(1, 0,13, GPIOB, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_1);

				}
		  }
//=============================================================================================
// joystick - LCD
		  lcd_clear();
		  lcd_put_cur(0, 0);
		  lcd_send_string("Joystick mode");
		  if(step_count_pitch_H == 0)
		  {
			  sprintf(joy_pitch_value,"p:-%d", step_count_pitch_L);
		  }
		  else if(step_count_pitch_L == 0)
		  {
			  sprintf(joy_pitch_value,"p:+%d", step_count_pitch_H);
		  }
		  lcd_put_cur(1, 0);
		  lcd_send_string(joy_pitch_value);

		  if(step_count_roll_H == 0)
		  {
			  sprintf(joy_roll_value,"r :-%d", step_count_roll_L);
		  }
		  else if(step_count_roll_L == 0)
		  {
			  sprintf(joy_roll_value,"r :+%d", step_count_roll_H);
		  }
		  lcd_put_cur(1, 9);
		  lcd_send_string(joy_roll_value);

	  }

//=============================================================================================
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//=====================================================================================================================
// Get kalman value
void func()
{
    double gyroXrate,gyroYrate,dt;
    /* Update all the IMU values */
    updateMPU6050();

    dt = 0.03;


    /* Roll and pitch estimation */
    updatePitchRoll();
    gyroXrate = gyroX / 131.0;
    gyroYrate = gyroY / 131.0;

    #ifdef RESTRICT_PITCH

    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        setAngle(&kalmanX,roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
    kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); 				// Calculate the angle using a Kalman filter

    if (fabs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; 										// Invert rate, so it fits the restricted accelerometer reading
    kalAngleY = getAngle(&kalmanY,pitch, gyroYrate, dt);

    #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    } else
    kalAngleY = getAngle(&kalmanY, pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
    kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    #endif

    /* Estimate angles using gyro only */
//    gyroXangle += gyroXrate * dt;				 // Calculate gyro angle without any filter
//    gyroYangle += gyroYrate * dt;
//    gyroZangle += gyroZrate * dt;
//    gyroXangle += kalmanX.getRate() * dt;		 // Calculate gyro angle using the unbiased rate from the Kalman filter
//    gyroYangle += kalmanY.getRate() * dt;
//    gyroZangle += kalmanZ.getRate() * dt;

    /* Estimate angles using complimentary filter */
//    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; 			// Calculate the angle using a Complimentary filter
//    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
//    compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

    /* Reset the gyro angles when they has drifted too much*/
//    if (gyroXangle < -180 || gyroXangle > 180)
//        gyroXangle = kalAngleX;
//    if (gyroYangle < -180 || gyroYangle > 180)
//        gyroYangle = kalAngleY;
//    if (gyroZangle < -180 || gyroZangle > 180)
//        gyroZangle = kalAngleZ;

}

//=====================================================================================================================
// Joystick control On/Off
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {
	if(GPIO_Pin == GPIO_PIN_7)
		{
		cur_time = __HAL_TIM_GET_COUNTER(&htim2);
		if(cur_time - pre_time >= duration)
		{
			if(state == 0)
			{

				if(step_count_pitch_H > step_count_pitch_L)
				{
					for(int i = 0; i<step_count_pitch_H +10  ; i++)
						{
							stepper_step_angle(1, 0,13, GPIOB, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_1);
						}
				}
				else
				{
					for(int i = 0; i <step_count_pitch_L+10;i++)
					{
						stepper_step_angle(1, 1,13, GPIOB, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_1);
					}
				}

				if(step_count_roll_H > step_count_roll_L)
				{

					for(int i = 0; i<step_count_roll_H; i++)
					{
						stepper_step_angle(1, 0, 13, GPIOC, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
					}
				}
				else
				{
					for(int i = 0 ; i<step_count_roll_L;i++)
					{
						stepper_step_angle(1, 1, 13, GPIOC, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
					}

				}

				step_count_pitch_H = 0;
				step_count_pitch_L = 0;
				step_count_roll_H = 0;
				step_count_roll_L = 0;
				state = 1;
			}
			else if(state == 1)
			{

				if(kalAngleX > 0)
				{
					step_count_roll_H = kalAngleX;
				}
				else if(kalAngleX < 0)
				{
					step_count_roll_L = kalAngleX *(-1);
				}
				if(kalAngleY > 0)
				{
					step_count_pitch_H = kalAngleY;
				}
				else if(kalAngleY < 0)
				{
					step_count_pitch_L = kalAngleY * (-1);
				}
				state = 0;
			}

			__HAL_TIM_SET_COUNTER(&htim2, 0);
		}
	}
  }


//=====================================================================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // Kalman calc
{
	time_count++;
}
//=====================================================================================================================

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
