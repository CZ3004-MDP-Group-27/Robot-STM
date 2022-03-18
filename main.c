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
#include "ICM20948.h"
#include <stdlib.h>

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for showTask */
osThreadId_t showTaskHandle;
const osThreadAttr_t showTask_attributes = {
  .name = "showTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ultraTask */
osThreadId_t ultraTaskHandle;
const osThreadAttr_t ultraTask_attributes = {
  .name = "ultraTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for icm20948_task */
osThreadId_t icm20948_taskHandle;
const osThreadAttr_t icm20948_task_attributes = {
  .name = "icm20948_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);
void motor(void *argument);
void encoder(void *argument);
void ultra(void *argument);
void uart(void *argument);
void icm20948(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//initializing global variables
uint8_t aRxBuffer[50]; //for taking in stuff from rpi thru UART

double pwmVal_L = 2000; // pwm values
double pwmVal_R = 2000;
#define DEFAULTPWM 5000
uint8_t dir = 1; // controls dir of wheels, 1 for front, 0 for back
uint8_t servoVal = 150; //set servo dir
double duration=0; //duration in ms
double reqduration=0, curduration=0;
uint8_t distprof = 0;
uint8_t servodefault = 150;
uint8_t correctionleft = 147, correctionright = 152;

//Ultrasonic
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

//For Profile Switching
uint8_t userBtnCount = 0;
uint8_t enableSw = 0;

//For Gyro
/* define ICM-20948 Device I2C address*/
#define I2C_ADD_ICM20948            0xD0
#define I2C_ADD_ICM20948_AK09916    0x0C
#define I2C_ADD_ICM20948_AK09916_READ  0x80
#define I2C_ADD_ICM20948_AK09916_WRITE 0x00
/* define ICM-20948 Register */
/* user bank 0 register */
#define REG_ADD_WIA             0x00
#define REG_VAL_WIA             0xEA
#define REG_ADD_USER_CTRL       0x03
#define REG_VAL_BIT_DMP_EN          0x80
#define REG_VAL_BIT_FIFO_EN         0x40
#define REG_VAL_BIT_I2C_MST_EN      0x20
#define REG_VAL_BIT_I2C_IF_DIS      0x10
#define REG_VAL_BIT_DMP_RST         0x08
#define REG_VAL_BIT_DIAMOND_DMP_RST 0x04
#define REG_ADD_PWR_MIGMT_1     0x06
#define REG_VAL_ALL_RGE_RESET   0x01
#define REG_VAL_DEVICE_RESET    0x41	//self added
#define REG_VAL_RUN_MODE        0x01    //Non low-power mode
#define REG_ADD_LP_CONFIG       0x05
#define REG_ADD_PWR_MGMT_1      0x06
#define REG_ADD_PWR_MGMT_2      0x07
#define REG_VAL_DISABLE_GYRO    0x07
#define REG_VAL_ENABLE_ALL      0x00
#define REG_ADD_ACCEL_XOUT_H    0x2D
#define REG_ADD_ACCEL_XOUT_L    0x2E
#define REG_ADD_ACCEL_YOUT_H    0x2F
#define REG_ADD_ACCEL_YOUT_L    0x30
#define REG_ADD_ACCEL_ZOUT_H    0x31
#define REG_ADD_ACCEL_ZOUT_L    0x32
#define REG_ADD_GYRO_XOUT_H     0x33
#define REG_ADD_GYRO_XOUT_L     0x34
#define REG_ADD_GYRO_YOUT_H     0x35
#define REG_ADD_GYRO_YOUT_L     0x36
#define REG_ADD_GYRO_ZOUT_H     0x37
#define REG_ADD_GYRO_ZOUT_L     0x38
#define REG_ADD_EXT_SENS_DATA_00 0x3B
#define REG_ADD_REG_BANK_SEL    0x7F
#define REG_VAL_REG_BANK_0  0x00
#define REG_VAL_REG_BANK_1  0x10
#define REG_VAL_REG_BANK_2  0x20
#define REG_VAL_REG_BANK_3  0x30

/* user bank 1 register */
/* user bank 2 register */
#define REG_ADD_GYRO_SMPLRT_DIV 0x00
#define REG_ADD_GYRO_CONFIG_1   0x01
#define REG_VAL_BIT_GYRO_DLPCFG_2   0x10 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_4   0x20 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_6   0x30 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_FS_250DPS  0x00 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_500DPS  0x02 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_1000DPS 0x04 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_2000DPS 0x06 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_DLPF       0x01 /* bit[0]   */
#define REG_ADD_ACCEL_SMPLRT_DIV_2  0x11
#define REG_ADD_ACCEL_CONFIG        0x14
#define REG_VAL_BIT_ACCEL_DLPCFG_2  0x10 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_4  0x20 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_6  0x30 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_FS_2g     0x00 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_4g     0x02 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_8g     0x04 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_16g    0x06 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_DLPF      0x01 /* bit[0]   */

/* user bank 3 register */
#define REG_ADD_I2C_SLV0_ADDR   0x03
#define REG_ADD_I2C_SLV0_REG    0x04
#define REG_ADD_I2C_SLV0_CTRL   0x05
#define REG_VAL_BIT_SLV0_EN     0x80
#define REG_VAL_BIT_MASK_LEN    0x07
#define REG_ADD_I2C_SLV0_DO     0x06
#define REG_ADD_I2C_SLV1_ADDR   0x07
#define REG_ADD_I2C_SLV1_REG    0x08
#define REG_ADD_I2C_SLV1_CTRL   0x09
#define REG_ADD_I2C_SLV1_DO     0x0A

/* define ICM-20948 Register  end */

/* define ICM-20948 MAG Register  */
#define REG_ADD_MAG_WIA1    0x00
#define REG_VAL_MAG_WIA1    0x48
#define REG_ADD_MAG_WIA2    0x01
#define REG_VAL_MAG_WIA2    0x09
#define REG_ADD_MAG_ST2     0x10
#define REG_ADD_MAG_DATA    0x11
#define REG_ADD_MAG_CNTL2   0x31
#define REG_VAL_MAG_MODE_PD     0x00
#define REG_VAL_MAG_MODE_SM     0x01
#define REG_VAL_MAG_MODE_10HZ   0x02
#define REG_VAL_MAG_MODE_20HZ   0x04
#define REG_VAL_MAG_MODE_50HZ   0x05
#define REG_VAL_MAG_MODE_100HZ  0x08
#define REG_VAL_MAG_MODE_ST     0x10
/* define ICM-20948 MAG Register  end */

typedef struct icm20948_st_sensor_data_tag
{
	int16_t s16X;
	int16_t s16Y;
	int16_t s16Z;
}ICM20948_ST_SENSOR_DATA;

typedef struct icm20948_st_avg_data_tag
{
	uint8_t u8Index;
	int16_t s16AvgBuffer[8];
}ICM20948_ST_AVG_DATA;

typedef struct imu_st_sensor_data_tag
{
	int16_t s16X;
	int16_t s16Y;
	int16_t s16Z;
}IMU_ST_SENSOR_DATA;
#define MAG_DATA_LEN    6
#define CONTROL_DELAY		1000

/* ICM20948 Variables --------------------------------------------------------*/
uint8_t icmData = 0x1;
uint8_t icmTempMsg[16] = {};
ICM20948_ST_SENSOR_DATA gstGyroOffset ={0,0,0};
IMU_ST_SENSOR_DATA gstMagOffset = {0,0,0};
int16_t TestmagnBuff[9]={0};
int16_t magn[3];
short Deviation_gyro[3],Original_gyro[3];
short s16Gyro[3], s16Accel[3], s16Magn[3];
int16_t gyro[3], accel[3],magnet[3];
uint16_t Deviation_Count = 0;
uint64_t gyrosum=0, gyrozero=0;
int64_t gyrosumsigned=0;

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
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

  HAL_UART_Transmit_IT(&huart3, (uint8_t *) aRxBuffer, 50);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of showTask */
  showTaskHandle = osThreadNew(show, NULL, &showTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(motor, NULL, &motorTask_attributes);

  /* creation of encoderTask */
  encoderTaskHandle = osThreadNew(encoder, NULL, &encoderTask_attributes);

  /* creation of ultraTask */
  ultraTaskHandle = osThreadNew(ultra, NULL, &ultraTask_attributes);

  /* creation of uartTask */
  uartTaskHandle = osThreadNew(uart, NULL, &uartTask_attributes);

  /* creation of icm20948_task */
  icm20948_taskHandle = osThreadNew(icm20948, NULL, &icm20948_task_attributes);

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
  htim2.Init.Period = 65535;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
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

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_SW_Pin */
  GPIO_InitStruct.Pin = ENABLE_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENABLE_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//User functions
void userFunctions(){}
void realignWheels(){
	htim1.Instance->CCR4 = 170; //right
	osDelay(400);
	htim1.Instance->CCR4 = 147; //left
	osDelay(300);
//	htim1.Instance->CCR4 = ; //center
}

//void motorActivate(double pwmVal_L, double pwmVal_R, uint8_t dir, float duration, uint8_t servoVal){
void motorActivate(){
//	duration = 2363; //1m
	if(dir == 1){
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	}
	htim1.Instance->CCR4 = servoVal;
//	osDelay(400);

	uint32_t tick = HAL_GetTick();

	double pwml = pwmVal_L;
	double pwmr = pwmVal_R;

	while((HAL_GetTick()-tick)<=(duration)){
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwml);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmr);
	}

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,0);
}

void straight(double local_pwmVal_L, double local_pwmVal_R, uint8_t local_dir, double distance){
	pwmVal_L = local_pwmVal_L; // pwm values
	pwmVal_R = local_pwmVal_R;
	dir = local_dir;
	servoVal = 150; //set servo dir
	duration = distance * 30 *0.415;
	motorActivate();
}

void gyrostraight(double local_pwmVal_L, double local_pwmVal_R, uint8_t local_dir, double distance){
	pwmVal_L = local_pwmVal_L; // pwm values
	pwmVal_R = local_pwmVal_R;
	dir = local_dir;
	reqduration = distance * 30 *0.415;
	curduration = 0;
	gyrosumsigned = 0;
	servodefault = 150;
	while(curduration<=(reqduration)*0.8){
		servoVal = servodefault;
		if(dir==1){
			if(gyrosumsigned>500){ //going left
				//servoVal = correctionright;
				servodefault+=1;
				duration = 30;
				motorActivate();
				gyrosumsigned=0;
			}else if(gyrosumsigned<-500){ //going right
				//servoVal = correctionright;
				servodefault-=1;
				duration = 30;
				motorActivate();
				gyrosumsigned=0;
			}else{
				duration = 30;
				motorActivate();
			}
		}else{
			if(gyrosumsigned>500){ //going left
				//servoVal = correctionright;
				servodefault-=1;
				duration = 30;
				motorActivate();
				gyrosumsigned=0;
			}else if(gyrosumsigned<-500){ //going right
				//servoVal = correctionright;
				servodefault+=1;
				duration = 30;
				motorActivate();
				gyrosumsigned=0;
			}else{
				duration = 30;
				motorActivate();
			}
		}

		sprintf(icmTempMsg,"G:%+07d,",gyro[2]);
		HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);

		curduration+=30;
	}
	while(curduration<=reqduration){
		servoVal = 150;
		pwmVal_L = pwmVal_R = 2000;
				duration = 30;
				motorActivate();


		sprintf(icmTempMsg,"G:%+07d,",gyro[2]);
		HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);

		curduration+=30;
	}
}

void turn(uint8_t local_dir, int leftright, int angle){
	dir = local_dir;
	if(distprof==1){ //red tiles/////////////////////////////////////////////////////////////
		if(dir==1){ //forward
			if(leftright == 0){//left
				if(angle==15) duration = 127;
				if(angle==45) duration = 399;
				//        if(angle==15) duration = 105;
				if(angle == 90)  {//making the forward turning cover 40cm by 25cm
					straight(DEFAULTPWM,DEFAULTPWM,1,0.7); //move forward by 0.7cm
					//duration = 815; //for actual red tiles
//					duration = 805;
					duration = 790;

				}
					pwmVal_L = 2400; // pwm values
					pwmVal_R = 4800;
					servoVal = 112; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,1,0.6); //move forward by 0.6cm
					straight(DEFAULTPWM,DEFAULTPWM,1,2.1); //move forward by 2.1cm
				}
			}else{//right
				//duration = 7.604762*angle + 40;
				if(angle==15) duration = 110;
				if(angle==45) duration = 355;
				//        if(angle==15) duration = 101;
				if(angle == 90)  {//making the forward turning cover 40cm by 25cm
					straight(DEFAULTPWM,DEFAULTPWM,1,3.1);//move forward by 3.1cm
					//duration =720;//lab
//					duration =715;//red tiles
					duration = 700;
				}
					pwmVal_L = 4800; // pwm values
					pwmVal_R = 2400;
					servoVal = 230; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,1,4.5); //move forward by 4.5cm
					straight(DEFAULTPWM,DEFAULTPWM,1,2); //move forward by 2cm
				}
			}
		} else {//backward
			if(leftright == 0){//left
				if(angle==15) duration = 127;
				if(angle==45) duration = 378.5;
				if(angle == 90)  {//making the forward turning cover 25cm by 40cm
					straight(DEFAULTPWM,DEFAULTPWM,0,1.5);//move back by 1.5 cm
					duration =757; //duration =750;
				}
					pwmVal_L = 2400; // pwm values
					pwmVal_R = 4800;
					servoVal = 112; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,0,4.2); //move backwards by 4.2cm
					straight(DEFAULTPWM,DEFAULTPWM,0,4.7); //move backwards by 4.2cm
				}
			}else{//right
				if(angle==15) duration = 127;
				if(angle==45) duration = 377.5;
				if(angle == 90)  {
					//straight(DEFAULTPWM,DEFAULTPWM,0,0);  //move back by 0.8cm
					duration =755;
				}
					pwmVal_L = 4800; // pwm values
					pwmVal_R = 2400;
					servoVal = 230; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
					straight(DEFAULTPWM,DEFAULTPWM,0,3.1); //move back by 4.6cm
				}
			}
		}
	}//end red tiles
	else if(distprof==2){ //lab floor/////////////////////////////////////////////////////////////
		if(dir==1){ //forward
			if(leftright == 0){//left
				if(angle==15) duration = 140;
				if(angle==45) duration = 420;
				//        if(angle==15) duration = 105;
				if(angle == 90)  {//making the forward turning cover 40cm by 25cm
					straight(DEFAULTPWM,DEFAULTPWM,1,0.7); //move forward by 0.7cm
					duration = 840;
				}
					pwmVal_L = 2400; // pwm values
					pwmVal_R = 4800;
					servoVal = 112; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,1,0.6); //move forward by 0.6cm
					straight(DEFAULTPWM,DEFAULTPWM,1,2.1); //move forward by 2.1cm
				}
			}else{//right
				//duration = 7.604762*angle + 40;
				if(angle==15) duration = 129;
				if(angle==45) duration = 385;
				//        if(angle==15) duration = 101;
				if(angle == 90)  {//making the forward turning cover 40cm by 25cm
					straight(DEFAULTPWM,DEFAULTPWM,1,3.1);//move forward by 3.1cm
					duration =770;//red tiles
				}
					pwmVal_L = 4800; // pwm values
					pwmVal_R = 2400;
					servoVal = 230; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,1,4.5); //move forward by 4.5cm
					straight(DEFAULTPWM,DEFAULTPWM,1,2); //move forward by 2cm
				}
			}
		} else {//backwards
			if(leftright == 0){//left
				if(angle==15) duration = 137;
				if(angle==45) duration = 410;
				if(angle == 90)  {//making the forward turning cover 25cm by 40cm
					straight(DEFAULTPWM,DEFAULTPWM,0,1.5);//move back by 1.5 cm
					duration =820; //duration =750;
				}
					pwmVal_L = 2400; // pwm values
					pwmVal_R = 4800;
					servoVal = 112; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,0,4.2); //move backwards by 4.2cm
					straight(DEFAULTPWM,DEFAULTPWM,0,4.7); //move backwards by 4.2cm
				}
			}else{//right
				if(angle==15) duration = 139;
				if(angle==45) duration = 420;
				if(angle == 90)  {
					//straight(DEFAULTPWM,DEFAULTPWM,0,0);  //move back by 0.8cm
					duration =840;
				}
					pwmVal_L = 4800; // pwm values
					pwmVal_R = 2400;
					servoVal = 230; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
					straight(DEFAULTPWM,DEFAULTPWM,0,3.1); //move back by 4.6cm
				}
			}
		}
	}//end lab


}

void gyroturn(uint8_t local_dir, int leftright, int angle){ //23,580 for 90 degrees
	gyrosum = 0;
	dir = local_dir;
	if(leftright==0) htim1.Instance->CCR4 = 105;
	else htim1.Instance->CCR4 = 210;
	osDelay(300);
	while (gyrosum<=(angle*275*4*2*0.7)){
			if(leftright==0){//left
				duration = 10;
				pwmVal_L = 1200; // pwm values
				pwmVal_R = 2400;
				servoVal = 105; //set servo dir
				motorActivate();
//				sprintf(icmTempMsg,"%+09d,",gyrosum);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);
			}else{//right
				duration = 10;
				pwmVal_L = 2400; // pwm values
				pwmVal_R = 1200;
				servoVal = 210; //set servo dir
				motorActivate();
//				sprintf(icmTempMsg,"%+09d,",gyrosum);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);
			}
	}
	while (gyrosum<=(angle*275*4*2)){ //last 30%
			if(leftright==0){//left
				duration = 10;
				pwmVal_L = 600; // pwm values
				pwmVal_R = 1200;
				servoVal = 105; //set servo dir
				motorActivate();
//				sprintf(icmTempMsg,"%+09d,",gyrosum);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);
			}else{//right
				duration = 10;
				pwmVal_L = 1200; // pwm values
				pwmVal_R = 600;
				servoVal = 210; //set servo dir
				motorActivate();
//				sprintf(icmTempMsg,"%+09d,",gyrosum);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);
			}
	}
	realignWheels();
}

void tpturn(int leftright){
	//three 30 turns
	int dur;
	//switch(angle){

	if(leftright==1){
		turn(1,1,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 16.12);

		turn(1,1,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 23.442);

		turn(1,1,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 9);
	}else{
		turn(1,0,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 16.12);

		turn(1,0,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 23.442);

		turn(1,0,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 9);
	}

}

void ipt90(int leftright){
	if (leftright ==0){//left
//		turn(1,0,15);
//		turn(0,1,15);
//		turn(1,0,15);
//		turn(0,1,15);
//		turn(1,0,15);
//		turn(0,1,15);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 2.3+1);
		gyroturn(1,0,40);
		gyroturn(0,1,40);
		straight(DEFAULTPWM, DEFAULTPWM, 1, 4.7+1);
	}else{
//		turn(1,1,15);
//		turn(0,0,15);
//		turn(1,1,15);
//		turn(0,0,15);
//		turn(1,1,15);
//		turn(0,0,15);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 4.6+1);
		gyroturn(1,1,40);
		gyroturn(0,0,40);
		straight(DEFAULTPWM, DEFAULTPWM, 1, 0.7+1);
	}
}


void ICMWriteOneByte(uint8_t RegAddr, uint8_t Data)
{
	HAL_I2C_Mem_Write(&hi2c1, I2C_ADD_ICM20948, RegAddr, I2C_MEMADD_SIZE_8BIT, &Data, 1, 0xffff);
}

uint8_t ICMReadOneByte(uint8_t RegAddr)
{
	uint8_t TempVal = 0;
	HAL_I2C_Mem_Read(&hi2c1, I2C_ADD_ICM20948, RegAddr, I2C_MEMADD_SIZE_8BIT, &TempVal, 1, 0xffff);
	return TempVal;
}

void ICMReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data)
{
    uint8_t i;
    uint8_t u8Temp;

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    ICMWriteOneByte(REG_ADD_I2C_SLV0_ADDR, u8I2CAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV0_REG,  u8RegAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|u8Len);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    u8Temp = ICMReadOneByte(REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);
    osDelay(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);

    for(i=0; i<u8Len; i++)
    {
        *(pu8data+i) = ICMReadOneByte(REG_ADD_EXT_SENS_DATA_00+i);

    }
    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

    u8Temp = ICMReadOneByte(REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    ICMWriteOneByte(REG_ADD_I2C_SLV0_CTRL,  u8Temp);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

}

void ICMWriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data)
{
    uint8_t u8Temp;
    ICMWriteOneByte(REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    ICMWriteOneByte(REG_ADD_I2C_SLV1_ADDR, u8I2CAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV1_REG,  u8RegAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV1_DO,   u8data);
    ICMWriteOneByte(REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN|1);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    u8Temp = ICMReadOneByte(REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);
    osDelay(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

    u8Temp = ICMReadOneByte(REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    ICMWriteOneByte(REG_ADD_I2C_SLV0_CTRL,  u8Temp);

   ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    return;
}

void ICMWhoIAm()
{
	uint8_t ICM_OK_Msg[8] = "ICM OK";
	if (REG_VAL_WIA == ICMReadOneByte(REG_ADD_WIA))
	{
		OLED_ShowString(0,20,ICM_OK_Msg);
	}
}

void ICMCalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{
	uint8_t i;

	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
  	*pIndex &= 0x07;

  	*pOutVal = 0;
	for(i = 0; i < 8; i ++)
  	{
    	*pOutVal += *(pAvgBuffer + i);
  	}
  	*pOutVal >>= 3;
}

void ICMGyroRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
    uint8_t u8Buf[6];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
    static int16_t ss16c = 0;
    ss16c++;

    u8Buf[0] = ICMReadOneByte(REG_ADD_GYRO_XOUT_L);
    u8Buf[1] = ICMReadOneByte(REG_ADD_GYRO_XOUT_H);
    s16Buf[0] =	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0] = ICMReadOneByte(REG_ADD_GYRO_YOUT_L);
    u8Buf[1] = ICMReadOneByte(REG_ADD_GYRO_YOUT_H);
    s16Buf[1] =	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0] = ICMReadOneByte(REG_ADD_GYRO_ZOUT_L);
    u8Buf[1] = ICMReadOneByte(REG_ADD_GYRO_ZOUT_H);
    s16Buf[2] =	(u8Buf[1]<<8)|u8Buf[0];

#if 1
    for(i = 0; i < 3; i ++)
    {
        ICMCalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    *ps16X = s32OutBuf[0] - gstGyroOffset.s16X;
    *ps16Y = s32OutBuf[1] - gstGyroOffset.s16Y;
    *ps16Z = s32OutBuf[2] - gstGyroOffset.s16Z;
#else
    *ps16X = s16Buf[0];
    *ps16Y = s16Buf[1];
    *ps16Z = s16Buf[2];
#endif
    return;
}

void ICMAccelRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
   uint8_t u8Buf[2];
   int16_t s16Buf[3] = {0};
   uint8_t i;
   int32_t s32OutBuf[3] = {0};
   static ICM20948_ST_AVG_DATA sstAvgBuf[3];

   u8Buf[0] = ICMReadOneByte(REG_ADD_ACCEL_XOUT_L);
   u8Buf[1] = ICMReadOneByte(REG_ADD_ACCEL_XOUT_H);
   s16Buf[0] = (u8Buf[1]<<8)|u8Buf[0];

   u8Buf[0] = ICMReadOneByte(REG_ADD_ACCEL_YOUT_L);
   u8Buf[1] = ICMReadOneByte(REG_ADD_ACCEL_YOUT_H);
   s16Buf[1] = (u8Buf[1]<<8)|u8Buf[0];

   u8Buf[0] = ICMReadOneByte(REG_ADD_ACCEL_ZOUT_L);
   u8Buf[1] = ICMReadOneByte(REG_ADD_ACCEL_ZOUT_H);
   s16Buf[2] = (u8Buf[1]<<8)|u8Buf[0];

#if 1
   for(i = 0; i < 3; i ++)
   {
       ICMCalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
   }
   *ps16X = s32OutBuf[0];
   *ps16Y = s32OutBuf[1];
   *ps16Z = s32OutBuf[2];

#else
   *ps16X = s16Buf[0];
   *ps16Y = s16Buf[1];
   *ps16Z = s16Buf[2];
#endif
   return;

}

void ICMMagRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
   uint8_t counter = 20;
   uint8_t u8Data[MAG_DATA_LEN];
   int16_t s16Buf[3] = {0};
   uint8_t i;
   int32_t s32OutBuf[3] = {0};
   static ICM20948_ST_AVG_DATA sstAvgBuf[3];
   while( counter>0 )
   {
       osDelay(10);
       ICMReadSecondary(I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                   REG_ADD_MAG_ST2, 1, u8Data);

       if ((u8Data[0] & 0x01) != 0)
           break;

       counter--;
   }

   if(counter != 0)
   {
       ICMReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                   REG_ADD_MAG_DATA,
                                   MAG_DATA_LEN,
                                   u8Data);
       s16Buf[0] = ((int16_t)u8Data[1]<<8) | u8Data[0];
       s16Buf[1] = ((int16_t)u8Data[3]<<8) | u8Data[2];
       s16Buf[2] = ((int16_t)u8Data[5]<<8) | u8Data[4];
   }
   else
   {
       printf("\r\n Mag is busy \r\n");
   }
#if 1
   for(i = 0; i < 3; i ++)
   {
       ICMCalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
   }

   *ps16X =  s32OutBuf[0];
   *ps16Y = -s32OutBuf[1];
   *ps16Z = -s32OutBuf[2];
#else
   *ps16X = s16Buf[0];
   *ps16Y = -s16Buf[1];
   *ps16Z = -s16Buf[2];
#endif
   return;
}

void ICMGyroOffset()
{
	uint8_t i;
    int16_t	s16Gx = 0, s16Gy = 0, s16Gz = 0;
	int32_t	s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
    for(i = 0; i < 32; i ++)
 	{
        ICMGyroRead(&s16Gx, &s16Gy, &s16Gz);
        s32TempGx += s16Gx;
		s32TempGy += s16Gy;
		s32TempGz += s16Gz;
        osDelay(10);
    }
    gstGyroOffset.s16X = s32TempGx >> 5;
	gstGyroOffset.s16Y = s32TempGy >> 5;
	gstGyroOffset.s16Z = s32TempGz >> 5;
    return;
}

void MPU_Get_Gyroscope(void)
{
	ICMGyroRead(&gyro[0], &gyro[1], &gyro[2]);
	//if(Deviation_Count==CONTROL_DELAY)
	//{
		//Save the raw data to update zero by clicking the user button
		Original_gyro[0] = gyro[0];
		Original_gyro[1] = gyro[1];
		Original_gyro[2] = gyro[2];

		//Removes zero drift data
		gyro[0] = Original_gyro[0]-Deviation_gyro[0];
		gyro[1] = Original_gyro[1]-Deviation_gyro[1];
		gyro[2] = Original_gyro[2]-Deviation_gyro[2];
	//}
	return;
}

void ICMInit()
{
	/* user bank 0 register */
	ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
	//ICMWriteOneByte(REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET);
	ICMWriteOneByte(REG_ADD_PWR_MIGMT_1, REG_VAL_DEVICE_RESET);
	osDelay(10);
	ICMWriteOneByte(REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE);


	/* user bank 2 register */

	ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);

	ICMWriteOneByte(REG_ADD_GYRO_SMPLRT_DIV, 0x07);
	ICMWriteOneByte(REG_ADD_GYRO_CONFIG_1,REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_500DPS | REG_VAL_BIT_GYRO_DLPF);

	ICMWriteOneByte(REG_ADD_ACCEL_SMPLRT_DIV_2,  0x07);
	ICMWriteOneByte(REG_ADD_ACCEL_CONFIG,REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);

	/* user bank 0 register */
	ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

	osDelay(100);
	/* offset */
	ICMGyroOffset();

	ICMWriteSecondary(I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_100HZ);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==htim4.Instance && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(&htim4, 0);  // reset the counter

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
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
//			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
		}
	}
}

uint32_t notes[] = {
	2272, // A - 440 Hz
	2024, // B - 494 Hz
	3816, // C - 262 Hz
	3401, // D - 294 Hz
	3030, // E - 330 Hz
	2865, // F - 349 Hz
	2551, // G - 392 Hz
	1136, // a - 880 Hz
	1012, // b - 988 Hz
	1912, // c - 523 Hz
	1703, // d - 587 Hz
	1517, // e - 659 Hz
	1432, // f - 698 Hz
	1275, // g - 784 Hz
};
uint8_t* song =(uint8_t*)"e2,d2,e2,d2,e2,B2,d2,c2,A2_C2,E2,A2,B2_E2,G2,B2,c4_E2,e2,d2,e2,d2,e2,B2,d2,c2,A2_C2,E2,A2,B2_E2,c2,B2,A4";
uint32_t getNote(uint8_t ch)
{
	if (ch >= 'A' && ch <= 'G')
	return notes[ch - 'A'];
	if (ch >= 'a' && ch <= 'g')
	return notes[ch - 'a' + 7];
	return 0;
}
uint32_t getDuration(uint8_t ch)
{
	if (ch < '0' || ch > '9') return 400;
	/* number of ms */
	return (ch - '0') * 200;
}
uint32_t getPause(uint8_t ch)
{
	switch (ch) {
		case '+':
		return 0;
		case ',':
		return 5;
		case '.':
		return 20;
		case '_':
		return 30;
		default:
		return 5;
	}
}
void playNote(uint32_t note, uint32_t durationMs)
{
	uint32_t t = 0;
	if (note > 0) {
		while (t < (durationMs*1000)) {
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1); // Turn on your buzzer (Please Edit)
			osDelay(note/2);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0); // Turn off your buzzer (Please Edit)
			osDelay(note/2);
			t += note;
		}
	}
	else {
		osDelay(durationMs); // ms timer
	}
}
void playSong(uint8_t *song) {
	uint32_t note = 0;
	uint32_t dur = 0;
	uint32_t pause = 0;
	/*
	* A song is a collection of tones where each tone is
	* a note, duration and pause, e.g.
	* "E2,F4,"
	*/
	while(*song != '\0') {
		note = getNote(*song++);
		if (*song == '\0')
		break;
		dur = getDuration(*song++);
		if (*song == '\0')
		break;
		pause = getPause(*song++);
		playNote(note, dur);
		osDelay(pause);
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//prevent unused argument(s) compilation warning
	UNUSED(huart);
	HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 20);
	HAL_UART_Transmit(&huart3,(uint8_t *)aRxBuffer, 20, 0xFFFF);

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
  uint8_t ch = 'A';

  for(;;)
  {
	  if(HAL_GPIO_ReadPin(GPIOD, USER_BTN_Pin)==0){
		  if(userBtnCount<13){
			  userBtnCount++;
			  osDelay(100);
		  }else{
			  userBtnCount=0;
			  osDelay(100);
		  }
	  }


	  osDelay(100);


//	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
//	if(ch<'Z')
//		ch++;
//	else ch = 'A';
//	osDelay(100);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the showTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
  /* Infinite loop */
//	uint8_t hello[20] = "";
	uint8_t disp[20] = "";

	for(;;)
	{
	if(distprof!=0){
		switch(userBtnCount){
		case 1:
		  strcpy(disp, "MOTOR TEST");
		  break;
		case 2:
		  strcpy(disp, "IPT LEFT  ");
		  break;
		case 3:
		  strcpy(disp, "IPT RIGHT ");
		  break;
		case 4:
		  strcpy(disp, "FLEFT 90 ");
		  break;
		case 5:
		  strcpy(disp, "FRIGHT 90");
		  break;
		case 6:
		  strcpy(disp, "BLEFT 90 ");
		  break;
		case 7:
		  strcpy(disp, "BRIGHT 90");
		  break;
		case 8:
		  strcpy(disp, "TPT L 45 ");
		  break;
		case 9:
		  strcpy(disp, "TPT R 45");
		  break;
		default:
		  strcpy(disp, "STOP      ");

		}	  //profile reading
	}else{
		switch(userBtnCount){
		case 1:
			strcpy(disp, "80 cm     ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=1;
				userBtnCount=0;
			}
			break;

		case 2:
			strcpy(disp, "90 cm     ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=2;
				userBtnCount=0;
			}
			break;

		case 3:
			strcpy(disp, "100 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=3;
				userBtnCount=0;
			}
			break;

		case 4:
			strcpy(disp, "110 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=4;
				userBtnCount=0;
			}
			break;
		case 5:
			strcpy(disp, "120 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=5;
				userBtnCount=0;
			}
			break;
		case 6:
			strcpy(disp, "130 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=6;
				userBtnCount=0;
			}
			break;
		case 7:
			strcpy(disp, "140 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=7;
				userBtnCount=0;
			}
			break;
		case 8:
			strcpy(disp, "150 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=8;
				userBtnCount=0;
			}
			break;
		case 9:
			strcpy(disp, "160 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=9;
				userBtnCount=0;
			}
			break;
		case 10:
			strcpy(disp, "170 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=10;
				userBtnCount=0;
			}
			break;
		case 11:
			strcpy(disp, "180 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=11;
				userBtnCount=0;
			}
			break;
		case 12:
			strcpy(disp, "190 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=12;
				userBtnCount=0;
			}
			break;
		case 13:
			strcpy(disp, "200 cm    ");
			if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1){
				distprof=13;
				userBtnCount=0;
			}
			break;



		default:
			strcpy(disp, "UNDEFINED ");
			break;
		}

	}

//	  sprintf(disp,"Profile:%2d",userBtnCount);
	  if(distprof==0) OLED_ShowString(0,50,"Dist:");
		  else  OLED_ShowString(0,50,"Prof:");
	  OLED_ShowString(50,50,disp);
	  OLED_Refresh_Gram();


//Ultrasonic Distance reading
	sprintf(disp,"Dist:%5d",Distance);
	OLED_ShowString(0,30,disp);
	OLED_Refresh_Gram();

//print current floor type
	if(distprof==0)	sprintf(disp, "%s","SELECT DIST:");
	else if(distprof==1)	sprintf(disp, "%s","80 cm        ");
	else if(distprof==2)	sprintf(disp, "%s","90 cm        ");
	else if(distprof==3)	sprintf(disp, "%s","100 cm       ");
	else if(distprof==4)	sprintf(disp, "%s","110 cm       ");
	else if(distprof==5)	sprintf(disp, "%s","120 cm       ");
	else if(distprof==6)	sprintf(disp, "%s","130 cm       ");
	else if(distprof==7)	sprintf(disp, "%s","140 cm       ");
	else if(distprof==8)	sprintf(disp, "%s","150 cm       ");
	else if(distprof==9)	sprintf(disp, "%s","160 cm       ");
	else if(distprof==10)	sprintf(disp, "%s","170 cm       ");
	else if(distprof==11)	sprintf(disp, "%s","180 cm       ");
	else if(distprof==12)	sprintf(disp, "%s","190 cm       ");
	else if(distprof==13)	sprintf(disp, "%s","200 cm       ");

	else sprintf(disp, "%d            ",distprof);
	//sprintf(disp, "%d",distprof);
	OLED_ShowString(0,40,disp);
	OLED_Refresh_Gram();
    osDelay(100);
  }
  /* USER CODE END show */
}

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
	//initialize timers
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	/* Infinite loop */
	int count=0;
	int reset=1;
	for(;;)
	{
		if(reset==1){
			realignWheels();
			reset=0;
		}
		if(distprof!=0){

			switch (userBtnCount){
				case 1: //MOTOR TEST
				break;

				case 2:
				if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1 && count<1){
					gyrostraight(DEFAULTPWM, DEFAULTPWM, 0, 200);
//								  ipt90(0);
//								  ipt90(0);
//								  ipt90(0);
					count++;
				}
				break;

				case 3:
				if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1 && count<1){
					ipt90(1);
//								  ipt90(1);
//								  ipt90(1);
//								  ipt90(1);
					count++;
				}
				break;

				case 4:
				if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1 && count<1){
					osDelay(500);
					gyroturn(1, 0, 90);
//							  turn(1, 0, 90);
//							  turn(1, 0, 90);
//							  turn(1, 0, 90);
					count++;
				}
				break;

				case 5:
				if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1 && count<1){
					osDelay(500);
					gyroturn(1, 1, 90);
//							  turn(1, 1, 90);
//							  turn(1, 1, 90);
//							  turn(1, 1, 90);
					count++;
				}
				break;

				case 6:
				if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1 && count<1){
					osDelay(500);
					gyroturn(0, 0, 90);
//							  turn(0, 0, 90);
//							  turn(0, 0, 90);
//							  turn(0, 0, 90);
					count++;
				}
				break;

				case 7:
				if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1 && count<1){
					osDelay(500);
					gyroturn(0, 1, 90);
//								turn(0, 1, 90);
//								turn(0, 1, 90);
//								turn(0, 1, 90);
					count++;
				}
				break;

				case 8://tpt l 45
				if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1 && count<1){
					dir = 1;    //
					tpturn(0);
					count++;
				}
				break;
				case 9://tpt r 45
				if(HAL_GPIO_ReadPin(GPIOE, ENABLE_SW_Pin)==1 && count<1){
					dir = 1;    //
					tpturn(1);
					count++;
				}
				break;

				default:
				break;
			}

		}

	}
	osDelay(200);
  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_encoder */
/**
* @brief Function implementing the encoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder */
void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
  /* Infinite loop */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //MOTORA Encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //MOTORB Encoder

  int tim2Cnt1, tim2Cnt2, tim2Diff; //initiate some counters and a diff variable
  int tim3Cnt1, tim3Cnt2, tim3Diff;
  int expected;
  uint32_t tick;

  tim2Cnt1 = __HAL_TIM_GET_COUNTER(&htim2); //Get count at rising edge
  tim3Cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
  tick = HAL_GetTick(); //Grab current tick and slap it into tick

  uint8_t disp[20];
  uint16_t dir2, dir3;


  for(;;)
  {
	if(HAL_GetTick()-tick > 1000L){
		tim2Cnt2 = __HAL_TIM_GET_COUNTER(&htim2); //get updated no of counts
		tim3Cnt2 = __HAL_TIM_GET_COUNTER(&htim3);
		//
		//tim2
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
			if(tim2Cnt2<tim2Cnt1) //underflow
				tim2Diff = tim2Cnt1 - tim2Cnt2;
			else
				tim2Diff = (65535 - tim2Cnt2)+tim2Cnt1;
		}
		else{
			if(tim2Cnt2 > tim2Cnt1)
				tim2Diff = tim2Cnt2 - tim2Cnt1;
			else
				tim2Diff = (65536 - tim2Cnt1)+tim2Cnt2;
		}
		//
		//tim3 doesnt work
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
			if(tim3Cnt2<tim3Cnt1) //underflow
				tim3Diff = tim3Cnt1 - tim3Cnt2;
			else
				tim3Diff = (65535 - tim3Cnt2)+tim3Cnt1;
		}
		else{
			if(tim3Cnt2 > tim3Cnt1)
				tim3Diff = tim3Cnt2 - tim3Cnt1;
			else
				tim3Diff = (65536 - tim3Cnt1)+tim3Cnt2;
		}

		sprintf(disp,"LSpeed:%5d",tim2Diff);
		OLED_ShowString(0,0,disp);
		sprintf(disp,"RSpeed:%5d",tim3Diff);
		OLED_ShowString(0,10,disp);

		expected = (tim2Diff+tim3Diff)/2;
		if(tim2Diff>expected){
			pwmVal_L - 15;
		}else if(tim2Diff<expected){
			pwmVal_L + 15;
		}
		if(tim3Diff>expected){
			pwmVal_R - 15;
		}else if(tim3Diff<expected){
			pwmVal_R + 15;
		}

//		dir2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
//		dir3 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
//		sprintf(disp,"Direct:%5d",dir2);
//		OLED_ShowString(0,20,disp);
//		sprintf(disp,"RDir:%5d",dir3);
//		OLED_ShowString(0,30,disp);
		//repeat
		tim2Cnt1 = __HAL_TIM_GET_COUNTER(&htim2); //Get count at rising edge
	    tim3Cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
	    tick = HAL_GetTick(); //Grab current time and slap it into tick

	}
    //osDelay(1);
  }
  osDelay(100);
  /* USER CODE END encoder */
}

/* USER CODE BEGIN Header_ultra */
/**
* @brief Function implementing the ultraTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ultra */
void ultra(void *argument)
{
  /* USER CODE BEGIN ultra */
  /* Infinite loop */

	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
  for(;;)
  {
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	osDelay(1);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

    osDelay(10);
  }
  /* USER CODE END ultra */
}

/* USER CODE BEGIN Header_uart */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart */
void uart(void *argument)
{
  /* USER CODE BEGIN uart */
  /* Infinite loop */
	uint8_t ch[10] = "Complete\n";
  for(;;)
  {
	  //if (userBtnCount == 8){
	  	  HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 20);

		  char * pch = malloc(20);
		  //char *str;
		  //sprintf(str, "%s\0",aRxBuffer);
		  int counter = 0;
		  pch = strtok (aRxBuffer," ");
//		  if (strcmp(pch, "START")==0) //task 2
//		  {
//				osDelay(200);
//				gyrostraight(DEFAULTPWM, DEFAULTPWM, 1, distprof*10+25); //200cm
//				osDelay(100);
//				gyroturn(1, 0, 90);
//				gyrostraight(DEFAULTPWM, DEFAULTPWM, 1, 25); //to the left
//				osDelay(100);
//				gyroturn(1, 1, 90);
//				gyrostraight(DEFAULTPWM, DEFAULTPWM, 1, 15);
//				gyroturn(1, 1, 90);
//				gyrostraight(DEFAULTPWM, DEFAULTPWM, 1, 101); //to the right
//				osDelay(100);
//				gyroturn(1, 1, 90);
//				gyrostraight(DEFAULTPWM, DEFAULTPWM, 1, 15);
//				gyroturn(1, 1, 90);
//				gyrostraight(DEFAULTPWM, DEFAULTPWM, 1, 30); //back to the left
//				osDelay(100);
//				gyroturn(1, 0, 90);
//				gyrostraight(DEFAULTPWM, DEFAULTPWM, 1, distprof*10+10); //200cm
//				pch = NULL;
//				sprintf(aRxBuffer[0], ' ');
//				exit(0);
//		  }


			{
			  if(counter==0){
				  if(strcmp(pch, "FORWARD")==0){
					  dir = 1; //forward, backward
				  }
				  else{ //normal turn
					  dir =0;
				  }
			  }
			  if(counter==1){
				  if(strcmp(pch, "TURN")==0){ //if turning


					  pch = strtok (NULL, " ");
					  if (strcmp(pch,"LEFT")==0){
						  if (dir==1) straight(DEFAULTPWM, DEFAULTPWM, 1, 2+1);  //forward left
						  else straight(DEFAULTPWM, DEFAULTPWM, 0, 4.6+1);       //backward left
						  gyroturn(dir, 0, 90);
						  if (dir==1) straight(DEFAULTPWM, DEFAULTPWM, 1, 4.6+1);
						  else straight(DEFAULTPWM, DEFAULTPWM, 0, 2+1);
						  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
					  }
					  else{
						  if (dir==1){
							  straight(DEFAULTPWM, DEFAULTPWM, 0, 2.3+1);
							  dir=1;
						  }
						  else straight(DEFAULTPWM, DEFAULTPWM, 0, 0.7+1);

						  gyroturn(dir, 1, 90);
						  if (dir==1) straight(DEFAULTPWM, DEFAULTPWM, 1, 0.7+1);
						  else{
							  straight(DEFAULTPWM, DEFAULTPWM, 1, 0.8+1);
							  dir=0;
						  }
						HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
					  }

				  }
				  else if(strcmp(pch, "INPLACE")==0){ //doing inplace turn
					  pch = strtok (NULL, " ");
					  if (strcmp(pch,"LE")==0){
						  ipt90(0);
						  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
					  }
					  else{
						  ipt90(1);
						  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
					  }
				  }
				  else{//if not turning

					  gyrostraight(DEFAULTPWM, DEFAULTPWM, dir, atoi(pch));
					  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);


				  }
			  }

			  pch = strtok (NULL, " ");
			  counter++;
			}
//		  FORWARD 20
//		  BACKWARD 100
//		  FORWARD TURN RIGHT
//		  FORWARD TURN LEFT
//		  BACKWARD TURN LEFT
//		    for(int i =0; i<20;i++){
//		    	sprintf(aRxBuffer[i], ' ');
//		    }


//	  }

    osDelay(100);
  }
  /* USER CODE END uart */
}

/* USER CODE BEGIN Header_icm20948 */
/**
* @brief Function implementing the icm20948_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_icm20948 */
void icm20948(void *argument)
{
  /* USER CODE BEGIN icm20948 */
	uint8_t mag_count = 0;
	uint32_t lastWakeTime = HAL_GetTick();
	ICMInit(); //initialize ICM
  /* Infinite loop */
  for(;;)
  {
	  //vTaskDelayUntil(&lastWakeTime, 0.01);
	  if(Deviation_Count<CONTROL_DELAY)
	  {
		  Deviation_Count++;
		  memcpy(Deviation_gyro,gyro,sizeof(gyro));
	  }
	  //Get acceleration sensor data
//	  ICMAccelRead(&accel[0], &accel[1], &accel[2]);
	  //Get gyroscope data
	  MPU_Get_Gyroscope();
	  //Get magnetometer data
//	  ICMMagRead(&magnet[0], &magnet[1], &magnet[2]);
//	  sprintf(icmTempMsg,"%d %d %d      ",magnet[0],magnet[1],magnet[2]);
	  sprintf(icmTempMsg,"Gyr%+07d",gyro[2]);
	  OLED_ShowString(0, 20, icmTempMsg);
	  if(abs(gyro[2]-gyrozero)>50){
		  gyrosum+=abs(gyro[2]-gyrozero);
		  gyrosumsigned+=gyro[2]-gyrozero;
	  }



	  osDelay(25);
  }
  /* USER CODE END icm20948 */
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

