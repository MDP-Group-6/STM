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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "Pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ShowTask */
osThreadId_t ShowTaskHandle;
const osThreadAttr_t ShowTask_attributes = {
  .name = "ShowTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor_Turn */
osThreadId_t Motor_TurnHandle;
uint32_t Motor_TurnBuffer[ 128 ];
osStaticThreadDef_t Motor_TurnControlBlock;
const osThreadAttr_t Motor_Turn_attributes = {
  .name = "Motor_Turn",
  .cb_mem = &Motor_TurnControlBlock,
  .cb_size = sizeof(Motor_TurnControlBlock),
  .stack_mem = &Motor_TurnBuffer[0],
  .stack_size = sizeof(Motor_TurnBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void receiveAndTransmit(void *argument);
void show(void *argument);
void motors(void *argument);
void encoder_task(void *argument);
void Turn(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[20];
uint8_t command[20];
uint8_t index=0;
uint8_t receive_msg[10] = {0};
uint16_t left_speed = 0;
uint16_t tar_left_speed = 0;
uint16_t right_speed = 0;
uint16_t tar_right_speed = 0;
uint16_t left_dir = '0';
uint16_t right_dir = '0';
uint16_t angel =72;
pidData_t m_pidData;
pidData_t* m_pidData_p;
uint32_t left_encoder_speed;
uint16_t left_ignore_time;

pidData_t m_pidData_right;
pidData_t* m_pidData_right_p;
uint32_t right_encoder_speed;
uint16_t right_ignore_time;

//used for sonic
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Capture = 0;
uint8_t Distance = 0;

//sending lock
uint8_t sending_size_in_transmit_fun = 0;


void my_PID_init()
{
	double m_kp = 0.3;
	double m_ki = 0.1;
	double m_kd = 0.1;
	ctrlDir_t m_controllerDir = PID_DIRECT;
	uint32_t m_samplePeriodMs = 200;

	m_pidData_p = &m_pidData;
	Pid_Init(m_pidData_p, m_kp, m_ki, m_kd, m_controllerDir, m_samplePeriodMs);

	m_pidData_right_p = &m_pidData_right;
	Pid_Init(m_pidData_right_p, m_kp, m_ki, m_kd, m_controllerDir, m_samplePeriodMs);
}


struct command1
{
	uint8_t mesg[20];
};

struct commandQueue
{
	int size;
	int capacity;
	int front;
	int rear;
	struct command1 command_set[25];//25 commands
};

struct commandQueue command_queue;

void Queue_init()
{
	command_queue.size = 0;
	command_queue.capacity = 25;
	command_queue.front = 0;//index of first command
	command_queue.rear = command_queue.capacity-1; // index of last command
}

int isEmpty()
{
	return (command_queue.size == 0);
}
int isFull()
{
	return (command_queue.size == command_queue.capacity);
}

int  getSize()
{
	return command_queue.size;
}


void Enqueue(uint8_t* command_in)
{
	//put a command at index rear
	if (isFull(command_queue))
	   return;
	command_queue.rear = (command_queue.rear + 1) % command_queue.capacity;
	//queue->commandSet[queue->rear] = command1_in;
	command_queue.size += 1;

	for(int i=0;i<20;i++){
		command_queue.command_set[command_queue.rear].mesg[i] = command_in[i];
	}
	//printf("%d enqueued to queue\n", item);
}

struct command1 Dequeue()
{
	/*
    if (isEmpty(command_queue))
        return command_queue.command_set[command_queue.front];
       */
    struct command1 command_out = command_queue.command_set[command_queue.front];
    command_queue.front = (command_queue.front + 1) % command_queue.capacity;
    command_queue.size = command_queue.size - 1;

    return command_out;
}

void update();
void process_command()
{

	uint8_t out_msg[20];
	uint16_t array_to_uint16_angel(uint8_t* uint8_array)
	{
		uint16_t res = 0;
		for(int i=0;i<3;i++){
			res*=10;
			res+=(uint8_array[i]-(uint8_t)48);
		}
		return res;
	}

	uint16_t array_to_uint16(uint8_t* uint8_array)
	{
		uint16_t res = 0;
		int len = sizeof(uint8_array)/sizeof(uint8_array[0]);
		for(int i=0;i<len;i++){
			res*=10;
			res+=(uint8_array[i]-(uint8_t)48);
		}
		return res;
	}

	if(!isEmpty())
	{
		struct command1 command_in_struct = Dequeue();
		uint8_t command_in[20];
		for(int i=0; i<20;i++)
		{
			command_in[i] = command_in_struct.mesg[i];
		}
		command_in[19]='\0';



		//get angel
		uint8_t cmd1[3] = {command_in[0],command_in[1],command_in[2]};
		//sscanf(cmd1, "%d", &angel);
		angel = array_to_uint16_angel(cmd1);

		sprintf(out_msg,"%s",command_in);//"%s",command_in);
		OLED_ShowString(0,0,out_msg);


		if(angel<59 ||angel>90)
			angel=72;




		//control for left wheel
		left_dir = command_in[3];
		right_dir = command_in[8];


		uint8_t cmd2[4] = {command_in[4],command_in[5],command_in[6],command_in[7]};
		uint8_t cmd3[4] = {command_in[9], command_in[10], command_in[11],command_in[12]};

		tar_left_speed = array_to_uint16(cmd2);
		left_speed = tar_left_speed;
		left_ignore_time = 0;
		m_pidData_p->output = tar_left_speed;
		m_pidData_p->prevOutput = tar_left_speed;
		Pid_SetSetPoint(m_pidData_p ,(double)tar_left_speed);
		Pid_SetOutputLimits(m_pidData_p, 0, 6000);


		tar_right_speed = array_to_uint16(cmd3);
		right_speed = tar_right_speed;
		right_ignore_time = 0;
		m_pidData_right_p->output = tar_right_speed;
		m_pidData_right_p->prevOutput = tar_right_speed;
		Pid_SetSetPoint(m_pidData_right_p ,(double)tar_right_speed);
		Pid_SetOutputLimits(m_pidData_right_p, 0, 6000);


		//Pid_SetOutputLimits(m_pidData_p,(tar_left_speed*0.5), (tar_left_speed*1.5));

		uint8_t cmd4[4] = {command_in[13], command_in[14], command_in[15],'0'};
		uint16_t time = array_to_uint16(cmd4);

		//call all the functions to quickly change all settings
		update();


		osDelay(time);
	}
}


void update()
{
//motor
	if(left_dir=='0')
	{
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	}else{
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
	}




	// Modify the comparison value for the duty cycle
	if(right_dir=='0')
	{
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
	}else{
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
	}
	// Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,left_speed);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,right_speed);


//turn

	htim1.Instance->CCR4 = angel;

}

void HCSR04_Read()
{
	HAL_GPIO_WritePin(Sonic_Trig_GPIO_Port, Sonic_Trig_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(Sonic_Trig_GPIO_Port, Sonic_Trig_Pin, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}
void send_queue_size()
{
	if(command_queue.size>=10)
		{//send two bytes, first is 0, next is the number, which ranges from 0-9
			uint8_t out_msg[1];
			sprintf(out_msg,"%d",(uint8_t)command_queue.size);
			HAL_UART_Transmit(&huart3,(uint8_t *)out_msg,2,0xFFFF);
		}else{
			//send two byte, directly
			uint16_t out_msg[2];
			sprintf(out_msg,"0%d",(uint8_t)command_queue.size);
			HAL_UART_Transmit(&huart3,(uint8_t *)out_msg,2,0xFFFF);
		}
}

void send_complete()//98
{
	uint16_t out_msg[2];
	sprintf(out_msg,"%d",98);
	HAL_UART_Transmit(&huart3,(uint8_t *)out_msg,2,0xFFFF);
}

void send_fin()//99
{
	uint16_t out_msg[2];
	sprintf(out_msg,"%d",99);
	HAL_UART_Transmit(&huart3,(uint8_t *)out_msg,2,0xFFFF);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  index=0;
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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer,20);

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

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  Queue_init();
  my_PID_init();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(receiveAndTransmit, NULL, &defaultTask_attributes);

  /* creation of ShowTask */
  ShowTaskHandle = osThreadNew(show, NULL, &ShowTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motors, NULL, &MotorTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder_task, NULL, &EncoderTask_attributes);

  /* creation of Motor_Turn */
  Motor_TurnHandle = osThreadNew(Turn, NULL, &Motor_Turn_attributes);

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Sonic_Trig_GPIO_Port, Sonic_Trig_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : Sonic_Echo_Pin */
  GPIO_InitStruct.Pin = Sonic_Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Sonic_Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Sonic_Trig_Pin */
  GPIO_InitStruct.Pin = Sonic_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Sonic_Trig_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*Prevent unused argument(s) compilation warning */
	//UNUSED(huart);
	//to show that the device has successful connection, we toggle the LED when new data is received
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

	//get the data received
	HAL_UART_Receive_IT(&huart3, (uint8_t *)receive_msg, 1);

	//"|" is a special character that marks the end of the command
	if(receive_msg[0]=='|')
	{
		//ensure the least length of the received command
		//if enough, enqueue the command to task queue
		//and set index to first to receive new data
		if (command[15]!=' ')
			Enqueue(command);
		index=0;
		//if not enough, it is viewed as invalid
		for(int i=0; i<20;i++) command[i] = ' ';
	}else{
		//"?" is for abort
		if(receive_msg[0]=='?' || index==20){
			index=0;
			//clear current receiving command
			for(int i=0; i<20;i++) command[i] = ' ';
			//clear saved and undone command from task queue
			while(command_queue.size>0) Dequeue();
			//create a idol command, and put robot to idol
			uint8_t default_command[20] = "0760000000000300";
			Enqueue(default_command);
		    process_command();
		}
		else{
			//not "|" or "?", then receiving part command,
			command[index] = receive_msg[0];
			index++;
		}
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == Sonic_Echo_Pin)
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		if(Is_First_Capture==0)
		{
			//IC_Val1 = HAL_TIM_ReadCaptureValue(htim, TIM_CHANNEL_1);
			IC_Val1 = __HAL_TIM_GET_COUNTER(&htim4);
			Is_First_Capture=1;
		}else if(Is_First_Capture==1)
		{
			//IC_Val2 = HAL_TIM_ReadCaptureValue(htim, TIM_CHANNEL_1);
			IC_Val2 = __HAL_TIM_GET_COUNTER(&htim4);
			__HAL_TIM_SET_COUNTER(&htim4, 0);
			//they set the timer to 0 but we are not

			if(IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
			}else if(IC_Val1 >IC_Val2)
			{
				Difference = ((1000) - IC_Val1 ) + IC_Val2;
			}

			Distance = ((Difference *0.68)*10)/45;
			Is_First_Capture = 0;
		}
	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_receiveAndTransmit */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_receiveAndTransmit */
void receiveAndTransmit(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  //default command, front wheels forward, back wheels stop, as idol state
 //continues for 3s
  uint8_t default_command[20] = "0760000000000300";
  for(;;)
  {

	  //central part for checking command
	  //one command finished, then come here to see if got new command
	  //if no new command, it will just enqueue a default idol command for robot to execute
	  if(!isEmpty())
	  {
		  //got command undone, do the command in task queue
		  process_command();
		  sending_size_in_transmit_fun=1;
		  if(command_queue.size==0)
		  {
			  send_fin();
		  }else
		  {
			  send_complete();
		  }
		  sending_size_in_transmit_fun=0;
	  }else
	  {
		  //no new undone command, execute the default command
		  Enqueue(default_command);
		  process_command();
	  }
	  //toggle the LED to see if the program is still functioning
	  //HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the ShowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
  //uint8_t hello[20] = "Hello World!\0";
	HAL_TIM_Base_Start(&htim4);
  /* Infinite loop */
  for(;;)
  {
	//part where the stm broad transmit data to rpi
	  //content is the number of the command in task queue
	  //task queue's max length is 25, so the rpi should avoid sending new command when length reaches 25

	//we want to send the length in two bytes, so number below 10 is processed differently
	//we pad a 0 before

	  //display dis and tick

	  uint32_t out_msg_sonic[20];
	  sprintf(out_msg_sonic,"                ");
	  OLED_ShowString(0,30,out_msg_sonic);
	  OLED_ShowString(0,40,out_msg_sonic);
	  OLED_ShowString(0,50,out_msg_sonic);
	  sprintf(out_msg_sonic,"distance :%d",Distance+100);
	  OLED_ShowString(0,30,out_msg_sonic);
	  sprintf(out_msg_sonic,"tick :%d",HAL_GetTick());
	  OLED_ShowString(0,40,out_msg_sonic);



	  Is_First_Capture=0;
	  HCSR04_Read();

	  if(sending_size_in_transmit_fun==0)
	  {
		  send_queue_size();


	  }
	  osDelay(100);

	  uint16_t out_msg[3];
	  sprintf(out_msg,"%d",Distance+100);
	  HAL_UART_Transmit(&huart3,(uint8_t *)out_msg,3,0xFFFF);


	  OLED_Refresh_Gram();
	  osDelay(400);
  }
  /* USER CODE END show */
}

/* USER CODE BEGIN Header_motors */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motors */
void motors(void *argument)
{
  /* USER CODE BEGIN motors */

	//actually this part of the code is not so useful for our program
	//the setting is modified in update() function
	//this thread, changes speed  every 2s, but this change is not needed


//start the PWM signal
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  /* Infinite loop */
  for(;;)
  {
	  //the old pid algo
	  //given up as it doesn't provide much help
	  //and possible cause unstable actions
	  /*
	  if( left_ignore_time > 5)
	  {
		  if( left_encoder_speed < 50000)
		  {
			  Pid_Run(m_pidData_p, (double)left_encoder_speed);
			  double new_left_change = m_pidData_p->output;

			  left_speed = (uint16_t)new_left_change;

			  uint16_t out_msg[20];
			  sprintf(out_msg,"                ");
			  OLED_Refresh_Gram();
		  }
	  }else
	  {
		  left_ignore_time+=1;
	  }

	  if( right_ignore_time > 5)
	  	  {
	  		  if( right_encoder_speed < 50000)
	  		  {
	  			  Pid_Run(m_pidData_right_p, (double)right_encoder_speed);
	  			  double new_right_change = m_pidData_right_p->output;

	  			  right_speed = (uint16_t)new_right_change;


	  		  }
	  	  }else
	  	  {
	  		  right_ignore_time+=1;
	  	  }
	  //displaying
	  uint16_t out_msg[20];
	  sprintf(out_msg,"                ");
	  OLED_ShowString(0,30,out_msg);
	  OLED_ShowString(0,40,out_msg);
	  OLED_ShowString(0,50,out_msg);
	  sprintf(out_msg,"enc speed:%d",right_encoder_speed);
	  OLED_ShowString(0,30,out_msg);
	  sprintf(out_msg,"pwm speed:%d",right_speed);
	  OLED_ShowString(0,40,out_msg);
	  OLED_Refresh_Gram();
	  */

	  //set the direction of the left wheel, 0 for forward and 1 for backward
	  if(left_dir=='0')
	  {
		  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	  }else{
		  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
	  }
	  //set the speed of the left wheel




	  //set the direction of the right wheel, 0 for forward and 1 for backward
	  if(right_dir=='0')
	  {
		  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
	  }else{
		  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
	  }
	  //set the speed of the right wheel
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,left_speed);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,right_speed);
	  osDelay(200);
  }

  /* USER CODE END motors */
}

/* USER CODE BEGIN Header_encoder_task */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder_task */
void encoder_task(void *argument)
{
  /* USER CODE BEGIN encoder_task */
  /* Infinite loop */

  //the encoder seems to be able to tell the direction of the wheel
  //it counts up when forward and count down
  //but since pid is not used
  //encoder doesn't have much usage already
  //also the encoder seems to be updating very slow, 1s every time

  //principle
  //there two counters, cnt1 and cnt2
  //every 0.1s, cnt1 records the current value of the timer's counter
  //at the end of the next 0.1s, before cnt1's update, cnt2 gets the value of the timer's counter
  //by do deduction on cnt1 and cnt2, we get how much the counter has changed
  //this allows us to get roughly how much the wheel has turned



  //start the timer
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

  //tick for the system
  uint32_t tick;
  //value for measuring wheel's pin
  int left_cnt1, left_cnt2, left_diff;
  int right_cnt1, right_cnt2, right_diff;
  uint16_t left_dir;
  uint16_t right_dir;

  //mesg buffer for sending for showing the mesg on OLED
  uint8_t hello[20];

  //get the tick of the system
  tick = HAL_GetTick();

  //get the counter1
  left_cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
  right_cnt1 = __HAL_TIM_GET_COUNTER(&htim3);

  //we will only explain the left wheel encoder, left wheel is the same idea
  for(;;)
  {
	//make sure the time passed 1000L, this is the system timer
	if(HAL_GetTick()-tick > 1000L){
		//get the value of the counter 2
		left_cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
			if(left_cnt2<=left_cnt1)
				left_diff = left_cnt1 - left_cnt2;
			else
				left_diff = (65535 - left_cnt2)+left_cnt1;
		}
		else{
			if(left_cnt2 >= left_cnt1)
				left_diff = left_cnt2 - left_cnt1;
			else
				left_diff = (65535 - left_cnt1) + left_cnt2;
		}

		//display the rotation speed
		/*
		sprintf(hello,"%5d\0",left_diff);
		left_encoder_speed = left_diff;
		OLED_ShowString(0,40,hello);
		left_dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
		sprintf(hello,"%5d\0",left_dir);
		OLED_ShowString(40,40,hello);
		left_cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
		*/

		//right wheel
		right_cnt2 = __HAL_TIM_GET_COUNTER(&htim3);
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
		{
			if(right_cnt2<=right_cnt1)
				right_diff = right_cnt1 - right_cnt2;
			else{
				right_diff = (65535 - right_cnt2)+right_cnt1;
			}
		}else{
			if(right_cnt2 >= right_cnt1){
				right_diff = right_cnt2 - right_cnt1;
			}else{
				right_diff = (65535 - right_cnt1) + right_cnt2;
			}
		}
		right_cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
		tick = HAL_GetTick();

		//display the rotation speed
		/*
		sprintf(hello,"%5d\0",right_diff);
		right_encoder_speed = right_diff;
		OLED_ShowString(0,50,hello);
		right_dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
		sprintf(hello,"%5d\0",right_dir);
		OLED_ShowString(40,50,hello);

		OLED_Refresh_Gram();
		*/

	}
    osDelay(100);
  }
  /* USER CODE END encoder_task */
}

/* USER CODE BEGIN Header_Turn */
/**
* @brief Function implementing the Motor_Turn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Turn */
void Turn(void *argument)
{
  /* USER CODE BEGIN Turn */
  /* Infinite loop */

	//similar to motor, this part of the code is not so useful for our program
	//the setting is modified in update() function
	//this thread, front wheel's angle every 0.1s, but this change is not needed

	//start the timer PWM
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	for(;;)
	{
		//set the front wheels' angle
		htim1.Instance->CCR4 = angel;//I typed wrongly
		osDelay(100);
	}
  /* USER CODE END Turn */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
