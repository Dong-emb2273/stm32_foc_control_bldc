/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include "driver.h"

#include "device_config.h"
#include "stm32f405_link.h"
#include "motor.h"
#include "as5048a.h"
#include "FOC_utils.h"
#include "self_commissioning.h"
#include "storage.h"
#include "sf_motion_usb_com.h"
#include "sf_motion_com.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;


AS5048A_t hencd1;
uint32_t adc_buff[4];
foc_t hfoc1;
self_commissioning_t hsc1;
storage_t hstorage1;

sfm_com_t husb_com;
sfm_com_t hcan_com;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* Definitions for CalibrationTask */
osThreadId_t CalibrationTaskHandle;
const osThreadAttr_t CalibrationTask_attributes = {
  .name = "CalibrationTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

osThreadId_t ErrorTaskHandle;
const osThreadAttr_t ErrorTask_attributes = {
  .name = "ErrorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};



uint32_t get_dt_us(void) {
#if 0
  static uint32_t last_us = 0;

  // Get current time (µs)
  uint32_t now_us = TIM10->CNT;
  uint32_t elapsed_us = now_us - last_us;
  last_us = now_us;
#else
  // Get current time (µs)
  uint32_t elapsed_us = TIM10->CNT;
  TIM10->CNT = 0;
#endif
  return elapsed_us;
}
uint32_t dt_us;



uint8_t Serial2RxBuffer[1];




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
void StartCalibrationTask(void *argument);

/* USER CODE BEGIN PFP */

int write_flash(void *data, uint32_t len) {
  return link_write_flash(data, len);
}

int read_flash(void *data, uint32_t len) {
  memcpy(data, (void*)FLASH_SECTOR_ADDR, len);
  return 0;
}
uint32_t motor1_get_pwm_res(void) {
  return link_get_max_pwm(&htim1);
}
float motor1_as5047p_get_mech_deg(void) {
  return AS5048A_get_degree(&hencd1);
}

float motor1_as5047p_get_continuous_mech_deg(void) {
  return AS5048A_get_continuous_degree(&hencd1);
}

float motor1_as5047p_get_rpm(void) {
  return AS5048A_get_rpm(&hencd1);
}

static void init_motor(void) {
  // motor pwm
  motor_init_pwm(&hfoc1.motor, &TIM1->CCR1, &TIM1->CCR2, &TIM1->CCR3);
  // current sense
  motor_init_adc_current_sense(&hfoc1.motor, &adc_buff[0], &adc_buff[1], &adc_buff[2]);
  // power voltage sense
  motor_init_adc_power_voltage_sense(&hfoc1.motor, &adc_buff[3]);
  motor_init_lpf_power_voltage_sense(&hfoc1.motor, hstorage1.memory.voltage_sens.filter_fc, BLDC_PWM_FREQ);
  // current sense amplifier calibration enable (for internal CSA only)
  // CSA_CAL1_GPIO_Port->BSRR = CSA_CAL1_Pin << 16;
}

static void init_encoder(void) {
  AS5048A_Config(&hencd1, SPI_EX_TransmitReceive_DMA, GPIO_AS5048A_EX_CS_EN, GPIO_AS5048A_EX_CS_DIS);
  AS5048A_Init(&hencd1, 
              hstorage1.memory.encoder_config.is_dir_inverted? SENSOR_DIR_REVERSE : SENSOR_DIR_NORMAL, 
              hstorage1.memory.encoder_config.scale);
  AS5048A_set_pll_filter(&hencd1, 
                        hstorage1.memory.encoder_config.pll_filt_bandwidth, 
                        hstorage1.memory.encoder_config.pll_filt_damping_ratio, 
                        FOC_TS);
}
int can_recv_data(uint8_t *data, uint16_t len) {
  (void)data;
  (void)len;
  return 0;
}

int can_send_data(uint8_t *data, uint16_t len) {
  uint32_t can_std_id = (data[0] << 8) | data[1];
  return link_can_send_data(can_std_id, data + 2, len);
}


static void init_foc(void) {
#if USB_TO_CAN
  link_can_init(USB_TO_CAN_DEVICE_ID);
  sfm_usb_com_init(&hsfm_usb_com, usb_send_data);
  sfm_com_init(&husb_com, 
              sfm_com_usb_recv_data, 
              sfm_com_usb_send_data, 
              HAL_GetTick,
              link_system_reset,
              &hfoc1, &hstorage1, &hsc1);
  sfm_com_init(&hcan_com, 
              can_recv_data, 
              can_send_data, 
              HAL_GetTick,
              link_system_reset,
              &hfoc1, &hstorage1, &hsc1);
#else
  storage_init(&hstorage1, write_flash, read_flash);
  storage_read_config(&hstorage1);
  storage_copy_to_local(&hstorage1, &hfoc1);

  link_set_pwm_freq(&htim1, BLDC_PWM_FREQ);
  init_motor();
  init_encoder();
  sc_init(&hsc1, &hfoc1);

  i2t_init(&hfoc1.i2t, FOC_TS);
  i2t_set_config(&hfoc1.i2t, 
                hstorage1.memory.protection.i2t_i_rated,
                hstorage1.memory.protection.i2t_i_max,
                hstorage1.memory.protection.i2t_t_max);
  float uv = hstorage1.memory.protection.under_voltage;
  float ov = hstorage1.memory.protection.over_voltage;
  vbus_init(&hfoc1.vbus_prot, uv, ov, FOC_TS);
  foc_inverter_init(&hfoc1, GPIO_DRV832xS_CS_EN , GPIO_DRV832xS_CS_DIS, motor1_get_pwm_res);
  foc_feedback_sensor_init(&hfoc1, 
                          motor1_as5047p_get_mech_deg, 
                          motor1_as5047p_get_continuous_mech_deg,
                          motor1_as5047p_get_rpm, 
                          hstorage1.memory.encoder_config.error_comp_deg);
  foc_cogging_comp_init(&hfoc1, hstorage1.memory.cogging_torque.current_LUT);
  foc_disable(&hfoc1);

  link_can_init(hfoc1.motor.device_id);
  // sfm_usb_com_init(&hsfm_usb_com, usb_send_data);
  // sfm_com_init(&husb_com, 
  //             sfm_com_usb_recv_data, 
  //             sfm_com_usb_send_data, 
  //             HAL_GetTick,
  //             link_system_reset,
  //             &hfoc1, &hstorage1, &hsc1);
  sfm_com_init(&hcan_com, 
              can_recv_data, 
              can_send_data, 
              HAL_GetTick,
              link_system_reset,
              &hfoc1, &hstorage1, &hsc1);

  // Id PI parameter
  pid_reset(&hfoc1.id_ctrl);
  pid_set_ts(&hfoc1.id_ctrl, FOC_TS);
  // Id PI parameter
  pid_reset(&hfoc1.iq_ctrl);
  pid_set_ts(&hfoc1.iq_ctrl, FOC_TS);
  // Speed PID parameter
  pid_reset(&hfoc1.speed_ctrl);
  pid_set_ts(&hfoc1.speed_ctrl, SPEED_TS);
  // Position PID parameter
  pid_reset(&hfoc1.pos_ctrl);
  pid_set_ts(&hfoc1.pos_ctrl, POSITION_TS);
  pid_set_max_d(&hfoc1.pos_ctrl, 100.0f);
  // field weakening
  pid_reset(&hfoc1.fw_ctrl);
  pid_set_ts(&hfoc1.fw_ctrl, FOC_TS);

  // foc_set_mode(&hfoc1, FOC_MODE_SENSORLESS_SMO_HFI_NEW);
  foc_sensorless_init(&hfoc1, BLDC_PWM_FREQ);

  // HAL_Delay(500);
  link_enable_pwm(&htim1);
  link_enable_adc(&hadc1);
  foc_enable(&hfoc1);
  // foc_set_motor_mode(&hfoc1, hstorage1.memory.motor_config.motor_mode);
#endif
}



static void self_commissioning_update(void) {
  if (sc_is_measure_done(&hsc1)) {
    switch(sc_get_seq(&hsc1)) {
      case SC_SEQUENCE_START_MEASURE_RS:
        foc_set_motor_Rs(&hfoc1, sc_get_Rs(&hsc1));
        break;
      case SC_SEQUENCE_START_MEASURE_LD:
        foc_set_motor_Ld(&hfoc1, sc_get_Ld(&hsc1));
        break;
      case SC_SEQUENCE_START_MEASURE_LQ:
        foc_set_motor_Lq(&hfoc1, sc_get_Lq(&hsc1));
        break;
      case SC_SEQUENCE_START_CALIBRATE_ABS_ENCODER:
        break;
      case SC_SEQUENCE_START_CALC_POLE_PAIRS:
        break;
      case SC_SEQUENCE_START_MEASURE_COGGING_TORQUE:
        husb_com.show_cogging_torque_lut = 1;
        break;
      default:
        break;
    }
    sfm_com_send_event(&hcan_com, SFM_EVENT_SELFCOM_DONE);
  }
}

static void event_update(void) {
  foc_error_t error_code = FOC_ERROR_NONE;
  if (foc_get_error_flag(&hfoc1, &error_code)) {
    switch (error_code) {
      case FOC_ERROR_I2T_TRIPPED: {
        sfm_com_send_event(&husb_com, SFM_EVENT_I2T_TRIPPED);
        sfm_com_send_event(&hcan_com, SFM_EVENT_I2T_TRIPPED);
        break;
      }
      case FOC_ERROR_OVER_VOLTAGE: {
        sfm_com_send_event(&husb_com, SFM_EVENT_OVER_VOLTAGE);
        sfm_com_send_event(&hcan_com, SFM_EVENT_OVER_VOLTAGE);
        break;
      }
      case FOC_ERROR_UNDER_VOLTAGE: {
        sfm_com_send_event(&husb_com, SFM_EVENT_UNDER_VOLTAGE);
        sfm_com_send_event(&hcan_com, SFM_EVENT_UNDER_VOLTAGE);
        break;
      }
      default:
        break;
    }
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ErrorTask(void *argument);

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

  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
//  GPIO_Init();
  MX_DMA_Init();
  MX_SPI_IN_Init();
  MX_SPI_EX_Init();
  MX_USART_UART_Init();
  MX_CAN_Init();
  MX_TIM1_Init();


  ADC_DMA_CONFIG();
  TIM_PWM_DMA_CONFIG();

	TIM_IQR_MS_CONGFIG();
  TIM_COUNT_US_CONFIG();


  init_foc();
  
  // Load Config 

  // flash_erase_ready();
  foc_position_control_update(&hfoc1,1);


	
	/*ENCODER setup   */


  /*CAN setup   */

  

  // Enable Timer
  TIM_COUNTER_ENABLE(TIM10);
	TIM_COUNTER_ENABLE(TIM3);
  TIM_COUNTER_ENABLE(TIM1); 


  /* Start the FSM */

  /* FLASH SETUP */

  
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
  /* creation of CalibrationTask */
  CalibrationTaskHandle = osThreadNew(StartCalibrationTask, NULL, &CalibrationTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  ErrorTaskHandle = osThreadNew(ErrorTask, NULL, &ErrorTask_attributes);  
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

    // indicator_update();
    #if USB_TO_CAN
        usb_to_can_update();
    #else
        AS5048A_update(&hencd1);
        event_update();
        sfm_com_update(&husb_com);
        sfm_com_update(&hcan_com);
        self_commissioning_update();
    #endif

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init spi2*/
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 7, 4);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 7, 5);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	
	/* DMA interrupt init spi1 */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 7, 6);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 7, 7);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	
	/* DMA interrupt init uart*/
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 7, 8);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 7, 9);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

void TIM3_IRQHandler(void){
	TIM3->SR &= ~TIM_SR_UIF;
  //	DRV8323_Get_Current(&bldc, &hfoc.ia, &hfoc.ib,&hfoc.ic);
  //	USART_Get_Angle(&usart_tx, hfoc.actual_angle);
  //	USART_Get_Vel(&usart_tx, hfoc.actual_rpm);
  //	USART_Get_Pos(&usart_tx, hfoc.actual_angle);
  //	USART_Get_Current(&usart_tx, hfoc.ia, hfoc.ib, hfoc.ic);	
  //	USART_TRANSMIT(&usart_tx);
	
	
}
//
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1024;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 25;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/*===========================================================*/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2 || hspi->Instance == SPI1)
	{

	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {

  }
}
/*===========================================================*/
void ADC_IRQHandler(void) {

	if (ADC1->SR & ADC_SR_JEOC) {
		ADC1->SR &= ~ADC_SR_JEOC;
    
	}
}

void ErrorTask(void *argument)
{
  osDelay(1000);
  
}

void StartCalibrationTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  osDelay(3000);
  /* Infinite loop */
  for(;;)
  {   
    

    osDelay(1);
  }
  /* USER CODE END 5 */
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCalibrationTask */
/**
  * @brief  Function implementing the CalibrationTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCalibrationTask */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
#ifdef USE_FULL_ASSERT
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
