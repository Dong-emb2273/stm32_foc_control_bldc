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
#include "device_config.h"
#include "stm32f405_link.h"
#include "motor.h"
#include "as5048a.h"
#include "FOC_utils.h"
#include "self_commissioning.h"
#include "storage.h"
#include "sf_motion_usb_com.h"
#include "sf_motion_com.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */


AS5048A_t hencd1;
uint32_t adc_buff[4];
foc_t hfoc1;
self_commissioning_t hsc1;
storage_t hstorage1;

sfm_usb_com_t hsfm_usb_com;
sfm_com_t husb_com;
sfm_com_t hcan_com;

char usb_send_buff[64];
uint8_t can_send_buff[64];
uint8_t can_recv_buff[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void motor1_inverter_enable(void) {
  DRV832xS_ENABLE_GPIO_Port->BSRR = DRV832xS_ENABLE_Pin;
  // link_enable_pwm(&htim1);
  // link_enable_adc(&hadc1);
}

void motor1_inverter_disable(void) {
  DRV832xS_ENABLE_GPIO_Port->BSRR = DRV832xS_ENABLE_Pin<<16;
  // link_disable_pwm(&htim1);
  // link_disable_adc(&hadc1);
}

uint32_t motor1_get_pwm_res(void) {
  return link_get_max_pwm(&htim1);
}

float motor1_as5048a_get_mech_deg(void) {
  return AS5048A_get_degree(&hencd1);
}

float motor1_as5048a_get_continuous_mech_deg(void) {
  return AS5048A_get_continuous_degree(&hencd1);
}

float motor1_as5048a_get_rpm(void) {
  return AS5048A_get_rpm(&hencd1);
}

int motor1_as5048a_spi_transmit(uint8_t *tx, uint8_t *rx, uint16_t len) {
  if (HAL_SPI_TransmitReceive_DMA(&hspi2, tx, rx, len) == HAL_OK) return 0;
  return -1;
}

// void motor1_as5047p_spi_cs(_Bool cs_state) {
//   if (cs_state) SPI_CS_GPIO_Port->BSRR = SPI_CS_Pin;
//   else SPI_CS_GPIO_Port->BSRR = SPI_CS_Pin<<16;
// }

int write_flash(void *data, uint32_t len) {
  return link_write_flash(data, len);
}

int read_flash(void *data, uint32_t len) {
  memcpy(data, (void*)FLASH_SECTOR_ADDR, len);
  return 0;
}

// int usb_send_data(uint8_t *data, uint16_t len) {
//   if (CDC_Transmit_FS(data, len) != USBD_OK) return -1;
//   return 0;
// }

int sfm_com_usb_send_data(uint8_t *data, uint16_t len) {
  if (sfm_usb_com_start_send_data(&hsfm_usb_com, data, len) == 0) return 0;
  return -1;
}

int sfm_com_usb_recv_data(uint8_t *data, uint16_t len) {
  (void)data;
  (void)len;
  return 0;
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

/**************************************************************************** */

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
  AS5048A_Config(&hencd1, motor1_as5048a_spi_transmit,GPIO_AS5048A_EX_CS_EN,GPIO_AS5048A_EX_CS_DIS);
  AS5048A_Init(&hencd1, 
              hstorage1.memory.encoder_config.is_dir_inverted? SENSOR_DIR_REVERSE : SENSOR_DIR_NORMAL, 
              hstorage1.memory.encoder_config.scale);
  AS5048A_set_pll_filter(&hencd1, 
                        hstorage1.memory.encoder_config.pll_filt_bandwidth, 
                        hstorage1.memory.encoder_config.pll_filt_damping_ratio, 
                        FOC_TS);
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
  foc_inverter_init(&hfoc1, motor1_inverter_enable, motor1_inverter_disable, motor1_get_pwm_res);
  foc_feedback_sensor_init(&hfoc1, 
                          motor1_as5048a_get_mech_deg, 
                          motor1_as5048a_get_continuous_mech_deg,
                          motor1_as5048a_get_rpm, 
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

// static void indicator_update(void) {
//   static uint32_t heartbeat_tick = 0;
//   if (HAL_GetTick() - heartbeat_tick >= 500) {
//     heartbeat_tick = HAL_GetTick();
//     HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
//   }
//   if (HAL_GPIO_ReadPin(NFAULT1_GPIO_Port, NFAULT1_Pin)) {
//     LED_R_GPIO_Port->BSRR = LED_R_Pin<<16;
//   }
//   else {
//     LED_R_GPIO_Port->BSRR = LED_R_Pin;
//   }
// }

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
    sfm_com_send_event(&husb_com, SFM_EVENT_SELFCOM_DONE);
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

void usb_to_can_update(void) {
  if (husb_com.incomming_data_flag) {
    husb_com.incomming_data_flag = 0;
    hcan_com.send_data(husb_com.data_rx, husb_com.data_rx_len);
  }
  if (hcan_com.incomming_data_flag) {
    hcan_com.incomming_data_flag = 0;
    husb_com.send_data(hcan_com.data_rx, hcan_com.data_rx_len);
  }
}

/**************************************************************************** */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
    AS5048A_set_spi_transfer_done(&hencd1);
	}
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
    adc_buff[0] = ADC1->JDR1; // current sense A
    adc_buff[1] = ADC1->JDR2; // current sense B
    adc_buff[2] = ADC1->JDR3; // current sense C
    adc_buff[3] = ADC1->JDR4; // power voltage sense
    motor_calculate_power_voltage(&hfoc1.motor);
    motor_get_power_voltage(&hfoc1.motor, &hfoc1.v_bus);
    AS5048A_pll_filter_update(&hencd1);
    sc_update(&hsc1, FOC_TS);
    foc_update(&hfoc1, FOC_TS);
  }
}

// CAN RX FIFO 0 Callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  uint32_t id;
  if (link_can_recv_data(&id, hcan_com.data_rx, &hcan_com.data_rx_len) == 0) {
    hcan_com.incomming_data_flag = 1;
  }
}

/**************************************************************************** */


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
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  init_foc();

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

    // indicator_update();
#if USB_TO_CAN
    usb_to_can_update();
#else
    AS5048A_update(&hencd1);
    event_update();
    // sfm_com_update(&husb_com);
    sfm_com_update(&hcan_com);
    self_commissioning_update();
#endif
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = 4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AS5048A_EX_CS_Pin|AS5048A_IN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DRV832xS_ENABLE_Pin|DRV8323_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AS5048A_EX_CS_Pin AS5048A_IN_CS_Pin */
  GPIO_InitStruct.Pin = AS5048A_EX_CS_Pin|AS5048A_IN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV832xS_FAULT_Pin */
  GPIO_InitStruct.Pin = DRV832xS_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRV832xS_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV832xS_ENABLE_Pin DRV8323_CS_Pin */
  GPIO_InitStruct.Pin = DRV832xS_ENABLE_Pin|DRV8323_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
