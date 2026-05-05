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


#include "uart.h"
#include "hw_config.h"
#include "user_config.h"
#include "spi.h"
#include "as5048a.h"
#include "drv8323s.h"
#include "flash.h"
#include "foc_utils.h"
#include "can.h"

#include "encoder.h"
#include "fsm.h"

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


/* Definitions for CalibrationTask */
osThreadId_t CalibrationTaskHandle;
const osThreadAttr_t CalibrationTask_attributes = {
  .name = "CalibrationTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
FSMStruct state;

/* foc control setup*/
foc_t hfoc;



motor_config_t m_config;

/* usart setup*/


/* can setup*/
CANTxMessage can_tx;
CANRxMessage can_rx;

/* joint setup*/
JointRobot_t robot_joints;


// test 
AS5048A_t my_as5048a_hw;



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

int torque_control_update(void) {
  int ret = 0;
  static uint8_t event_speed_loop_count = 0;
	static float rpm_temp = 0.0f;
	
  
	foc_current_control_update(&hfoc);
//	foc_get_mech_degree(&hfoc);

  if (event_speed_loop_count > SPEED_CONTROL_CYCLE) {
    event_speed_loop_count = 0;
    dt_us = get_dt_us();
    float rpm_encd = ENCODER_GetRPM(&encoder, dt_us);
    foc_calc_mech_rpm_encoder(&hfoc, rpm_encd);
    foc_set_flag();
    ret = 1;
  }

  event_speed_loop_count++;

  return ret;
}

void control_init(void) {
	hfoc.angle_filtered = &encoder.angle_filtered;

  // Id PI parameter
  pid_reset(&hfoc.id_ctrl);
  pid_set_ts(&hfoc.id_ctrl, FOC_TS);
  pid_set_kp(&hfoc.id_ctrl, m_config.id_kp);
  pid_set_ki(&hfoc.id_ctrl, m_config.id_ki);
  pid_set_out_constraint(&hfoc.id_ctrl, m_config.id_out_max, -m_config.id_out_max);
  pid_set_deadband(&hfoc.id_ctrl, m_config.id_e_deadband);
  // Id PI parameter
  pid_reset(&hfoc.iq_ctrl);
  pid_set_ts(&hfoc.iq_ctrl, FOC_TS);
  pid_set_kp(&hfoc.iq_ctrl, m_config.iq_kp);
  pid_set_ki(&hfoc.iq_ctrl, m_config.iq_ki);
  pid_set_out_constraint(&hfoc.iq_ctrl, m_config.iq_out_max, -m_config.iq_out_max);
  pid_set_deadband(&hfoc.iq_ctrl, m_config.iq_e_deadband);
  // Speed PID parameter
  pid_reset(&hfoc.speed_ctrl);
  pid_set_ts(&hfoc.speed_ctrl, FOC_TS * SPEED_CONTROL_CYCLE);
  pid_set_kp(&hfoc.speed_ctrl, m_config.speed_kp);
  pid_set_ki(&hfoc.speed_ctrl, m_config.speed_ki);
  pid_set_kd(&hfoc.speed_ctrl, 0.0001f);
  pid_set_d_filter_fc(&hfoc.speed_ctrl, 100.0f);
  pid_set_max_d(&hfoc.speed_ctrl, 10.0f);
  pid_set_out_constraint(&hfoc.speed_ctrl, m_config.speed_out_max, -m_config.speed_out_max);
  pid_set_deadband(&hfoc.speed_ctrl, m_config.speed_e_deadband);
  // Position PID parameter
  pid_reset(&hfoc.pos_ctrl);
  pid_set_ts(&hfoc.pos_ctrl, FOC_TS * SPEED_CONTROL_CYCLE);
  pid_set_kp(&hfoc.pos_ctrl, m_config.pos_kp);
  pid_set_ki(&hfoc.pos_ctrl, m_config.pos_ki);
  pid_set_kd(&hfoc.pos_ctrl, m_config.pos_kd);
  pid_set_d_filter_fc(&hfoc.pos_ctrl, 20.0f);
  pid_set_max_d(&hfoc.pos_ctrl, 100.0f);
  pid_set_out_constraint(&hfoc.pos_ctrl, m_config.pos_out_max, -m_config.pos_out_max);
  pid_set_deadband(&hfoc.pos_ctrl, m_config.pos_e_deadband);


	
	foc_pwm_init(&hfoc, &(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3), hfoc.drv8323s.pwm_resolution);
  foc_motor_init(&hfoc, POLE_PAIR, 750);
  foc_sensor_init(&hfoc, m_config.encd_offset, REVERSE_DIR);
  foc_gear_reducer_init(&hfoc, 1.0);
  foc_set_limit_current(&hfoc, 10.0);
 
}
uint8_t Serial2RxBuffer[1];

float __float_reg[64];
int __int_reg[256];

float sPoint_Vel = 50.0f;
float sPoint_Pos = 0;
float sPoint_Tor = 0;
volatile uint8_t is_calibrating = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
void StartCalibrationTask(void *argument);

/* USER CODE BEGIN PFP */

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

  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  MX_DMA_Init();

	MX_SPI1_Init();
	MX_SPI2_Init();
  MX_USART6_UART_Init();

  ADC1_DMA_CONFIG3();

  TIM1_PWM_DMA_CONFIG();
	TIM_IQR_MS_CONGFIG();
  TIM_COUNT_US_CONFIG();


  /* Sanitize configs in case flash is empty*/
  if(E_ZERO==-1){E_ZERO = 0;}
  if(M_ZERO==-1){M_ZERO = 0;}
  if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
  if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
  if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}
  if(CAN_ID==-1){CAN_ID = 1;}
  if(CAN_MASTER==-1){CAN_MASTER = 0;}
  if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
  if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
  if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
  if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
  if(isnan(I_CAL)||I_CAL==-1){I_CAL = 5.0f;}
  if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 21.0f;}
  if(isnan(GR) || GR==-1){GR = 1.0f;}
  if(isnan(KT) || KT==-1){KT = 1.0f;}
  if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
  if(isnan(KP_MIN) || KP_MIN==-1){KP_MIN = 0.0f;}
  if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 5.0f;}
  if(isnan(KD_MIN) || KD_MIN==-1){KD_MIN = 0.0f;}
  if(isnan(P_MAX)){P_MAX = 12.5f;}
  if(isnan(P_MIN)){P_MIN = -12.5f;}
  if(isnan(V_MAX)){V_MAX = 65.0f;}
  if(isnan(V_MIN)){V_MIN = -65.0f;}

  
  // Load Config 
  flash_read_config(&m_config);

  init_trig_lut();

	bldc_config(&hfoc.drv8323s); 
	control_init();
	
	/*AS5048A setup   */
//	Encoder_Init_Manual_Test();

	ENCODER_Setup();
	ENCODER_AutoDetect();
	
  /* Start the FSM */
  state.state = MENU_MODE;
  state.next_state = MENU_MODE;
  state.ready = 1;

  HAL_UART_Receive_IT(&huart, (uint8_t *)Serial2RxBuffer, 1);

  // Enable Timer
  TIM_COUNTER_ENABLE(TIM10);
	TIM_COUNTER_ENABLE(TIM3);
  TIM_COUNTER_ENABLE(TIM1); 


  // Output PWM
  DRV8323_Start_PWM(&hfoc.drv8323s); //  MOE và CCxE

  // // Calibration
  // hfoc.control_mode = CALIBRATION_MODE;
	// is_calibrating = 1;
  // foc_cal_encoder(&hfoc);
	// is_calibrating = 0;
	hfoc.control_mode = POSITION_CONTROL_MODE;
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
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	
	/* DMA interrupt init spi1 */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	
	/* DMA interrupt init uart*/
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
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


/*===========================================================*/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2)
	{
		foc_sensored_calc_electric_angle(&hfoc);

	}
}
/*===========================================================*/
void ADC_IRQHandler(void) {

	if (ADC1->SR & ADC_SR_JEOC) {
		ADC1->SR &= ~ADC_SR_JEOC;
	

		if (ENCODER_GetFlag()) {
      ENCODER_Reset_Flag();
      encoder.start_read(encoder.hw_encoder);
		}
    run_fsm(&state);

		
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCalibrationTask */
/**
  * @brief  Function implementing the CalibrationTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCalibrationTask */
void StartCalibrationTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    // if(!hfoc.done_orderphase){foc_auto_calibration(&hfoc);}
		// if(!hfoc.done_cal_encoder){foc_cal_encoder(&hfoc);}
            
    osDelay(10);
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
