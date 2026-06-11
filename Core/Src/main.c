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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

osThreadId_t ErrorTaskHandle;
const osThreadAttr_t ErrorTask_attributes = {
  .name = "ErrorTask",
  .stack_size = 512 * 4, // Cấp phát 512 bytes RAM cho ngăn xếp (Stack)
  .priority = (osPriority_t) osPriorityNormal, // Mức ưu tiên
};

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



void control_init(void) {
	hfoc.angle_filtered = &encoder.angle_filtered;

  // Id PI parameter
  pid_reset(&hfoc.id_ctrl);
  pid_set_ts(&hfoc.id_ctrl, FOC_TS);
  pid_set_kp(&hfoc.id_ctrl, m_config.id_kp);
  pid_set_ki(&hfoc.id_ctrl, m_config.id_ki);
  pid_set_out_constraint(&hfoc.id_ctrl, V_MAX_VOLTAGE, -V_MAX_VOLTAGE);
  pid_set_deadband(&hfoc.id_ctrl, m_config.id_e_deadband);
  // Id PI parameter
  pid_reset(&hfoc.iq_ctrl);
  pid_set_ts(&hfoc.iq_ctrl, FOC_TS);
  pid_set_kp(&hfoc.iq_ctrl, m_config.iq_kp);
  pid_set_ki(&hfoc.iq_ctrl, m_config.iq_ki);
  pid_set_out_constraint(&hfoc.iq_ctrl, V_MAX_VOLTAGE, -V_MAX_VOLTAGE);
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


	
	foc_pwm_init(&hfoc, &(TIM1->CCR3), &(TIM1->CCR2), &(TIM1->CCR1), hfoc.drv8323s.pwm_resolution);

  foc_set_limit_current(&hfoc, 15.0);
 
}
uint8_t Serial2RxBuffer[1];





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
  MX_DMA_Init();

	MX_SPI1_Init();
	MX_SPI2_Init();
  MX_USART6_UART_Init();
  // MX_CAN2_Init();


  ADC1_DMA_CONFIG3();

  TIM1_PWM_DMA_CONFIG();
	TIM_IQR_MS_CONGFIG();
  TIM_COUNT_US_CONFIG();


  
  // Load Config 
  flash_read_config(&m_config);
  flash_erase_ready();

  init_trig_lut();

	bldc_config(&hfoc.drv8323s); 
	control_init();
	
	/*ENCODER setup   */
	ENCODER_Setup();
  ENCODER_Init_From_Config();

	
  /* Start the FSM */
  state.state = MENU_MODE;
  state.next_state = STATE;
  state.ready = 1;

  hfoc.control_mode = CONTROL_MODE;

  HAL_UART_Receive_IT(&huart, (uint8_t *)Serial2RxBuffer, 1);

  // Enable Timer
  TIM_COUNTER_ENABLE(TIM10);
	TIM_COUNTER_ENABLE(TIM3);
  TIM_COUNTER_ENABLE(TIM1); 


  // Output PWM
  DRV8323_Start_PWM(&hfoc.drv8323s); //  MOE và CCxE

  
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
	if (hspi->Instance == SPI2 || hspi->Instance == SPI1)
	{
		foc_sensored_calc_electric_angle(&hfoc);

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
	
		if (ENCODER_GetFlag()) {
      ENCODER_Reset_Flag();
      encoder.start_read(&encoder);
		}

    foc_get_power_voltage(&hfoc);

    run_fsm(&state);
	}
}

void ErrorTask(void *argument)
{

  osDelay(3000);

  for(;;)
  {
    if(encoder.is_connected == 0) printf("Encoder not detected. Please check the connection.\r\n");
    if (POWER_FLAG == 1) printf("Power voltage is too low: %.2f V\r\n", hfoc.v_bus);
    

    osDelay(1000); 
  }
}

void StartCalibrationTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  hfoc.done_orderphase = 1;
  hfoc.done_cal_encoder = 1;
  float tx[3] = {0};
  osDelay(3000);
  /* Infinite loop */
  for(;;)
  {   
    switch (state.state) {
      case CALIBRATION_MODE: {
        ENCODER_AutoDetect();
        ENCODER_CHECK();
        DRV8323_Calibrate_Current_Offset(&hfoc.drv8323s);
        foc_auto_calibration(&hfoc);
        foc_cal_encoder(&hfoc);
        
        calibration_seq();
        if(hfoc.done_cal_encoder == 1 && hfoc.done_orderphase == 1){
          printf("Calibration Successful!\r\n");
          update_fsm(&state, 27);
        }
        break;
      }
      
      case TEST_MODE: {
        TestModeView();
        osDelay(9);
        break;
      }
      case MENU_MODE: {
      
        break;
      }
      case ENCODER_MODE: {
        // printf("Encoder Angle: %.2f\n\r", hfoc.actual_angle);
        // ENCODER_AutoDetect();
        ENCODER_CHECK();
        update_fsm(&state, 27);
        break;
      }
      default:
                  
        break;
              
    }
    
    
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
