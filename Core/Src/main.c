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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "uart.h"
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

/* USER CODE BEGIN PV */
FSMStruct state;
/* foc control setup*/
foc_t hfoc;

/* drv8323rs setup*/
DRV8323_t bldc;

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

static int torque_control_update(void) {
  int ret = 0;
  static uint8_t event_speed_loop_count = 0;
	static float rpm_temp = 0.0f;
	
  DRV8323_Get_Current(&bldc, &hfoc.ia, &hfoc.ib,&hfoc.ic);
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

	
  pid_init(&hfoc.id_ctrl, m_config.id_kp, m_config.id_ki, 0.0f, FOC_TS, m_config.id_out_max, m_config.id_e_deadband);
  pid_init(&hfoc.iq_ctrl, m_config.iq_kp, m_config.iq_ki, 0.0f, FOC_TS, m_config.iq_out_max, m_config.iq_e_deadband);
  pid_init(&hfoc.speed_ctrl, m_config.speed_kp, m_config.speed_ki, 0.0f, FOC_TS * SPEED_CONTROL_CYCLE, m_config.speed_out_max, m_config.speed_e_deadband);
	pid_init(&hfoc.pos_ctrl, m_config.pos_kp, m_config.pos_ki, m_config.pos_kd, FOC_TS * SPEED_CONTROL_CYCLE, m_config.pos_out_max, m_config.pos_e_deadband);
  hfoc.pos_ctrl.d_alpha_filter = 0.85;
	
	foc_pwm_init(&hfoc, &(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3), bldc.pwm_resolution);
  foc_motor_init(&hfoc, POLE_PAIR, 750);
  foc_sensor_init(&hfoc, m_config.encd_offset, NORMAL_DIR);
  foc_gear_reducer_init(&hfoc, 1.0);
  foc_set_limit_current(&hfoc, 10.0);
 
}
uint8_t Serial2RxBuffer[1];

float __float_reg[64];
int __int_reg[256];

float sPoint_Vel = 0;
float sPoint_Pos = 0;
float sPoint_Tor = 0;
volatile uint8_t is_calibrating = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
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
  MX_GPIO_Init();
  MX_DMA_Init();

	MX_SPI1_Init();
	MX_SPI2_Init();
  MX_USART6_UART_Init();


  ADC1_DMA_CONFIG3();

  TIM1_PWM_DMA_CONFIG();
	TIM_IQR_MS_CONGFIG();
  TIM_COUNT_US_CONFIG();
  
  // Load Config 
  flash_read_config(&m_config);

  init_trig_lut();

	bldc_config(&bldc); 
	control_init();
	
	/*AS5048A setup   */
//	Encoder_Init_Manual_Test();

	ENCODER_Setup();
	ENCODER_AutoDetect();
	
  
  // Enable Timer
  TIM_COUNTER_ENABLE(TIM10);
	TIM_COUNTER_ENABLE(TIM3);
  TIM_COUNTER_ENABLE(TIM1); 


  // Output PWM
  DRV8323_Start_PWM(&bldc); //  MOE và CCxE

  // Calibration
  hfoc.control_mode = CALIBRATION_MODE;
	is_calibrating = 1;
  foc_cal_encoder(&hfoc);
	is_calibrating = 0;
	hfoc.control_mode = POSITION_CONTROL_MODE;



  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	
	/* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	
	/* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
uint32_t crc_check, crc_rev;
float k = -1.0f, p = 0.1f ;
//void USART6_IRQHandler(void){
//	HAL_UART_IRQHandler(&huart);
	
//	if(usart_rx.rx_buff[ID_HEADER] == HEADER){

//		memcpy(&crc_rev, &usart_rx.rx_buff[ID_CRC], 4);

//		switch(usart_rx.rx_buff[ID_MODE]){
//			case TORQUE_CONTROL_MODE: {
//				hfoc.control_mode = TORQUE_CONTROL_MODE;
//				memcpy(&sPoint_Tor, &usart_rx.rx_buff[ID_TOR], 4);
//			break;
//			}
//			case SPEED_CONTROL_MODE: {
//				hfoc.control_mode = SPEED_CONTROL_MODE;
//				memcpy(&sPoint_Vel, &usart_rx.rx_buff[ID_VEL], 4);
//				memcpy(&hfoc.iq_ctrl.out_max, &usart_rx.rx_buff[ID_TOR], 4);
//				break;
//			}
//			case POSITION_CONTROL_MODE: {
//				hfoc.control_mode = POSITION_CONTROL_MODE;
//				memcpy(&sPoint_Pos, &usart_rx.rx_buff[ID_POS], 4);
//				memcpy(&hfoc.pos_ctrl.out_max, &usart_rx.rx_buff[ID_VEL], 4);
//				memcpy(&hfoc.iq_ctrl.out_max, &usart_rx.rx_buff[ID_TOR], 4);
//				break;
//			}
////			case POSITION_CONTROL_MODE: {
////				hfoc.control_mode = TORQUE_CONTROL_MODE;
////				memcpy(&sPoint_Pos, &usart_rx.rx_buff[ID_POS], 4);
//////				sPoint_Tor = k*(sPoint_Pos - hfoc.actual_angle);
////				
////				break;
////			}
//			default:{
//				
//				break;
//			}
//		}
//	}


//}
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

		if (is_calibrating == 1) {
			return;
		}
		
		hfoc.v_bus = 19.420f; 
//		hfoc.v_bus = 12.4f; 

		switch (hfoc.control_mode) {
      case TORQUE_CONTROL_MODE: {
				float deg_encd = ENCODER_GetActualDegree(&encoder);
        foc_calc_mech_pos_encoder(&hfoc, deg_encd);
				
				sPoint_Tor = k*(sPoint_Pos - hfoc.actual_angle) + p*hfoc.actual_rpm ;
        hfoc.id_ref = 0.0f;
        hfoc.iq_ref = sPoint_Tor;
        torque_control_update();
        break;
			}
			case POSITION_CONTROL_MODE: {
        if (torque_control_update() == 1) {
          float deg_encd = ENCODER_GetActualDegree(&encoder);
          foc_calc_mech_pos_encoder(&hfoc, deg_encd);
          foc_position_control_update(&hfoc, sPoint_Pos);
        }
        break;
			}
			
			case SPEED_CONTROL_MODE: {
        if (torque_control_update() == 1) {
          foc_speed_control_update(&hfoc, sPoint_Vel);

        }
        break;

			default:
				
				break;
			}
		}
		run_fsm(&state);
		
	}
}
/* USER CODE END 4 */

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
