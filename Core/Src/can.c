/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "hw_config.h"

/* USER CODE BEGIN 0 */

#include "math_ops.h"



/* USER CODE END 0 */

CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan2.Instance = CAN1;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

	/* USER CODE END CAN2_MspInit 0 */
	/* CAN1 clock enable */
	__HAL_RCC_CAN2_CLK_ENABLE();
  __HAL_RCC_CAN1_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**CAN1 GPIO Configuration
	PA11     ------> CAN1_RX
	PA12     ------> CAN1_TX
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* CAN1 interrupt Init */
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
	

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    CAN_RX_PIN     ------> CAN1_RX
    CAN_TX_PIN     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11);
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void can_rx_init(CANRxMessage *msg){
	msg->filter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 
	msg->filter.FilterBank = 14;               
	msg->filter.SlaveStartFilterBank = 14;
	msg->filter.FilterIdHigh=0x000<<5; 				// CAN ID
	msg->filter.FilterIdLow=0x000;
	msg->filter.FilterMaskIdHigh=0x000<<5;
	msg->filter.FilterMaskIdLow=0x000;
	msg->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	msg->filter.FilterScale=CAN_FILTERSCALE_32BIT;
	msg->filter.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan2, &msg->filter);
}
//

void can_tx_init(CANTxMessage *msg){
	msg->tx_header.DLC = 8; 			
	msg->tx_header.IDE=CAN_ID_STD; 		
	msg->tx_header.RTR=CAN_RTR_DATA; 
}
//


/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void pack_cmd(CANTxMessage *msg, JointCommand_t *cmd){
	int p_int  = float_to_uint(cmd->p_des, P_MIN, P_MAX, 16);
  int v_int  = float_to_uint(cmd->v_des, V_MIN, V_MAX, 12);
	int kp_int = float_to_uint(cmd->kp, KP_MIN, KP_MAX, 12);
	int kd_int = float_to_uint(cmd->kd, KD_MIN, KD_MAX, 12);
	int t_int  = float_to_uint(cmd->t_ff, T_MIN, T_MAX, 12);
	
	msg->tx_header.StdId = cmd->can_id;
	
	msg->data[0] = p_int >> 8;
	msg->data[1] = p_int & 0xFF;
	msg->data[2] = v_int >> 4;
	msg->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
	msg->data[4] = kp_int & 0xFF;
	msg->data[5] = kd_int >> 4;
	msg->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
	msg->data[7] = t_int & 0xFF;
}
/// unpack_cmd Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void unpack_state(CANRxMessage *msg, JointState_t *state){// ControllerStruct * controller){
	int p_int = (msg->data[1] << 8) | msg->data[2];	
	int v_int = (msg->data[3] << 4) | (msg->data[4] >> 4);
	int t_int = ((msg->data[4] & 0xF) << 8) | msg->data[5];
	int vb_int = msg->data[6];

	state->p_act  = uint_to_float(p_int, P_MIN, P_MAX, 16);
	state->v_act  = uint_to_float(v_int, V_MIN, V_MAX, 12);
	state->t_act  = uint_to_float(t_int, T_MIN, T_MAX, 12);
	state->v_batt = uint_to_float(vb_int, VB_MIN, VB_MAX, 8);

}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
























