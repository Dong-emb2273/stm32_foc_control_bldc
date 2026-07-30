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
#include "user_config.h"

/* USER CODE BEGIN 0 */

#include "math_ops.h"



/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
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
  /* USER CODE BEGIN CAN1_MspInit 0 */

	/* USER CODE END CAN1_MspInit 0 */
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
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
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
	msg->filter.FilterBank = 0;               
	msg->filter.SlaveStartFilterBank = 14;
	msg->filter.FilterIdHigh=CAN_SID<<5; 				// CAN ID
	msg->filter.FilterIdLow=0x000;
	msg->filter.FilterMaskIdHigh=0x000<<5;
	msg->filter.FilterMaskIdLow=0x000;
	msg->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	msg->filter.FilterScale=CAN_FILTERSCALE_32BIT;
	msg->filter.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &msg->filter);
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
/// 12 bit velocity command, between -30 and 30 rad/s
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
/* =========================================================================
 * CÁC HÀM DÀNH CHO MASTER
 * ========================================================================= */

// MASTER GỬI LỆNH: Đóng gói dữ liệu điều khiển để gửi xuống Slave
void Master_Pack_Cmd(CANTxMessage *msg, JointCommand_t *cmd) {
    int p_int  = float_to_uint(cmd->p_des, P_MIN, P_MAX, 16);
    int v_int  = float_to_uint(cmd->v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(cmd->kp, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(cmd->kd, KD_MIN, KD_MAX, 12);
    int t_int  = float_to_uint(cmd->t_ff, T_MIN, T_MAX, 12);
    
    msg->tx_header.StdId = cmd->can_id; // ID Slave

    msg->data[0] = p_int >> 8;
    msg->data[1] = p_int & 0xFF;
    msg->data[2] = v_int >> 4;
    msg->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg->data[4] = kp_int & 0xFF;
    msg->data[5] = kd_int >> 4;
    msg->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg->data[7] = t_int & 0xFF;
}

void Master_Unpack_State(CANRxMessage *msg, JointState_t *state) {
    // Byte [0] thường chứa Slave ID, ta bắt đầu giải mã từ Byte [1]
    int p_int  = (msg->data[1] << 8) | msg->data[2];    
    int v_int  = (msg->data[3] << 4) | (msg->data[4] >> 4);
    int t_int  = ((msg->data[4] & 0xF) << 8) | msg->data[5];
    int vb_int = msg->data[6];

    // Chuyển đổi số nguyên ngược lại thành số thực
    state->can_id = msg->data[0]; // Lấy ID từ Byte [0]
    state->p_act  = uint_to_float(p_int, P_MIN, P_MAX, 16);
    state->v_act  = uint_to_float(v_int, V_MIN, V_MAX, 12);
    state->t_act  = uint_to_float(t_int, T_MIN, T_MAX, 12);
    state->v_batt = uint_to_float(vb_int, VB_MIN, VB_MAX, 8);
}
/// unpack_cmd Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
/* =========================================================================
 * CÁC HÀM DÀNH CHO SLAVE (MOTOR CONTROLLER)
 * ========================================================================= */

// SLAVE NHẬN LỆNH: Giải mã gói tin điều khiển từ Master
void Slave_Unpack_Cmd(CANRxMessage *msg, JointCommand_t *cmd) {
  int p_int  = (msg->data[0] << 8) | msg->data[1];
  int v_int  = (msg->data[2] << 4) | (msg->data[3] >> 4);
  int kp_int = ((msg->data[3] & 0xF) << 8) | msg->data[4];
  int kd_int = (msg->data[5] << 4) | (msg->data[6] >> 4);
  int t_int  = ((msg->data[6] & 0xF) << 8) | msg->data[7];

  cmd->can_id = msg->rx_header.StdId; // Lấy ID từ gói tin nhận được

  cmd->p_des = uint_to_float(p_int, P_MIN, P_MAX, 16);
  cmd->v_des = uint_to_float(v_int, V_MIN, V_MAX, 12);
  cmd->kp    = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
  cmd->kd    = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
  cmd->t_ff  = uint_to_float(t_int, T_MIN, T_MAX, 12);

  SPOINT_POS = RAD_TO_DEG(cmd->p_des)*GR;
  SPOINT_VEL = RADS_TO_RPM(cmd->v_des)*GR; 

}

void Slave_Pack_State(CANTxMessage *msg, JointState_t *state, foc_t *hfoc) {
  state->p_act  = DEG_TO_RAD(hfoc->actual_angle)/GR; // rad
  state->v_act  = RPM_TO_RADS(hfoc->actual_rpm)/GR; // rad/s
  state->t_act  = hfoc->iq*KT; //

  int p_int  = float_to_uint(state->p_act, P_MIN, P_MAX, 16);
  int v_int  = float_to_uint(state->v_act, V_MIN, V_MAX, 12);
  int t_int  = float_to_uint(state->t_act, T_MIN, T_MAX, 12);
  int vb_int = float_to_uint(state->v_batt, VB_MIN, VB_MAX, 8);
  
  msg->tx_header.StdId = CAN_MID; // ID Master
  
  // Đóng gói theo chuẩn MIT Cheetah (5 bytes dữ liệu + 1 byte ID)
  msg->data[0] = CAN_SID;                 // Byte 0 chứa ID của động cơ
  msg->data[1] = p_int >> 8;
  msg->data[2] = p_int & 0xFF;
  msg->data[3] = v_int >> 4;
  msg->data[4] = ((v_int & 0xF) << 4) | (t_int >> 8);
  msg->data[5] = t_int & 0xFF;
  msg->data[6] = vb_int;
  msg->data[7] = 0x00;                     // Byte 7 không sử dụng
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
























