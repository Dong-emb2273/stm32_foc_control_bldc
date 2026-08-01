/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "stm32f4xx_hal_can.h"
#include "joint_acc.h"
#include "fsm.h"
#include "foc_utils.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */
// #define P_MIN -12.5f
// #define P_MAX 12.5f
// #define V_MIN -65.0f
// #define V_MAX 65.0f
// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f
// #define T_MIN -18.0f
// #define T_MAX 18.0f
// #define VB_MIN 0.0f
// #define VB_MAX 40.0f

// #define I_MAX 30.0f
// #define KT 1.0f
// #define GR 1.0f
// #define SENSE_BUFFER 0.0f

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct{
	uint8_t data[8];
	CAN_RxHeaderTypeDef rx_header;
	CAN_FilterTypeDef filter;
}CANRxMessage ;

typedef struct{
	uint8_t data[8];
	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;



extern CAN_HandleTypeDef hcan1;
extern CANTxMessage can_tx;
extern CANRxMessage can_rx;

void can_rx_init(CANRxMessage *msg);
void can_tx_init(CANTxMessage *msg);
void Master_Pack_Cmd(CANTxMessage *msg, JointCommand_t *cmd);
void Master_Unpack_State(CANRxMessage *msg, JointState_t *state);
void Slave_Pack_State(CANTxMessage *msg, JointState_t *state, foc_t *hfoc);
void Slave_Unpack_Cmd(CANRxMessage *msg, JointCommand_t *cmd);

void Slave_Pack_State_2(CANTxMessage *msg, JointState_t *state, foc_t *hfoc);
void Slave_Unpack_Cmd_2(CANRxMessage *msg, JointCommand_t *cmd);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/