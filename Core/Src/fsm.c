/*
 * fsm.cpp
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */
#include "main.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "fsm.h"
#include "hw_config.h"
#include "user_config.h"
#include "foc_utils.h"
#include "encoder.h"
#include "flash.h"
#include "uart.h"


extern UART_HandleTypeDef huart;
extern JointCommand_t joint_cmd;
// extern JointRobot_t robot_joints;
extern float sPoint_Vel ;
extern float sPoint_Pos ;
extern float sPoint_Tor;



void run_fsm(FSMStruct * fsmstate){
	/* run_fsm is run every commutation interrupt cycle */

	/* state transition management */
	if(fsmstate->next_state != fsmstate->state){
		fsm_exit_state(fsmstate);		// safely exit the old state
		if(fsmstate->ready){			// if the previous state is ready, enter the new state
			fsmstate->state = fsmstate->next_state;
			fsm_enter_state(fsmstate);
		}
	}

	switch(fsmstate->state){
		case MENU_MODE:
			
			break;

		case CALIBRATION_MODE:
			meas_inj_dq_process(&hfoc, FOC_TS);
			break;

		case MOTOR_MODE:
			/* If CAN has timed out, reset all commands */
			foc_control_loop(&hfoc);
			break;

		case TEST_MODE:
			/* If CAN has timed out, reset all commands */
			
			foc_control_loop(&hfoc);
			break;
		case SETUP_MODE:
			break;

		case ENCODER_MODE:
			
			break;

		case SET_PID_MODE:
			break;
	}

}

void fsm_enter_state(FSMStruct * fsmstate){
	/* Called when entering a new state
	* Do necessary setup   */
	STATE = fsmstate->state;
	NEXT_STATE = fsmstate->next_state;
	switch(fsmstate->state){
			case MENU_MODE:
			//printf("Entering Main Menu\r\n");
			MOTOR_RUNNING = 0;
			DRV8323RS_Disnable;
			enter_menu_state();
			break;
		case SETUP_MODE:
			printf("Entering Setup\r\n");
			enter_setup_state();
			break;
		case TEST_MODE:
			DRV8323RS_Disnable;
			MOTOR_RUNNING = 0;
			enter_test_state();
			break;
		case ENCODER_MODE:
			//printf("Entering Encoder Mode\r\n");
			break;
		case MOTOR_MODE:
			printf("Entering Motor Mode\r\n");
			DRV8323RS_Enable;
			MOTOR_RUNNING = 1;
			break;
		case CALIBRATION_MODE:
			printf("Starting Calibration Mode\r\n");
			DRV8323RS_Enable;
            foc_start_calibration(&hfoc);  
			break;

	}
}

void fsm_exit_state(FSMStruct * fsmstate){
	/* Called when exiting the current state
	* Do necessary cleanup  */

	switch(fsmstate->state){
		case MENU_MODE:
			//printf("Leaving Main Menu\r\n");
			// DRV8323_Set_PWM(&hfoc.drv8323s, 0, 0, 0);
			fsmstate->ready = 1;
			break;
		case SETUP_MODE:
			printf("Leaving Setup Menu\r\n");
			SAVE_FLAG = 1;
			fsmstate->ready = 1;
			break;
		case TEST_MODE:
			HAL_UART_AbortTransmit(&huart);
			printf("Leaving Test Mode\r\n");
			// DRV8323_Set_PWM(&hfoc.drv8323s, 0, 0, 0);
			pid_reset(&hfoc.id_ctrl);
			pid_reset(&hfoc.iq_ctrl);
			SAVE_FLAG = 1;
			MOTOR_RUNNING = 0;
			
			fsmstate->ready = 1;
			break;
		case ENCODER_MODE:
			//printf("Leaving Encoder Mode\r\n");
			fsmstate->ready = 1;
			break;
		case MOTOR_MODE:
			/* Don't stop commutating if there are high currents or FW happening */
			// DRV8323_Set_PWM(&hfoc.drv8323s, 0, 0, 0);
			pid_reset(&hfoc.id_ctrl);
			pid_reset(&hfoc.iq_ctrl);
			fsmstate->ready = 1;
			break;
		case CALIBRATION_MODE:
			printf("Exiting Calibration Mode\r\n");
			fsmstate->ready = 1;
			break;
	}

}

void update_fsm(FSMStruct * fsmstate, char fsm_input){
	/*update_fsm is only run when new state-change information is received
	* on serial terminal input or CAN input
	*/
	if(fsm_input == MENU_CMD){	// escape to exit to rest mode
		fsmstate->next_state = MENU_MODE;
		fsmstate->ready = 0;
		return;
	}
	switch(fsmstate->state){
		case MENU_MODE:
			switch (fsm_input){
				case CAL_CMD:
					fsmstate->next_state = CALIBRATION_MODE;
					fsmstate->ready = 0;
					break;
				case MOTOR_CMD:
					fsmstate->next_state = MOTOR_MODE;
					fsmstate->ready = 0;
					break;
				case ENCODER_CMD:
					fsmstate->next_state = ENCODER_MODE;
					fsmstate->ready = 0;
					break;
				case SETUP_CMD:
					fsmstate->next_state = SETUP_MODE;
					fsmstate->ready = 0;
					break;
				case TEST_CMD:
					fsmstate->next_state = TEST_MODE;
					fsmstate->ready = 0;
					break;
				case ZERO_CMD:
					M_ZERO = ENCODER_GetActualDegree(&encoder); 
					
					SPOINT_POS = 0.0f;
					// hfoc.sPoint_Pos = 0.0f;
					printf("\n\r new zero position: %.3f deg\r\n", M_ZERO);
					
					break;
				}
			break;
		case SETUP_MODE:
			if(fsm_input == 10 || fsm_input == ' '){ 
        		break; 
    		}	
			if(fsm_input == ENTER_CMD){
				process_user_input(fsmstate);
				break;
			}
			if(fsmstate->bytecount == 0){fsmstate->cmd_id = fsm_input;}
			else{
				fsmstate->cmd_buff[fsmstate->bytecount-1] = fsm_input;
				fsmstate->bytecount = fsmstate->bytecount%(sizeof(fsmstate->cmd_buff)/sizeof(fsmstate->cmd_buff[0])); // reset when buffer is full
			}
			HAL_UART_Transmit(&huart, (uint8_t *)&fsm_input, 1, 2);
			fsmstate->bytecount++;
			/* If enter is typed, process user input */

			break;
		case TEST_MODE:
			if(fsm_input == 10 || fsm_input == ' '){ 
        		break; 
    		}	
			if(fsm_input == ENTER_CMD){
				set_pid_mode(fsmstate);
				break;
			}
			if(fsmstate->bytecount == 0){fsmstate->cmd_id = fsm_input;}
			else{
				fsmstate->cmd_buff[fsmstate->bytecount-1] = fsm_input;
				fsmstate->bytecount = fsmstate->bytecount%(sizeof(fsmstate->cmd_buff)/sizeof(fsmstate->cmd_buff[0])); // reset when buffer is full
			}
			HAL_UART_Transmit(&huart, (uint8_t *)&fsm_input, 1, 2);
			fsmstate->bytecount++;
			/* If enter is typed, process user input */

			break;
		

		case ENCODER_MODE:
			break;
		case MOTOR_MODE:
			switch (fsm_input){
				case ZERO_CMD:
					M_ZERO = ENCODER_GetActualDegree(&encoder); 
					
					SPOINT_POS = 0.0f;
					printf("\n\r new zero position: %.3f deg\r\n", M_ZERO);
					break;
			}
		break;
	}
//printf("FSM State: %d  %d\r\n", fsmstate.state, fsmstate.state_change);
}

void TestModeView(void) {
    if (CUR_VIEW == 0 && VEL_VIEW == 0 && POS_VIEW == 0) {
        return; 
    }

    static uint8_t tx_buf[18];

    tx_buf[0] = 0xAA; 
    tx_buf[1] = 0xBB;

    float id = (CUR_VIEW == 1) ? hfoc.id_filtered : 0.0f;
	float iq = (CUR_VIEW == 1) ? hfoc.iq_filtered : 0.0f;
    float vel = (VEL_VIEW == 1) ? hfoc.actual_rpm : 0.0f;
    float pos = (POS_VIEW == 1) ? hfoc.actual_angle : 0.0f;
	

    memcpy(&tx_buf[2],  &id, 4);
    memcpy(&tx_buf[6],  &iq, 4);
    memcpy(&tx_buf[10], &vel, 4);
    memcpy(&tx_buf[14], &pos, 4);

    HAL_UART_Transmit_DMA(&huart, tx_buf, 18);
}

void enter_test_state(void){

    printf("\r\n Test & PID Tuning Options \n\r");
    printf(" %-4s %-31s %-5s %-6s %-2s\r\n", "prefix", "parameter", "min", "max", "current value");

    printf("\r\n Active Setpoint:\r\n");
    float current_setpoint = (hfoc.control_mode == TORQUE_CONTROL_MODE) ? SPOINT_TOR : 
                             (hfoc.control_mode == SPEED_CONTROL_MODE) ? SPOINT_VEL : SPOINT_POS;
    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "t", "Target (Tor/Spd/Pos)", "-", "-", current_setpoint);
	printf(" %-4s %-31s %-5s %-6s %d\n\r", "o", "Control Mode(0:Tor,1:Spd,2:Pos,3:Imp)", "0", "3", CONTROL_MODE);
    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "l", "Current Limit (A)", "0.0", "75.0", I_MAX);
	printf(" %-4s %-31s %-5s %-6s %.2f\n\r", "p", "Voltage Limit (V)", "0.0", "40.0", V_MAX_VOLTAGE);
	printf(" %-4s %-31s %-5s %-6s %d\n\r", "m", "Motor On/Off", "0", "1", MOTOR_RUNNING);

	printf("\r\n Data View (0:Off, 1:On):\r\n");
	printf(" %-4s %-31s %-5s %-6s %d\n\r", "v0", "Current View", "0", "1", CUR_VIEW);
	printf(" %-4s %-31s %-5s %-6s %d\n\r", "v1", "Velocity View", "0", "1", VEL_VIEW);
	printf(" %-4s %-31s %-5s %-6s %d\n\r", "v2", "Position View", "0", "1", POS_VIEW);
	
	printf("\r\n Current Loop (Id/Iq):\r\n");
    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "a", "Current Kp", "0", "-", ID_KP);
    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "b", "Current Ki", "0", "-", ID_KI);

	if (hfoc.control_mode != IMPEDANCE_CONTROL_MODE) {
		printf("\r\n Speed Loop:\r\n");
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "c", "Speed Kp", "0", "-", SPEED_KP);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "d", "Speed Ki", "0", "-", SPEED_KI);

		printf("\r\n Position Loop:\r\n");
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "x", "Position Kp", "0", "-", POS_KP);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "y", "Position Ki", "0", "-", POS_KI);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "z", "Position Kd", "0", "-", POS_KD);

	}
	else {
		printf("\r\n Impedance Control:\r\n");
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "k", "Torque Constant (N-m/A)", "0", "-", KT);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "c", "Position Gain (N-m/rad)", "0", "-", joint_cmd.kp);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "d", "Velocity Gain (N-m-s/rad)", "0", "-", joint_cmd.kd);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "x", "Position (Rad)", "0", "-", joint_cmd.p_des);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "y", "Velocity (Rad/s)", "0", "-", joint_cmd.v_des);

	}
    printf(" \n\r To change a value, type 'prefix''value''ENTER'\n\r e.g. 'c0.15''ENTER'\r\n ");
    printf("VALUES UPDATE IMMEDIATELY IN TEST MODE! \n\r\n\r");

	
}

void set_pid_mode(FSMStruct * fsmstate){

	switch (fsmstate->cmd_id){
		case 't':{
			switch (hfoc.control_mode){
				case TORQUE_CONTROL_MODE:
					SPOINT_TOR = atof(fsmstate->cmd_buff);
					printf("\n\r  Updated torque Setpoint: %.3f\r\n", SPOINT_TOR);
					break;
				case SPEED_CONTROL_MODE:
					SPOINT_VEL = atof(fsmstate->cmd_buff);
					printf("\n\r  Updated velocity Setpoint: %.3f\r\n", SPOINT_VEL);
					break;
				case POSITION_CONTROL_MODE:
					SPOINT_POS = atof(fsmstate->cmd_buff);
					printf("\n\r  Updated position Setpoint: %.3f\r\n", SPOINT_POS);
					break;
			}
			break;
		}
		case 'o': {
            int mode_val = atoi(fsmstate->cmd_buff);
            if (mode_val == 0) {
                hfoc.control_mode = TORQUE_CONTROL_MODE;
                printf("Control Mode set to TORQUE (0)\r\n");
            } 
            else if (mode_val == 1) {
                hfoc.control_mode = SPEED_CONTROL_MODE;
                printf("Control Mode set to SPEED (1)\r\n");
            } 
            else if (mode_val == 2) {
                hfoc.control_mode = POSITION_CONTROL_MODE;
                printf("Control Mode set to POSITION (2)\r\n");
            } 
            else if (mode_val == 3) {
                hfoc.control_mode = IMPEDANCE_CONTROL_MODE;
                printf("Control Mode set to IMPEDANCE (3)\r\n");
            }
            else {
                printf("Invalid Mode! Please enter 0, 1, 2, or 3.\r\n");
            }
			CONTROL_MODE = hfoc.control_mode;
            break;
        }
		case 'm':{
			if(MOTOR_RUNNING != 1){
				MOTOR_RUNNING = 1;
				pid_reset(&hfoc.id_ctrl);
				pid_reset(&hfoc.iq_ctrl);	
				pid_reset(&hfoc.speed_ctrl);
				pid_reset(&hfoc.pos_ctrl);
				DRV8323RS_Enable;
				printf("Start motor.\r\n");
				break;
			}
			else{
				MOTOR_RUNNING = 0;
				pid_reset(&hfoc.id_ctrl);
				pid_reset(&hfoc.iq_ctrl);	
				pid_reset(&hfoc.speed_ctrl);
				pid_reset(&hfoc.pos_ctrl);
				DRV8323RS_Disnable;
				printf("Stop motor.\r\n");				
				break;
			}
			break;
		}
		case 'p':
			V_MAX_VOLTAGE = fmaxf(fminf(atof(fsmstate->cmd_buff), 40.0f), 0.0f);
			pid_reset(&hfoc.id_ctrl);
			pid_reset(&hfoc.iq_ctrl);
			pid_set_out_constraint(&hfoc.id_ctrl, V_MAX_VOLTAGE, -V_MAX_VOLTAGE);
			pid_set_out_constraint(&hfoc.iq_ctrl, V_MAX_VOLTAGE, -V_MAX_VOLTAGE);
			printf("\n\r  Updated max voltage setpoint: %.2f\r\n", V_MAX_VOLTAGE);
			break;

		case 'v':{
			int mode_view = atoi(fsmstate->cmd_buff);
			if (mode_view == 0){
				if(CUR_VIEW != 1){
					CUR_VIEW = 1;
					break;
				}
				else{
					CUR_VIEW = 0;
					break;
				}
			}
			if (mode_view == 1){
				if(VEL_VIEW != 1){
					VEL_VIEW = 1;
					break;
				}
				else{
					VEL_VIEW = 0;
					break;
				}
			}
			if (mode_view == 2){
				if(POS_VIEW != 1){
					POS_VIEW = 1;
					break;
				}
				else{
					POS_VIEW = 0;
					break;
				}
			}
			break;
		}
		case 'l':
			I_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 75.0f), 0.0f);
			foc_set_limit_current(&hfoc, I_MAX);
			printf("I_MAX set to %f\r\n", I_MAX);
			break;
		case 'k':
			KT = fmaxf(atof(fsmstate->cmd_buff), 0.0001f);	// Limit prevents divide by zero.  Seems like a reasonable LB?
			printf("KT set to %f\r\n", KT);
			break;

		case 'a':
			ID_KP = atof(fsmstate->cmd_buff);
			IQ_KP = ID_KP;
			hfoc.id_ctrl.kp = ID_KP;
			hfoc.iq_ctrl.kp = IQ_KP;
			printf("\n\r  Updated i kp: %.3f\r\n", ID_KP);
			break;
		case 'b':
			ID_KI = atof(fsmstate->cmd_buff);
			IQ_KI = ID_KI;
			hfoc.id_ctrl.ki = ID_KI;
			hfoc.iq_ctrl.ki = IQ_KI;
			printf("\n\r  Updated i ki: %.3f\r\n", ID_KI);
			break;
		case 'c':{
			if (hfoc.control_mode == IMPEDANCE_CONTROL_MODE){
				joint_cmd.kp = atof(fsmstate->cmd_buff);
				printf("\n\r  Updated position kp: %.3f\r\n", joint_cmd.kp);
				break;
			}
			SPEED_KP = atof(fsmstate->cmd_buff);
			hfoc.speed_ctrl.kp = SPEED_KP;
			printf("\n\r  Updated speed kp: %.3f\r\n", SPEED_KP);
			break;
		}
		case 'd':{
			if (hfoc.control_mode == IMPEDANCE_CONTROL_MODE){
				joint_cmd.kd = atof(fsmstate->cmd_buff);
				printf("\n\r  Updated position kd: %.3f\r\n", joint_cmd.kd);
				break;
			}
			SPEED_KI = atof(fsmstate->cmd_buff);
			hfoc.speed_ctrl.ki = SPEED_KI;
			printf("\n\r  Updated speed ki: %.3f\r\n", SPEED_KI);
			break;
		}
		case 'x':
			if (hfoc.control_mode == IMPEDANCE_CONTROL_MODE){
				joint_cmd.p_des = atof(fsmstate->cmd_buff);
				printf("\n\r  Updated position setpoint: %.3f\r\n", joint_cmd.p_des);
				break;
			}
			POS_KP = atof(fsmstate->cmd_buff);
			hfoc.pos_ctrl.kp = POS_KP;
			printf("\n\r  Updated position kp: %.3f\r\n", POS_KP);
			break;
		case 'y':
			if (hfoc.control_mode == IMPEDANCE_CONTROL_MODE){
				joint_cmd.v_des = atof(fsmstate->cmd_buff);
				printf("\n\r  Updated velocity setpoint: %.3f\r\n", joint_cmd.v_des);
				break;
			}
			POS_KI = atof(fsmstate->cmd_buff);
			hfoc.pos_ctrl.ki = POS_KI;
			printf("\n\r  Updated position ki: %.3f\r\n", POS_KI);
			break;
		case 'z':
			POS_KD = atof(fsmstate->cmd_buff);
			hfoc.pos_ctrl.kd = POS_KD;
			printf("\n\r  Updated position kd: %.3f\r\n", POS_KD);
			break;
		default:
			printf("\n\r '%c' Not a valid command prefix\n\r\n\r", fsmstate->cmd_id);
			break;

		}
	enter_test_state();
	fsmstate->bytecount = 0;
	fsmstate->cmd_id = 0;
	memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
}

void enter_menu_state(void){
	//drv.disable_gd();
	//reset_foc(&controller);
	//gpio.enable->write(0);
	DRV8323RS_Disnable;
	printf("\n\r\n\r");
	printf(" Commands:\n\r");
	printf(" m - Motor Mode\n\r");
	printf(" t - Test Mode\n\r");
	printf(" c - Calibrate\n\r");
	printf(" s - Setup\n\r");
	printf(" e - Display Encoder\n\r");
	printf(" z - Set Zero Position\n\r");
	printf(" esc - Exit to Menu\n\r");

	//gpio.led->write(0);
}
   
void enter_setup_state(void){
	printf("\r\n Configuration Options \n\r");
	printf(" %-4s %-31s %-5s %-6s %-2s\r\n", "prefix", "parameter", "min", "max", "current value");
	printf("\r\n Motor:\r\n");
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "g", "Gear Ratio", "0", "-", GR);
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "k", "Torque Constant (N-m/A)", "0", "-", KT);
	
	printf("\r\n Control:\r\n");
	printf(" %-4s %-31s %-5s %-6s %d\n\r", "o", "Control Mode(0:Tor,1:Spd,2:Pos,3:Imp)", "0", "3", CONTROL_MODE);
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "b", "Current Bandwidth (Hz)", "50", "2000", I_BW);
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "l", "Current Limit (A)", "0.0", "75.0", I_MAX);
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "p", "Max Position Setpoint (rad)", "-", "-", P_MAX);
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "v", "Max Velocity Setpoint (rad)/s", "-", "-", V_MAX);
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "x", "Max Position Gain (N-m/rad)", "0.0", "1000.0", KP_MAX);
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "d", "Max Velocity Gain (N-m/rad/s)", "0.0", "5.0", KD_MAX);
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
	//printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "h", "Temp Cutoff (C) (0 = none)", "0", "150", TEMP_MAX);
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "c", "Continuous Current (A)", "0.0", "40.0", I_MAX_CONT);
	printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "a", "Calibration Current (A)", "0.0", "2.0", I_CAL);
	printf("\r\n CAN:\r\n");
	printf(" %-4s %-31s %-5s %-6s %-5li\n\r", "i", "CAN ID", "0", "127", CAN_SID);
	printf(" %-4s %-31s %-5s %-6s %-5li\n\r", "m", "CAN TX ID", "0", "127", CAN_MID);
	printf(" %-4s %-31s %-5s %-6s %-5li\n\r", "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
	printf(" \n\r To change a value, type 'prefix''value''ENTER'\n\r e.g. 'b1000''ENTER'\r\n ");
	printf("VALUES NOT ACTIVE UNTIL POWER CYCLE! \n\r\n\r");
}

void process_user_input(FSMStruct * fsmstate){
	/* Collects user input from serial (maybe eventually CAN) and updates settings */	

	switch (fsmstate->cmd_id){
		case 'o': {
            int mode_val = atoi(fsmstate->cmd_buff);
            if (mode_val == 0) {
                hfoc.control_mode = TORQUE_CONTROL_MODE;
                printf("Control Mode set to TORQUE (0)\r\n");
            } 
            else if (mode_val == 1) {
                hfoc.control_mode = SPEED_CONTROL_MODE;
                printf("Control Mode set to SPEED (1)\r\n");
            } 
            else if (mode_val == 2) {
                hfoc.control_mode = POSITION_CONTROL_MODE;
                printf("Control Mode set to POSITION (2)\r\n");
            } 
            else if (mode_val == 3) {
                hfoc.control_mode = IMPEDANCE_CONTROL_MODE;
                printf("Control Mode set to IMPEDANCE (3)\r\n");
            }
            else {
                printf("Invalid Mode! Please enter 0, 1, 2, or 3.\r\n");
            }
			CONTROL_MODE = hfoc.control_mode;
            break;
        }
		case 'b':
			I_BW = fmaxf(fminf(atof(fsmstate->cmd_buff), 2000.0f), 50.0f);
			printf("I_BW set to %f\r\n", I_BW);
			break;
		case 'i':
			CAN_SID = atoi(fsmstate->cmd_buff);
			printf("CAN_SID set to %ld\r\n", CAN_SID);
			break;
		case 'm':
			CAN_MID = atoi(fsmstate->cmd_buff);
			printf("CAN_MID set to %ld\r\n", CAN_MID);
			break;
		case 't':
			CAN_TIMEOUT = atoi(fsmstate->cmd_buff);
			printf("CAN_TIMEOUT set to %ld\r\n", CAN_TIMEOUT);
			break;

		case 'l':
			I_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 75.0f), 0.0f);
			pid_set_out_constraint(&hfoc.id_ctrl, I_MAX, -I_MAX);
			pid_set_out_constraint(&hfoc.iq_ctrl, I_MAX, -I_MAX);
			printf("I_MAX set to %f\r\n", I_MAX);
			break;
		// case 'f':
		// 	I_FW_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 33.0f), 0.0f);
		// 	printf("I_FW_MAX set to %f\r\n", I_FW_MAX);
		// 	break;

		// case 'h':
		// 	TEMP_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 150.0f), 0.0f);
		// 	printf("TEMP_MAX set to %f\r\n", TEMP_MAX);
		// 	break;
		// case 'c':
		// 	I_MAX_CONT = fmaxf(fminf(atof(fsmstate->cmd_buff), 40.0f), 0.0f);
		// 	printf("I_MAX_CONT set to %f\r\n", I_MAX_CONT);
		// 	break;
		case 'a':
			I_CAL = fmaxf(fminf(atof(fsmstate->cmd_buff), 20.0f), 0.0f);
			printf("I_CAL set to %f\r\n", I_CAL);
			break;
		case 'g':
			GR = fmaxf(atof(fsmstate->cmd_buff), .001f);	// Limit prevents divide by zero if user tries to enter zero
			printf("GR set to %f\r\n", GR);
			break;
		case 'k':
			KT = fmaxf(atof(fsmstate->cmd_buff), 0.0001f);	// Limit prevents divide by zero.  Seems like a reasonable LB?
			printf("KT set to %f\r\n", KT);
			break;
		case 'x':
			KP_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			printf("KP_MAX set to %f\r\n", KP_MAX);
			break;
		case 'd':
			KD_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			printf("KD_MAX set to %f\r\n", KD_MAX);
			break;
		case 'p':
			P_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			P_MIN = -P_MAX;
			printf("P_MAX set to %f\r\n", P_MAX);
			break;
		case 'v':
			V_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			V_MIN = -V_MAX;
			printf("V_MAX set to %f\r\n", V_MAX);
			break;
		default:
			 printf("\n\r '%c' Not a valid command prefix\n\r\n\r", fsmstate->cmd_id);
			break;

		}

	enter_setup_state();
	

	fsmstate->bytecount = 0;
	fsmstate->cmd_id = 0;
	memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
}

void enter_motor_mode(void){

}

