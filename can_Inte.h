/**
  * ****************************************************************************
  * @file           : can_Int.h
  * @brief          : Header for can_Int.c file
  *                   This file contains the common defines of the application.
  * ****************************************************************************
  */
#ifndef __CAN_INT_H
#define __CAN_INT_H

/* include-------------------------------------------------------------------- */
#include "main.h"
#include "can.h"
#include "pid.h"
#include "remote_control_unit.h"
#include "base_plate.h"


/* Structure------------------------------------------------------------------- */
/** @brief The data type received by the CAN bus.
  * @{
  */
typedef struct CAN_ReceivesDataMember
{
  fp32_t RotorAngle;
  fp32_t RotorSpeed_SlowingBef;
  fp32_t RotorSpeed_SlowingAft;
  fp32_t WheelLinearSpeed;
  fp32_t ActualTorqueCurrent;
  fp32_t MotorTemperature;
}CANRxDataMember;
/**
  * @}
  */

/** @brief The data received by the CAN bus that corresponds to eight motors.
  * @{
  */
typedef struct CAN_Receives_FIFO_DataMember
{
  uint8_t passage0[8];
  uint8_t passage1[8];
  uint8_t passage2[8];
  uint8_t passage3[8];
  uint8_t passage4[8];
  uint8_t passage5[8];
  uint8_t passage6[8];
  uint8_t passage7[8];
}RxDataCanFifo;
/**
  * @}
  */


/* CAN receives motor data values.   <define: uint8_t pid_ActualData[8]; */
/* pid_ActualData[0]    > The rotor mechanical Angle high 8 bit. <  -Highest symbol bit. 0x00(+) 0x80(-) */
/* pid_ActualData[1]    > The rotor mechanical Angle low 8 bit. <                                        */
/* pid_ActualData[2]    > Rotor speed high 8 bit. <  -Highest symbol bit. 0x00(+) 0x80(-)                */              
/* pid_ActualData[3]    > Rotor speed low 8 bit. <                                                       */
/* pid_ActualData[4]    > Actual torque current high 8 bit. <  -Highest symbol bit. 0x00(+) 0x80(-)      */
/* pid_ActualData[5]    > Actual torque current low 8 bit. <                                             */
/* pid_ActualData[6]    > Motor temperature <                                                            */
/* pid_ActualData[7]    > NULL <                                                                         */

/* CAN pin number */
/* F407IGHx */
/* CAN1_Rx PD0 */
/* CAN1_Tx PD1 */


/* define--------------------------------------------------------------------- */
/** @defgroup CAN_Tx_Motor_ID Motor ID 
  * @{
  */
#define Motor_ID1 (uint16_t)0x0001U
#define Motor_ID2 (uint16_t)0x0002U
#define Motor_ID3 (uint16_t)0x0004U
#define Motor_ID4 (uint16_t)0x0008U
/**
  * @}
  */


/* Global variable------------------------------------------------------------- */
/** @brief  The data received by the CAN bus. (extern)
  * @{
  */
extern RxDataCanFifo RxData_hcan1_fifo0;
extern RxDataCanFifo RxData_hcan1_fifo1;
extern RxDataCanFifo RxData_hcan2_fifo0;
extern RxDataCanFifo RxData_hcan2_fifo1;
/**
  * @}
  */


/* Function declaration--------------------------------------------------------*/
uint32_t CAN_Sand_Data(CAN_HandleTypeDef *hcan, uint16_t ID, fp32_t Data, uint16_t Motor_ID);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

void CAN_Filter_Config(CAN_HandleTypeDef *hcan, uint32_t FIFO);
void CAN_Rx_Data_Filter_Config(CAN_RxHeaderTypeDef header, RxDataCanFifo* RxData_hcanx_fifox, uint8_t* RxData_hcanx_fifox_pre_filter);

void scram(fp32_t wheel0, fp32_t wheel1, fp32_t wheel2, fp32_t wheel3);
void Motor_return_data_solution(CANRxDataMember* CANRxDatax, uint8_t* RxData_hcan1_fifox_passagex);


#endif // !__CAN_INT_H
