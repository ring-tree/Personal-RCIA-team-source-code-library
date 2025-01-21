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


/* Structure------------------------------------------------------------------- */
typedef struct CAN_ReceivesDataMember
{
  fint32_t RotorAngle;
  fint32_t RotorSpeed;
  fint32_t ActualTorqueCurrent;
  fint32_t MotorTemperature;
}CANRxDataMember;


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
extern uint8_t RxData_hcan1_fifo0[8];
extern uint8_t RxData_hcan1_fifo1[8];
extern uint8_t RxData_hcan2_fifo0[8];
extern uint8_t RxData_hcan2_fifo1[8];
/**
  * @}
  */


/* Function declaration--------------------------------------------------------*/
uint32_t CAN_Sand_Data(CAN_HandleTypeDef *hcan, uint16_t ID, fint32_t Data, uint16_t Motor_ID);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN_Filter_Config(CAN_HandleTypeDef *hcan, uint32_t FIFO);


#endif // !__CAN_INT_H
