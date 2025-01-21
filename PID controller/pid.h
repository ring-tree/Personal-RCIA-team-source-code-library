/**
  * ****************************************************************************
  * @file           : pid.h
  *
  * @brief          : Header for pid.c file
  *                   This file contains the common defines of the application.
  * ****************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* include -------------------------------------------------------------------- */
#include "main.h"                    


/* typedef -------------------------------------------------------------------- */
typedef float fint32_t;

/* define --------------------------------------------------------------------- */
#define SPEED_MODE 0x0001u
#define ANGLE_MODE 0x0002u

#define EMPTY __FLT_MAX__

/* Structure --------------------------------------------------------------- */
typedef struct PID_Struture
{
  /* p , i , d Control parameter. */
  fint32_t kp;  /* Proportional term */
  fint32_t ki;  /* Integral term */
  fint32_t kd;  /* Differential term */

  /* Importance value. */
  fint32_t ExpectationInput;  
  fint32_t ActualOutput;
  fint32_t Last_ActualOutput;
  fint32_t Error;  /* Error = ExpectationInput - ActualOutput */
  fint32_t Last_Error;  /* Last_Error = ExpectationInput - Last_ActualOutput */

  /* Amplitude limiting.*/
  fint32_t IntegralMaxOutput;        /* Integral limiting */
  fint32_t DifferentialMaxOutput;  /* Differential limiting */

  /* I , D output. */
  fint32_t IOutput;  /* IOutput += Error */
  fint32_t DOutput;  /* DOutput = (Error - Last_Error) */

  fint32_t Time;  /* Time */

}pid;

typedef struct PID_OutputType
{
  fint32_t AnglePID;
  fint32_t SpeedPID;
  fint32_t TorqueCurrentPID;
}pidout;

/* extern ------------------------------------------------------------------- */
extern uint8_t VOFA_DebugData[sizeof(fint32_t) * 8];

/* Function declaration ----------------------------------------------------- */
void LimitMax(fint32_t* value, fint32_t max);
void LimitMin(fint32_t* value, fint32_t min);
fint32_t angle_to_radian(fint32_t angle);

void PID_Initialize_SpeedMode(pid* pidx);
void PID_Initialize_PositionMode(pid* pidx, uint32_t Mode);
void PID_AdjustParameters_Initialize(pid* pidx, fint32_t kp, fint32_t ki, fint32_t kd, fint32_t iMax, fint32_t dMax, fint32_t Time);
fint32_t PID_Controller(pid pidx, fint32_t ExpectationData, fint32_t ActualData, uint32_t Mode);

fint32_t DataExtraction(uint8_t data0, uint8_t data1);
void DataStorage(fint32_t data, uint8_t* data0, uint8_t* data1);

fint32_t ZeroCrossing(fint32_t ExpectedAngle, fint32_t ActualAngle);
void VOFA_DebugDataEncapsulation ( 
    fint32_t passage0, 
    fint32_t passage1, 
    fint32_t passage2, 
    fint32_t passage3, 
    fint32_t passage4,  
    fint32_t passage5, 
    fint32_t passage6  );

#ifdef __cplusplus
}
#endif

#endif /* !pid.h */

/* pid 测试 --- */
// pid pid1;
// pidout pidout1;
// 
// pid pid2_Angle;
// pid pid2_speed;
// pidout pidout2_AngleLoop;
// pidout pidout2_speedLoop;
// 
// PID_Initialize_SpeedMode(&pid1);
// PID_Initialize_PositionMode(&pid2_Angle, ANGLE_MODE);
// PID_Initialize_PositionMode(&pid2_speed, SPEED_MODE);
//
/* 速度 */
// pidout1.SpeedPID = PID_Controller(pid1, TxData, RxData1.RotorSpeed, SPEED_MODE);
// CAN_Sand_Data(&hcan1, 0x200, pidout1.SpeedPID, Motor_ID1 | Motor_ID2 | Motor_ID3 | Motor_ID4);
/* 角度 */
// pidout2_AngleLoop.AnglePID = PID_Controller(pid2_Angle, TxData, RxData1.RotorAngle, ANGLE_MODE);
// pidout2_speedLoop.SpeedPID = PID_Controller(pid2_speed, pidout2_AngleLoop.AnglePID, RxData1.RotorSpeed, SPEED_MODE);
// CAN_Sand_Data(&hcan1, 0x200, pidout2_speedLoop.SpeedPID, Motor_ID1 | Motor_ID2 | Motor_ID3 | Motor_ID4);
