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
typedef float fp32_t;
typedef double fp64_t;

/* define --------------------------------------------------------------------- */
#define SPEED_MODE 0x0001u
#define ANGLE_MODE 0x0002u

#define EMPTY __FLT_MAX__

/* Structure --------------------------------------------------------------- */
typedef struct PID_Struture
{
  /* p , i , d Control parameter. */
  fp32_t kp;  /* Proportional term */
  fp32_t ki;  /* Integral term */
  fp32_t kd;  /* Differential term */

  /* Importance value. */
  fp32_t ExpectationInput;  
  fp32_t ActualOutput;
  fp32_t Last_ActualOutput;
  fp32_t Error;  /* Error = ExpectationInput - ActualOutput */
  fp32_t Last_Error;  /* Last_Error = ExpectationInput - Last_ActualOutput */

  /* Amplitude limiting.*/
  fp32_t IntegralMaxOutput;        /* Integral limiting */
  fp32_t DifferentialMaxOutput;  /* Differential limiting */

  /* I , D output. */
  fp32_t IOutput;  /* IOutput += Error */
  fp32_t DOutput;  /* DOutput = (Error - Last_Error) */

  fp32_t Time;  /* Time */

}pid;

typedef struct PID_OutputType
{
  fp32_t AnglePID;
  fp32_t SpeedPID;
  fp32_t TorqueCurrentPID;
}pidout;

/* extern ------------------------------------------------------------------- */
extern uint8_t VOFA_DebugData[sizeof(fp32_t) * 8];  //Debug data with VOFA++

/* Function declaration ----------------------------------------------------- */
void LimitMax(fp32_t* value, fp32_t max);
void LimitMin(fp32_t* value, fp32_t min);
fp32_t angle_to_radian(fp32_t angle);

void PID_Initialize_SpeedMode(pid* pidx);
void PID_Initialize_PositionMode(pid* pidx, uint32_t Mode);
void PID_AdjustParameters_Initialize(pid* pidx, fp32_t kp, fp32_t ki, fp32_t kd, fp32_t iMax, fp32_t dMax, fp32_t Time);
fp32_t PID_Controller(pid pidx, fp32_t ExpectationData, fp32_t ActualData, uint32_t Mode);

fp32_t DataExtraction(uint8_t data0, uint8_t data1);
void DataStorage(fp32_t data, uint8_t* data0, uint8_t* data1);

fp32_t ZeroCrossing(fp32_t ExpectedAngle, fp32_t ActualAngle);
void VOFA_DebugDataEncapsulation ( 
    fp32_t passage0, 
    fp32_t passage1, 
    fp32_t passage2, 
    fp32_t passage3, 
    fp32_t passage4,  
    fp32_t passage5, 
    fp32_t passage6  );


#ifdef __cplusplus
}
#endif

#endif /* !__PID_H */

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
// pidout1.SpeedPID = PID_Controller(pid1, TxData, RxData1.RotorSpeed_SlowingBef, SPEED_MODE);
// CAN_Sand_Data(&hcan1, 0x200, pidout1.SpeedPID, Motor_ID1 | Motor_ID2 | Motor_ID3 | Motor_ID4);
/* 角度 */
// pidout2_AngleLoop.AnglePID = PID_Controller(pid2_Angle, TxData, RxData1.RotorAngle, ANGLE_MODE);
// pidout2_speedLoop.SpeedPID = PID_Controller(pid2_speed, pidout2_AngleLoop.AnglePID, RxData1.RotorSpeed_SlowingBef, SPEED_MODE);
// CAN_Sand_Data(&hcan1, 0x200, pidout2_speedLoop.SpeedPID, Motor_ID1 | Motor_ID2 | Motor_ID3 | Motor_ID4);
