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
  fint32_t PointsMaxOutput;        /* Integral limiting */
  fint32_t DifferentialMaxOutput;  /* Differential limiting */

  /* I , D output. */
  fint32_t IOutput;  /* IOutput += Error */
  fint32_t DOutput;  /* DOutput = (Error - Last_Error) */

}pid;

typedef struct PID_OutputType
{
  fint32_t AnglePID;
  fint32_t SpeedPID;
  fint32_t TorqueCurrentPID;
}pidout;


/* Function declaration ----------------------------------------------------- */
void LimitMax(fint32_t* value, fint32_t max);
fint32_t angle_to_radian(fint32_t angle);

void PID_Initialize(pid* pidx, fint32_t iMax, fint32_t dMax);
void PID_AdjustParameters_Initialize(pid* pidx, fint32_t kp, fint32_t ki, fint32_t kd, fint32_t iMax, fint32_t dMax);
fint32_t PID_Controller(pid pidx, fint32_t ExpectationData, fint32_t ActualData);

fint32_t DataExtraction(uint8_t data0, uint8_t data1);
void DataStorage(fint32_t data, uint8_t* data0, uint8_t* data1);

#ifdef __cplusplus
}
#endif

#endif /* !pid.h */
