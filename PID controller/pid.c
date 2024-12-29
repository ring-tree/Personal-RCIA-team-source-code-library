/**
 * *****************************************************************************
 * @file    pid.c
 * 
 * @brief   PID control.
 * *****************************************************************************
 */

/* include -------------------------------------------------------------------- */
#include "pid.h"
#include "math.h"


/* Function prototypes -------------------------------------------------------- */
/**
 * @brief  Limit the maximum value.
 * 
 * @param  value: The value to be limited.
 * @param  max: The maximum value.
 * 
 * @retval None
 */
void LimitMax(fint32_t* value, fint32_t max)
{
    if ( *value >= max ) 
        *value = max;    
}


/**
 * @brief  Converts an angle in degrees to radians.
 * 
 * @param  angle: The angle in degrees to be converted.
 * 
 * @retval fint32_t The converted angle in radians.
 */
fint32_t angle_to_radian(fint32_t angle)
{
    return angle * 0.0174533f;
}


/**
 * @brief  Initializes a PID controller with the given maximum integral and differential outputs.
 * 
 * @param  pidx: A pointer to the PID controller structure to be initialized.
 * @param  iMax: The maximum integral output of the PID controller.
 * @param  dMax: The maximum differential output of the PID controller.
 * 
 * @retval None
 */
void PID_Initialize(pid* pidx, fint32_t iMax, fint32_t dMax)
{
    pidx->kp = 20000.0f;
    pidx->ki = 0.0f;
    pidx->kd = 2.0f;

    pidx->ExpectationInput = 0.0f;
    pidx->ActualOutput = 0.0f;
    pidx->Last_ActualOutput = 0.0f;
    pidx->Error = 0.0f;
    pidx->Last_Error = 0.0f;

    pidx->PointsMaxOutput = iMax;
    pidx->DifferentialMaxOutput = dMax;

    pidx->IOutput = 0.0f;
    pidx->DOutput = 0.0f;   
}

/**
 * @brief Initializes the PID controller with adjustable parameters.
 *
 * @param pidx A pointer to the PID controller structure.
 * @param kp The proportional gain.
 * @param ki The integral gain.
 * @param kd The derivative gain.
 * @param iMax The maximum integral output.
 * @param dMax The maximum differential output.
 * 
 * @retval None
 */
void PID_AdjustParameters_Initialize(pid* pidx, fint32_t kp, fint32_t ki, fint32_t kd, fint32_t iMax, fint32_t dMax)
{
    pidx->kp = kp;
    pidx->ki = ki;
    pidx->kd = kd;
    pidx->ExpectationInput = 0.0f;
    pidx->ActualOutput = 0.0f;
    pidx->Last_ActualOutput = 0.0f;
    pidx->Error = 0.0f;
    pidx->Last_Error = 0.0f;
    pidx->PointsMaxOutput = iMax;
    pidx->DifferentialMaxOutput = dMax;
    pidx->IOutput = 0.0f;
    pidx->DOutput = 0.0f;   
}

/**
 * @brief  Calculates the PID controller output based on the expectation and actual data.
 * 
 * @param  pidx: A pointer to the PID controller structure.
 * @param  ExpectationData: The expected input value.
 * @param  ActualData: The actual output value.
 * 
 * @retval fint32_t The calculated PID controller output value.
 */
fint32_t PID_Controller(pid pidx, fint32_t ExpectationData, fint32_t ActualData)
{       
    pidx.ExpectationInput = ExpectationData;
    pidx.ActualOutput = ActualData;

    pidx.Error = pidx.ExpectationInput - pidx.ActualOutput;

    pidx.IOutput += pidx.Error;
    LimitMax(&pidx.IOutput, pidx.PointsMaxOutput);

    pidx.DOutput = pidx.Error - pidx.Last_Error;
    LimitMax(&pidx.DOutput, pidx.DifferentialMaxOutput);

    pidx.Last_ActualOutput = pidx.ActualOutput;
    pidx.Last_Error = pidx.Error;    

    return pidx.kp * pidx.Error + pidx.ki * pidx.IOutput + pidx.kd * pidx.DOutput;
}


/**
 * @brief  Extracts a 16-bit signed integer value from two 8-bit unsigned integers.
 * 
 * @param  data0: The high 8 bits of the 16-bit integer, highest symbol bit.
 * @param  data1: The low 8 bits of the 16-bit integer.
 * 
 * @retval fint32_t The extracted 16-bit signed integer value.
 */
fint32_t DataExtraction(uint8_t data0, uint8_t data1)
{
    uint16_t intdata = ((data0 << 8) | data1) << 1;
    fint32_t fdata = (fint32_t)(((data0 & 0x80) >> 7) ? -((intdata >> 1)) : (intdata >> 1));
    return fdata;
}

/**
 * @brief  Stores a 16-bit signed integer value into two 8-bit unsigned integers.
 * 
 * @param  data: The 16-bit signed integer value to be stored.
 * @param  data0: A pointer to the high 8 bits of the 16-bit integer, highest symbol bit.
 * @param  data1: A pointer to the low 8 bits of the 16-bit integer.
 * 
 * @retval None
 */
void DataStorage(fint32_t data, uint8_t* data0, uint8_t* data1)
{
    fint32_t float_to_int_data = roundf(data);
    uint16_t int_data = (uint16_t)fabsf(float_to_int_data);
    *data0 = (float_to_int_data <= 0) ? ((int_data >> 8) | 0x80) : (int_data >> 8);
    *data1 = int_data;
}

