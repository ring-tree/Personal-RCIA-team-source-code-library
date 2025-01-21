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

/* variable ------------------------------------------------------------------- */
uint8_t VOFA_DebugData[sizeof(fint32_t) * 8];

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
 * @brief  Limit the minimum value.
 * 
 * @param  value: A pointer to the value to be limited.
 * @param  min: The minimum value.
 * 
 * @retval None
 */
void LimitMin(fint32_t* value, fint32_t min)
{
    if ( *value <= min ) 
        *value = min;    
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
 * @brief Initializes a PID speed controlle with default gains and given maximum integral and differential outputs.
 * 
 * This function sets the PID speed controller's proportional, integral, and derivative gains to default values (0.5, 0.2, and 0.0 respectively), 
 * and initializes the PID controller's state variables (expectation input, actual output, error, etc.) to zero.
 * It also sets the maximum integral and differential outputs to the given values.
 * 
 * @param pidx A pointer to the PID speed controller structure to be initialized.
 * 
 * @retval None
 */
void PID_Initialize_SpeedMode(pid* pidx)
{
    pidx->kp = 1.2f;
    pidx->ki = 2.0f;
    pidx->kd = 10.0f;

    pidx->ExpectationInput = 0.0f;
    pidx->ActualOutput = 0.0f;
    pidx->Last_ActualOutput = 0.0f;
    pidx->Error = 0.0f;
    pidx->Last_Error = 0.0f;

    pidx->IntegralMaxOutput = 3000.0f;     
    pidx->DifferentialMaxOutput = 3000.0f; 

    pidx->IOutput = 0.0f;
    pidx->DOutput = 0.0f;   

    pidx->Time = 10.0f;
}

/**
 * @brief Initializes a PID position controller with given maximum integral and differential outputs.
 * 
 * This function sets the PID position controller's proportional, integral, and derivative gains based on the given mode (SPEED_MODE or ANGLE_MODE),
 * and initializes the PID position controller's state variables (expectation input, actual output, error, etc.) to zero.
 * It also sets the maximum integral and differential outputs to the given values.
 * 
 * @param pidx A pointer to the PID position controller structure to be initialized.
 * @param Mode Position pid controller inner ring speed ring and outer ring Angle ring.
 *             Can be SPEED_MODE or ANGLE_MODE.
 * 
 * @retval None
 */
void PID_Initialize_PositionMode(pid* pidx, uint32_t Mode) 
{
    if (Mode == ANGLE_MODE) {
        pidx->kp = 0.06f;
        pidx->ki = 0.2f;    
        pidx->kd = 0.0f;
    }
    else if (Mode == SPEED_MODE) {
        pidx->kp = 12.0f;
        pidx->ki = 0.0f;    
        pidx->kd = 0.0f;
    }

    pidx->ExpectationInput = 0.0f;
    pidx->ActualOutput = 0.0f;
    pidx->Last_ActualOutput = 0.0f;
    pidx->Error = 0.0f;
    pidx->Last_Error = 0.0f;

    pidx->IntegralMaxOutput = 300.0f;     
    pidx->DifferentialMaxOutput = 300.0f; 

    pidx->IOutput = 0.0f;
    pidx->DOutput = 0.0f;   

    pidx->Time = 10.0f;
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
 * @param Time Pid calculates the time constant.
 * 
 * @retval None
 */
void PID_AdjustParameters_Initialize(pid* pidx, fint32_t kp, fint32_t ki, fint32_t kd, fint32_t iMax, fint32_t dMax, fint32_t Time)
{
    pidx->kp = kp;
    pidx->ki = ki;
    pidx->kd = kd;
    pidx->ExpectationInput = 0.0f;
    pidx->ActualOutput = 0.0f;
    pidx->Last_ActualOutput = 0.0f;
    pidx->Error = 0.0f;
    pidx->Last_Error = 0.0f;
    pidx->IntegralMaxOutput = iMax;
    pidx->DifferentialMaxOutput = dMax;
    pidx->IOutput = 0.0f;
    pidx->DOutput = 0.0f;   
    pidx->Time = Time;
}

/**
 * @brief Calculates the PID controller output based on the expectation data, actual data, and mode.
 * 
 * This function takes in a PID controller structure, expected input value, actual output value, and mode.
 * It calculates the error, updates the integral and differential outputs, and returns the PID controller output value.
 * 
 * @param pidx A pointer to the PID controller structure.
 * @param ExpectationData The expected input value.
 * @param ActualData The actual output value.
 * @param Mode The mode of the PID controller, either SPEED_MODE or ANGLE_MODE.
 * 
 * @retval fint32_t The calculated PID controller output value.
 */
fint32_t PID_Controller(pid pidx, fint32_t ExpectationData, fint32_t ActualData, uint32_t Mode)
{       
    pidx.ExpectationInput = ExpectationData;
    pidx.ActualOutput = ActualData;

    if (Mode == SPEED_MODE) {
        pidx.Error = pidx.ExpectationInput - pidx.ActualOutput;
    }
    else if (Mode == ANGLE_MODE) {
        pidx.Error = ZeroCrossing(pidx.ExpectationInput, pidx.ActualOutput);
    }

    pidx.IOutput += pidx.Error;
    LimitMax(&pidx.IOutput, pidx.IntegralMaxOutput);

    pidx.DOutput = (pidx.Error - pidx.Last_Error) / pidx.Time;
    LimitMax(&pidx.DOutput, pidx.DifferentialMaxOutput);

    pidx.Last_ActualOutput = pidx.ActualOutput;
    pidx.Last_Error = pidx.Error;    

    fint32_t PID_Output = pidx.kp * pidx.Error + pidx.ki * pidx.IOutput + pidx.kd * pidx.DOutput;
    LimitMax(&PID_Output, 8191.0f);
    LimitMin(&PID_Output, -8292.0f);

    return PID_Output;
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
    uint16_t uintdata = (data0 << 8) | data1;
    int16_t nintdata = ~((uintdata - 1) << 1);

    fint32_t fdata = (fint32_t)((data0 >> 7) ? -((nintdata >> 1)) : (uintdata));
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
    fint32_t float_to_uint_data = roundf(data);
    uint16_t uint_data = (uint16_t)fabsf(float_to_uint_data);
    int16_t int_data = ((~uint_data) + 1) << 1;

    *data0 = (data >= 0) ? (uint_data >> 8) : ((int_data >> 9) | 0x80);
    *data1 = (data >= 0) ? (uint_data) : (int_data >> 1);
}

/**
 * @brief Adjusts the actual angle to match the expected angle based on the zero-crossing point.
 * 
 * This function takes in the expected angle and actual angle values, calculates the difference, 
 * and adjusts the actual angle to match the expected angle by considering the zero-crossing point.
 * 
 * @param ExpectedAngle The expected angle value.
 * @param ActualAngle The actual angle value to be adjusted.
 * 
 * @retval fint32_t The adjusted actual angle value.
 */
fint32_t ZeroCrossing(fint32_t ExpectedAngle, fint32_t ActualAngle) {
    const fint32_t MAX_ANGLE = 8192.0f;
    fint32_t DifferenceValue = ExpectedAngle - ActualAngle;
    if ( fabs(DifferenceValue) >= (MAX_ANGLE / 2.0f) )
        return ( (DifferenceValue >= 0) ? (DifferenceValue - MAX_ANGLE) : (DifferenceValue + MAX_ANGLE) );
    return DifferenceValue;
}

/**
 * @brief Encapsulates debug data into the VOFA_DebugData array.
 * 
 * The function accepts 7 fint32_t parameters corresponding to 7 channels and 
 * stores them in the VOFA_DebugData array encapsulation, 
 * 7 fint32_t parameters can be set to EMPTY to indicate that they are not used,
 * and the last 4 bits are set to 0x7f800000f to indicate the frame tail.
 *
 * @param passage0 The first passage.   or EMPTY
 * @param passage1 The second passage.  or EMPTY
 * @param passage2 The third passage.   or EMPTY
 * @param passage3 The fourth passage.  or EMPTY
 * @param passage4 The fifth passage.   or EMPTY
 * @param passage5 The sixth passage.   or EMPTY
 * @param passage6 The seventh passage. or EMPTY
 * 
 * @retval None
 */
void VOFA_DebugDataEncapsulation (   
    fint32_t passage0, 
    fint32_t passage1, 
    fint32_t passage2, 
    fint32_t passage3, 
    fint32_t passage4,  
    fint32_t passage5, 
    fint32_t passage6  )
{
    if (passage0 != EMPTY)
        *((fint32_t*)&VOFA_DebugData[sizeof(fint32_t) * 0]) = passage0;
    if (passage1 != EMPTY)
        *((fint32_t*)&VOFA_DebugData[sizeof(fint32_t) * 1]) = passage1;
    if (passage2 != EMPTY)
        *((fint32_t*)&VOFA_DebugData[sizeof(fint32_t) * 2]) = passage2;
    if (passage3 != EMPTY)
        *((fint32_t*)&VOFA_DebugData[sizeof(fint32_t) * 3]) = passage3;
    if (passage4 != EMPTY)
        *((fint32_t*)&VOFA_DebugData[sizeof(fint32_t) * 4]) = passage4;
    if (passage5 != EMPTY)
        *((fint32_t*)&VOFA_DebugData[sizeof(fint32_t) * 5]) = passage5;
    if (passage6 != EMPTY)
        *((fint32_t*)&VOFA_DebugData[sizeof(fint32_t) * 6]) = passage6;

    *((uint32_t*)&VOFA_DebugData[sizeof(fint32_t) * 7]) = 0x7f800000;
}
