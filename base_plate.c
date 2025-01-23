/**
 * *****************************************************************************
 * @file    base_plate.c
 * @brief   This file provides firmware functions to manage the base plate.
 * *****************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "base_plate.h"
#include "pid.h"


/* Global variable -----------------------------------------------------------*/
fp32_t wheel0; 
fp32_t wheel1;  
fp32_t wheel2; 
fp32_t wheel3; 

fp32_t Vx; 
fp32_t Vy; 
fp32_t Vz; 

/* Functions -----------------------------------------------------------------*/
/**
 * @brief  Calculates the motion solution for the wheat wheels based on the remote control data.
 * 
 * This function takes the remote control data from RC_CtrlData_AT and calculates the motion solution for the wheat wheels.
 * It uses the formulas for VA, VB, VC, and VD wheels to calculate the wheel speeds.
 * 
 * @param  None
 * 
 * @return None
 */
void Wheat_wheel_motion_solution()  
{
    Vx = RC_CtrlData_AT.ch2;
    Vy = RC_CtrlData_AT.ch3;
    Vz = RC_CtrlData_AT.ch0;

    wheel0 = Vx + Vy + Vz * (WHEEL_BASE / 2 + WHEEL_TRACK / 2);
    wheel1 = Vy - Vx + Vz * (WHEEL_BASE / 2 + WHEEL_TRACK / 2);
    wheel2 = -(Vx + Vy - Vz * (WHEEL_BASE / 2 + WHEEL_TRACK / 2));
    wheel3 = -(Vy - Vx - Vz * (WHEEL_BASE / 2 + WHEEL_TRACK / 2));
}
