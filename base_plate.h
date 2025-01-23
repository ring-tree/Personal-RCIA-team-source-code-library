/**
 * *****************************************************************************
 * @file    base_plate.c
 * @brief   This file provides firmware functions to manage the base plate.
 * *****************************************************************************
 */
#ifndef __BASE_PLATE_H
#define __BASE_PLATE_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
// #include "pid.h"
#include "remote_control_unit.h"
#include "can_Inte.h"


/* define --------------------------------------------------------------------*/
#define WHEEL_TRACK 0.2f  //Left and right wheats distance (m)
#define WHEEL_BASE 0.2f  //Front and rear wheats distance (m)

#define WHEEL_RADIUS 0.05f  //Wheel radius (m)

#ifndef M_PI
#define M_PI 3.14159265358979323846

#define REDUCTION_RATIO_M3508 19.2f  //M3508 reduction ratio
#define ROTOR_MECHANICAL_ANGLE_MAX 8192.0f  //M3508




/* Global variable -----------------------------------------------------------*/
extern fp32_t wheel0; // (m/s)
extern fp32_t wheel1; // (m/s)
extern fp32_t wheel2; // (m/s)
extern fp32_t wheel3; // (m/s)

extern fp32_t Vx;  //Speed in the x direction (left- and right+) (m/s)
extern fp32_t Vy;  //Speed in the y direction (front+ and rear-) (m/s)
extern fp32_t Vz;  //Speed in the z direction (rotation clockwise+ and counterclockwise-) (rad/s)


/* Functions -----------------------------------------------------------------*/
void Wheat_wheel_motion_solution();  


#endif /* ！M_PI */

#endif /* ！__BASE_PLATE_H */


