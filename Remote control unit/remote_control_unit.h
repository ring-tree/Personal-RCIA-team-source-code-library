/**
 * ****************************************************************************
 * @file           : remote_control_unit.h
 * 
 * @brief          : Header for remote_control_unit.c file
 * ****************************************************************************
 */

/* include-------------------------------------------------------------------- */
#include "main.h"
#include "usart.h"
#include "tim.h"


/* define--------------------------------------------------------------------- */
/* RC Channel Definition */
#define RC_CH_VALUE_MIN ((uint16_t)364 )              /* Maximum output value of rocker channel */
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)           /* Intermediate output value of rocker channel */
#define RC_CH_VALUE_MAX ((uint16_t)1684)              /* Minimum output value of rocker channel */

/* RC Switch Definition */
#define RC_SW_UP ((uint16_t)1)                         /* Minimum output value of Three-position switch channel */
#define RC_SW_MID ((uint16_t)3)                        /* Intermediate output value of Three-position switch channel */
#define RC_SW_DOWN ((uint16_t)2)                       /* Maximum output value of Three-position switch channel */      

#define RC_FRAME_LENGTH 18u                            /* The total number of bytes of data frames received from the remote control */

/* Connection state */
#define RECEIVER_STATE_UNSYNC ((uint16_t)0x0000 )      /* Usart state unsync*/
#define RECEIVER_STATE_SYNC ((uint16_t)0x0001 )        /* Usart state sync*/


/* struct--------------------------------------------------------------------- */
/**
 * @defgroup RC_Ctl_t Remote control data structure.
 * @{
 */
typedef struct Remote_control
{
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint8_t s1;
    uint8_t s2;
} RC_Ctl_t;
/**
 * @}
 */

