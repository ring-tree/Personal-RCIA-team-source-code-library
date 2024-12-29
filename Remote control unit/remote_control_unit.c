/**
 * ****************************************************************************
 * @file remote_control_unit.c
 * 
 * @brief Remote control unit implementation.
 * *****************************************************************************
 */

/* include--------------------------------------------------------------------- */
#include "remote_control_unit.h"


/* Internal Data-------------------------------------------------------------- */
static uint8_t sbus_rx_buffer[RC_FRAME_LENGTH];  /* A variable that stores data received from the remote control */
static RC_Ctl_t RC_CtrlData;                     /* The RC_Ctl_t struct variable that stores the decoded received data */

static uint32_t Detection = 1;        /* Total value of sent frames to determine whether the connection is disconnected */
static uint32_t Last_Detection = 0;   /* Historical cumulative value */ 


/* Function definition-------------------------------------------------------- */
/**
 * @brief  Detects if there is a wire break in the remote control unit.
 * 
 * @retval uint16_t RECEIVER_STATE_UNSYNC if the states are the same,
 *                  RECEIVER_STATE_SYNC if the states are different.
 */
uint16_t WireBreakDetection(void)
{
    if (Detection == Last_Detection) {
        MX_USART1_UART_Init();
        return RECEIVER_STATE_UNSYNC;
    } else {
        return RECEIVER_STATE_SYNC;
    }
    Last_Detection = Detection;
}

/**
 * @brief Callback function for the TIM period elapsed.
 *
 * @param htim Pointer to a TIM_HandleTypeDef structure that contains
 *             the configuration information for the specified TIM module.
 *
 * @return The return value of the WireBreakDetection function.
 */
uint16_t HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2_BASE) {
        return WireBreakDetection();
    }
}

/**
 * @brief  Remote control unit data processing.
 * 
 * @param  rcdata Pointer to the data received from the remote control unit.
 * 
 * @retval None
 */
void RemoteDataProcess(uint8_t* rcdata)
{
    if (rcdata == NULL) {
        return; 
    }    
    RC_CtrlData.ch0 = ((int16_t)rcdata[0] | ((int16_t)rcdata[1] << 8)) & 0x07FF; 
    RC_CtrlData.ch1 = (((int16_t)rcdata[1] >> 3) | ((int16_t)rcdata[2] << 5)) & 0x07FF;
    RC_CtrlData.ch2 = (((int16_t)rcdata[2] >> 6) | ((int16_t)rcdata[3] << 2) | ((int16_t)rcdata[4] << 10)) & 0x07FF;
    RC_CtrlData.ch3 = (((int16_t)rcdata[4] >> 1) | ((int16_t)rcdata[5]<<7)) & 0x07FF;

    RC_CtrlData.s1 = ((rcdata[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.s2 = ((rcdata[5] >> 4) & 0x0003);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Detection++;        
        RemoteDataProcess(sbus_rx_buffer);
        HAL_UART_Receive_DMA(&huart1, sbus_rx_buffer, RC_FRAME_LENGTH);
    }
}










