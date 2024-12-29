/**
 * @file    can_Inte.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.             
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_INTE_H
#define __CAN_INTE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/** @defgroup CAN_Filter_Config Filter scale and mode.
  * @{
  */
#define CAN_FILTERSET_32BIT_IDAMASK       (0x00000000U)  /*!< One 32-bit filter and Identifier mask mode  */
#define CAN_FILTERSET_16BIT_IDAMASK       (0x00000001U)  /*!< Two 16-bit filters and Identifier mask mode */
#define CAN_FILTERSET_32BIT_IDLIST        (0x00000002U)  /*!< One 32-bit filter and Identifier list mode  */
#define CAN_FILTERSET_16BIT_IDLIST        (0x00000003U)  /*!< Two 16-bit filters and Identifier list mode */
/**
  * @}
  */


/* Function declaration --------------------------------------------------------*/

/* Configures Filter */
void CAN_Filter_Config(CAN_HandleTypeDef* hcan, uint32_t FilterNumber, uint32_t scale_mode
            , uint32_t RTR,uint32_t id, uint32_t mask_id, uint32_t fifo, uint32_t MyCAN_SlaveStartFilterBank);

/* Send Data */
uint32_t CAN_Send(CAN_HandleTypeDef *hcan, uint32_t CanSendID, uint8_t CanSendData[], uint32_t Length
       , uint32_t MyCan_IDE, uint32_t MyCan_RTR, FunctionalState MyCan_TransmitGlobalTime);

/* Interrupts mode receive Data */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


#ifdef __cplusplus
}
#endif  

#endif /*! can_Inte.h */   
