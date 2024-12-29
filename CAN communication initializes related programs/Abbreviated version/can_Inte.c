/**
 * *******************************************************************************
 * @file can_Inte.c
 * 
 * @brief CAN interface implementation
 * *******************************************************************************
 */

/* include---------------------------------------------------------------------*/
#include "can_Inte.h"


/* Global variable------------------------------------------------------------- */
/** @brief  The data received by the CAN bus, external use requires a call. "extern"
  * @{
  */
uint8_t RxData_hcan1_fifo0[8];
uint8_t RxData_hcan1_fifo1[8];
uint8_t RxData_hcan2_fifo0[8];
uint8_t RxData_hcan2_fifo1[8];
/**
  * @}
  */


/* Function definition---------------------------------------------------------*/
/**
  * @brief  Sends a message over the CAN bus.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains the configuration information for the CAN module.
  * @param  ID The standard identifier of the message to be sent.
  * @param  Data The data to be sent.
  * @param  Motor_ID The ID of the motor to be controlled.  
  *                  This parameter can be a value of @arg CAN_Tx_Motor_ID.
  * @retval The status of the transmission request.
  */
uint32_t CAN_Sand_Data(CAN_HandleTypeDef *hcan, uint16_t ID, fint32_t Data, uint16_t Motor_ID)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t used_mailbox;
  uint8_t TxData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  //Detects key parameter passing
  assert_param(hcan != NULL);

  tx_header.StdId = ID;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 0X08;

  if (Motor_ID & 0x01)
  {
    DataStorage(Data, &TxData[0], &TxData[1]);
  }
  if ((Motor_ID & 0x02) >> 1)
  {
    DataStorage(Data, &TxData[2], &TxData[3]);
  }
  if ((Motor_ID & 0x04) >> 2)
  {
    DataStorage(Data, &TxData[4], &TxData[5]);
  }
  if ((Motor_ID & 0x08) >> 3)
  {
    DataStorage(Data, &TxData[6], &TxData[7]);
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan))
  {
    HAL_StatusTypeDef TxStatus = HAL_CAN_AddTxMessage(hcan, &tx_header, TxData, &used_mailbox);
    uint32_t WaitTime = 0;
    while ( TxStatus != HAL_OK )
    {
      TxStatus = HAL_CAN_AddTxMessage(hcan, &tx_header, TxData, &used_mailbox);
      WaitTime++;
      if (WaitTime > 100000) 
        return TxStatus;     
    }
    return used_mailbox;
  }     
   
  return HAL_CAN_GetTxMailboxesFreeLevel(hcan);
}


/**
  * @brief  Rx FIFO 0 message pending callback.
  *         This function is called when a message is pending in Rx FIFO 0.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  
  //Detects key parameter passing
  assert_param(hcan != NULL);

  if (hcan == &hcan1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, RxData_hcan1_fifo0);
  }
  else if (hcan == &hcan2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, RxData_hcan2_fifo0);
  }
}


/**
  * @brief  Rx FIFO 1 message pending callback.
  *         This function is called when a message is pending in Rx FIFO 1.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;

  if (hcan == &hcan1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, RxData_hcan1_fifo1);
  }
  else if (hcan == &hcan2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, RxData_hcan2_fifo1);
  }
}


/**
  * @brief  Configures the CAN filter according to the specified parameters. (Configuration all-pass, 16-bit_MASK mode).
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains the configuration information for the CAN module.
  * @param  FIFO The FIFO to be configured. Can be CAN_FILTER_FIFO0 or CAN_FILTER_FIFO1.
  * @retval None
  */
void CAN_Filter_Config(CAN_HandleTypeDef *hcan, uint32_t FIFO)
{
    CAN_FilterTypeDef can_filter_init_structure;

    //Detects key parameter passing
    assert_param(hcan != NULL);

    can_filter_init_structure.FilterBank = 0;
    can_filter_init_structure.FilterFIFOAssignment = FIFO;

    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;

    can_filter_init_structure.FilterIdHigh = 0x0000;
    can_filter_init_structure.FilterIdLow = 0x0000;
    can_filter_init_structure.FilterMaskIdHigh = 0x0000;
    can_filter_init_structure.FilterMaskIdLow = 0x0000;
    can_filter_init_structure.SlaveStartFilterBank = 14;
    can_filter_init_structure.SlaveStartFilterBank = 14;

    can_filter_init_structure.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);

    HAL_CAN_Start(hcan);

    if (FIFO == CAN_FILTER_FIFO0)
    {
      HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (FIFO == CAN_FILTER_FIFO1)
    {
      HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    }

  }
