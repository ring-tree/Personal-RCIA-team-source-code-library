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
RxDataCanFifo RxData_hcan1_fifo0;
RxDataCanFifo RxData_hcan1_fifo1;
RxDataCanFifo RxData_hcan2_fifo0;
RxDataCanFifo RxData_hcan2_fifo1;
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
uint32_t CAN_Sand_Data(CAN_HandleTypeDef *hcan, uint16_t ID, fp32_t Data, uint16_t Motor_ID)
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
 CAN_RxHeaderTypeDef header;
uint8_t RxData_hcan1_fifo0_pre_filter[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  // CAN_RxHeaderTypeDef header;
  // uint8_t RxData_hcan1_fifo0_pre_filter[8];
  uint8_t RxData_hcan2_fifo0_pre_filter[8];
  
  //Detects key parameter passing
  assert_param(hcan != NULL);

  if (hcan == &hcan1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, RxData_hcan1_fifo0_pre_filter);
    CAN_Rx_Data_Filter_Config(header, &RxData_hcan1_fifo0, RxData_hcan1_fifo0_pre_filter);
  }
  else if (hcan == &hcan2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, RxData_hcan2_fifo0_pre_filter);
    CAN_Rx_Data_Filter_Config(header, &RxData_hcan2_fifo0, RxData_hcan2_fifo0_pre_filter);
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
  uint8_t RxData_hcan1_fifo1_pre_filter[8];
  uint8_t RxData_hcan2_fifo1_pre_filter[8];

  if (hcan == &hcan1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, RxData_hcan1_fifo1_pre_filter);
    CAN_Rx_Data_Filter_Config(header, &RxData_hcan1_fifo1, RxData_hcan1_fifo1_pre_filter);
  }
  else if (hcan == &hcan2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, RxData_hcan2_fifo1_pre_filter);
    CAN_Rx_Data_Filter_Config(header, &RxData_hcan2_fifo1, RxData_hcan2_fifo1_pre_filter);
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

  /**
   *  @brief Configures the CAN receive data filter based on the received message header.
   *  
   *  This function filters the received CAN data based on the StdId in the message header.
   *  It assigns the pre-filtered data to the corresponding passage in the RxDataCanFifo structure.
   *  
   *  @param[in] header The CAN message header containing the StdId.
   *  @param[out] RxData_hcanx_fifox The RxDataCanFifo structure to store the filtered data.
   *  @param[in] RxData_hcanx_fifox_pre_filter The pre-filtered data to be assigned to the RxDataCanFifo structure.
   *  
   *  @return None
   */
  void CAN_Rx_Data_Filter_Config(CAN_RxHeaderTypeDef header, RxDataCanFifo* RxData_hcanx_fifox, uint8_t* RxData_hcanx_fifox_pre_filter)
  {
    switch (header.StdId)
    {
      case 0x201:
        for (int i = 0; i < 8; i++)
          RxData_hcanx_fifox->passage0[i] = RxData_hcanx_fifox_pre_filter[i];
        break;
      case 0x202:
        for (int i = 0; i < 8; i++)
          RxData_hcanx_fifox->passage1[i] = RxData_hcanx_fifox_pre_filter[i];
        break;
      case 0x203:
        for (int i = 0; i < 8; i++)
          RxData_hcanx_fifox->passage2[i] = RxData_hcanx_fifox_pre_filter[i];
        break;
      case 0x204:
        for (int i = 0; i < 8; i++)
          RxData_hcanx_fifox->passage3[i] = RxData_hcanx_fifox_pre_filter[i];
        break;
      case 0x205:
        for (int i = 0; i < 8; i++) 
          RxData_hcanx_fifox->passage4[i] = RxData_hcanx_fifox_pre_filter[i];
        break;
      case 0x206:
        for (int i = 0; i < 8; i++)
          RxData_hcanx_fifox->passage5[i] = RxData_hcanx_fifox_pre_filter[i];
        break;
      case 0x207:
        for (int i = 0; i < 8; i++)
          RxData_hcanx_fifox->passage6[i] = RxData_hcanx_fifox_pre_filter[i];
        break;
    }
  }

  /**
   *  @brief Resets the wheel values to 0.0f if any of them exceed 10000.0f or if a specific remote control condition is met.
   *  
   *  This function checks if any of the wheel values have exceeded 10000.0f or if the remote control data structure RC_CtrlData_AT has s1 equal to 2 and s2 equal to 2.
   *  If either condition is true, it resets all wheel values to 0.0f.
   *  
   *  @param[in,out] wheel0  The first wheel value to be checked and potentially reset.
   *  @param[in,out] wheel1  The second wheel value to be checked and potentially reset.
   *  @param[in,out] wheel2  The third wheel value to be checked and potentially reset.
   *  @param[in,out] wheel3  The fourth wheel value to be checked and potentially reset.
   *  
   *  @return None
   */
  void scram(fp32_t wheel0, fp32_t wheel1, fp32_t wheel2, fp32_t wheel3)
  {
    if ((wheel0 >= 10000.0f || wheel1 >= 10000.0f || wheel2 >= 10000.0f || wheel3 >= 10000.0f) || (RC_CtrlData_AT.s1 == 2 && RC_CtrlData_AT.s2 == 2))
    {
      RC_CtrlData_AT.ch0 = 0.0f;
      RC_CtrlData_AT.ch1 = 0.0f;
      RC_CtrlData_AT.ch2 = 0.0f;
      RC_CtrlData_AT.ch3 = 0.0f;
    }
  }

  /**
   *  @brief Extracts and processes motor data from the received CAN message.
   *  
   *  This function takes in a pointer to a CANRxDataMember structure and a pointer to a uint8_t array containing the received CAN message data.
   *  It extracts the rotor angle, rotor speed, wheel linear speed, actual torque current, and motor temperature from the received data and stores them in the corresponding fields of the CANRxDataMember structure.
   *  
   *  @param[in,out] CANRxDatax A pointer to a CANRxDataMember structure to store the extracted motor data.
   *  @param[in] RxData_hcan1_fifox_passagex A pointer to a uint8_t array containing the received CAN message data.
   *  
   *  @return None
   */
  void Motor_return_data_solution(CANRxDataMember* CANRxDatax, uint8_t* RxData_hcan1_fifox_passagex)
  {
    CANRxDatax->RotorAngle = DataExtraction(RxData_hcan1_fifox_passagex[0], RxData_hcan1_fifox_passagex[1]);
    
    CANRxDatax->RotorSpeed_SlowingBef = DataExtraction(RxData_hcan1_fifox_passagex[2], RxData_hcan1_fifox_passagex[3]);
    CANRxDatax->RotorSpeed_SlowingAft = CANRxDatax->RotorSpeed_SlowingBef * REDUCTION_RATIO_M3508;
    CANRxDatax->WheelLinearSpeed = CANRxDatax->RotorSpeed_SlowingAft * 2.0f * M_PI * WHEEL_RADIUS;
 
    CANRxDatax->ActualTorqueCurrent = DataExtraction(RxData_hcan1_fifox_passagex[4], RxData_hcan1_fifox_passagex[5]);
    
    CANRxDatax->MotorTemperature = DataExtraction(0x00, RxData_hcan1_fifox_passagex[6]);
  }
