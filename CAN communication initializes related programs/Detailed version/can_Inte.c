/**
 * @file    can_Inte.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "can_Inte.h"
#include "string.h"


/**
 * @brief Initializes the CAN interface and enables interrupts for message pending in both FIFOs.
 * 
 * @param hcan Pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
 * 
 * @retval None
 */
void CAN_Inte(CAN_HandleTypeDef* hcan)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}


/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Configures the CAN filter according to the specified parameters.
 * 
 * @param  hcan         Pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
 * @param  number       Filter number. Can be (0 ~ 13).
 * @param  scale_mode   Filter scale and mode. Can be one of the following:
 *                       - CAN_FILTERSET_32BIT_IDAMASK  > One 32-bit filter and Identifier mask mode 
 *                       - CAN_FILTERSET_16BIT_IDAMASK  > Two 16-bit filters and Identifier mask mode
 *                       - CAN_FILTERSET_32BIT_IDLIST   > One 32-bit filter and Identifier list mode 
 *                       - CAN_FILTERSET_16BIT_IDLIST   > Two 16-bit filters and Identifier list mode
 * @param  RTR          Remote transmission request. Can be one of the following:
 *                       - CAN_RTR_DATA   
 *                       - CAN_RTR_REMOTE  
 * @param  id           Filter ID.                                  
 * @param  mask_id      Filter mask ID. id and mask_id can be one of the following: 
 *                       - scale is "32BIT"  > one of "0x0000 0000 - 0x1fff ffff"
 *                       - scale is "16BIT"  > tow of "0x0000 - 0x07ff"
 * @param  fifo         Filter FIFO assignment. can be one of the following: 
 *                       - CAN_FILTER_FIFO0 
 *                       - CAN_FILTER_FIFO1
 * @param  SlaveStartFilterBank   Slave start filter bank.
 * 
 * @retval None
 */
void CAN_Filter_Config(CAN_HandleTypeDef* hcan, uint32_t FilterNumber, uint32_t scale_mode, uint32_t RTR, uint32_t id, uint32_t mask_id, uint32_t fifo, uint32_t MyCAN_SlaveStartFilterBank) 
{
    /* Filter configuration structure */
    CAN_FilterTypeDef MyCAN_FilterStruct;

    /* Intermediate processing variable used when the filter is set to 16 bits */
    uint32_t Can_16Bit_SetFilterIdHigh;
    uint32_t Can_16Bit_SetFilterMaskIdHigh;

    /* Filter number */
    MyCAN_FilterStruct.FilterBank = FilterNumber;

    /* Filter set Scale and Mode as well as the id */
    switch (scale_mode)
    {
        case CAN_FILTERSET_32BIT_IDLIST : 
        {
            MyCAN_FilterStruct.FilterScale = CAN_FILTERSCALE_32BIT;
            MyCAN_FilterStruct.FilterMode = CAN_FILTERMODE_IDLIST;
            /* Set id */
            MyCAN_FilterStruct.FilterIdHigh = (id << 3) << 16;
            MyCAN_FilterStruct.FilterIdLow = (id << 3) | ((RTR == CAN_RTR_DATA) ? (0x00000004) : (0x00000006));
            /* Set mask id */
            MyCAN_FilterStruct.FilterMaskIdHigh = (mask_id << 3) << 16;
            MyCAN_FilterStruct.FilterMaskIdLow = (mask_id << 3) | ((RTR == CAN_RTR_DATA) ? (0x00000004) : (0x00000006));
        }
        case CAN_FILTERSET_16BIT_IDLIST :
        {
            MyCAN_FilterStruct.FilterScale = CAN_FILTERSCALE_16BIT;
            MyCAN_FilterStruct.FilterMode = CAN_FILTERMODE_IDLIST;
            /* Set id */
            Can_16Bit_SetFilterIdHigh = (id >> 11);
            MyCAN_FilterStruct.FilterIdHigh = ((Can_16Bit_SetFilterIdHigh << 5) << 16) | ((RTR == CAN_RTR_DATA) ? (0x00000000) : (0x00100000));
            MyCAN_FilterStruct.FilterIdLow = (id << 5) | ((RTR == CAN_RTR_DATA) ? (0x00000000) : (0x00000010));
            /* Set mask id */
            Can_16Bit_SetFilterMaskIdHigh = (mask_id >> 11);
            MyCAN_FilterStruct.FilterMaskIdHigh = ((Can_16Bit_SetFilterMaskIdHigh << 5) << 16) | ((RTR == CAN_RTR_DATA) ? (0x00000000) : (0x00100000));
            MyCAN_FilterStruct.FilterMaskIdLow = (mask_id << 5) | ((RTR == CAN_RTR_DATA) ? (0x00000000) : (0x00000010));
        }
        case CAN_FILTERSET_32BIT_IDAMASK :
        {
            MyCAN_FilterStruct.FilterScale = CAN_FILTERSCALE_32BIT;
            MyCAN_FilterStruct.FilterMode = CAN_FILTERMODE_IDMASK;
            /* Set id */
            MyCAN_FilterStruct.FilterIdHigh = (id << 3) << 16;
            MyCAN_FilterStruct.FilterIdLow = (id << 3) | ((RTR == CAN_RTR_DATA) ? (0x00000004) : (0x00000006));
            /* Set mask id */
            MyCAN_FilterStruct.FilterMaskIdHigh = (mask_id << 3) << 16;
            MyCAN_FilterStruct.FilterMaskIdLow = (mask_id << 3) | (0x00000007);
        }
        case CAN_FILTERSET_16BIT_IDAMASK :
        {
            MyCAN_FilterStruct.FilterScale = CAN_FILTERSCALE_16BIT;
            MyCAN_FilterStruct.FilterMode = CAN_FILTERMODE_IDMASK;
            /* Set id */
            Can_16Bit_SetFilterIdHigh = (id >> 11);
            MyCAN_FilterStruct.FilterIdHigh = ((Can_16Bit_SetFilterIdHigh << 5) << 16) | ((RTR == CAN_RTR_DATA) ? (0x00000000) : (0x00100000));
            MyCAN_FilterStruct.FilterIdLow = (id << 5) | ((RTR == CAN_RTR_DATA) ? (0x00000000) : (0x00000010));
            /* Set mask id */
            Can_16Bit_SetFilterMaskIdHigh = (id >> 11);
            MyCAN_FilterStruct.FilterMaskIdHigh = ((Can_16Bit_SetFilterMaskIdHigh << 5) << 16) | (0x001F0000);
            MyCAN_FilterStruct.FilterMaskIdLow = (id << 5) | (0x0000001f);    
        }
    }
    /* Filter FIFO assignment */
    MyCAN_FilterStruct.FilterFIFOAssignment = fifo;

    /* Slave start filter bank. */
    MyCAN_FilterStruct.SlaveStartFilterBank = MyCAN_SlaveStartFilterBank;
    
    /* Filter activation */
    MyCAN_FilterStruct.FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(hcan, &MyCAN_FilterStruct);

    /* Enable the FIFO message pending interrupt */
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}


/**
 * @brief Sends a CAN message using the specified handle.
 *
 * @param hcan Pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
 * @param CanSendID ID of the CAN message to be sent.
 * @param CanSendData Pointer to an array of uint8_t containing the data to be sent.
 * @param Length The length of the data to be sent.
 * @param MyCan_IDE The ID type of the CAN message. Can be CAN_ID_STD or CAN_ID_EXT.
 * @param MyCan_RTR The type of the CAN message. Can be CAN_RTR_DATA or CAN_RTR_REMOTE.
 * @param MyCan_TransmitGlobalTime The time triggered communication mode. Can be ENABLE or DISABLE.
 *
 * @retval Sent email
 */
uint32_t CAN_Send(CAN_HandleTypeDef *hcan, uint32_t CanSendID, uint8_t CanSendData[], uint32_t Length, uint32_t MyCan_IDE, uint32_t MyCan_RTR, FunctionalState MyCan_TransmitGlobalTime) 
{
    /* Email address for sending CAN packets */
    uint32_t MyCanSendMailbox;
    /* CAN message structure */
    CAN_TxHeaderTypeDef MyCanSendDataStruct;

    MyCanSendDataStruct.StdId = CanSendID; /* Standard ID */
    MyCanSendDataStruct.ExtId = CanSendID; /* Extended ID */
    MyCanSendDataStruct.DLC = Length;        /* Data length */
    MyCanSendDataStruct.IDE = MyCan_IDE;     /* Id type Standard or Extended */
    MyCanSendDataStruct.RTR = MyCan_RTR;     /* Id type Data or Remote */
    MyCanSendDataStruct.TransmitGlobalTime = MyCan_TransmitGlobalTime; /* Time Triggered Communication Mode */
    /* Send CAN message and judgment the status */
    HAL_StatusTypeDef MyCanTx_status = HAL_CAN_AddTxMessage(hcan, &MyCanSendDataStruct, CanSendData, &MyCanSendMailbox);
    
    /* Wait for the message to be sent */
    int32_t SendStatus_OK_Time = 0;
    while (MyCanTx_status != HAL_OK) {
        SendStatus_OK_Time++;
        if (SendStatus_OK_Time > 10000)
           break;
    }

    /* Handling to send CAN message failed status */
    if (MyCanTx_status != HAL_OK) {
        switch (MyCanTx_status) 
        {
        case HAL_ERROR: /* The mailbox may be full or some other serious error */
            /* code */
            break;
        case HAL_BUSY: /* The CAN module is busy and cannot process the send request immediately */
            /* code */
            break;
        case HAL_TIMEOUT: /* Operation timeout */
            /* code */
            break;
        }
    }
    return MyCanSendMailbox;
}


/**
 * @brief  Handles the reception of a CAN message in FIFO0.
 * 
 * @param  hcan  Pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
 * 
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* CAN message structure */
    CAN_RxHeaderTypeDef MyCanReceiveDataStruct;

    /* CAN message data */
    uint8_t CanRxData[8];

    /* Receive CAN message and judgment the status */
    HAL_StatusTypeDef MyCanRx_status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &MyCanReceiveDataStruct, CanRxData);
     
    /* Wait for the message to be received */
    int32_t ReceiveStatus_OK_Time = 0;
    while (MyCanRx_status != HAL_OK) {
        ReceiveStatus_OK_Time++;
        HAL_StatusTypeDef MyCanRx_status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &MyCanReceiveDataStruct, CanRxData);
        if (ReceiveStatus_OK_Time > 10000)
            break;
    }
    
    for (size_t i = 0; i < MyCanReceiveDataStruct.DLC; i++)
    {
        MyCanRxData[i] = CanRxData[i];
    }

    MyCanRxLengh = MyCanReceiveDataStruct.DLC;

    /* Gets the id type and reads the id */
    if (MyCanRx_status != HAL_OK) {
        switch (MyCanRx_status) 
        {
        case HAL_ERROR: /* The mailbox may be full or some other serious error */

            break;
        case HAL_BUSY: /* The CAN module is busy and cannot process the send request immediately */

            break;
        case HAL_TIMEOUT: /* Operation timeout */

            break;
        }
    }
}

