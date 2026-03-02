/***************************************************************************//**
* \file usb_echo_device.c
* \version 1.0
*
* Implements the data loopback and source/sink logic for the FX2G3 USB Echo
* Device application.
*
*
* \copyright
* (c) (2026), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**********************************************************************************/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "cy_pdl.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "usb_echo_device.h"
#include "usb_app.h"
#include "cy_fault_handlers.h"
#include "cy_debug.h"

extern bool Cy_USB_ConnectionEnable (cy_stc_usb_app_ctxt_t *pAppCtxt);

extern void OutEpDma_ISR(uint8_t endpNumber);
extern void InEpDma_ISR(uint8_t endpNumber);
extern cy_stc_hbdma_buf_mgr_t HBW_BufMgr;

cy_israddress GetEPInDmaIsr(uint8_t epNum);
cy_israddress GetEPOutDmaIsr(uint8_t epNum);

#if APP_SRC_SNK_EN
unsigned int echodevice = ECHO_DEVICE_SRC_SNK;
#else
unsigned int echodevice = ECHO_DEVICE_LOOPBACK;
#endif /* APP_SRC_SNK_EN */

extern uint32_t Ep0TestBuffer[1024U];


/**
 * \name Cy_USB_InitializeWriteBuffer
 * \brief This Function initialize egress/write buffer to a value.
 * \param pUsbdCtxt USBD layer context pointer
 * \retval None
 */
void 
Cy_USB_InitializeWriteBuffer (uint8_t *pWriteBuffer, uint32_t len)
{
    uint32_t i;
    for (i = 0x00; i < len; i++) {
        *(pWriteBuffer + i) = 0x55;
    }
}   /* End of function */

/**
 * \name HbDma_Cb
 * \brief   HBDMA callback function to manage data transfers on the USB Audio Class endpoint
 * \param handle HBDMA channel handle
 * \param cy_en_hbdma_cb_type_t HBDMA channel type
 * \param pbufStat fHBDMA buffer status
 * \param userCtx user context
 * \retval None
 */
void HbDma_Cb (cy_stc_hbdma_channel_t *handle,
                          cy_en_hbdma_cb_type_t type,
                          cy_stc_hbdma_buff_status_t *pbufStat,
                          void *userCtx)
                          
 {
    
    cy_en_hbdma_mgr_status_t   status;
    cy_stc_hbdma_buff_status_t rdBufStat;
    cy_stc_hbdma_buff_status_t wrBufStat;
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;
    uint8_t endp = 0;

    if (type == CY_HBDMA_CB_PROD_EVENT)
    {
        if(echodevice == ECHO_DEVICE_SRC_SNK)
        {
    
            /* Discard the buffer to drop the data */
            status = Cy_HBDma_Channel_DiscardBuffer(handle, &rdBufStat);
            if (status != CY_HBDMA_MGR_SUCCESS)
            {
                DBG_APP_ERR("Echo: HB-DMA GetBuffer Error\r\n");
                return;
            }
            
            
        }
        else if (echodevice == ECHO_DEVICE_LOOPBACK) 
        {
            
            for(uint8_t i = 1; i <= CY_USB_NUM_ENDP_CONFIGURED; i++)
            {
                if(handle == pAppCtxt->pOutEpDma[i])
                {
                    endp = i;
                    break;
                }
            }
            
            if(endp != 0)
            {
                /* Get the read buffer. */
                status = Cy_HBDma_Channel_GetBuffer(handle, &rdBufStat);
                if (status != CY_HBDMA_MGR_SUCCESS)
                {
                    DBG_APP_ERR("Echo: HB-DMA GetBuffer Error\r\n");
                    return;
                }
                
                /* Get the write buffer */
                status = Cy_HBDma_Channel_GetBuffer(pAppCtxt->pInEpDma[endp],&wrBufStat);
                if (status != CY_HBDMA_MGR_SUCCESS)
                {
                    DBG_APP_ERR("Echo: HB-DMA GetBuffer Error\r\n");
                    return;
                }
                
                memcpy((uint8_t*)wrBufStat.pBuffer,(uint8_t*)rdBufStat.pBuffer, rdBufStat.count);
                
                wrBufStat.count = rdBufStat.count;
                
                /* Commit the data */
                status = Cy_HBDma_Channel_CommitBuffer(pAppCtxt->pInEpDma[endp], &wrBufStat);
                if (status != CY_HBDMA_MGR_SUCCESS)
                {
                    DBG_APP_ERR("Echo: HB-DMA GetBuffer Error\r\n");
                    return;
                }
                
                /* Discard the buffer to free up the read buffer */ 
                status = Cy_HBDma_Channel_DiscardBuffer(handle, &rdBufStat);
                if (status != CY_HBDMA_MGR_SUCCESS)
                {
                    DBG_APP_ERR("Echo: HB-DMA GetBuffer Error\r\n");
                    return;
                }
                 
            }
        }
    }
    
    if(type == CY_HBDMA_CB_CONS_EVENT)
    {
        if(echodevice == ECHO_DEVICE_SRC_SNK)
        {
            /* Wait for free write  buffer. */
            status = Cy_HBDma_Channel_GetBuffer(handle, &wrBufStat);
            if (status != CY_HBDMA_MGR_SUCCESS)
            {
                DBG_APP_ERR("Echo: HB-DMA GetBuffer Error\r\n");
                return;
            }
            
            wrBufStat.count = wrBufStat.size;
            
            /* Commit the write buffer */
            status = Cy_HBDma_Channel_CommitBuffer(handle, &wrBufStat);
            if (status != CY_HBDMA_MGR_SUCCESS)
            {
                DBG_APP_ERR("Echo: HB-DMA CommitBuffer Error\r\n");
                return;
            }
        }
    }
    
 }

/**
 * \name Cy_USB_EchoDevicePrepareToStartDataXfer
 * \brief Function takes care of all preparation to start data transfer.
 * \param pAppCtxt application layer context pointer
 * \retval None
 */
void
Cy_USB_EchoDevicePrepareToStartDataXfer (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    uint32_t endpNum;
    cy_en_hbdma_mgr_status_t mgrStat;
    cy_stc_hbdma_buff_status_t buffStat;

    /*
     * For loopback:
     * It is expected that data transfer should be started by host so
     * just call DMAread. Whatever data comes from host will be given
     * back t host.
     *
     * For Src/Snk:
     * This is infinite src and sink implementation so call DMAread/Write both.
     */
    DBG_APP_TRACE("Echo: Prepare for Data Transfer\r\n");
    switch (echodevice) {

        case ECHO_DEVICE_LOOPBACK:
        default:
            DBG_APP_INFO("Echo: LoopBack Mode\r\n");
            break;

        case ECHO_DEVICE_SRC_SNK:
            DBG_APP_INFO("Echo: SRC-SNK mode\r\n");
            /* Endpoint 0 will be taken care by USBD layer. */
            for (endpNum = 0x01; endpNum <= CY_USB_NUM_ENDP_CONFIGURED; endpNum++) 
            {
                
                if(pAppCtxt->pInEpDma[endpNum] != NULL)
                {  

                        for(uint8_t i = 0; i < CY_IFX_ECHO_MAX_QUEUE_SIZE; i++)
                        {
                            /* Wait for a free buffer. */
                            mgrStat = Cy_HBDma_Channel_GetBuffer(pAppCtxt->pInEpDma[endpNum], &buffStat);
                            if (mgrStat != CY_HBDMA_MGR_SUCCESS)
                            {
                                DBG_APP_ERR("Echo: HB-DMA GetBuffer Error %x for ep %x\r\n",mgrStat,endpNum);
                                return;
                            }
                            
                            
                            Cy_USB_InitializeWriteBuffer((uint8_t*)buffStat.pBuffer, buffStat.size);
                            buffStat.count = buffStat.size;
                         

                            /* Commit buffer  */
                            mgrStat = Cy_HBDma_Channel_CommitBuffer(pAppCtxt->pInEpDma[endpNum], &buffStat);
                            if (mgrStat != CY_HBDMA_MGR_SUCCESS)
                            {
                                DBG_APP_ERR("Echo: HB-DMA Commit Buffer Error\r\n");
                                return;
                            }
                            else 
                            {
                                DBG_APP_TRACE("Echo: HB-DMA Commit Buffer ep %x\r\n",endpNum);
                            }
                        }
                    
                    }
            }
            break;
        }
    return;
}   /* end of function */

/**
 * \name Cy_USB_EchoDeviceTaskHandler
 * \brief This function handles data transfer for source/sink echo device.
 * \param pTaskParam Task param
 * \param qMsg messgae queue pointer
 * \retval None
 */
void 
Cy_USB_EchoDeviceTaskHandler (void *pTaskParam)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    cy_stc_usbd_app_msg_t queueMsg;

    /* endpAddress  will have endpNum and direction. */
    uint8_t endpAddr;
    uint32_t endpNum;
    uint32_t lpEntryTime = 0;

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;

    BaseType_t xStatus;
    uint32_t idleLoopCnt = 0;
    cy_en_hbdma_mgr_status_t mgrStat;

    DBG_APP_INFO("Echo: Task Handler\r\n");

    /* Enable USB-3 connection and wait until it is stable. */
    vTaskDelay(250);

    /* If VBus is present, enable the USB connection. */
    pAppCtxt->vbusPresent =
    (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
#if USBFS_LOGS_ENABLE
    vTaskDelay(500);
#endif /* USBFS_LOGS_ENABLE */

    if (pAppCtxt->vbusPresent) {
        Cy_USB_ConnectionEnable(pAppCtxt);
    }

    do {
#if WATCHDOG_RESET_EN
        /* Kick The WDT to prevent RESET */
        KickWDT();
#endif /* WATCHDOG_RESET_EN */

#if LPM_ENABLE
        if ((pAppCtxt->isLpmEnabled == false) && (pAppCtxt->lpmEnableTime != 0)) {

            if (
                    (Cy_USBD_GetTimerTick() >= pAppCtxt->lpmEnableTime)
               )
            {
                pAppCtxt->isLpmEnabled  = true;
                pAppCtxt->lpmEnableTime = 0;
                Cy_USBD_LpmEnable(pAppCtxt->pUsbdCtxt);
            }
        }
#endif /* LPM_ENABLE */

        /*
         * Wait until some data is received from the queue.
         * Timeout after 100 ms.
         */
        xStatus = xQueueReceive(pAppCtxt->xQueue, &queueMsg, 1);
        if (xStatus != pdPASS) {
            idleLoopCnt++;
            if (idleLoopCnt >= 10000UL) {
                idleLoopCnt = 0;
                DBG_APP_INFO("Echo: Task Idle\r\n");
            }
        }
        else
        {
            idleLoopCnt = 0;
            endpAddr = queueMsg.data[0];
            endpNum = (endpAddr & CY_USBD_ENDP_NUM_MASK);

            switch (queueMsg.type) {
             case CY_USB_VBUS_CHANGE_INTR:
                  /* Start the debounce timer. */
                  xTimerStart(pAppCtxt->vbusDebounceTimer, 0);
                  break;

              case CY_USB_VBUS_CHANGE_DEBOUNCED:
                  /* Check whether VBus state has changed. */
                  pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);

                  if (pAppCtxt->vbusPresent) {
                      if (!pAppCtxt->usbConnected) {
                          DBG_APP_INFO("USB: Enabling USB connection due to VBus detect\r\n");
                          Cy_USB_ConnectionEnable(pAppCtxt);
                      }
                  } else {
                      if (pAppCtxt->usbConnected) {
                          DBG_APP_INFO("USB: Disabling USB connection due to VBus removal\r\n");
                          Cy_USB_ConnectionDisable(pAppCtxt);
                      }
                  }
                  break;

                case CY_USB_ECHO_DEVICE_MSG_SETUP_DATA_XFER:
                    /*
                    * setup_data should be common for src/snk and loopback.
                    * Initialize endpoint number, max packet size.
                    * Device endpoint related to interface/functionality starts 
                    * from endpoint 1 (if there).
                    * For loopback endpoint pair will be 1-1,2-2...15-15.
                    */
                    DBG_APP_INFO("Echo: Setup data transfer\r\n");
                    for (endpNum = 0x01; endpNum <= CY_USB_NUM_ENDP_CONFIGURED; endpNum++) {
                        
                        if(pAppCtxt->pInEpDma[endpNum] != NULL)
                        {
                            /* Enable the channel for data transfer. */
                            mgrStat = Cy_HBDma_Channel_Enable(pAppCtxt->pInEpDma[endpNum], 0);
                            DBG_APP_TRACE("Echo: IN  channel enable status: %x\r\n", mgrStat);
                        }
                        
                        if(pAppCtxt->pOutEpDma[endpNum] != NULL)
                        {
                            /* Enable the channel for data transfer. */
                            mgrStat = Cy_HBDma_Channel_Enable(pAppCtxt->pOutEpDma[endpNum], 0);
                            DBG_APP_TRACE("Echo: OUT  channel enable status: %x\r\n", mgrStat);
                           
                        }
                        
                    }
                    break;

                case CY_USB_ECHO_DEVICE_MSG_START_DATA_XFER:
                    DBG_APP_INFO("Echo: Start data transfer\r\n");
                    Cy_USB_EchoDevicePrepareToStartDataXfer(pAppCtxt);
                    break;
    
                case CY_USB_ECHO_DEVICE_MSG_STOP_DATA_XFER:

                    /* same for loopback and src/sink */
                    for (endpNum = 0x01; endpNum <= CY_USB_NUM_ENDP_CONFIGURED; endpNum++) 
                    {
                        if (pAppCtxt->pInEpDma[endpNum] != NULL)
                        {
                            Cy_HBDma_Channel_Reset(pAppCtxt->pInEpDma[endpNum]);
                        }
                        
                        /* Flush and reset the endpoint and clear the STALL bit. */
                        Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, endpNum, CY_USB_ENDP_DIR_IN);
                        Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt, endpNum, CY_USB_ENDP_DIR_IN, false);
                        Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, (cy_en_usb_endp_dir_t)endpNum, CY_USB_ENDP_DIR_IN, false);
                    }
   
    
                    for (endpNum = 0x01; endpNum <= CY_USB_NUM_ENDP_CONFIGURED; endpNum++)
                    {
                        if (pAppCtxt->pOutEpDma[endpNum] != NULL)
                        {
                            Cy_HBDma_Channel_Reset(pAppCtxt->pOutEpDma[endpNum]);
                        }
                        
                        /* Flush and reset the endpoint and clear the STALL bit. */
                        Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, endpNum, CY_USB_ENDP_DIR_OUT);
                        Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt, endpNum, CY_USB_ENDP_DIR_OUT, false);
                        Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, (cy_en_usb_endp_dir_t)endpNum, CY_USB_ENDP_DIR_OUT, false);

                    }

                    break;

                case CY_USB_ECHO_DEVICE_MSG_CTRL_XFER_SETUP:
                    DBG_APP_TRACE("Echo: Control request\r\n");
                    Cy_USB_EchoDeviceHandleCtrlSetup((void *)pAppCtxt, &queueMsg);
                    break;

                case CY_USB_ENDP0_READ_TIMEOUT:
                    DBG_APP_TRACE("Echo: EP0 Read timeout\r\n");
                    /*
                    * When application layer wants to recieve data from
                    * host through endpoint 0 then device initiate RcvEndp0
                    * function call and start timer. When timer ends and still
                    * data is not recieved then TIMER interrupt will send
                    * CY_USB_ENDP0_READ_TIMEOUT message. If data is recieved then
                    * case which handles data should stop timer.
                    */
                    Cy_USB_USBD_RetireRecvEndp0Data(pAppCtxt->pUsbdCtxt);
                    break;

                case CY_USB_ECHO_DEVICE_MSG_L1_SLEEP:
                    DBG_APP_TRACE("Echo:L1 Sleep \r\n");
                    /* As of now nothing needs to be done here. */
                    break;

                case CY_USB_ECHO_DEVICE_MSG_L1_RESUME:
                    DBG_APP_TRACE("Echo:L1 Resume \r\n");
                    /* As of now nothing needs to be done here. */
                    break;

                default:
                    DBG_APP_ERR("EchoMsgDefault %d\r\n", queueMsg.type);
                    break;
            }   /* end of switch() */
        }

        /*
         * If the link has been in USB2-L1 for more than 0.5 seconds, initiate LPM exit so that
         * transfers do not get delayed significantly.
         */
        if ((MXS40USBHSDEV_USBHSDEV->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L1_SLEEP) != 0)
        {
            if ((Cy_USBD_GetTimerTick() - lpEntryTime) >= 500UL) {
                lpEntryTime = Cy_USBD_GetTimerTick();
                Cy_USBD_GetUSBLinkActive(pAppCtxt->pUsbdCtxt);
            }
        } else {
            lpEntryTime = Cy_USBD_GetTimerTick();
        }

    } while (1);
}   /* End of function  */

/**
 * \name Cy_USB_EchoDeviceHandleCtrlSetup
 * \brief This function handles control command given to application.
 * \param pApp application layer context pointer
 * \param pMsg app messgae queue pointer
 * \retval None
 */
void
Cy_USB_EchoDeviceHandleCtrlSetup (void *pApp, cy_stc_usbd_app_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    cy_en_usb_endp_dir_t endpDir = CY_USB_ENDP_DIR_INVALID;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    uint32_t  setupData0;
    uint32_t  setupData1;
    uint8_t bmRequest, bRequest, bTarget;
    uint16_t wValue, wIndex, wLength;
    uint8_t   reqType;
    bool isReqHandled = false;
    uint8_t loopCount = 250u;

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pApp;

    setupData0 = pMsg->data[0];
    setupData1 = pMsg->data[1];

    DBG_APP_TRACE("Echo: Control Setup Handler\r\n");
    /* Decode the fields from the setup request. */
    bmRequest = (uint8_t)((setupData0 & CY_USB_BMREQUEST_SETUP0_MASK) >>
                           CY_USB_BMREQUEST_SETUP0_POS);
    bRequest =  (uint8_t)((setupData0 & CY_USB_BREQUEST_SETUP0_MASK) >>
                           CY_USB_BREQUEST_SETUP0_POS);
    wValue = (uint16_t)((setupData0 & CY_USB_WVALUE_SETUP0_MASK) >>
                         CY_USB_WVALUE_SETUP0_POS);
    wIndex = (uint16_t)((setupData1 & CY_USB_WINDEX_SETUP1_MASK) >>
                         CY_USB_WINDEX_SETUP1_POS);
    wLength = (uint16_t)((setupData1 & CY_USB_WLENGTH_SETUP1_MASK) >>
                          CY_USB_WLENGTH_SETUP1_POS);

    reqType = ((bmRequest & CY_USB_CTRL_REQ_TYPE_MASK) >>
                                                CY_USB_CTRL_REQ_TYPE_POS);
    bTarget = (bmRequest & CY_USB_CTRL_REQ_RECIPENT_MASK);

    DBG_APP_TRACE("USB: Ctrl request - reqType:%x bRequest:%x bTarget:%x wValue:%x wLength: %x \r\n",reqType,bRequest,bTarget,wValue,wLength);

    switch (reqType) {

        case CY_USB_CTRL_REQ_STD:
            DBG_APP_TRACE("USB: Standard request\r\n");
            if ((bRequest == CY_USB_SC_SET_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
                (wValue == CY_USB_FEATURE_ENDP_HALT)) {
                DBG_APP_INFO("USB: Set Feature - EndpHalt\r\n");
                endpDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) :
                         (CY_USB_ENDP_DIR_OUT));
                Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt,
                                              ((uint32_t)wIndex & 0x7FUL),
                                               endpDir, true);
                Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;
            }
            
            if ((bRequest == CY_USB_SC_SET_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE)) {
                switch (wValue) {
                    case CY_USB_FEATURE_DEVICE_REMOTE_WAKE:
                        DBG_APP_INFO("USB: Set Feature - Remote Wakeup\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;
                    case CY_USB_FEATURE_U1_ENABLE:
                        DBG_APP_INFO("USB: Set Feature - U1 Enable\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;

                    case CY_USB_FEATURE_U2_ENABLE:
                        DBG_APP_INFO("USB: Set Feature - U2 Enable\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;
                    
                    default:
                    /* Unknown feature selector: Request will be stalled below. */
                    break;
                }
            }

            /* Handle FUNCTION_SUSPEND here */
            if ((bRequest == CY_USB_SC_SET_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) &&
                (wValue == 0x00)) {
                Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;
            }

            if ((bRequest == CY_USB_SC_CLEAR_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
                (wValue == CY_USB_FEATURE_ENDP_HALT)) {
                DBG_APP_TRACE("USB: Clear Feature\r\n");

                endpDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) :
                         (CY_USB_ENDP_DIR_OUT));
                Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt,
                                  ((uint32_t)wIndex & 0x7FUL), endpDir);
                Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt,
                                  ((uint32_t)wIndex & 0x7FUL), endpDir, false);
                Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt,
                                              ((uint32_t)wIndex & 0x7FUL),
                                              endpDir, false);
                Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;
            }
            
            if ((bRequest == CY_USB_SC_CLEAR_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE)) {
                switch (wValue) {
                    case CY_USB_FEATURE_DEVICE_REMOTE_WAKE:
                        DBG_APP_INFO("USB: Clear Feature - Remote wakeup\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        Cy_USBD_LpmEnable(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;

                    case CY_USB_FEATURE_U1_ENABLE:
                        DBG_APP_INFO("USB: Clear Feature - U1 Enable\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;

                    case CY_USB_FEATURE_U2_ENABLE:
                         DBG_APP_INFO("USB: Clear Feature - U2 Enable\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;
                    
                    default:
                        /*
                         * Unknown feature selector so dont handle here.
                         * just send stall.
                         */
                        isReqHandled = false;
                    break;
                }
            }

            /* Handle Microsoft OS String Descriptor request. */
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                (bRequest == CY_USB_SC_GET_DESCRIPTOR) &&
                (wValue == ((CY_USB_STRING_DSCR << 8) | 0xEE))) {

                /* Make sure we do not send more data than requested. */
                if (wLength > glOsString[0]) {
                    wLength = glOsString[0];
                }

                DBG_APP_INFO("USB: OS String\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                                      (uint8_t *)glOsString, wLength);
                if (retStatus != CY_USBD_STATUS_SUCCESS) {
                    DBG_APP_ERR("USB: Send Ep0 Failed %x\r\n",retStatus);
                }
                isReqHandled = true;
            }

            break;

        case CY_USB_CTRL_REQ_CLASS:
        case CY_USB_CTRL_REQ_VENDOR:
            DBG_APP_INFO("USB: Class requests - bRequest: %x\r\n",bRequest);

            if ((bRequest == 0xB8) && (wLength != 0) &&
                ((wValue & 0x3) == 0) && ((wValue + wLength) <= 4096U)) {
                if ((bmRequest & 0x80) != 0) {
                    retStatus =
                    Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                              ((uint8_t *)Ep0TestBuffer) + wValue,
                                              wLength);
                } else {
                    retStatus =
                    Cy_USB_USBD_RecvEndp0Data(pAppCtxt->pUsbdCtxt,
                                              ((uint8_t *)Ep0TestBuffer) + wValue,
                                              wLength);

                    while (!Cy_USBD_IsEp0ReceiveDone(pAppCtxt->pUsbdCtxt) && loopCount--) {
                        Cy_SysLib_DelayUs(10);
                    }
                }

                if (retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }

            /* Handle OS Compatibility and OS Feature requests */
            if (bRequest == MS_VENDOR_CODE) {
                /*
                 * this one is VENDOR request. As off now class and vendor
                 * request under fallback case statement.
                 */
                if (wIndex == 0x04) {
                    if (wLength > *((uint16_t *)glOsCompatibilityId)) {
                        wLength = *((uint16_t *)glOsCompatibilityId);
                    }
                    DBG_APP_INFO("USB: OS Compat\r\n");
                    retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                                 (uint8_t*)glOsCompatibilityId, wLength);
                    if (retStatus != CY_USBD_STATUS_SUCCESS) {
                        DBG_APP_ERR("USB: Send Ep0 Failed %x\r\n",retStatus);
                    }
                    isReqHandled = true;

                } else if (wIndex == 0x05) {

                    if (wLength > *((uint16_t *)glOsFeature)) {
                        wLength = *((uint16_t *)glOsFeature);
                    }
                    DBG_APP_INFO("USB: OS Feature\r\n");
                    retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                                          (uint8_t *)glOsFeature, wLength);
                    if (retStatus != CY_USBD_STATUS_SUCCESS) {
                        DBG_APP_ERR("USB: Send Ep0 Failed %x\r\n",retStatus);
                    }
                    isReqHandled = true;
                }
            }

            break;

        default:
            DBG_APP_INFO("Echo: Control setup :Default\r\n");
            break;
    }

    if(!isReqHandled) {
        Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
    return;

}   /* end of function() */


