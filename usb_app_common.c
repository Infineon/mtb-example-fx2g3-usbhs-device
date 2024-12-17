/***************************************************************************//**
* \file usb_app_common.c
* \version 1.0
*
* Provides functions to configure USB endpoint and DMA resources for USB
* 2.x connection.
*
*******************************************************************************
* \copyright
* (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usb_usbd.h"
#include "cy_usbhs_dw_wrapper.h"
#include "usb_echo_device.h"
#include "usb_app.h"
#include "usb_app_common.h"
#include "cy_debug.h"

extern void OutEpDma_ISR(uint8_t endpNumber);
extern void InEpDma_ISR(uint8_t endpNumber);

cy_israddress GetEPInDmaIsr(uint8_t epNum);
cy_israddress GetEPOutDmaIsr(uint8_t epNum);

/*******************************************************************************
* Function name: Cy_USB_AppConfigureEndp
****************************************************************************//**
*
* This Function is used by application to configure endpoints after set
* configuration. This function should be used for all endpoints except endp0.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USB_AppConfigureEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr)
{
    cy_stc_usb_endp_config_t endpConfig;
    cy_en_usb_endp_dir_t endpDirection;
    bool valid;
    uint32_t endpType;
    uint32_t endpNum, dir;
    uint16_t maxPktSize;
    uint32_t isoPkts = 0x00;
    uint8_t burstSize = 0x00;
    uint8_t maxStream = 0x00;
    uint8_t interval = 0x00;

    DBG_APP_TRACE("Cy_USB_AppConfigureEndp >> \r\n");
    /* If it is not endpoint descriptor then return */
    if (!Cy_USBD_EndpDscrValid(pEndpDscr)) {
        DBG_APP_ERR("EndpDscrNotValid \r\n");
        return;
    }
    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNum, &maxPktSize, &dir);
    
    if (dir) {
        DBG_APP_TRACE("DIR-IN endpNum:%d \n", endpNum);
        endpDirection = CY_USB_ENDP_DIR_IN;
    } else {
        DBG_APP_TRACE("DIR-OUT endpNum:%d \n", endpNum);
        endpDirection = CY_USB_ENDP_DIR_OUT;
    }
    Cy_USBD_GetEndpType(pEndpDscr, &endpType);

    if ((CY_USB_ENDP_TYPE_ISO == endpType) ||
        (CY_USB_ENDP_TYPE_INTR == endpType)) {
        /*
         * The ISOINPKS setting in the USBHS register is the actual
         * packets per microframe value.
         */
        isoPkts = 
        ((*((uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) & CY_USB_ENDP_ADDL_XN_MASK)
        >> CY_USB_ENDP_ADDL_XN_POS) + 1;
    }

    valid = 0x01;
    Cy_USBD_GetEndpInterval(pEndpDscr, &interval);

    /* Prepare endpointConfig parameter. */
    endpConfig.endpType = (cy_en_usb_endp_type_t)endpType;
    endpConfig.endpDirection = endpDirection;
    endpConfig.valid = valid;
    endpConfig.endpNumber = endpNum;
    endpConfig.maxPktSize = (uint32_t)maxPktSize;
    endpConfig.isoPkts = isoPkts;
    endpConfig.burstSize = burstSize;
    endpConfig.streamID = (maxStream & 0x1F);
    endpConfig.interval = interval;
    /*
     * allowNakTillDmaRdy = true means device will send NAK
     * till DMA setup is ready. This field is applicable to only
     * ingress direction ie OUT transfer/OUT endpoint.
     * For Egress ie IN transfer, this field is ignored.
     */
    endpConfig.allowNakTillDmaRdy = TRUE;
    Cy_USB_USBD_EndpConfig(pUsbdCtxt, endpConfig);

    DBG_APP_TRACE("Cy_USB_AppConfigureEndp << \r\n");
    return;
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USB_AppDestroyEndpDmaParamsHs
****************************************************************************//**
*
* This Function de-couple endpoint and DMA channel for HS controller. It also
* distroys DMA channel.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* None
*
*******************************************************************************/
void
Cy_USB_AppDestroyEndpDmaParamsHs (cy_stc_usb_app_ctxt_t *pUsbApp,
                                  uint8_t *pEndpDscr)
{
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    uint32_t endpNum, endpDir;
    uint16_t maxPktSize;

    DBG_APP_TRACE("Cy_USB_AppDestroyEndpDmaParamsHs >> \r\n");
    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNum, &maxPktSize, &endpDir);

    Cy_USB_AppInitCpuDmaIntr(endpNum,
                             endpDir ? (CY_USB_ENDP_DIR_IN):(CY_USB_ENDP_DIR_OUT),
                             NULL);

    /* Updated endpoint related functions in CAL layer through USBD */
    Cy_USBD_EnableEndp(pUsbApp->pUsbdCtxt, endpNum,
                       endpDir ? (CY_USB_ENDP_DIR_IN):(CY_USB_ENDP_DIR_OUT),
                       false);
    Cy_USBD_FlushEndp(pUsbApp->pUsbdCtxt, endpNum,
                      endpDir ? (CY_USB_ENDP_DIR_IN):(CY_USB_ENDP_DIR_OUT));
    Cy_USBD_ResetEndp(pUsbApp->pUsbdCtxt, endpNum,
                      endpDir ? (CY_USB_ENDP_DIR_IN):(CY_USB_ENDP_DIR_OUT), false);

    /* This function takes care of retrieving channel from endpDmaSet */
    if (endpDir) {
        Cy_USB_AppTerminateCpuDma(pUsbApp, endpNum, CY_USB_ENDP_DIR_IN);
    } else {
        Cy_USB_AppTerminateCpuDma(pUsbApp, endpNum, CY_USB_ENDP_DIR_OUT);
    }

    if (endpDir) {
        pEndpDmaSet = &(pUsbApp->endpInDma[endpNum]);
        memset(pEndpDmaSet, 0, sizeof(cy_stc_app_endp_dma_set_t));
        pEndpDmaSet->channel = endpNum;

    } else {
        pEndpDmaSet = &(pUsbApp->endpOutDma[endpNum]);
        memset(pEndpDmaSet, 0, sizeof(cy_stc_app_endp_dma_set_t));
        pEndpDmaSet->channel = endpNum;
    }
    pUsbApp->hbChannelCreated = false;
    DBG_APP_TRACE("Cy_USB_AppDestroyEndpDmaParamsHs << \r\n");
    return;
}   /* end of function() */


/*******************************************************************************
* Function name: Cy_USB_AppTerminateCpuDma
****************************************************************************//**
*
* Function will disable associate central DMA channel.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction
*
* \return
* None
*
********************************************************************************/
void
Cy_USB_AppTerminateCpuDma (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNum,
                           cy_en_usb_endp_dir_t endpDir)
{
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;

    DBG_APP_TRACE("Cy_USB_AppTerminateCpuDma >>\r\n");

    if (endpDir == CY_USB_ENDP_DIR_OUT) {
        /* Parameter validity checks. */
        if ((pAppCtxt == NULL) || (pAppCtxt->pCpuDw0Base == NULL)) {
            DBG_APP_ERR("TerminateCpuDma: BadParam\r\n");
            return;
        }

        if (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
            /*
             * while disabling DMA channel for OUT endpoint, enable sending
             * NAK also.
             */
                Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt,
                                                endpNum,
                                                CY_USB_ENDP_DIR_OUT, true);
            }
            pEndpDmaSet = &(pAppCtxt->endpOutDma[endpNum]);
            Cy_DMA_Channel_Disable(pAppCtxt->pCpuDw0Base, pEndpDmaSet->channel);
    } else {
        /* Parameter validity checks. */
        if ((pAppCtxt == NULL) || (pAppCtxt->pCpuDw1Base == NULL)) {
            DBG_APP_ERR("TerminateCpuDma: BadParam\r\n");
            return;
        }
        /* If the DMA channel is already enabled, disable it. */
        pEndpDmaSet = &(pAppCtxt->endpInDma[endpNum]);
        Cy_DMA_Channel_Disable(pAppCtxt->pCpuDw1Base, pEndpDmaSet->channel);
    }
    DBG_APP_TRACE("Cy_USB_AppTerminateCpuDma <<\r\n");
    return;
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USB_AppTerminateDma
****************************************************************************//**
*
* Function will disable associate DMA channel.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction
*
* \return
* None
*
*******************************************************************************/
void
Cy_USB_AppTerminateDma (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNum,
                        cy_en_usb_endp_dir_t endpDir)
{
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;

    if (endpDir == CY_USB_ENDP_DIR_OUT) {
        /* Parameter validity checks. */
        if ((pAppCtxt == NULL) || (pAppCtxt->pCpuDw0Base == NULL)) {
            DBG_APP_ERR("TerminateDma: BadParam\r\n");
            return;
        }
        
        /*
            * while disabling DMA channel for OUT endpoint, enable sending
            * NAK also.
            */
        if (endpDir == CY_USB_ENDP_DIR_OUT) {
            Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt,
                                            endpNum,
                                            CY_USB_ENDP_DIR_OUT, true);
        }
        pEndpDmaSet = &(pAppCtxt->endpOutDma[endpNum]);
        Cy_DMA_Channel_Disable(pAppCtxt->pCpuDw0Base, pEndpDmaSet->channel);

    } else {
        /* Parameter validity checks. */
        if ((pAppCtxt == NULL) || (pAppCtxt->pCpuDw1Base == NULL)) {
            DBG_APP_ERR("TerminateDma: BadParam\r\n");
            return;
        }

        /* If the DMA channel is already enabled, disable it. */
        pEndpDmaSet = &(pAppCtxt->endpInDma[endpNum]);
        Cy_DMA_Channel_Disable(pAppCtxt->pCpuDw1Base, pEndpDmaSet->channel);
        
    }

    return;
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USB_AppFindValidInEndpNumber
****************************************************************************//**
*
* Find valid IN endpoint number.
*
* \param pAppCtxt
* application layer context pointer.
*
* \return
* 0x00 or endpoint number.
*
*******************************************************************************/
uint8_t
Cy_USB_AppFindValidInEndpNumber (cy_stc_usb_app_ctxt_t *pUsbApp)
{
    uint8_t endpNum = 0x00;
    uint8_t index;

    
    for (index = 0x01; index < CY_USB_NUM_ENDP_CONFIGURED; index++) {
        if  (pUsbApp->endpInDma[index].valid) {
            endpNum = index;
            break;
        }
    }
    

    return (endpNum);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USB_AppFindValidOutEndpNumber
****************************************************************************//**
*
* Find valid OUT endpoint number.
*
* \param pAppCtxt
* application layer context pointer.
*
* \return
* 0x00 or endpoint number.
*
*******************************************************************************/
uint8_t
Cy_USB_AppFindValidOutEndpNumber (cy_stc_usb_app_ctxt_t *pUsbApp)
{
    uint8_t endpNum = 0x00;
    uint8_t index;
    
    for (index = 0x01; index < CY_USB_NUM_ENDP_CONFIGURED; index++) {
        if  (pUsbApp->endpOutDma[index].valid) {
            endpNum = index;
            break;
        }
    }
    return (endpNum);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USB_AppIsOutEndpValidEnable
****************************************************************************//**
*
* Valid bit is set or not for given OUT endpoint.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \return
* TRUE if valid bit is set. FALSE if valid bit is cleared.
*
*******************************************************************************/
bool
Cy_USB_AppIsOutEndpValidEnable (cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t endpNum)
{
    if (pUsbApp->endpOutDma[endpNum].valid) {
        return TRUE;
    }
    return(FALSE);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USB_AppIsInEndpValidEnable
****************************************************************************//**
*
* Valid bit is set or not for given IN endpoint.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \return
* TRUE if valid bit is set. FALSE if valid bit is cleared.
*
*******************************************************************************/
bool
Cy_USB_AppIsInEndpValidEnable (cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t endpNum)
{
    if (pUsbApp->endpInDma[endpNum].valid) {
        return TRUE;
    }
    return(FALSE);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USB_AppGetMaxPktSize
****************************************************************************//**
*
* Function finds max packet size of an endpoint.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction
*
* \return
* Max Packet size.
*
*******************************************************************************/
uint32_t
Cy_USB_AppGetMaxPktSize (cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t endpNum,
                         cy_en_usb_endp_dir_t dir)
{
    uint32_t maxPktSize = 0x00;
    if (dir == CY_USB_ENDP_DIR_IN) {
        maxPktSize = pUsbApp->endpInDma[endpNum].maxPktSize;
    } else {
        maxPktSize = pUsbApp->endpOutDma[endpNum].maxPktSize;
    }
    return (maxPktSize);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USB_AppInitCpuDmaIntr
****************************************************************************//**
*
* Function to register an ISR for a USB endpoint DMA channel and enable
* the interrupt.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction
*
* \param userIsr
* user provided ISR function pointer.
*
* \return
* None
*
*******************************************************************************/
void 
Cy_USB_AppInitCpuDmaIntr (uint32_t endpNum, cy_en_usb_endp_dir_t endpDir,
                          cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;
    
    DBG_APP_TRACE("Cy_USB_AppInitCpuDmaIntr >>\r\n");

    if ((endpNum == 0x00) && (endpDir == CY_USB_ENDP_DIR_OUT)) {
        /* To make "RcevEndp0Data" non blocking, register ISR */
#if (!CY_CPU_CORTEX_M4)

        /* For CM0 yet to enable interrupt */
        intrCfg.intrPriority = 3;
        intrCfg.intrSrc = NvicMux6_IRQn;
        /* DW0 channels 0 onwards are used for OUT endpoints. */
        intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dmac_1_IRQn);

#else
        /* Only for OUT transfer interrupt registration required. */
        intrCfg.intrPriority = 5;
        intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dmac_1_IRQn);
#endif

       if (userIsr != NULL)  {
            /* If an ISR is provided, register it and enable the interrupt. */
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        } else {
            /* ISR is NULL. Disable the interrupt. */
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }

    }

    if ((endpNum > 0) && (endpNum < CY_USB_NUM_ENDP_CONFIGURED)) {
        DBG_APP_TRACE("Registring ISR for endp:%d endpDir %d \r\n",endpNum,endpDir);
#if (!CY_CPU_CORTEX_M4)
        intrCfg.intrPriority = 3;
        intrCfg.intrSrc = NvicMux7_IRQn;
        if (endpDir == CY_USB_ENDP_DIR_IN) {
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw1_0_IRQn + endpNum);
        } else {
            /* DW0 channels 0 onwards are used for OUT endpoints. */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw0_0_IRQn + endpNum);
        }
#else
        intrCfg.intrPriority = 5;
        if (endpDir == CY_USB_ENDP_DIR_IN) {
            DBG_APP_TRACE("DIR-IN \r\n");
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.intrSrc =
                          (IRQn_Type)(cpuss_interrupts_dw1_0_IRQn + endpNum);
        } else {
            /* DW0 channels 0 onwards are used for OUT endpoints. */
            DBG_APP_TRACE("DIR-OUT \r\n");
            intrCfg.intrSrc =
                          (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + endpNum);
        }
#endif /* (CY_CPU_CORTEX_M4) */

        if (userIsr != NULL)  {
            /* If an ISR is provided, register it and enable the interrupt. */
            DBG_APP_TRACE("Registering ISR \r\n");
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        } else {
            /* ISR is NULL. Disable the interrupt. */
            DBG_APP_TRACE("Disabling ISR\r\n");
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }
    }
    DBG_APP_TRACE("Cy_USB_AppInitCpuDmaIntr << \r\n");
}   /* end of function. */

/*******************************************************************************
* Function name: Cy_USB_AppClearCpuDmaInterrupt
****************************************************************************//**
*
* Function to clear the pending DMA interrupt associated with an endpoint.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction
*
* \return
* None
*
*******************************************************************************/
void 
Cy_USB_AppClearCpuDmaInterrupt (cy_stc_usb_app_ctxt_t *pAppCtxt,
                                uint32_t endpNum, cy_en_usb_endp_dir_t endpDir)
{

    if ((pAppCtxt != NULL) && (endpNum == 0x00) &&
        (endpDir == CY_USB_ENDP_DIR_OUT)) {
        Cy_DMAC_Channel_ClearInterrupt(pAppCtxt->pCpuDmacBase,
                                       pAppCtxt->pUsbdCtxt->channel1,
                                       CY_DMAC_INTR_COMPLETION);
    }

    if ((pAppCtxt != NULL) && (endpNum > 0) && 
        (endpNum < CY_USB_NUM_ENDP_CONFIGURED)) {
        if (endpDir == CY_USB_ENDP_DIR_IN) {
            Cy_DMA_Channel_ClearInterrupt(pAppCtxt->pCpuDw1Base, 
                                      pAppCtxt->endpInDma[endpNum].channel);
        } else  {
            Cy_DMA_Channel_ClearInterrupt(pAppCtxt->pCpuDw0Base,
                                     pAppCtxt->endpOutDma[endpNum].channel);
        }
    }
}   /* end of function. */

/*******************************************************************************
* Function name: Cy_USB_AppHandleSetCfgCommon
****************************************************************************//**
*
* Function handles common portion of set configuration call back to application.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD layer  context pointer.
*
* \param pMsg
* pointer to message coming from lower layer.
*
* \return
* CY_USB_APP_STATUS_SUCCESS in case of set Config handle without error.
* CY_USB_APP_STATUS_FAILURE in all other case.
*
*******************************************************************************/
cy_en_usb_app_ret_code_t
Cy_USB_AppHandleSetCfgCommon (cy_stc_usb_app_ctxt_t *pAppCtxt,
                              cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_setup_req_t *pSetupReq;
    cy_en_usb_speed_t devSpeed;
    uint8_t *pActiveCfg, *pIntfDscr, *pEndpDscr;
    uint8_t index, numOfIntf, numOfEndp;

    DBG_APP_TRACE("Cy_USB_AppHandleSetCfgCommon >>\r\n");

    devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);

    pAppCtxt->devSpeed = devSpeed;
    DBG_APP_INFO("SetCfg:devSpeed:%d\r\n",devSpeed);

    /* Get setup packet from message. */
    pSetupReq = (cy_stc_usb_setup_req_t *)(&(pMsg->data[0]));
    /* Disable DMA if Set Config request on Config index 0 is received */
    if(pSetupReq->wValue == 0) {
        /* TBD: Device should move to adress state so unconfigure everything. */
        DBG_APP_INFO("Set CFG 0\r\n");
        return CY_USB_APP_STATUS_FAILURE;
    }

    /* Enable Datawire. This application uses data wire for data transfer. */
    Cy_DMA_Enable(pAppCtxt->pCpuDw0Base);
    Cy_DMA_Enable(pAppCtxt->pCpuDw1Base);

    pActiveCfg = Cy_USB_USBD_GetActiveCfgDscr(pUsbdCtxt);
    if (!pActiveCfg) {
        /* Set config should be called when active config value > 0x00. */
        DBG_APP_ERR("Active config not available\r\n");
        return CY_USB_APP_STATUS_FAILURE;
    }

    numOfIntf = Cy_USBD_FindNumOfIntf(pActiveCfg);
    if (numOfIntf == 0x00) {
        DBG_APP_ERR("numOfIntf-0x00\r\n");
        return CY_USB_APP_STATUS_FAILURE;
    }
    DBG_APP_TRACE("NumOfIntf:%d\r\n", numOfIntf);

    for (index = 0x00; index < numOfIntf; index++) {
        /* During Set Config command always altSetting 0 will be active. */
        pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, index, 0x00);
        if (pIntfDscr == NULL) {
            DBG_APP_INFO("pIntfDscrNull\r\n");
            return CY_USB_APP_STATUS_FAILURE;
        }

        numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
        DBG_APP_TRACE("numOfEndp:%d\r\n", numOfEndp);
        if (numOfEndp == 0x00) {
            /* If current interface has 0 endpoint then move to next intf */
            DBG_APP_ERR("numOfEndp-0x00\r\n");
            continue;
        }

        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00) {
            /* first cleanup all channels related info then configure new. */
            Cy_USB_AppDestroyEndpDmaParamsHs(pAppCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParamsHs(pAppCtxt, pEndpDscr);
            numOfEndp--;

            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
            
        }
    }
    DBG_APP_TRACE("All endpoint Configured\r\n");

    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_CONFIGURED;
    pAppCtxt->devState = CY_USB_DEVICE_STATE_CONFIGURED;

    DBG_APP_TRACE("Cy_USB_AppHandleSetCfgCommon <<\r\n");
    return CY_USB_APP_STATUS_SUCCESS;
}   /* end of function. */

