/***************************************************************************//**
* \file usb_app.h
* \version 1.0
*
* \brief Provides definition used by the FX2G3 USB Echo Device application.
*
*******************************************************************************
* \copyright
* (c) (2025), Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* P4.0 is used for VBus detect functionality. */
#define VBUS_DETECT_GPIO_PORT                   (P4_0_PORT)
#define VBUS_DETECT_GPIO_PIN                    (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR                   (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE                       (0u)


/*Related to WINUSB*/
extern uint8_t glOsString[];
extern uint8_t glOsCompatibilityId[];
extern uint8_t glOsFeature[];

typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;

/* USBD layer return code shared between USBD layer and Application layer. */
typedef enum cy_en_usb_app_ret_code_ {
    CY_USB_APP_STATUS_SUCCESS=0,
    CY_USB_APP_STATUS_FAILURE,
}cy_en_usb_app_ret_code_t;

/* 
 * USB applicatio data structure which is bridge between USB system and device
 * functionality.
 * It maintains some usb system information which comes from USBD and it also
 * maintains information about device functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    uint8_t firstInitDone;
    uint8_t devAddr;
    uint8_t activeCfgNum;
    uint8_t currentAltSetting;
    cy_en_usb_device_state_t devState;
    cy_en_usb_device_state_t prevDevState;
    cy_en_usb_speed_t devSpeed;
    cy_en_usb_enum_method_t enumMethod;

    cy_stc_app_endp_dma_set_t endpInDma[CY_USB_NUM_ENDP_CONFIGURED];
    cy_stc_app_endp_dma_set_t endpOutDma[CY_USB_NUM_ENDP_CONFIGURED];
    uint8_t intfAltSetEndp[4][4];    /* Max 4INTF AND EACH INTERFACE 4 alt setting */ 
    /* Next three are related to central DMA */
    DMAC_Type *pCpuDmacBase;
    DW_Type *pCpuDw0Base;
    DW_Type *pCpuDw1Base;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;
    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt;

    /* Global Task handles */
    TaskHandle_t echoDevicetaskHandle;
    QueueHandle_t xQueue;
    void *pDevFuncCtxt;
    /* Timer functionality for endp0Rcvd */
    TimerHandle_t endp0OutTimerHandle;
    uint32_t endp0OuttimerExpiry;
    bool dataXferIntrEnabled;
    /* VBus detect status */
    bool vbusPresent;
    bool usbConnected;                                              /** Whether USB connection is enabled. */
    /* USB connection status */
    bool usbConnectDone;

    bool vbusChangeIntr;                        /** VBus change interrupt received flag. */
    TimerHandle_t vbusDebounceTimer;            /** VBus change debounce timer handle. */

    bool          isLpmEnabled;                         /* Whether LPM transitions are enabled. */
    uint32_t      lpmEnableTime;                        /* Timestamp at which LPM should be re-enabled. */
};

/**
 * \name Cy_USB_EchoDeviceInit
 * \brief function to initialize EchoDevice for source/sink.
 * \param pAppCtxt application layer context pointer
 * \retval None
 */
void  *Cy_USB_EchoDeviceInit(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_USB_AppSetupEndpDmaParamsHs
 * \details This Function will setup Endpoint and DMA related parameters for high speed
 *          device before transfer initiated.
 * \param pAppCtxt application layer context pointer.
 * \param pEndpDscr pointer to endpoint descriptor.
 * \retval None
 */
void Cy_USB_AppSetupEndpDmaParamsHs(cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *pEndpDscr);

/**
 * \name Cy_USB_AppInit
 * \brief   This function Initializes application related data structures, register callback
 *          creates task for device function.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer Context pointer
 * \param pCpuDmacBase DMAC base address
 * \param pCpuDw0Base DataWire 0 base address
 * \param pCpuDw1Base DataWire 1 base address
 * \param pHbDmaMgrCtxt HBDMA Manager Context
 * \retval None
 */
void Cy_USB_AppInit (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, 
                    DMAC_Type *pCpuDmacBase, DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);

/**
 * \name Cy_USB_AppRegisterCallback
 * \brief This function will register all calback with USBD layer.
 * \param pAppCtxt application layer context pointer.
 * \retval None
 */
void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_USB_AppInitEndpCpuDmaDscrConfig
 * \brief This function initializes DMA descriptor for endpoint 0 IN transfer.
 * \param pEndpCpuDmaDscrConfig Endpoint DMa descriptor configuration pointer
 * \param pSrcAddr data source address
 * \param pDstAddr data destination address
 * \param endpSize Endpoint size
 * \param endpDirection Endpoint direction
 * \retval None
 */
void Cy_USB_AppInitEndpCpuDmaDscrConfig(cy_stc_dma_descriptor_config_t *pEndpCpuDmaDscrConfig, 
    uint32_t *pSrcAddr, uint32_t *pDstAddr, uint32_t endpSize, 
    cy_en_usb_endp_dir_t endpDirection);

/**
 * \name Cy_USB_AppQueueRead
 * \brief Function to queue read operation on an OUT endpoint.
 * \param pAppCtxt application layer context pointer.
 * \param endpNum endpoint number.
 * \param endpDir endpoint direction
 * \param pBuffer pointer to buffer where data will be stored.
 * \param dataSize expected data size.
 * \retval None
 */
void Cy_USB_AppQueueRead(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);

/**
 * \name Cy_USB_AppReadShortPacket
 * \brief   Function to modify an ongoing DMA read operation to take care of a short
 *          packet.
 * \param pAppCtxt application layer context pointer.
 * \param endpNum endpoint number.
 * \param pktSize Size of the short packet to be read out. Can be zero in case of ZLP.
 * \retval 0x00 or Data size.
 */
uint16_t Cy_USB_AppReadShortPacket(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint16_t pktSize);

/**
 * \name Cy_USB_AppQueueWrite
 * \brief Function to queue write operation on an IN endpoint
 * \param pAppCtxt
 * \brief application layer context pointer.
 * \param endpNum endpoint number.
 * \param endpDir endpoint direction
 * \param pBuffer pointer to buffer where data is available.
 * \param dataSize size of data available in buffer.
 * \retval None
 */
void Cy_USB_AppQueueWrite(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);

/**
 * \name Cy_USB_RecvEndp0TimerCallback
 * \brief   This Function will be called when timer expires.
 *          This function posts TIMER Expiry message to echo device.
 * \param xTimer Timer handle
 * \retval None
 */
void Cy_USB_RecvEndp0TimerCallback(TimerHandle_t xTimer);

/**
 * \name Cy_USB_AppSetCfgCallback=
 * \brief Callback function will be invoked by USBD when set configuration is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer.
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetCfgCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppBusResetCallback
 * \brief Callback function will be invoked by USBD when bus detects RESET
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusResetCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppBusResetDoneCallback
 * \brief Callback function will be invoked by USBD when RESET is completed
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppBusSpeedCallback
 * \brief   Callback function will be invoked by USBD when speed is identified or
 *          speed change is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppSetupCallback
 * \brief Callback function will be invoked by USBD when SETUP packet is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetupCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppSuspendCallback
 * \brief Callback function will be invoked by USBD when Suspend signal/message is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSuspendCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppResumeCallback
 * \brief Callback function will be invoked by USBD when Resume signal/message is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppResumeCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppSetIntfCallback
 * \brief Callback function will be invoked by USBD when SET_INTERFACE is  received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppL1SleepCallback
 * \brief Callback function will be invoked by USBD when L1 Sleep message comes.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppL1SleepCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppL1ResumeCallback
 * \brief Callback function will be invoked by USBD when L1 Resume message comes.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppL1ResumeCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppZlpCallback
 * \brief Callback function will be invoked by USBD when bus ZLP is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppSlpCallback
 * \brief Callback function will be invoked by USBD when SLP message comes.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppSetFeatureCallback
 * \brief Callback function will be invoked by USBD when Set Feature comes.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetFeatureCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppClearFeatureCallback
 * \brief Callback function will be invoked by USBD when clear Feature comes.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppClearFeatureCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_ConnectionEnable
 * \brief Function used to enable USB 2.x connection.
 * \param pAppCtxt Application context structure.
 * \retval None
 */
bool Cy_USB_ConnectionEnable(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_USB_ConnectionDisable
 * \brief Function which disables the USB 2.x connection.
 * \param pAppCtxt Application context structure pointer.
 * \retval None
 */
void Cy_USB_ConnectionDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_USB_AppLightDisable
 * \brief Function which disables the USB connection.
 * \param pAppCtxt Application context structure pointer.
 * \retval None
 */
void Cy_USB_AppLightDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_USB_VbusDebounceTimerCallback
 * \brief Function for VBUS Debounce Timer callback
 * \param pAppCtxt Application context structure pointer.
 * \retval None
 */
void
Cy_USB_VbusDebounceTimerCallback (TimerHandle_t xTimer);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */

