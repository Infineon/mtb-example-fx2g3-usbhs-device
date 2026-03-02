/***************************************************************************//**
* \file usb_echo_device.h
* \version 1.0
*
* Defines the messages and constants used in the USB Echo Device implementation.
*
*******************************************************************************
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
*******************************************************************************/

#ifndef _CY_USB_ECHO_DEVICE_H_
#define _CY_USB_ECHO_DEVICE_H_


#if defined(__cplusplus)
extern "C" {
#endif

#define CY_USB_ECHO_DEVICE_MSG_SETUP_DATA_XFER      (0x01)
#define CY_USB_ECHO_DEVICE_MSG_START_DATA_XFER      (0x02)
#define CY_USB_ECHO_DEVICE_MSG_STOP_DATA_XFER       (0x03)
#define CY_USB_ECHO_DEVICE_MSG_L1_SLEEP             (0x04)
#define CY_USB_ECHO_DEVICE_MSG_L1_RESUME            (0x05)
#define CY_USB_ENDP0_READ_TIMEOUT                   (0x06)
#define CY_USB_ECHO_DEVICE_MSG_CTRL_XFER_SETUP      (0x07)
#define CY_USB_VBUS_CHANGE_INTR                     (0x1E)
#define CY_USB_VBUS_CHANGE_DEBOUNCED                (0x1F)
#define MS_VENDOR_CODE                              (0xF0)
#define CY_USB_ECHO_DEVIE_MSG_QUEUE_SIZE            (16)
#define CY_USB_ECHO_DEVIE_MSG_SIZE                  (sizeof (cy_stc_usbd_app_msg_t))


#if APP_SRC_SNK_EN
#define CY_IFX_ECHO_MAX_QUEUE_SIZE                  (4)
#define CY_USB_MAX_DATA_BUFFER_SIZE                 (8192)
#else
#define CY_IFX_ECHO_MAX_QUEUE_SIZE                  (8)
#define CY_USB_MAX_DATA_BUFFER_SIZE                 (1024)
#endif /* APP_SRC_SNK_EN */

/* Used by application layer based on number of endp configured.*/
#define CY_USB_NUM_ENDP_CONFIGURED                  (5)
/*
 * Echo device can work in two different way.
 * 1. Loopback: Whatever data comes from Host will be given back to
 *    host.
 * 2. Soruce/Sink: Device will consuming data coming from host and it will
 *    be sending data whenever host ask for the same.
 * variable "unsigned int echodevice" should be initialize to one of them.
 */
#define ECHO_DEVICE_LOOPBACK                        (0x00)
#define ECHO_DEVICE_SRC_SNK                         (0x01)

/**
 * \name Cy_USB_EchoDeviceTaskHandler
 * \brief This function handles data transfer for source/sink echo device.
 * \param pTaskParam Task param
 * \param qMsg messgae queue pointer
 * \retval None
 */
void Cy_USB_EchoDeviceTaskHandler(void *pTaskParam);

/**
 * \name Cy_USB_EchoDeviceHandleCtrlSetup
 * \brief This function handles control command given to application.
 * \param pApp application layer context pointer
 * \param pMsg app messgae queue pointer
 * \retval None
 */
void Cy_USB_EchoDeviceHandleCtrlSetup(void *pApp, cy_stc_usbd_app_msg_t *pMsg);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_ECHO_DEVICE_H_ */

/* End of File */

