/*******************************************************************************
* File Name: app_bt_hid_bsa.h
*
* Description:
* This file contains the BLE HID function defintions related to Linux BSA Stack.
*
* Related Document: See README.md
*
*******************************************************************************
 * (c) 2022-2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
 *                          INCLUDE HEADERS
 ******************************************************************************/
#ifndef __APP_BT_HID_BSA_H__
#define __APP_BT_HID_BSA_H__

#include "FreeRTOS.h"
#include "timers.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
#if defined(BSA_OPUS)

#define KBD_USAGE               (0x07)
#define CC_USAGE                (0x0C)
#ifdef BSA_OPUS
#define NUM_CC_KEYS             (8u)
#endif
#define NUM_KBD_KEYS            (9u)
#define TOTAL_KEYS              (NUM_KBD_KEYS + NUM_CC_KEYS)
#define HID_KBD_POWER           (0x66)
#define HID_KBD_DPAD_UP         (0x52)
#define HID_KBD_DPAD_DOWN       (0x51)
#define HID_KBD_DPAD_LEFT       (0x50)
#define HID_KBD_DPAD_RIGHT      (0x4F)
#define HID_KBD_DPAD_SELECT     (0x58)
#define HID_KBD_BACK            (0xF1)
#define HID_KBD_PAGE_UP         (0x4B)
#define HID_KBD_PAGE_DOWN       (0x4E)
#define HID_CC_MULTIFUNCTION    (0x0089)
#define HID_CC_SETTINGS         (0x0183)
#define HID_CC_MUTE             (0x00E2)
#define HID_CC_MENU             (0x0040)
#define HID_CC_MIC              (0x0221)
#define HID_CC_HOME             (0x0223)
#define HID_CC_VOL_DOWN         (0x00EA)
#define HID_CC_VOL_UP           (0x00E9)

#endif // BSA_OPUS

/*******************************************************************************
 *                                Variables
 ******************************************************************************/

extern TimerHandle_t pair_mode_timer_h;

/*******************************************************************************
 *                          Function Declarations
 ******************************************************************************/

void app_send_report(uint8_t keyCode, uint8_t upDownFlag);

void app_send_voice_in_report(uint8_t *p_buf);

void app_bsa_audio_event_handler(uint8_t* bsa_command);


#endif // __APP_BT_HID_BSA_H__