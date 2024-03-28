/*******************************************************************************
* File Name: app_bt_hid_bsa.h
*
* Description:
* This file contains the BLE HID function defintions related to Linux BSA Stack.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2022-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
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