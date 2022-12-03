/*******************************************************************************
* File Name: app_bt_hid_atv.h
*
* Description:
* This file contains the BLE HID function defintions related to Android TV
* Bluetooth Stack.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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
 *                                Include Headers
 ******************************************************************************/
#ifndef __APP_BT_HID_ATV_H__
#define __APP_BT_HID_ATV_H__

#include "FreeRTOS.h"
#include "timers.h"

/*******************************************************************************
 *                                Constants
 ******************************************************************************/
#ifdef ATV_ADPCM
#define NUM_CC_KEYS             (17u)
#endif
#define TOTAL_KEYS              (NUM_CC_KEYS)
#define HID_CC_POWER            (0x0030)
#define HID_CC_MULTIFUNCTION    (0x0089)
#define HID_CC_SETTINGS         (0x0183)
#define HID_CC_MUTE             (0x00E2)
#define HID_CC_DPAD_UP          (0x0042)
#define HID_CC_DPAD_DOWN        (0x0043)
#define HID_CC_DPAD_LEFT        (0x0044)
#define HID_CC_DPAD_RIGHT       (0x0045)
#define HID_CC_DPAD_SELECT      (0x0041)
#define HID_CC_MENU             (0x0040)
#define HID_CC_MIC              (0x0221)
#define HID_CC_BACK             (0x0224)
#define HID_CC_PAGE_UP          (0x0042) //dpad up
#define HID_CC_PAGE_DOWN        (0x0043) //dpad down
#define HID_CC_HOME             (0x0223)
#define HID_CC_VOL_DOWN         (0x00EA)
#define HID_CC_VOL_UP           (0x00E9)

#ifdef ATV_ADPCM

// ATV Voice Service - AB5E0001-5A21-4F05-BC7D-AF01F617B664
#define UUID_ATV_VOICE_SERVICE 0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, 0x7d, 0xbc, \
                                0x05, 0x4f, 0x21, 0x5a, 0x01, 0x0, 0x5e, 0xab

// Write Characteristic - Characteristic AB5E0002-5A21-4F05-BC7D-AF01F617B664
// Used by ATV to send information to the Remote.
#define UUID_ATV_VOICE_TX_CHARACTERISTIC  0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, \
                                          0x7d, 0xbc, 0x05, 0x4f, 0x21, 0x5a, \
                                          0x02, 0x0, 0x5e, 0xab

// Read Characteristic - Characteristic AB5E0003-5A21-4F05-BC7D-AF01F617B664
// Used to return audio data in packets of 20 bytes each.
#define UUID_ATV_VOICE_RX_CHARACTERISTIC  0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, \
                                          0x7d, 0xbc, 0x05, 0x4f, 0x21, 0x5a, \
                                          0x03, 0x0, 0x5e, 0xab

// Control Characteristic - Characteristic AB5E0004-5A21-4F05-BC7D-AF01F617B664
// Used to return control data in packets of 20 bytes each.
#define UUID_ATV_VOICE_CTL_CHARACTERISTIC 0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, \
                                          0x7d, 0xbc, 0x05, 0x4f, 0x21, 0x5a, \
                                          0x04, 0x0, 0x5e, 0xab

/* Attribute Handles for Android TVs Voice over Bluetooth LE Remote */
#define HANDLE_ATV_VOICE_SERVICE                                        0xfe00
#define HANDLE_ATV_VOICE_TX_CHARACTERISTIC                              0xfe01
#define HANDLE_ATV_VOICE_TX_CHARACTERISTIC_VALUE                        0xfe02
#define HANDLE_ATV_VOICE_RX_CHARACTERISTIC                              0xfe03
#define HANDLE_ATV_VOICE_RX_CHARACTERISTIC_VALUE                        0xfe04
#define HANDLE_ATV_VOICE_RX_CLIENT_CONFIGURATION_DESCRIPTOR             0xfe05
#define HANDLE_ATV_VOICE_CTL_CHARACTERISTIC                             0xfe06
#define HANDLE_ATV_VOICE_CTL_CHARACTERISTIC_VALUE                       0xfe07
#define HANDLE_ATV_VOICE_CTL_CLIENT_CONFIGURATION_DESCRIPTOR            0xfe08
#define HANDLE_ATV_VOICE_SERVICE_END                                    HANDLE_ATV_VOICE_CTL_CLIENT_CONFIGURATION_DESCRIPTOR

/* Android TV Commands and capabilities */
#define ATV_VOICE_SERVICE_GET_CAPS_REQUEST          0x0A /* TV to remote */
#define ATV_VOICE_SERVICE_MIC_OPEN                  0x0C /* TV to remote */
#define ATV_VOICE_SERVICE_MIC_CLOSE                 0x0D /* TV to remote */
#define ATV_VOICE_SERVICE_MIC_EXTEND                0x0E /* TV to remote - 1.0 Spec */

#define ATV_VOICE_SERVICE_AUDIO_STOP                0x00 /* Remote to TV */
#define ATV_VOICE_SERVICE_AUDIO_START               0x04 /* Remote to TV */
#define ATV_VOICE_SERVICE_START_SEARCH              0x08 /* Remote to TV */
#define ATV_VOICE_SERVICE_AUDIO_SYNC                0x0A /* Remote to TV */
#define ATV_VOICE_SERVICE_GET_CAPS_RESP             0x0B /* Remote to TV */
#define ATV_VOICE_SERVICE_MIC_OPEN_ERROR            0x0C /* Remote to TV */


#define NUM_OF_20B_FRAMES                           (6u)
#define RESIDUAL_FRAME_SIZE                         (14u)

enum MAJOR
{
    DRAFT=0x00,
    VERSION_1,
    VERSION_2,
};

enum MINOR
{
    VERSION_P1=0x01,
    VERSION_P2,
    VERSION_P3,
    VERSION_P4
};

enum SUPPORTED_CODECS
{
    ADPCM_8KHZ_16BIT = 0x0001,
    ADPCM_8KHZ_16KHZ_16BIT = 0x0003,
    OPUS_ADPCM_8KHZ_16BIT = 0x0005,
    OPUS_ADPCM_8KHZ_16KHZ_16BIT = 0x0007,
};

/* 134 bytes default, DLE Dependent ; Future specs will detail exact values */
#define BYTES_PER_FRAME                         (0x0086)

/* 20 Bytes default, DLE Dependent; Future specs will detail exact values */
#define BYTES_PER_CHARACTERISTIC                (0x0014)

/* start timer when AUDIO_START is sent and stop timer when AUDIO_END is sent */
/* Terminate the timer when MIC_CLOSE is received from the ATV */
#define TYPICAL_AUDIO_TIMEOUT_IN_MS             (15000)

/* Frame Number */
#define EXACT_FRAME_NUMBER                      (0x0001)

/* ERROR CODE */
#define INVALID_CODEC_ERROR                     (0x0F01)

/* There are specifications for the quality of audio recorded on the remote */

#endif // ANDROID_REMOTE_SPEC

/*******************************************************************************
 *                                Variables
 ******************************************************************************/

extern TimerHandle_t pair_mode_timer_h;

/*******************************************************************************
 *                          Function Declarations
 ******************************************************************************/

/* ADPCM */
/* Common Function prototype to send HID Report */
void app_send_report(uint8_t keyCode, uint8_t upDownFlag);

void app_audio_timer_create(void);

void app_send_adpcm_data(uint8_t *enc_buffer);

void app_atv_audio_event_handler(uint8_t* atv_command);


#endif // __APP_BT_HID_ATV_H__