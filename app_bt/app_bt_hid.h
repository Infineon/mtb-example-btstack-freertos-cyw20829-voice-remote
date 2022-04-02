/*******************************************************************************
* File Name: app_bt_hid.h
*
* Description:
* This file contains the BLE HID task to send reports on keyscan, audio and
* battery change events to the connected HID Host.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
#ifndef __APP_BT_HID_H__
#define __APP_BT_HID_H__
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "audio.h"
#include "timers.h"
#include "cy_utils.h"
#ifdef ATV_ADPCM
#include "app_bt_hid_atv.h"
#endif
#ifdef BSA_OPUS
#include "app_bt_hid_bsa.h"
#endif

/*******************************************************************************
 *                                Constants
 ******************************************************************************/

/* Message type to differentiate Keyscan msg and Battery level msg */
#define KS_MSG_TYPE                             (1u)
#define BATT_MSG_TYPE                           (2u)

#define GENERIC_HID_SPEC
#define APP_SPECIFIC_HID

#define KEY_PRESSED                             (0u)
#define KEY_RELEASED                            (1u)

/* Make Sure the keycodes are sorted */
#define KC_MULTIFUNCTION                        (6u)
#define KC_POWER                                (8u)
#define KC_MUTE                                 (15u)
#define KC_DPAD_UP                              (16u)
#define KC_SETTINGS                             (17u)
#define KC_DPAD_RIGHT                           (18u)
#define KC_DPAD_SELECT                          (19u)
#define KC_DPAD_LEFT                            (20u)
#define KC_BACK                                 (21u)
#define KC_DPAD_DOWN                            (22u)
#define KC_MENU                                 (23u)
#define KC_VOL_UP                               (24u)
#define KC_MIC                                  (25u)
#define KC_PAGE_UP                              (26u)
#define KC_VOL_DOWN                             (27u)
#define KC_HOME                                 (28u)
#define KC_PAGE_DOWN                            (29u)

// Offset for Keyscan gaps for faster search
#define KEYCODE_OFFSET1                          (7u)
#define KEYCODE_OFFSET2                          (13u)

/*******************************************************************************
 *                                Conditional Macros
 ******************************************************************************/

#ifdef APP_SPECIFIC_HID

#define KEYBOARD_IN_REP_ID                      (0x01u)
#define CONSUMER_CTRL_IN_REP_ID                 (0x02u)
#define BATTERY_REPORT_ID                       (0x03u)


/* We might not need CCCD for Output report */
/* For Ginger Spec
 * KBD
 * CC
 * Audio in (voice data)
 * Audio Config
 * Battery
 */
#define MAX_NUM_CCCD                            (5u)

#endif // APP_SPECIFIC_HID

#ifdef GENERIC_HID_SPEC

/* Report Type */
#define INPUT_REPORT                            (0x01u)
#define OUTPUT_REPORT                           (0x02u)
#define FEATURE_REPORT                          (0x03u)

/* Client Characteristic Configuration Descriptor Size */
#define CCCD_SIZE                                (2u)   /* In Bytes (16bit) */

/* Recommended ADV Duration and Interval from HoGP */
#define HIGH_DUTY_ADV_DURATION_IN_SEC            (180u)
#define MIN_HIGH_DUTY_ADV_INTERVAL_IN_MS          (30u)
#define MAX_HIGH_DUTY_ADV_INTERVAL_IN_MS          (50u)

/* Device Initiated Connection Procedure for Bonded Devices from HoGP */
/* Start with low duty and then move to high duty as per HoGP */
#define PAIRED_LOW_DUTY_ADV_DURATION_IN_MS        (128u)   /* Directed */
#define PAIRED_HIGH_DUTY_ADV_DURATION_IN_SEC       (30u)   /* Undirected */
#define PAIRED_MIN_HIGH_DUTY_ADV_INTERVAL_IN_MS    (20u)
#define PAIRED_MAX_HIGH_DUTY_ADV_INTERVAL_IN_MS    (30u)

/* Host Initiated Connection Procedure for Bonded Devices from HoGP */
#define PAIRED_ADV_DURATION                         (0u)   /* Continuous non-stop undirected low duty */
#define PAIRED_MIN_ADV_INTERVAL_IN_MS             (100u)
#define PAIRED_MAX_ADV_INTERVAL_IN_MS             (250u)

#endif // GENERIC_HID_SPEC

/*******************************************************************************
 *                          Variable Declarations
 ******************************************************************************/

/* Keyscan Keycode value and press/released flag */
struct ks_code
{
    uint8_t keycode;
    uint8_t upDownFlag;
};

/* HID Message type : Keyscan Message or Battery level */
union msg
{
    struct ks_code ks;
    uint8_t batt_level;
};


/* HID Message and HID Message type */
struct hid_rpt_msg
{
    uint8_t msg_type;
    union msg data;
};

extern TaskHandle_t ble_task_h;
extern QueueHandle_t hid_rpt_q;

#define HID_MSG_Q_SZ            (10u)
#define HID_MSG_Q_ITEM_SZ       (sizeof(struct hid_rpt_msg))

/*******************************************************************************
 *                          Function Declarations
 ******************************************************************************/

void app_ble_task(void* pvParameters);
void app_send_audio_data(uint8_t *enc_buffer);
void app_send_batt_report(uint8_t battery_percentage);

#endif // __APP_BT_HID_H__
