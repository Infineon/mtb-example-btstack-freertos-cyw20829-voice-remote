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
 *                                Include Headers
 ******************************************************************************/
#ifndef __APP_BT_HID_H__
#define __APP_BT_HID_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


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
void ble_task_init(void);

#endif // __APP_BT_HID_H__
