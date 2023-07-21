/*******************************************************************************
* File Name: app_bt_hid_bsa.c
*
* Description:
* This file contains the BLE HID function defintions related to Linux BSA Stack.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 *                              INCLUDE HEADERS
 ******************************************************************************/
#include "app_bt_hid_bsa.h"
#include "app_bt_hid.h"
#include "audio.h"
#include "cycfg_gatt_db.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "stdlib.h"
#include "app_hw_handler.h"

/*******************************************************************************
 *                              GLOBAL VARIABLES
 ******************************************************************************/
/* HID Code */
union usage_page
{
    uint16_t cc_code;
    uint8_t kbd_code;
};

/* struct type for mapping HID code to its corresponding keycode*/
typedef struct key_usage
{
    uint8_t kc;
    uint8_t usage_page;
    union usage_page hid_code;
}key_usage_t;

/* Look up table for HID code for its corresponding keycode*/
const key_usage_t keyusage_table[TOTAL_KEYS] =
{ /* Make Sure the keycodes are sorted */
    { KC_MULTIFUNCTION , CC_USAGE  , {HID_CC_MULTIFUNCTION} },
    { KC_POWER         , KBD_USAGE , {HID_KBD_POWER       } },
    { KC_MUTE          , CC_USAGE  , {HID_CC_MUTE         } },
    { KC_DPAD_UP       , KBD_USAGE , {HID_KBD_DPAD_UP     } },
    { KC_SETTINGS      , CC_USAGE  , {HID_CC_SETTINGS     } },
    { KC_DPAD_RIGHT    , KBD_USAGE , {HID_KBD_DPAD_RIGHT  } },
    { KC_DPAD_SELECT   , KBD_USAGE , {HID_KBD_DPAD_SELECT } },
    { KC_DPAD_LEFT     , KBD_USAGE , {HID_KBD_DPAD_LEFT   } },
    { KC_BACK          , KBD_USAGE , {HID_KBD_BACK        } },
    { KC_DPAD_DOWN     , KBD_USAGE , {HID_KBD_DPAD_DOWN   } },
    { KC_MENU          , CC_USAGE  , {HID_CC_MENU         } },
    { KC_VOL_UP        , CC_USAGE  , {HID_CC_VOL_UP       } },
    { KC_MIC           , CC_USAGE  , {HID_CC_MIC          } },
    { KC_PAGE_UP       , KBD_USAGE , {HID_KBD_PAGE_UP     } },
    { KC_VOL_DOWN      , CC_USAGE  , {HID_CC_VOL_DOWN     } },
    { KC_HOME          , CC_USAGE  , {HID_CC_HOME         } },
    { KC_PAGE_DOWN     , KBD_USAGE , {HID_KBD_PAGE_DOWN   } },
};

/* Timer to handle Pairing Mode */
TimerHandle_t pair_mode_timer_h;

/* Variable to add sequence ID to ADPCM / BSA_OPUS header */
static uint16_t g_seq_id = 0x0000;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
/* Opus - BSA Transport */

// static void app_stop_voice_in_report(void);

static void app_send_stop_voice_ctl_msg(void);

static void app_send_start_voice_ctl_msg(void);


/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/**
 * Function Name:
 * find_hid_index
 *
 * Function Description:
 * @brief Find the index in the keyusage table.
 *
 * @param keyCode uint8_t keyCode from keyscan driver.
 *
 * @return int index for the respective keycode.
 */
int find_hid_index(uint8_t keyCode)
    {
        if( (keyCode >= KC_MUTE) && (keyCode <= KC_PAGE_DOWN) )
        {
            return keyCode - KEYCODE_OFFSET2;
        }
        else if(keyCode == KC_MULTIFUNCTION)
        {
            return keyCode - KC_MULTIFUNCTION;
        }
        else if(keyCode == KC_POWER)
        {
            return keyCode - KEYCODE_OFFSET1;
        }
        else
        {
            return 0; // Not a valid keycode
        }
    }


/**
 * Function Name:
 * app_send_report
 *
 * Function Description:
 * @brief Sends HID reports based on the usage page and keys available and
 * configured. This function also decides between Keyboard usage page and Consumer
 * Usage page.
 *
 * @param keyCode uint8_t keyCode from keyscan driver
 * @param upDownFlag uint8_t Key pressed or released flag
 *
 * @return void
 */
void app_send_report(uint8_t keyCode, uint8_t upDownFlag)
{
    int key_table_index = find_hid_index(keyCode);

    key_usage_t *key_p = (key_usage_t*)&keyusage_table[key_table_index];

    if(key_p != NULL)
    {
        // check the usage page for the keycode
        if( KBD_USAGE == (key_p->usage_page) &&
            (app_hids_kbd_in_report_client_char_config[0] == 0x01) )
        {
            wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

            // check the key press or release flag
            if(upDownFlag == KEY_PRESSED)
            {
                uint8_t kbd_code[3]={0};
                kbd_code[0] = key_p->hid_code.kbd_code;
                memcpy( (uint8_t*)&app_hids_kbd_in_report,
                kbd_code,app_hids_kbd_in_report_len);
                // send the HID report data
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                        HDLC_HIDS_KBD_IN_REPORT_VALUE,
                                                        app_hids_kbd_in_report_len,
                                                        app_hids_kbd_in_report,
                                                        NULL);

            }
            if(upDownFlag == KEY_RELEASED)
            {
                memset(app_hids_kbd_in_report, 0, app_hids_kbd_in_report_len);
                // send the HID report data
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                        HDLC_HIDS_KBD_IN_REPORT_VALUE,
                                                        app_hids_kbd_in_report_len,
                                                        app_hids_kbd_in_report,
                                                        NULL);
            }

            if( WICED_BT_GATT_SUCCESS == gatt_status)
            {
                printf("Notification sent\r\n");
            }
        }

        else if( CC_USAGE == (key_p->usage_page) &&
            (app_hids_cc_in_report_client_char_config[0] == 0x01) )
        {
            wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

            // check the key press or release flag
            if(upDownFlag == KEY_PRESSED)
            {
                memcpy( (uint8_t*)&app_hids_cc_in_report,
                        (uint8_t*)&(key_p->hid_code.cc_code),
                        app_hids_cc_in_report_len);
                // send the HID report data
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                        HDLC_HIDS_CC_IN_REPORT_VALUE,
                                                        app_hids_cc_in_report_len,
                                                        app_hids_cc_in_report,
                                                        NULL);

                if(KC_MIC == keyCode)
                {
                    if( (app_hids_voice_ctl_in_report_client_char_config[0] == 0x01) &&
                        (app_hids_voice_in_report_client_char_config[0] == 0x01) ){
                        app_send_start_voice_ctl_msg();
                    }else{
                        printf("CCCD for opus voice is disabled\r\n");
                    }
                }

                if(KC_HOME == keyCode)
                {
                    if (pdPASS != xTimerStart(pair_mode_timer_h, 10u))
                    {
                        printf("Failed to start pair mode timer!\r\n");
                        CY_ASSERT(0);
                    }
                }

            }
            if(upDownFlag == KEY_RELEASED)
            {
                if(KC_HOME == keyCode)
                {
                    if(xTimerIsTimerActive(pair_mode_timer_h))
                    {
                        if (pdPASS != xTimerStop(pair_mode_timer_h, 10u))
                        {
                            printf("Failed to stop pair mode timer!\r\n");
                            CY_ASSERT(0);
                        }
                    }
                }

                memset(app_hids_cc_in_report, 0, app_hids_cc_in_report_len);
                // send the HID report data
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                        HDLC_HIDS_CC_IN_REPORT_VALUE,
                                                        app_hids_cc_in_report_len,
                                                        app_hids_cc_in_report,
                                                        NULL);


                if(KC_MIC == keyCode)
                {
                    app_send_stop_voice_ctl_msg();
                }
            }

            if( WICED_BT_GATT_SUCCESS == gatt_status)
            {
                printf("Notification sent\r\n");
            }
        }
        else
        {
            printf("Error: CC or KBD Notification disabled or incorrect usage page\r\n");
        }
    }
    else
    {
        printf("Error: Invalid keycode\r\n");
    }
}


/**
 * Function Name:
 * app_send_voice_in_report
 *
 * Function Description:
 * @brief Sends Dummy opus data for BSA compatibility
 *
 * @param p_buf uint8_t* pointer for audio data
 *
 * @return void
 */
void app_send_voice_in_report(uint8_t *p_buf)
{

    uint32_t ulNotifiedValue;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    p_buf[0] = g_seq_id & 0xFF;
    g_seq_id++;

    gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                            HDLC_HIDS_VOICE_IN_REPORT_VALUE,
                                            app_hids_voice_in_report_len,
                                            p_buf,
                                            NULL);
     while (gatt_status != WICED_BT_GATT_SUCCESS)
     {
     printf("send audio gatt_status %d\r\n",gatt_status);
     xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
     gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                         HDLC_HIDS_VOICE_IN_REPORT_VALUE,
                                             app_hids_voice_in_report_len,
                                             p_buf,
                                            NULL);
     }
}

/**
 * Function Name:
 * app_bsa_audio_event_handler
 * Function Description:
 * @brief Function to start or stop audio
 *
 * @param bsa_command Data written to voice feature report
 *
 * @return void
 */
void app_bsa_audio_event_handler(uint8_t* bsa_command)
{
    if(0x02 == bsa_command[0])
    {
        printf("start streaming\r\n");
        // Start Opus encoding and send over Bluetooth LE
        send_start_streaming_msg();
    }
    else if(0x03 == bsa_command[0])
    {
        printf("Stop streaming\r\n");
        // Stop ADPCM Encoding and stop sending over Bluetooth LE
        send_stop_streaming_msg();
        printf("Number of Congestions during audio stream: %u\r\n", num_of_congestions);
    }
}

/**
 * Function Name:
 * app_send_start_voice_ctl_msg
 *
 * Function Description:
 * @brief This is a BSA specific OPUS implementation for voice control
 *
 * @param void
 *
 * @return void
 */
static void app_send_start_voice_ctl_msg(void)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    /* Start Voice CTL Message for BSA */
    app_hids_voice_ctl_in_report[0] = 0x0C;
    app_hids_voice_ctl_in_report[2] = 0x01;

    gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                HDLC_HIDS_VOICE_CTL_IN_REPORT_VALUE,
                                                app_hids_voice_ctl_in_report_len,
                                                app_hids_voice_ctl_in_report,
                                                NULL);

    memset( (uint8_t*)&app_hids_voice_ctl_in_report, 0, app_hids_voice_ctl_in_report_len);

    gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                HDLC_HIDS_VOICE_CTL_IN_REPORT_VALUE,
                                                app_hids_voice_ctl_in_report_len,
                                                app_hids_voice_ctl_in_report,
                                                NULL);
    if(gatt_status == WICED_BT_GATT_SUCCESS)
        printf("Starting voice. Voice ctl msg start sent\r\n");

}

/**
 * Function Name:
 * app_send_stop_voice_ctl_msg
 *
 * Function Description:
 * @brief This is a BSA specific OPUS implementation for voice control
 *
 * @param void
 *
 * @return void
 */
static void app_send_stop_voice_ctl_msg(void)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    /* Stop Voice CTL Message for BSA */
    app_hids_voice_ctl_in_report[0] = 0x0D;
    app_hids_voice_ctl_in_report[2] = 0x01;

    gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                HDLC_HIDS_VOICE_CTL_IN_REPORT_VALUE,
                                                app_hids_voice_ctl_in_report_len,
                                                app_hids_voice_ctl_in_report,
                                                NULL);
    if(gatt_status == WICED_BT_GATT_SUCCESS)
        printf("Stopping Voice. Voice ctl stop msg sent\r\n");
}


/**
 * @brief Function to send battery level percentage
 *
 * @param battery_percentage Battery level value in percentage
 *
 * @return void
 */
void app_send_batt_report(uint8_t battery_percentage)
{

    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    /* Check the cccd */
    if( app_bas_battery_level_client_char_config[0] == 0x01 )
    {
        memcpy( (uint8_t*)&app_bas_battery_level,
                (uint8_t*)&battery_percentage,
                app_bas_battery_level_len);

        /* send the Battery HID report data */
        gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                             HDLC_BAS_BATTERY_LEVEL_VALUE,
                                                             app_bas_battery_level_len,
                                                             app_bas_battery_level,
                                                             NULL);
        if( WICED_BT_GATT_SUCCESS == gatt_status)
        {
            printf("Battery level Notification sent\r\n\r\n");
        }
    }
}
