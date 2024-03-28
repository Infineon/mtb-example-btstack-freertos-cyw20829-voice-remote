/*******************************************************************************
* File Name: app_bt_hid_atv.c
*
* Description:
* This file contains the BLE HID function defintions related to Android TV
* Bluetooth Stack.
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
 *                                Include Headers
 ******************************************************************************/
#include "app_bt_hid.h"
#include "app_bt_hid_atv.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_bt_utils.h"
#include "app_hw_handler.h"
#include "cycfg_gatt_db.h"
#include "audio.h"
#include "wiced_bt_l2c.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
/* Length in bytes as specified in the Voice over BLE specification */
#define ATVS_READ_CHAR_LEN              (134)

/*******************************************************************************
 *                            Global Variables
 ******************************************************************************/
/* struct type for mapping HID code to its corresponding keycode*/
typedef struct map_kc_cc_rpt
{
    uint8_t kc;
    uint16_t cc_code;
}map_kc_cc_rpt_t;

/* Look up table for HID code for its corresponding keycode*/
const map_kc_cc_rpt_t kc_cc_map[NUM_CC_KEYS] =
{ /* Make Sure the keycodes are sorted */
    { KC_MULTIFUNCTION, HID_CC_MULTIFUNCTION },
    { KC_POWER        , HID_CC_POWER         },
    { KC_MUTE         , HID_CC_MUTE          },
    { KC_DPAD_UP      , HID_CC_DPAD_UP       },
    { KC_SETTINGS     , HID_CC_SETTINGS      },
    { KC_DPAD_RIGHT   , HID_CC_DPAD_RIGHT    },
    { KC_DPAD_SELECT  , HID_CC_DPAD_SELECT   },
    { KC_DPAD_LEFT    , HID_CC_DPAD_LEFT     },
    { KC_BACK         , HID_CC_BACK          },
    { KC_DPAD_DOWN    , HID_CC_DPAD_DOWN     },
    { KC_MENU         , HID_CC_MENU          },
    { KC_VOL_UP       , HID_CC_VOL_UP        },
    { KC_MIC          , HID_CC_MIC           },
    { KC_PAGE_UP      , HID_CC_PAGE_UP       },
    { KC_VOL_DOWN     , HID_CC_VOL_DOWN      },
    { KC_HOME         , HID_CC_HOME          },
    { KC_PAGE_DOWN    , HID_CC_PAGE_DOWN     },
};

/* Timer to handle streaming start and stop */
TimerHandle_t audio_timer_h;

/* Timer to handle Pairing Mode */
TimerHandle_t pair_mode_timer_h;

/* Variable to add sequence ID to ADPCM / BSA_OPUS header */
static uint16_t g_seq_id = 0x0000;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

/* ADPCM */
static void app_check_n_kill_timer(void);

static void app_start_adpcm_transfer(void );

static void app_stop_adpcm_transfer(void);

static void app_atv_audio_cmd_dispatcher(uint8_t cmd);
#ifdef VOICE_REMOTE
static uint16_t find_hid_cc_code(uint8_t keyCode);
#endif
static void app_audio_timer_cb(TimerHandle_t cb_params);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
#ifdef VOICE_REMOTE
/**
 * Function Name:
 * find_hid_cc_code
 *
 * Function Description:
 * @brief This is a search function with offset for key map with gaps.
 *
 * @param uint8_t keyCode from KeyScan driver
 *
 * @return uint16_t HID Consumer control code
 *
 */
uint16_t find_hid_cc_code(uint8_t keyCode)
{
    if( (keyCode >= KC_MUTE) && (keyCode <= KC_PAGE_DOWN) )
    {
        return kc_cc_map[keyCode - KEYCODE_OFFSET2].cc_code ;
    }
    else if(keyCode == KC_MULTIFUNCTION)
    {
        return kc_cc_map[keyCode - KC_MULTIFUNCTION].cc_code;
    }
    else if(keyCode == KC_POWER)
    {
        return kc_cc_map[keyCode - KEYCODE_OFFSET1].cc_code;
    }
    else
    {
        return 0; // Not a valid keycode
    }
}
#endif

/**
 * Function Name:
 * app_send_report
 *
 * Function Description:
 * @brief Sends HID reports based on the usage page and keys available and
 * configured.
 *
 * @param uint8_t keyCode from KeyScan driver
 * @param uint8_t upDownFlag to indicate keypressed or released
 *
 * @return void
 *
 */
void app_send_report(uint8_t keyCode, uint8_t upDownFlag)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    // check for valid keycode and get the hid code for the key code
    uint16_t hid_cc_code;
    #ifdef VOICE_REMOTE
    hid_cc_code = find_hid_cc_code(keyCode);
    #else
    hid_cc_code=HID_CC_MIC;
    #endif

    if(hid_cc_code)
    {
        // check the cccd
        if( app_hids_cc_in_report_client_char_config[0] == 0x01 )
        {
            // check the key press or release flag
            if(upDownFlag == KEY_PRESSED)
            {
                memcpy( (uint8_t*)&app_hids_cc_in_report,
                        (uint8_t*)&hid_cc_code,
                        app_hids_cc_in_report_len);
                // send the HID report data
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                        HDLC_HIDS_CC_IN_REPORT_VALUE,
                                                        app_hids_cc_in_report_len,
                                                        app_hids_cc_in_report,
                                                        NULL);

                if( WICED_BT_GATT_SUCCESS == gatt_status)
                {
                    printf("Notification sent\r\n\r\n");
                }

                if(KC_MIC == keyCode)
                {
                    printf("send conn param request 16 16 10 5\r\n");
                    if (0 != wiced_bt_l2cap_update_ble_conn_params(
                            peer_bd_addr,
                            16,
                            16,
                            10,
                            SUPERVISION_TO))
                    {
                        printf("Connection parameter update successful\r\n");
                    }
                    app_atv_audio_cmd_dispatcher(ATV_VOICE_SERVICE_START_SEARCH);
                    if (pdPASS != xTimerStart(audio_timer_h, 10u))
                    {
                        printf("Failed to start audio timer!\r\n");
                        CY_ASSERT(0);
                    }
                    // printf("GATT Bearer MTU: %d \r\n",wiced_bt_gatt_get_bearer_mtu(0));
                    printf("audio timer started!\r\n");
                }
            }

            if(upDownFlag == KEY_RELEASED)
            {
                memset(app_hids_cc_in_report, 0, app_hids_cc_in_report_len);
                // send the HID report data
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                        HDLC_HIDS_CC_IN_REPORT_VALUE,
                                                        app_hids_cc_in_report_len,
                                                        app_hids_cc_in_report,
                                                        NULL);

                if( WICED_BT_GATT_SUCCESS == gatt_status)
                {
                    printf("Notification sent\r\n\r\n");
                }
            }

        }
        else
        {
            printf("Error: CC Notification disabled or incorrect usage page\r\n");
        }
    }
    else
    {
        printf("Error Invalid keycode\r\n");
    }

}


/**
 * Function Name:
 * app_atv_audio_event_handler
 *
 * Function Description:
 * @brief Android TV Transport Implementation : Handling ATV write char
 *
 * @param atv_command Command received from the GATT write operation
 *
 * @return void
 */
void app_atv_audio_event_handler(uint8_t* atv_command)
{

    switch(atv_command[0])
    {
        case ATV_VOICE_SERVICE_GET_CAPS_REQUEST:
            printf("Received ATV Get Capabilities Request\r\n");
            app_atv_audio_cmd_dispatcher(ATV_VOICE_SERVICE_GET_CAPS_RESP);
            break;

        case ATV_VOICE_SERVICE_MIC_OPEN:
            printf("Recieved MIC Open command\r\n");
            if(app_atvs_atv_read_char_atv_read_cccd[0] == 0x01)
            {
                app_start_adpcm_transfer();
                app_atv_audio_cmd_dispatcher(ATV_VOICE_SERVICE_AUDIO_START);
            }
            else
            {
                app_atv_audio_cmd_dispatcher(ATV_VOICE_SERVICE_MIC_OPEN_ERROR);
                printf("ATV Read char CCCD disabled\r\n");
            }
            break;

        case ATV_VOICE_SERVICE_MIC_CLOSE:
            printf("Recieved MIC close command\r\n");
            app_atv_audio_cmd_dispatcher(ATV_VOICE_SERVICE_AUDIO_STOP);
            app_stop_adpcm_transfer();
            printf("Number of Congestions during audio stream: %u\r\n", num_of_congestions);
            break;

        case ATV_VOICE_SERVICE_MIC_EXTEND:
            /* For 1.0 Spec */
            printf("MIC Extend\r\n");
            break;

        default:
            printf("ERROR: Unknown ATV command 0x%x\r\n", atv_command[0]);
            break;
    }
}

/**
 * Function Name:
 * app_atv_audio_cmd_dispatcher
 *
 * Function Description:
 * @brief Android TV Transport Implementation : Handling ATV control char
 *
 * @param cmd Command is the type of message specified in the spec.
 *
 * @return void
 */
void app_atv_audio_cmd_dispatcher(uint8_t cmd)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    if(app_atvs_atv_control_char_atv_control_cccd[0] == 0x01)
    {
        switch(cmd)
        {
            case ATV_VOICE_SERVICE_START_SEARCH:
                printf("ATV_VOICE_SERVICE_START_SEARCH\r\n");
                /* START_SEARCH command from remote to ATV */
                app_atvs_atv_control_char[0] = cmd;
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                        HDLC_ATVS_ATV_CONTROL_CHAR_VALUE,
                                        1,
                                        app_atvs_atv_control_char,
                                        NULL);
                break;

            case ATV_VOICE_SERVICE_AUDIO_START:
                printf("ATV_VOICE_SERVICE_AUDIO_START\r\n");
                /* ATV audio control msg */
                /* AUDIO_START from Remote to ATV */
                app_atvs_atv_control_char[0] = cmd;
                /* ADPCM implementation for ATV when the host issues command to open the Mic */
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                            HDLC_ATVS_ATV_CONTROL_CHAR_VALUE,
                                            1,
                                            app_atvs_atv_control_char,
                                            NULL);
                break;

            case ATV_VOICE_SERVICE_AUDIO_STOP:
                printf("ATV_VOICE_SERVICE_AUDIO_STOP\r\n");
                /* ATV audio control msg */
                /* AUDIO_STOP from Remote to ATV */
                app_atvs_atv_control_char[0] = cmd;
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                    HDLC_ATVS_ATV_CONTROL_CHAR_VALUE,
                                    1,
                                    app_atvs_atv_control_char,
                                    NULL);
                break;

            case ATV_VOICE_SERVICE_MIC_OPEN_ERROR:
                printf("ATV_VOICE_SERVICE_MIC_OPEN_ERROR\r\n");
                /* Timeout1 mentioned in the 0.4e Spec that will start a 1 sec timer
                * on receiving MIC_OPEN from the TV and then trigger this command
                *
                * The voice search command also needs a MIC_OPEN_ERROR to be sent
                * as per 0.4e spec
                */
                app_atvs_atv_control_char[0] = cmd;
                app_atvs_atv_control_char[1] = FROM_BIT16_TO_8(INVALID_CODEC_ERROR);
                app_atvs_atv_control_char[2] = (uint8_t)INVALID_CODEC_ERROR;
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                    HDLC_ATVS_ATV_CONTROL_CHAR_VALUE,
                                    3,
                                    app_atvs_atv_control_char,
                                    NULL);
                break;

            case ATV_VOICE_SERVICE_AUDIO_SYNC:
                app_atvs_atv_control_char[0] = cmd;
                printf("Send Audio Sync Packet\r\n");
                break;

            case ATV_VOICE_SERVICE_GET_CAPS_RESP:
                printf("ATV_VOICE_SERVICE_GET_CAPS_RESP\r\n");
                /* ADPCM implementation for ATV when the host issues command to get
                the audio capabilities */
                /* ATV audio control msg */
                app_atvs_atv_control_char[0] = 0x0B; // Get Capabilities response Command
                app_atvs_atv_control_char[1] = 0x00; // Major Version
                app_atvs_atv_control_char[2] = 0x04; // Minor Version : Draft
                app_atvs_atv_control_char[3] = 0x00; // Codec supported
                app_atvs_atv_control_char[4] = 0x01; // Codec supported: ADPCM_8KHZ_16BIT
                app_atvs_atv_control_char[5] = 0x00; // Bytes per frame
                app_atvs_atv_control_char[6] = 0x86; // Bytes per frame
                app_atvs_atv_control_char[7] = 0x00; // Bytes per characteristic
                app_atvs_atv_control_char[8] = 0x86; // Bytes per characteristic
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                    HDLC_ATVS_ATV_CONTROL_CHAR_VALUE,
                                    app_atvs_atv_control_char_len,
                                    app_atvs_atv_control_char,
                                    NULL);
                break;

            default:
                printf("ERROR: Unknown Command %x\r\n", cmd);
                break;
        }

    }
    else
    {
        printf("ATV ctrl cccd is disabled\r\n");
    }

    if(WICED_BT_GATT_SUCCESS == gatt_status)
    {
        printf("ATV ctrl char Notification sent\r\n\r\n");
    }

}


/**
 * Function Name:
 * app_start_adpcm_transfer
 *
 * Function Description:
 * @brief Start ADPCM Encoding and send over the Bluetooth LE
 *
 * @param void
 *
 * @return void
 */
static void app_start_adpcm_transfer(void)
{
    send_start_streaming_msg();
    g_seq_id = 0;
}


/**
 * Function Name:
 * app_stop_adpcm_transfer
 *
 * Function Description:
 * @brief Stop ADPCM Encoding and stop sending over Bluetooth LE
 *
 * @param void
 *
 * @return void
 */
static void app_stop_adpcm_transfer(void)
{

    send_stop_streaming_msg();

    printf("send conn param request 24 24 33 5\r\n");
    if (0 != wiced_bt_l2cap_update_ble_conn_params(
            peer_bd_addr,
            MIN_CI,
            MAX_CI,
            SLAVE_LATENCY,
            SUPERVISION_TO))
    {
        printf("Connection parameter update successful\r\n");
    }

    /* Kill the active timer as per the ATV specification */
    app_check_n_kill_timer();

}

/**
 * Function Name:
 * app_send_adpcm_data
 * Function Description:
 * @brief Send ADPCM data over Bluetooth LE
 *
 * @param uint8_t* enc_buffer The encoded buffer
 *
 * @return void
 */
void app_send_adpcm_data(uint8_t *enc_buffer)
{
    uint32_t ulNotifiedValue;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    enc_buffer[0] = (g_seq_id >> 8) & 0xFF;
    enc_buffer[1] = g_seq_id & 0xFF;
    enc_buffer[2] = 0;

    g_seq_id++;

    gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                    HDLC_ATVS_ATV_READ_CHAR_VALUE,
                                                    ATVS_READ_CHAR_LEN,
                                                    enc_buffer,
                                                    NULL);

        while (gatt_status != WICED_BT_GATT_SUCCESS)
        {
               printf("send audio gatt_status %d\r\n",gatt_status);
               xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
               gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                               HDLC_ATVS_ATV_READ_CHAR_VALUE,
                                                               ATVS_READ_CHAR_LEN,
                                                               enc_buffer,
                                                               NULL);
        }

}


/**
 * Function Name:
 * app_audio_timer_create
 *
 * Function Description:
 * @brief Timer init for mic open and close
 *
 * @param void
 *
 * @return void
 */
void app_audio_timer_create(void)
{
    audio_timer_h = xTimerCreate("Audio Timer",
                                TYPICAL_AUDIO_TIMEOUT_IN_MS,
                                pdFALSE,
                                NULL ,
                                app_audio_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL ==  audio_timer_h)
    {
        printf("Audio timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
}

/**
 * Function Name:
 * app_audio_timer_cb
 *
 * Function Description:
 * @brief Timer callback to stop streaming voice
 *
 * @param cb_params argument to the callback
 *
 * @return void
 */
static void app_audio_timer_cb(TimerHandle_t cb_params)
{
   (void)cb_params;
    app_stop_adpcm_transfer();
    printf("15 sec Voice Timed out. Num of Congestions: %u\r\n", num_of_congestions);
}

/**
 * Function Name:
 * app_check_n_kill_timer
 *
 * Function Description:
 * @brief This function stops the active timer.
 *
 * @param void
 *
 * @return void
 */
void app_check_n_kill_timer(void)
{
    if(xTimerIsTimerActive(audio_timer_h))
    {
        if (pdPASS != xTimerStop(audio_timer_h, 10u))
        {
            printf("Failed to stop audio timer!\r\n");
            CY_ASSERT(0);
        }
    }
}

/**
 * Function Name:
 * app_send_batt_report
 *
 * Function Description:
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
