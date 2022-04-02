/*******************************************************************************
* File Name: ble_hid.c
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
 *                              INCLUDES
 ******************************************************************************/
#include "app_bt_hid.h"
#include "app_bt_advert.h"
#include "app_bt_bonding.h"
#include "app_bt_gatt_handler.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
/* 'Home' Key press time for Pairing Mode */
#define HOME_KEY_TIMEOUT_IN_MS                  (5000)

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

/* Queue for sending KS events and battery reports to Bluetooth LE Task */
QueueHandle_t   hid_rpt_q;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

static void app_keyscan_activity_handler(uint8_t keyCode, uint8_t upDownFlag);

static void app_pair_mode_timer_create(void);

static void app_pair_mode_timer_cb(TimerHandle_t cb_params);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/


/**
 * @brief Wrapper function to send ADPCM or OPUS data depending on the flag
 *
 * @param p_buf
 */
void app_send_audio_data(uint8_t *p_buf)
{
#if defined ATV_ADPCM
    app_send_adpcm_data(p_buf);
#endif
#ifdef BSA_OPUS
    app_send_voice_in_report(p_buf);
#endif
}


/**
 * @brief Send Reports through BLE GATT notifications
 *
 * @param pvParameters
 */
void app_ble_task(void* pvParameters)
{

    struct hid_rpt_msg rpt_msg;
    BaseType_t xResult = pdFAIL;

#if  defined (ATV_ADPCM)
    /* Necessary for Android - ADPCM but optionally for BSA */
    app_audio_timer_create();

#endif // ATV_ADPCM

    /* To allow new hosts to pair */
    app_pair_mode_timer_create();

    while(1)
    {
        /* Block until a command is received */
        xResult = xQueueReceive(hid_rpt_q, &(rpt_msg), portMAX_DELAY);
        if(xResult != pdPASS)
        {
            continue;
        }

        // Msg from Keyscan task
        if(rpt_msg.msg_type == KS_MSG_TYPE)
        {
            if(KC_HOME == rpt_msg.data.ks.keycode)
            {
                if((current_remote_state == PAIRED_ADVERTISING_ANY_HOST) ||
                   (current_remote_state == CONNECTED_NON_ADVERTISING) ||
                   (current_remote_state == PAIRED_IDLE))
                {
                    if(!rpt_msg.data.ks.upDownFlag)
                    {
                        if (pdPASS != xTimerStart(pair_mode_timer_h, 10u))
                        {
                            printf("Failed to start pair mode timer!\r\n");
                            CY_ASSERT(0);
                        }
                    }
                    else
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
                }
            }
            app_keyscan_activity_handler(rpt_msg.data.ks.keycode, rpt_msg.data.ks.upDownFlag);
        }

        // Msg from Battery monitor task
        if(rpt_msg.msg_type == BATT_MSG_TYPE)
        {
            app_send_batt_report(rpt_msg.data.batt_level) ;
        }
    }
}


/**
 * @brief Send Reports through BLE GATT notifications
 *
 * @param keyCode       The unique keycode generated for each button
 * @param upDownFlag    Key pressed or released flag
 *
 */
static void app_keyscan_activity_handler(uint8_t keyCode, uint8_t upDownFlag)
{
    switch(current_remote_state)
    {
        case PAIRED_ON:
            /* Pairing Succesful, GATT connection is not yet done */
            printf("Device is in PAIRED_ON state\r\n");
            /* Start advertisements to any host */
            app_bt_start_adv_any_host();
            break;

        case UNPAIRED_ADVERTISING:
            /* Remote is already advertising */
            printf("Device is in UNPAIRED_ADVERTISING state\r\n");
            break;

        case PAIRED_ADVERTISING_KNOWN_HOST:
            /* TODO Remote is already advertising */
            printf("Device is in PAIRED_ADVERTISING_KNOWN_HOST state\r\n");
            break;

        case PAIRED_ADVERTISING_ANY_HOST:
            /* TODO Remote is already advertising */
            printf("Device is in PAIRED_ADVERTISING_ANY_HOST state\r\n");
            break;

        case UNPAIRED_IDLE:
            /* When woke up from sleep */
            printf("Device is in UNPAIRED_IDLE state\r\n");
            /* Start advertisements to any host */
            current_remote_state = UNPAIRED_ADVERTISING;
            app_bt_start_adv_any_host();
            break;

        case PAIRED_IDLE:
            /* When woke up from sleep */
            printf("Device is in PAIRED_IDLE state\r\n");
            /* Start advertisements to known host */
            current_remote_state = PAIRED_ADVERTISING_ANY_HOST;
            app_bt_start_adv_any_host(); // TODO change to known host before final deployment
            break;

        case CONNECTED_NON_ADVERTISING:
            /* Device is paired and connected. Send data to connected device */
            printf("Device is in CONNECTED_NON_ADVERTISING state\r\n");
            app_send_report(keyCode, upDownFlag);
            // TODO start timer on down flag of home buton
            break;

        case CONNECTED_ADVERTISING:
            /* TODO When the device is put into pairing mode for a new device to
             * pair other than the connected device
            */
            printf("Device is in CONNECTED_ADVERTISING state\r\n");
            break;

        case UNPAIRED_ON:
            /* Device is ON but the stack/hardware is not yet initialized */
            printf("Device is in UNPAIRED_ON state\r\n");
            break;

        default:
            printf("ERROR: Unknown Remote state\r\rn");
    }
}



/**
 * @brief Timer init for Home Key pairing mode
 *
 */
static void app_pair_mode_timer_create(void)
{
    pair_mode_timer_h = xTimerCreate("Pair Mode Timer",
                                    HOME_KEY_TIMEOUT_IN_MS,
                                    pdFALSE,
                                    NULL ,
                                    app_pair_mode_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL ==  pair_mode_timer_h)
    {
        printf("Pair mode timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
}

/**
* @brief Timer callback to enter pairing mode
*
* @param cb_params argument to the callback
*/
static void app_pair_mode_timer_cb(TimerHandle_t cb_params)
{
   (void)cb_params;

    app_bt_delete_bond_info();
    wiced_bt_set_pairable_mode(TRUE, FALSE);
    app_bt_conn_id = 0;
    app_bt_start_adv_any_host();
    printf("Bond Info removed \r\n");
    current_remote_state = UNPAIRED_ADVERTISING;

}
