/*******************************************************************************
* File Name: app_bt_advert.c
*
* Description: This File provides the implementations necessary for
* Bluetooth Advertisements.
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
 *                              INCLUDES
 ******************************************************************************/
#include "app_bt_advert.h"
#include "app_hw_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_bt_bonding.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"

/*******************************************************************************
 *                      MACROS / VARIABLE DEFINITIONS
 ******************************************************************************/

/* Remote Application state */
remote_app_ble_state_t current_remote_state = UNPAIRED_ON;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
/* This function sets the dummy advertisement data */
static void app_bt_set_advertisement_data(void);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/**
 *  Function name:
 *  app_bt_adv_start
 *
 *  Function Description:
 *  @brief    This function is executed if BTM_ENABLED_EVT event occurs in
 *            Bluetooth management callback.
 *
 *  @param    void
 *
 *  @return    void
 */
void app_bt_adv_start(void)
{

    /* Set Advertisement Data */
    app_bt_set_advertisement_data();

    /* Restore Bond Information from NV Storage */
    if (0 != bondinfo.slot_data[NUM_BONDED])
    {
        /* No Bonded Device */
        // printf("No Bonded Device information present \r\n");
        /* Allow new devices to bond */
        wiced_bt_set_pairable_mode(TRUE, FALSE);            // TODO Change to FALSE before final deployment
        current_remote_state = PAIRED_ADVERTISING_ANY_HOST; // TODO Change to PAIRED_ADVERTISING_KNOWN_HOST before final deployment
        app_bt_start_adv_any_host(); // TODO Change to known_host before final deployment
    }
    else
    {
        /* Bonded Device found in Non Volatile storage but no bonded device
         * found in the range or vicinity
         */
        /* Allow pairing. This depends on use case. Some Applications may not
         * allow multiple bonded devices. For quick testing we are allowing.
         */
        wiced_bt_set_pairable_mode(TRUE, FALSE);
        current_remote_state = UNPAIRED_ADVERTISING;
        app_bt_start_adv_any_host();
    }

}

/**
 * Function Name:
 * app_bt_set_advertisement_data
 *
 * Function Description:
 * @brief  Set Advertisement Data
 *
 * @param void
 *
 * @return wiced_result_t WICED_SUCCESS or WICED_failure
 */
static void app_bt_set_advertisement_data(void)
{
    /* TODO Need to clean this up using configurator as Generic Remote Control
     * appearance works across multiple Hosts and is mandated for Android TV
     */

#ifdef ATV_ADPCM

    wiced_bt_ble_advert_elem_t adv_elem[NUM_ADV_ELEM] = { 0 };
    uint8_t index           = 0;
    uint8_t flag            = BTM_BLE_LIMITED_DISCOVERABLE_MASK | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint16_t service_uuid   = UUID_SERVICE_HID;
    uint16_t appearance     = APPEARANCE_GENERIC_REMOTE_CONTROL;

    /* Advertisement Element for Advertisement Flags */
    adv_elem[index].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[index].len         = sizeof(uint8_t);
    adv_elem[index].p_data      = &flag;
    index++;

    /* Advertisement Element for Appearance */
    adv_elem[index].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    adv_elem[index].len         = app_gap_appearance_len;
    adv_elem[index].p_data      = (uint8_t*)&appearance;
    index++;

    /* Advertisement Element for Service */
    adv_elem[index].advert_type = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    adv_elem[index].len         = sizeof(uint16_t);
    adv_elem[index].p_data      = (uint8_t*)&service_uuid;
    index++;


    /* Advertisement Element for Name */
    adv_elem[index].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[index].len         = app_gap_device_name_len;
    adv_elem[index].p_data      = app_gap_device_name;
    index++;

    /* Start Advertisement */
    if( WICED_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(NUM_ADV_ELEM, adv_elem))
    {
        printf("Setting advertisement data Failed\r\n");
    }
#else
    /* Start Advertisement */
    if( WICED_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(NUM_ADV_ELEM, cy_bt_adv_packet_data))
    {
        printf("Setting advertisement data Failed\r\n");
    }
#endif

}

/**
 *  Function Name:
 *  app_bt_set_advertisement_data
 *
 * Function Description:
 * @brief Do directed advertising for the devices in the whitelist
 *
 * @param bdaddr BD Address to be added to the whitelist
 *
 * @return void
 */
void app_bt_set_adv_filter(wiced_bt_device_address_t bdaddr)
{
    /* Add the device to white list on pairing complete event */
    if( WICED_TRUE !=                                                   \
            wiced_bt_ble_update_advertising_filter_accept_list(TRUE,    \
                                                    BLE_ADDR_PUBLIC,    \
                                                    bdaddr))
    {
        printf("Failed to update White List\r\n");
    }
    /* TODO Accept all devices for now
     * Later accept only connections from bonded devices
     * BTM_BLE_ADVERT_FILTER_WHITELIST_CONNECTION_REQ_WHITELIST_SCAN_REQ
     */
    if( WICED_TRUE != wiced_btm_ble_update_advertisement_filter_policy(       \
            BTM_BLE_ADV_POLICY_FILTER_CONN_ACCEPT_SCAN) )
    {
        printf("Failed to update advertisement filter policy \r\n");
    }
}


/**
 *  Function name:
 *  app_bt_advert_state_handler
 *
 *  Function Description:
 *  @brief Remote State change handler based on advertisement state.
 *
 *  @param    current_adv_mode
 *
 *  @return    void
 */
void app_bt_advert_state_handler(wiced_bt_ble_advert_mode_t current_adv_mode)
{
    switch (current_adv_mode)
    {
        case BTM_BLE_ADVERT_OFF:
            /* If advertisements was turned off due to time out after disconnection,
             * then the previous state is PAIRED_ADVERTISING_ANY_HOST
            */
            if(app_bt_conn_id == 0)
            {
                if( (current_remote_state == PAIRED_ADVERTISING_ANY_HOST) ||
                    (current_remote_state == PAIRED_ADVERTISING_KNOWN_HOST) )
                {
                    current_remote_state = PAIRED_IDLE;
                }
                else if (current_remote_state == UNPAIRED_ADVERTISING)
                {
                    current_remote_state = UNPAIRED_IDLE;
                }
            }
            else
            {
                current_remote_state = CONNECTED_NON_ADVERTISING;
            }
            break;
        case BTM_BLE_ADVERT_DIRECTED_HIGH:
            break;
        case BTM_BLE_ADVERT_DIRECTED_LOW:
            break;
        case BTM_BLE_ADVERT_UNDIRECTED_HIGH:
            break;
        case BTM_BLE_ADVERT_UNDIRECTED_LOW:
            break;
        case BTM_BLE_ADVERT_NONCONN_HIGH:
            break;
        case BTM_BLE_ADVERT_NONCONN_LOW:
            break;
        case BTM_BLE_ADVERT_DISCOVERABLE_HIGH:
            break;
        default:
            printf("ERROR: Unknown advertisement state\r\n");
            break;
    }

}

/**
 *  Function name:
 *  app_bt_start_adv_any_host
 *
 *  Function Description:
 *  @brief This Function starts undirected Bluetooth advertisement
 *
 *  @param    void
 *
 *  @return    void
 */
void app_bt_start_adv_any_host(void)
{
    /* Start Undirected LE Advertisements */
    if(WICED_SUCCESS !=
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                      BLE_ADDR_PUBLIC,
                                      NULL))
    {
        printf("Starting undirected Bluetooth LE advertisements Failed\r\n");
    }
}


/**
 *  Function name:
 *  app_bt_stop_adv
 *
 *  Function Description:
 *  @brief This Function stops ongoing Bluetooth advertisement
 *
 *  @param    void
 *
 *  @return    void
 */
void app_bt_stop_adv(void)
{
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, BLE_ADDR_PUBLIC, NULL);
}