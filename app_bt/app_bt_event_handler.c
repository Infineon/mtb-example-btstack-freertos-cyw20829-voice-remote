/*******************************************************************************
* File Name: app_bt_event_handler.c
*
* Description:
* This file contains the starting point of Bluetooth LE Voice Remote application.
* The wiced_bt_stack_init() registers for Bluetooth events in this main function.
* The Bluetooth Management callback manages the Bluetooth events and the
* application developer can customize the functionality and behavior depending on
* the Bluetooth events. The Bluetooth Management callback acts like a
* Finite State Machine (FSM) for the SoC.
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
*                               Includes
*******************************************************************************/
#include "app_bt_event_handler.h"
#include "app_bt_advert.h"
#include "app_bt_utils.h"
#include "app_bt_gatt_handler.h"
#include "app_hw_handler.h"
#include "app_bt_bonding.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#else
#include "cy_retarget_io.h"
#endif
#include "wiced_bt_gatt.h"
#if defined (RED_LED_ENABLE)
#include "app_hw_gpio.h"
#endif
#include "wiced_bt_l2c.h"
#ifdef ATV_ADPCM
#include "app_bt_hid_atv.h"
#endif

/*******************************************************************************
*                                Macros
*******************************************************************************/

/* Bitflags for LE secure pairing keys IO capabilities event */
#define PAIRING_CAPS_KEYS_FLAG      \
        (BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_PLK)
/* Key Size for LE secure pairing key IO capabilities event */
#define PAIRING_CAPS_KEY_SIZE       (16u)

/*******************************************************************************
*                               Global Variables
*******************************************************************************/
/* Status variable for connection ID */
uint16_t app_bt_conn_id;
uint8_t reset_bond_data = 0;

/*******************************************************************************
*                           Function Prototypes
*******************************************************************************/
/* This function initializes Bluetooth sub procedures */
static void app_bt_init(void);

/* This function initializes GATT DB and registers callback for GATT events */
static void app_bt_gatt_db_init(void);

void create_cpu_sleep_cb(void);
/*******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/**
 * Function Name: app_bt_management_callback()
 *
 * Function Description:
 * @brief
 *  This is a Bluetooth stack event handler function to receive management events
 *  from the Bluetooth LE stack and process as per the application.
 *
 * @param wiced_bt_management_evt_t  Bluetooth LE event code of one byte length
 * @param wiced_bt_management_evt_data_t  Pointer to Bluetooth LE management event
 *                                        structures
 *
 * @return wiced_result_t Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 */
wiced_result_t
app_bt_management_callback(wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_dev_status_t       bt_dev_status       = WICED_SUCCESS;
    wiced_bt_ble_advert_mode_t *p_adv_mode          = NULL;
    wiced_bt_dev_ble_pairing_info_t *p_pairing_info = NULL;
    wiced_bt_device_link_keys_t *p_link_key         = NULL;
    cy_rslt_t rslt;

    printf("\nBluetooth Management Event: \t");
    printf(app_get_btm_event_name(event));
    printf("\r\n");

    switch (event)
    {

    case BTM_ENABLED_EVT:
        /* Set Preferred PHY */

        /* Perform application-specific initialization */
        app_bt_init();
#if (!defined  PDM_MIC)  && (!defined ENABLE_BT_SPY_LOG)
        create_cpu_sleep_cb();
#endif
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        /* Advertisement State Changed */
        p_adv_mode = &p_event_data->ble_advert_state_changed;
        printf("\nAdvertisement state changed to ");
        printf(app_get_btm_advert_mode_name(*p_adv_mode));
        printf("\r\n");

        app_bt_advert_state_handler(*p_adv_mode);
#if defined (RED_LED_ENABLE)
        if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF )
        {
            // If the condition is true the advert is in off condition //
            printf("Entered the non advert led section \n");
            app_status_led_blinky_off(RED);

        }
         else
        {
            // If the condition is false the advert is on on condition //
            printf("entered the advert led section \n");
            app_status_led_blinky_on(RED);

        }
#endif
        break;

    case BTM_SECURITY_REQUEST_EVT:
        /* Need to compare with BT-SDK remote here for this event */
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                    WICED_BT_SUCCESS);
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        /* Set the IO capabilities */
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap =    \
            BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data =        \
            BTM_OOB_NONE;
        /* LE sec bonding */
        p_event_data->pairing_io_capabilities_ble_request.auth_req =        \
            (BTM_LE_AUTH_REQ_SC | BTM_LE_AUTH_REQ_BOND);
        p_event_data->pairing_io_capabilities_ble_request.init_keys =       \
            PAIRING_CAPS_KEYS_FLAG;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys =       \
            PAIRING_CAPS_KEYS_FLAG;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size =    \
            PAIRING_CAPS_KEY_SIZE;
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
        printf("Paired Device Link keys Request Event for device ");
        app_print_bd_address((uint8_t *)(p_event_data->paired_device_link_keys_request.bd_addr));
        /* Need to search to see if the BD_ADDR we are looking for is in Flash. If not, we return WICED_BT_ERROR and the stack */
        /* will generate keys and will then call BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that they can be stored */
        bt_dev_status = WICED_BT_ERROR;  /* Assume the device won't be found. If it is, we will set this back to WICED_BT_SUCCESS */

        /* This call will return BOND_MAX if the device is not found*/
        bondindex = app_bt_find_device_in_flash(p_event_data->paired_device_link_keys_request.bd_addr);
        if ( bondindex < BOND_MAX)
        {
            /* Check if BD_ADDR is in NV storage ; If not, throw error */
            p_link_key = &p_event_data->paired_device_link_keys_request;
            if(!app_bt_restore_cccd_using_link_key(p_link_key))
            {
                printf("Device not found\r\n");
            }
            bt_dev_status = WICED_BT_SUCCESS;
        memcpy( peer_bd_addr,
                p_event_data->paired_device_link_keys_request.bd_addr,
                sizeof(wiced_bt_device_address_t));
        printf("Peer Device BD ADDR:\r\n");
        app_print_bd_address(peer_bd_addr);
        }
        else
        {
            printf("Device Link Keys not found in the database! \n");
            bondindex = 0;
        }
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* Save the link keys of paired device to Non volatile storage */
        printf( "Paired Device Key Update \r\n");
        /* Check for valid BD address and store it in a Bond Info list */
        rslt = app_bt_save_device_link_keys(&(p_event_data->paired_device_link_keys_update));
        if (CY_RSLT_SUCCESS == rslt)
        {
            printf("Successfully Bonded to ");
            app_print_bd_address(p_event_data->paired_device_link_keys_update.bd_addr);
        }
        else
        {
            printf("Failed to bond! \r\n");
        }

        /* Print all security keys if needed */

        break;

    case BTM_PAIRING_COMPLETE_EVT:
        /* Received Pairing Complete Event */
        p_pairing_info = &p_event_data->pairing_complete.pairing_complete_info.ble;

        if ( SMP_SUCCESS == p_pairing_info->reason ) /* Bonding successful */
        {
            /* Check if device is already in bonded list;
             * if not add it and increment the number of bonded devices.
             */
            /* Update Num of bonded devices and next free slot in slot data*/
            rslt = app_bt_update_slot_data();

            /*Check if the data was updated successfully*/
            if (CY_RSLT_SUCCESS == rslt)
            {
                /* remember that the device is now bonded, so disable bonding */
                wiced_bt_set_pairable_mode(FALSE, FALSE);
                printf("Slot Data saved to Flash \r\n");
                /* Print Bond information stats once a new device is paired.
                (pairing complete event) */
                printf("Successfully Bonded to: ");
                app_print_bd_address(p_event_data->pairing_complete.bd_addr);
            }
            else
            {
                printf("Flash Write Error \r\n");
            }
            /* Set Bonded flag for state machine handling */
            current_remote_state = CONNECTED_NON_ADVERTISING;
            app_print_bond_info_stats();
        }
        else
        {
            printf("Bonding failed. Reason for failure: ");
            printf(app_get_pairing_status_name(p_pairing_info->reason));
            printf("\r\n");
            /* Delete host info and update bonded list */
        }
        /* Set Advertising Filter Policy if you do not want pairing
         * to happen with other devices */
        // app_bt_set_adv_filter(p_event_data->pairing_complete.bd_addr);

        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* Save the identity keys to the NV storage */
        printf( "Local Identity Key Update\n");
        rslt = app_bt_save_local_identity_key(p_event_data->local_identity_keys_update);
        if (CY_RSLT_SUCCESS != rslt)
        {
            bt_dev_status = WICED_BT_ERROR;
        }
        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* Retreive the identity keys from the NV storage */
        // Initialise kvstor library
        app_kv_store_init();

        if(reset_bond_data)
        {
            app_bt_delete_bond_info();
            app_bt_update_bond_data();
        }

        /*
         * If the key type is Identity keys; throw WICED_ERROR to cause the
         * BT stack to generate new keys and then call
         * BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that the keys can be stored
         * */
        printf("Local Identity Key Request\r\n");
        /*Read Local Identity Resolution Keys*/
        rslt = app_bt_read_local_identity_keys();
        if(CY_RSLT_SUCCESS == rslt)
        {
            memcpy( &(p_event_data->local_identity_keys_request),
                    &(identity_keys),
                    sizeof(wiced_bt_local_identity_keys_t));
            app_print_byte_array(&identity_keys, sizeof(wiced_bt_local_identity_keys_t));
            bt_dev_status = WICED_BT_SUCCESS;
        }
        else
        {
            bt_dev_status = WICED_BT_ERROR;
        }
        break;


    case BTM_ENCRYPTION_STATUS_EVT:
        /* Encryption Status Change */
        printf("Encryption Status event for: ");
        app_print_bd_address(p_event_data->encryption_status.bd_addr);
        printf("Encryption Status event result: %d \r\n", p_event_data->encryption_status.result);
        /* Check and retreive the index of the bond data of the device that got connected */
        /* This call will return BOND_MAX if the device is not found */
        bondindex = app_bt_find_device_in_flash(p_event_data->encryption_status.bd_addr);
        if(bondindex < BOND_MAX)
        {
            printf("Bond info present in Flash for device: ");
            app_print_bd_address(p_event_data->encryption_status.bd_addr);
        }
        else{
            printf("No Bond info present in Flash for device: ");
            app_print_bd_address(p_event_data->encryption_status.bd_addr);
            bondindex = 0;
        }

        break;

    case BTM_BLE_CONNECTION_PARAM_UPDATE:

        printf("Remote Connection parameter update status:%d,\r\n"
               "Remote Connection Interval: %d,\r\n"
               "Remote Connection Latency: %d,\r\n"
               "Remote Connection Timeout: %d\r\n",
                p_event_data->ble_connection_param_update.status,
                p_event_data->ble_connection_param_update.conn_interval,
                p_event_data->ble_connection_param_update.conn_latency,
                p_event_data->ble_connection_param_update.supervision_timeout);

        break;

    case BTM_SECURITY_FAILED_EVT:
        /* Handle pairing Failure */
        printf("Pairing Failed\r\n");
        break;

    case BTM_BLE_PHY_UPDATE_EVT:
        printf("BTM_BLE_PHY_UPDATE_EVT,\r\n "
                "PHY Tx value is: %d, \r\n"
                "PHY Rx value is: %d \r\n",
                p_event_data->ble_phy_update_event.tx_phy,
                p_event_data->ble_phy_update_event.rx_phy);
        break;

    case BTM_BLE_DATA_LENGTH_UPDATE_EVENT:
        printf("BTM_BLE_DATA_LENGTH_UPDATE_EVENT, \r\n"
                "Max tx octets is :%d ,\r\n"
                "Max rx octets is :%d \r\n",
                p_event_data->ble_data_length_update_event.max_tx_octets,
                p_event_data->ble_data_length_update_event.max_rx_octets);
#ifdef ATV_ADPCM
        // Fix for chromecast
        if( app_hids_cc_in_report_client_char_config[0] == 0x01 )
        {
            app_send_report(28u, 0u);
            app_send_report(28u, 1u);
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
        }
#endif
        break;


    default:
        printf("\nUnhandled Bluetooth Management Event: %d\r\n", event);
        break;
    }

    return (bt_dev_status);
}

/**
 * Function Name: hci_trace_cback
 *
 * Function Description:
 *   @brief This callback routes HCI packets to debug uart.
 *
 *   @param wiced_bt_hci_trace_type_t type : HCI trace type
 *   @param uint16_t length : length of p_data
 *   @param uint8_t* p_data : pointer to data
 *
 *   @return None
 *
 */
#ifdef ENABLE_BT_SPY_LOG
void hci_trace_cback(wiced_bt_hci_trace_type_t type,
                     uint16_t length, uint8_t* p_data)
{
    cybt_debug_uart_send_hci_trace(type, length, p_data);
}
#endif

/**
 * Function Name:
 * app_bt_init
 *
 * Function Description:
 * @brief This Function initializes the Bluetooth functions such as GATT DB
 * initialization, Bonding info and advertisement
 *
 * @param void
 *
 * @return void
 */
static void app_bt_init(void)
{
#ifdef ENABLE_BT_SPY_LOG
        wiced_bt_dev_register_hci_trace(hci_trace_cback);
#endif
        wiced_bt_device_address_t local_bda = {0x20, 0x82, 0x9B, 0x00, 0x00, 0x00};
        wiced_bt_set_local_bdaddr( local_bda, BLE_ADDR_PUBLIC);
        wiced_bt_device_address_t  local_device_bd_addr = {0};
        wiced_bt_dev_read_local_addr(local_device_bd_addr);

        printf("\nBluetooth Device Address: ");
        app_print_bd_address(local_device_bd_addr);

        /* GATT DB Initialization */
        app_bt_gatt_db_init();

        /* Retrieve Bond information from NV Storage to Bondinfo structure in RAM */
        /* The​ ​value​ ​of​ ​the​ ​​Client​ ​Characteristic​ ​Configuration​ ​​descriptor​ ​is​ ​persistent​
         * ​for​ ​bonded devices​ ​when​ ​not​ ​in​ ​a​ ​connection
         */
        /* CCCD needs to be restored */
        app_bt_restore_bond_data();

        /* Prints the Number of Bonded devices and free slots */
        app_print_bond_info_stats();

        /* Print the Bond data(BD_ADDR, Link Keys & Identity keys) from EmEEPROM */
        app_print_bond_data();

        /* Advertisement Data */
        app_bt_adv_start();
}


/**
 * Function Name:
 * app_bt_gatt_db_init
 *
 * Function Description:
 * @brief Initialize the Bluetooth LE GATT Database with the necessary services and
 * characteristics and register GATT callback function.
 *
 * @param void
 *
 * @return void
 *
 */
static void app_bt_gatt_db_init(void)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    /* Initialize GATT Database */
    if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_db_init(gatt_database,
                                                      gatt_database_len,
                                                      NULL))
    {
        printf("\r\n GATT DB Initialization not successful\r\n");
    }

    /* Register with Bluetoth stack to receive GATT callback */
    if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_register(app_bt_gatt_event_handler))
    {
        printf("\nGATT event gatt_status:\t");
        printf(app_get_gatt_status_name(gatt_status));
        printf("\r\n");
    }
}
