/******************************************************************************
* File Name: app_bt_bonding.c
*
* Description: This is the source code for Bluetooth bonding implementation using
*              kv-store library.
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
*        Header Files
*******************************************************************************/
#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg_bt_settings.h"
#include "cycfg_gatt_db.h"
#include "app_bt_bonding.h"
#include "mtb_kvstore.h"
#include "app_bt_utils.h"
#include <inttypes.h>
#include "cy_pdl.h"
#include "cycfg.h"
#include "cycfg_qspi_memslot.h"
#include "app_hw_handler.h"
#include "cybsp_smif_init.h"

/*******************************************************************************
 *                              Macro Definitions
 ******************************************************************************/
/* 1 ms timeout for all blocking functions */
#define TIMEOUT_1_MS                    (1000ul)

/* Set it high enough for the sector erase operation to complete */
#define MEMORY_BUSY_CHECK_RETRIES       (750ul)

/*
 * Position refers to the bit position from the LSB.
 * Position starts from 0 to N-1.
 * N refers to size of the variable. Byte has N = 8.
 * x refers to cccd_flags variable
 */
#define SET_CCCD_BIT(x, pos)    (x |= (0x01 << (pos)))
#define CLEAR_CCCD_BIT(x, pos)  (x &= ~(0x01 << (pos)))
#define IS_THIS_BIT_SET(x, pos) (x & (0x01 << (pos)))


/*******************************************************************************
 *                              Variable Definitions
 ******************************************************************************/
/*Structure to store bond data*/
tBondInfo       bondinfo = {0};
wiced_bt_local_identity_keys_t  identity_keys;                  /* Local Indentity Key */
/* This is the index for the link keys, , identity keys, cccd and privacy mode
 * of the host we are currently bonded to */
uint8_t         bondindex = 0;

static mtb_kvstore_t   kvstore_obj;

cy_stc_smif_context_t SMIFContext;

extern uint16_t app_bt_conn_id;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
static uint8_t app_bt_find_cccd_bit(uint16_t attr_handle);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 *              QSPI, SMIF and KVstor Init API for Non-Voltaile Storage
 ******************************************************************************/

/**
 * Function Name:
 * app_kv_store_init
 *
 * Function Description:
 * @brief This function initializes the kv store used for write the bond data
 *        in Non-Volatile Storage.
 *
 * @param void
 *
 * @return void
 *
 */
void  app_kv_store_init(void)
{
    uint32_t sector_size = (size_t)smifBlockConfig.memConfig[0]->deviceCfg->eraseSize;
    uint32_t length =  sector_size * 2;
    uint32_t start_addr = 0;
    cy_rslt_t rslt;

    cybsp_smif_init();
    start_addr = smifMemConfigs[0]->deviceCfg->memSize - sector_size * 2;

    /*Initialize kv-store library*/
    rslt = mtb_kvstore_init(&kvstore_obj, start_addr, length, &block_device);
    /*Check if the kv-store initialization was successful*/
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("failed to initialize kv-store \r\n");
        CY_ASSERT(0);
    }
}


/*******************************************************************************
 *              Peer Device and Bond Info Management APIs
 ******************************************************************************/

/**
 * Function Name:
 * app_bt_restore_bond_data
 *
 * Function Description:
 * @brief  This function restores the bond information from the Flash
 *
 * @param   void
 *
 * @return  cy_rslt_t: CY_RSLT_SUCCESS if the restoration was successful,
 * an error code otherwise.
 *
 */
cy_rslt_t app_bt_restore_bond_data(void)
{
    /* Read and restore contents of Serial flash */
    uint32_t data_size = sizeof(bondinfo);
    cy_rslt_t rslt = mtb_kvstore_read(  &kvstore_obj,
                                        "bondinfo",
                                        (uint8_t *)&bondinfo,
                                        &data_size);
    if (rslt != CY_RSLT_SUCCESS)
    {
        printf("Bond data not present in the flash!\r\n");
        return rslt;
    }

    // TODO Is it required?
    app_bt_add_devices_to_address_resolution_db();
    return rslt;
}

/**
* Function Name:
* app_bt_update_slot_data
*
* Function Description:
* @brief  This function updates the slot data in the Flash
*
* @param  void
*
* @return cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
* an error code otherwise.
*/
cy_rslt_t app_bt_update_slot_data(void)
{
    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;
    /* Increment number of bonded devices and next free slot and save them in Flash */
    if ( BOND_MAX > bondinfo.slot_data[NUM_BONDED])
    {
        /* Increment only if the bonded devices are less than BOND_MAX */
        bondinfo.slot_data[NUM_BONDED]++;
    }
    /* Update Next Slot to be used for next incoming Device */
    bondinfo.slot_data[NEXT_FREE_INDEX] = (bondinfo.slot_data[NEXT_FREE_INDEX] + 1) % BOND_MAX;
    rslt = app_bt_update_bond_data();
    return rslt;
}

/**
* Function Name:
* app_bt_update_bond_data
*
* Function Description:
* @brief This function updates the bond information in the Flash
*
* @param   void
*
* @return  cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
*              an error code otherwise.
*
*/
cy_rslt_t app_bt_update_bond_data(void)
{
    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;
    rslt = mtb_kvstore_write(&kvstore_obj, "bondinfo", (uint8_t *)&bondinfo, sizeof(bondinfo));
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Flash Write Error,Error code: %" PRIu32 "\r\n", rslt );
    }

    return rslt;
}

/**
* Function Name:
* app_bt_delete_bond_info
*
* Function Description:
* @brief  This deletes the bond information from the Flash
*
* @param  void
*
* @return  cy_rslt_t: CY_RSLT_SUCCESS if the deletion was successful,
*              an error code otherwise.
*
*/
cy_rslt_t app_bt_delete_bond_info(void)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    for (uint8_t i = 0; i < bondinfo.slot_data[NUM_BONDED]; i++)
    {
       wiced_result_t result = app_bt_delete_device_info(i);
       if (WICED_BT_SUCCESS != result)
       {
           rslt = CY_RSLT_TYPE_ERROR;
           printf("Delete bond info failed\r\n");
           return rslt;
       }
    }

    /* Remove bonding information in RAM */
    memset(&bondinfo, 0, sizeof(bondinfo));

    /*Update bond information*/
    rslt = app_bt_update_bond_data();
    return rslt;
}

/**
* Function Name:
* app_bt_delete_device_info
*
* Function Description:
* @brief  This function deletes the bond information of the device from the RAM
*         and address resolution database.
*
* @param  index: Index of the device whose data is to be deleted
*
* @return  wiced_result_t: WICED_BT_SUCCESS if the deletion was successful,
*                   an error code otherwise.
*
*/
wiced_result_t app_bt_delete_device_info(uint8_t index)
{
    wiced_result_t result = WICED_BT_SUCCESS;

    /* Disconnect the peer device before deleting the bondinfo */
    if(0 != wiced_bt_gatt_disconnect(app_bt_conn_id))
    {
        printf("Disconnect failed\r\n");
    };

    /* Remove from the bonded device list after disconnect only */
    result = wiced_bt_dev_delete_bonded_device(bondinfo.link_keys[index].bd_addr);
    if(WICED_BT_SUCCESS != result)
    {
        printf("Delete bond info failed\r\n");
        return result;
    }

    /* Remove link keys from address resolution database */
    result = wiced_bt_dev_remove_device_from_address_resolution_db(&(bondinfo.link_keys[index]));
    if (WICED_BT_SUCCESS != result)
    {
        printf("Delete addr resolution DB failed\r\n");
        return result;
    }

    return result;
}

/**
* Function Name:
* app_bt_find_device_in_flash
*
* Function Description:
* @brief This function searches provided bd_addr in bonded devices list
*
* @param *bd_addr: pointer to the address of the device to be searched
*
* @return uint8_t: Index of the device in the bond data stored in the flash if found,
*            else returns  BOND_MAX to indicate the device was not found.
*
*/
uint8_t app_bt_find_device_in_flash(uint8_t *bd_addr)
{
    uint8_t index =  BOND_MAX; /*Return out of range value if device is not found*/
    for (uint8_t count = 0; count < bondinfo.slot_data[NUM_BONDED]; count++)
    {
        if (0 == memcmp(&(bondinfo.link_keys[count].bd_addr),
                        bd_addr,
                        sizeof(wiced_bt_device_address_t)))
        {
            printf("Found device in the flash!\r\n");
            index = count;
            break; /* Exit the loop since we found what we want */
        }
    }
    return(index);
}

/**
* Function Name:
* app_bt_add_devices_to_address_resolution_db
*
* Function Description:
* @brief This function adds the bonded devices to address resolution database
*
* @param void
*
* @return void
*
*/
void app_bt_add_devices_to_address_resolution_db(void)
{
    /* Copy in the keys and add them to the address resolution database */
    for (uint8_t i = 0; i < bondinfo.slot_data[NUM_BONDED]; i++)
    {
        /* Add device to address resolution database */
        wiced_result_t result = wiced_bt_dev_add_device_to_address_resolution_db(&bondinfo.link_keys[i]);

        // TODO This may cause issue in re-pairing when bondinfo is already present.
        // result = wiced_bt_ble_set_privacy_mode(bondinfo.link_keys[i].bd_addr,
        //                                        bondinfo.link_keys[i].key_data.ble_addr_type,
        //                                        BTM_BLE_PRIVACY_MODE_DEVICE);
        if (WICED_BT_SUCCESS == result)
        {
            printf("Device added to address resolution database: ");
            app_print_bd_address((uint8_t *)&bondinfo.link_keys[i].bd_addr);
        }
        else
        {
            printf("Error adding device to address resolution database, Error Code %d \n", result);
        }
    }
}

/*******************************************************************************
 *              Security Keys Management APIs
 ******************************************************************************/
/**
* Function Name:
* app_bt_save_device_link_keys
*
* Function Description:
* @brief This function saves peer device link keys to the Flash
*
* @param link_key: Save link keys of the peer device.
*
* @return cy_rslt_t: CY_RSLT_SUCCESS if the save was successful,
*              an error code otherwise.
*
*/
cy_rslt_t app_bt_save_device_link_keys(wiced_bt_device_link_keys_t *link_key)
{
    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;
    memcpy(&bondinfo.link_keys[bondinfo.slot_data[NEXT_FREE_INDEX]],
           (uint8_t *)(link_key),
           sizeof(wiced_bt_device_link_keys_t));

    rslt = mtb_kvstore_write(&kvstore_obj,
                            "bondinfo",
                            (uint8_t *)&bondinfo,
                            sizeof(bondinfo));
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Flash Write Error,Error code: %" PRIu32 "\r\n", rslt );
    }
    return rslt;
}


/**
* Function Name:
* app_bt_save_local_identity_key
*
* Function Description:
* @brief This function saves local device identity keys to the Flash
*
* @param id_key: Local identity keys to store in the flash.
*
* @return cy_rslt_t: CY_RSLT_SUCCESS if the save was successful,
*              an error code otherwise.
*
*/
cy_rslt_t app_bt_save_local_identity_key(wiced_bt_local_identity_keys_t id_key)
{
    memcpy( &identity_keys,
            (uint8_t *)&(id_key),
            sizeof(wiced_bt_local_identity_keys_t));
    cy_rslt_t rslt = mtb_kvstore_write( &kvstore_obj,
                                        "local_irk",
                                        (uint8_t *)&identity_keys,
                                        sizeof(wiced_bt_local_identity_keys_t));
    if (CY_RSLT_SUCCESS == rslt)
    {
        printf("Local identity Keys saved to Flash \r\n");
    }
    else
    {
        printf("Flash Write Error,Error code: %" PRIu32 "\r\n", rslt );
    }

    return rslt;
}

/**
* Function Name:
* app_bt_read_local_identity_keys
*
* Function Description:
* @brief This function reads local device identity keys from the Flash
*
* @param void
*
* @return cy_rslt_t: CY_RSLT_SUCCESS if the read was successful,
*              an error code otherwise.
*
*/
cy_rslt_t app_bt_read_local_identity_keys(void)
{

    uint32_t data_size = sizeof(identity_keys);
    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;
    rslt = mtb_kvstore_read(&kvstore_obj,
                            "local_irk",
                            (uint8_t *)&identity_keys,
                            &data_size);
    if(identity_keys.key_type_mask == 0)
        {
            printf("New Keys need to be generated! \r\n");
        }
    else
        {
            printf("Identity keys are available in the database.\r\n");
            printf("Local identity keys read from Flash: \r\n");
        }
    return rslt;
}

/*******************************************************************************
 *              CCCD Management APIs
 ******************************************************************************/
/**
* Function Name:
* app_bt_update_cccd
*
* Function Description:
* @brief  This function updates the CCCD data in the Flash
*
* @param  cccd: cccd value to be updated in flash
* @param  index: Index of the device in the flash
*
* @return cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
*              an error code otherwise.
*/
cy_rslt_t app_bt_update_cccd(uint16_t cccd, uint8_t index)
{
        cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;
        bondinfo.cccd_flags[index]= (uint8_t)cccd;
        printf("Updating CCCD Value to: %d \r\n",cccd);
        rslt = mtb_kvstore_write(&kvstore_obj,
                                "bondinfo",
                                (uint8_t *)&bondinfo,
                                sizeof(bondinfo));
        return rslt;
}


/**
 * Function Name:
 * app_bt_find_cccd_bit
 *
 * Function Description:
 * @brief This function finds the bit position of the respective CCCD attribute
 *        handle
 *
 * @param attr_handle CCCD attribute handle
 *
 * @return uint8_t bit position from LSB
 */
static uint8_t app_bt_find_cccd_bit(uint16_t attr_handle)
{
    uint8_t position = 0;
    switch(attr_handle)
    {
        case HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG :
            position = 0;
            break;

        case HDLD_HIDS_KBD_IN_REPORT_CLIENT_CHAR_CONFIG:
            position = 1;
            break;

        case HDLD_HIDS_CC_IN_REPORT_CLIENT_CHAR_CONFIG:
            position = 2;
            break;

/* BSA Opus */
#ifdef BSA_OPUS
        case HDLD_HIDS_VOICE_IN_REPORT_CLIENT_CHAR_CONFIG:
            position = 3;
            break;

        case HDLD_HIDS_VOICE_CTL_IN_REPORT_CLIENT_CHAR_CONFIG:
            position = 4;
            break;
#endif

/* For ADPCM  */
#ifdef ATV_ADPCM
        case HDLD_ATVS_ATV_CONTROL_CHAR_ATV_CONTROL_CCCD:
            position = 3;
            break;

        case HDLD_ATVS_ATV_READ_CHAR_ATV_READ_CCCD:
            position = 4;
            break;
#endif
                default:
                    printf("Unknown CCCD Attr Handle: 0x%x \r\n", attr_handle);
                    break;
    }
    return position;
}


/**
 * Function Name:
 * app_modify_cccd_in_nv_storage
 *
 * Function Description:
 * @brief This function writes 0 or 1 the respective bits for a particular CCCD
 * and writes to NV Storage
 *
 * @param attr_handle attr_handle of the CCCD
 * @param p_val Pointer to the CCCD attribute
 *
 * @return void
 */
void app_modify_cccd_in_nv_storage(uint16_t attr_handle, uint8_t* p_val)
{

    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;
    /* Position refers to the Bit position from the LSB */
    uint8_t position = 0;
    position = app_bt_find_cccd_bit(attr_handle);

    // bondindex which stores the location of the current peer device in
    // NV storage is updated in BTM_ENCRYPTION_STATUS_EVT

    // printf("bondinfo.cccd_flags Value Before: 0x%x\r\n", bondinfo.cccd_flags[bondindex]);
    if(0x0001 == *p_val)
    {
        printf("Notification is enabled\r\n");
        SET_CCCD_BIT(bondinfo.cccd_flags[bondindex], position);
    }
    else if(0x0000 == *p_val)
    {
        printf("Notification is disabled\r\n");
        CLEAR_CCCD_BIT(bondinfo.cccd_flags[bondindex], position);
    }
    // printf("bondinfo.cccd_flags Value After: 0x%x\r\n", bondinfo.cccd_flags[bondindex]);

    rslt = mtb_kvstore_write(&kvstore_obj,
                            "bondinfo",
                            (uint8_t *)&bondinfo,
                            sizeof(bondinfo));

    if(rslt == CY_RSLT_TYPE_ERROR)
    {
        printf("KVStor write error\r\n");
    }
}

/**
 * Function Name:
 * app_bt_restore_cccd_using_link_key
 *
 * Function Description:
 * @brief Function to copy link key and restore CCCD back from the NV Storage
 *
 * @param p_link_key
 *
 * @return cy_rslt_t
 */
cy_rslt_t app_bt_restore_cccd_using_link_key(wiced_bt_device_link_keys_t *p_link_key)
{
    cy_rslt_t status = FALSE;

    for(uint8_t count = 0; count < bondinfo.slot_data[NUM_BONDED]; count++)
    {
        if( 0 == memcmp(&bondinfo.link_keys[count].bd_addr,
                        &(p_link_key->bd_addr),
                        sizeof(wiced_bt_device_address_t) ) )
        {
            printf( "Device Matching BD_ADDR is found.\r\n"   \
                    "Link key request received\r\n");
            if( NULL == memcpy( p_link_key,
                                &(bondinfo.link_keys[count]),
                                sizeof(wiced_bt_device_link_keys_t)))
            {
                printf("Link key request memcpy failed\r\n");
            }
            else
            {
                printf("\nPaired Device Link Key:\r\n");
                app_print_byte_array(&(bondinfo.link_keys[count]),
                                sizeof(wiced_bt_device_link_keys_t));

                printf("Found the device. Revoking CCCD status\r\n");

            /* This function restores the bondinfo cccd_flags values to the actual
             * GATT DB attribute */

            for(uint8_t i = 0 ; i < NUM_OF_CCCD ; i++ )
            {
                if(IS_THIS_BIT_SET(bondinfo.cccd_flags[count], i))
                {
                    printf("Position %d is set\r\n", i);
                    switch(i)
                    {
                        case 0:
                            app_bas_battery_level_client_char_config[0] = 0x01;
                            break;

                        case 1:
                            app_hids_kbd_in_report_client_char_config[0] = 0x01;
                            break;

                        case 2:
                            app_hids_cc_in_report_client_char_config[0] = 0x01;
                            break;

                        /* BSA Opus */
#ifdef BSA_OPUS
                        case 3:
                            app_hids_voice_in_report_client_char_config[0] = 0x01;
                            break;

                        case 4:
                            app_hids_voice_ctl_in_report_client_char_config[0] = 0x01;
                            break;
#endif
                        /* Android TV */
#ifdef ATV_ADPCM
                        case 3:
                            /* ATV voice service */
                            app_atvs_atv_control_char_atv_control_cccd[0] = 0x01;
                            break;

                        case 4:
                            /* ATV voice service */
                            app_atvs_atv_read_char_atv_read_cccd[0] = 0x01;
                            break;
#endif
                    }
                }
            }

                printf("cccd_flags value: 0x%x \r\n", bondinfo.cccd_flags[bondindex]);
                status = TRUE;
            }

            break;
        }
    }

    return status;
}

/*******************************************************************************
 *              Helper APIs
 ******************************************************************************/
/**
 * Function Name:
 * app_print_bond_data
 *
 * Function Description:
 * @brief This function prints the bond data stored in the Flash
 *
 * @param void
 *
 * @return void
 *
 */
void app_print_bond_data(void)
{
    for (uint8_t i = 0; i < bondinfo.slot_data[NUM_BONDED]; i++)
    {
        printf("Slot: %d",i+1);
        printf(" Device Bluetooth Address: ");
        app_print_bd_address(bondinfo.link_keys[i].bd_addr);
        printf("Link Keys: \n");
        app_print_byte_array(&(bondinfo.link_keys[i].key_data), sizeof(wiced_bt_device_sec_keys_t));
        printf("\n");
    }
}

/**
 * Function Name:
 * app_print_bond_info_stats
 *
 * Function Description:
 * @brief Prints the status of bonding information in the NV storage.
 *
 * @param void
 *
 * @return void
 */
void app_print_bond_info_stats(void)
{
    printf( "\r\nNumber of bonded devices: %d, "
            "\r\nNext free slot: %d,"
            "\r\nNumber of slots free: %d\r\n",
            bondinfo.slot_data[NUM_BONDED],
            bondinfo.slot_data[NEXT_FREE_INDEX]+1,
            (BOND_MAX - bondinfo.slot_data[NUM_BONDED]) );
}
