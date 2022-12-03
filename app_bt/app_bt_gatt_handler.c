/*******************************************************************************
* File Name: app_bt_gatt_handler.c
*
* Description: This file consists of the function defintions that are
*              necessary for developing the Bluetooth LE applications with GATT
*              Server callbacks.
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
 *                              INCLUDES
 ******************************************************************************/
#include "app_bt_gatt_handler.h"
#include "app_bt_utils.h"
#include "app_bt_advert.h"
#include "app_hw_handler.h"
#include "app_bt_bonding.h"
#ifdef ATV_ADPCM
#include "app_bt_hid_atv.h"
#endif
#ifdef BSA_OPUS
#include "app_bt_hid_bsa.h"
#endif
#include "cycfg_gatt_db.h"
#include "cybt_platform_trace.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_l2c.h"
#include "wiced_memory.h"

/*******************************************************************************
 *                      VARIABLE DEFINITIONS
 ******************************************************************************/
/* This variable tracks the number of congestion events during audio transfer */
uint16_t num_of_congestions = 0;

/* Flag to check for GATT congestion */
uint8_t is_gatt_congested;

/* Update this flag once connection parameter update is successful */
uint8_t conn_param_updated_flag = FALSE;

/* MTU size negotiated between local and peer device */
uint16_t preferred_mtu_size = CY_BT_MTU_SIZE;

/* Bluetooth Transmit task */
extern TaskHandle_t xmit_task_h;

wiced_bt_device_address_t peer_bd_addr = {0};

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

static void app_free_buffer(uint8_t *p_event_data);

static uint8_t *app_alloc_buffer(uint16_t len);

extern wiced_result_t BTM_SetDataChannelPDULength(wiced_bt_device_address_t bd_addr, uint16_t tx_pdu_length);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/**
 * Function Name:
 * app_free_buffer
 *
 *  Function Description:
 * @brief    This function frees up the buffer memory.
 *
 * @param    uint8_t *p_data: Pointer to the buffer to be free
 *
 * @return void
 */
static void app_free_buffer(uint8_t *p_event_data)
{
    wiced_bt_free_buffer(p_event_data);
}

/**
 * Function Name:
 * app_free_buffer
 *
 *  Function Description:
 * @brief    This function allocates the memory from BT buffer pool.
 *
 * @param    uint8_t len : Length to be allocated
 *
 * @return uint8_t * : Pointer to the start address of allocated memory
 */
static uint8_t *app_alloc_buffer(uint16_t len)
{
    uint8_t *p_mem = (uint8_t *)wiced_bt_get_buffer(len);
    if(!p_mem)
    {
        printf("OOM\r\n");
        CY_ASSERT(0);
    }

    return p_mem;
}

/**
 * Function Name:
 * app_bt_gatt_event_handler
 *
 * Function Description:
 * @brief  This Function handles the all the GATT events - GATT Event Handler
 *
 * @param event            Bluetooth LE GATT event type
 * @param p_event_data     Pointer to Bluetooth LE GATT event data
 *
 * @return wiced_bt_gatt_status_t  Bluetooth LE GATT status
 */
wiced_bt_gatt_status_t
app_bt_gatt_event_handler(wiced_bt_gatt_evt_t event,
                         wiced_bt_gatt_event_data_t *p_event_data)
{

    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;
    pfn_free_buffer_t pfn_free;

    /*
     * Call the appropriate callback function based on the GATT event type,
     * and pass the relevant event parameters to the callback function
     */
    switch (event)
    {

    case GATT_CONNECTION_STATUS_EVT:
        gatt_status = app_gatt_connection_status_change_cb(&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        p_attr_req = &p_event_data->attribute_request;
        gatt_status = app_gatt_attr_request_cb(p_attr_req);
        break;

    case GATT_OPERATION_CPLT_EVT:
        printf("GATT_OPERATION_CPLT_EVT\r\n");
        break;

    case GATT_CONGESTION_EVT:
        num_of_congestions++;
        is_gatt_congested = (p_event_data->congestion.congested) ? true : false;

        if(!is_gatt_congested)
        {
            xTaskNotify(xmit_task_h, 0, eNoAction);
        }

        break;

    case GATT_GET_RESPONSE_BUFFER_EVT:
        printf("GATT_GET_RESPONSE_BUFFER_EVT len_req %d \r\n", p_event_data->buffer_request.len_requested);

        p_event_data->buffer_request.buffer.p_app_rsp_buffer =
                app_alloc_buffer(p_event_data->buffer_request.len_requested);
        p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_free_buffer;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT:
        pfn_free  = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

        /* If the buffer is dynamic, the context will point to a function to
         * free it.
         */
        if (pfn_free)
        {
            pfn_free(p_event_data->buffer_xmitted.p_app_data);
        }
        break;

    default:
        printf("ERROR: Unhandled GATT event: %d \r\n", event);
        break;

    }

    if (WICED_BT_GATT_SUCCESS != gatt_status)
    {
        printf("\nGATT event status: %d \t", gatt_status);
        printf("\nFor the ATT handle: 0x%x\r\n", p_attr_req->data.read_req.handle);
        printf("\r\n");
    }

    return gatt_status;
}


/**
 * Function Name:
 * app_gatt_connection_status_change_cb
 *
 * Function Description:
 * @brief  The callback function is invoked when GATT_CONNECTION_STATUS_EVT occurs
 *         in GATT Event handler function
 *
 * @param p_conn_status     Pointer to Bluetooth LE GATT connection status
 *
 * @return wiced_bt_gatt_status_t  Bluetooth LE GATT status
 */
wiced_bt_gatt_status_t
app_gatt_connection_status_change_cb(wiced_bt_gatt_connection_status_t *p_conn_status)
{

    wiced_result_t gatt_status = WICED_BT_GATT_SUCCESS;

    if ((p_conn_status->connected) && (0 == app_bt_conn_id))
    {
        BTM_SetDataChannelPDULength(p_conn_status->bd_addr,251);

        /* Device has connected */
        printf("Connected to BDA:\r\n");
        app_print_bd_address(p_conn_status->bd_addr);
        printf("Connection ID: '%d'\r\n", p_conn_status->conn_id);
        printf("Peer device addr type : %d\r\n", p_conn_status->addr_type );

        app_bt_conn_id  = p_conn_status->conn_id;
        // Stop Advertisement as new device is connected
        app_bt_stop_adv();

        /* Enable all IN Report notifications. The HID Host will automatically
         * enable all CCCDs. On some cases, the HID device is done independently
         * for example chromecast. So Use it for debug purposes only.
         */
        // app_enable_all_cccds();

        // Enable battery notifications
        app_bas_battery_level_client_char_config[0] = 0x01;

        if(wiced_bt_l2cap_enable_update_ble_conn_params(peer_bd_addr, TRUE))
        {
            printf("conn param enabled\r\n");
        }
        else
        {
            printf("connection param disabled\r\n");
        }


    }
    else
    {
        /* Device has disconnected */
        printf("Disconnected to BDA:\r\n");
        app_print_bd_address(p_conn_status->bd_addr);
        printf("Connection ID: '%d'\r\n", p_conn_status->conn_id);
        printf("\nReason for disconnection: %d \t", p_conn_status->reason);
        printf(app_get_gatt_disconn_reason_name(p_conn_status->reason));
        printf("\r\n");

        /* Handle the disconnection */
        app_bt_conn_id  = 0;

        /*
         * Reset the CCCD value so that on a reconnect CCCD (notifications)
         * will be off. For HID Bonded devices, CCCDs has to be retained.
         */
        app_disable_all_cccds();

        /* TODO New devices allowed to bond after current device is disconnected
         * for quick testing. Ideally not */
        wiced_bt_set_pairable_mode(TRUE, FALSE);

        app_bt_start_adv_any_host(); // TODO Change to known host before final deployment
        current_remote_state = PAIRED_ADVERTISING_ANY_HOST; // TODO Change to known host before final deployment
    }

    return gatt_status;
}

/**
 * Function Name:
 * app_gatt_attr_request_cb
 *
 * Function Description:
 * @brief  The callback function is invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs
 *         in GATT Event handler function. GATT Server Event Callback function.
 *
 * @param GATT Request attribute  Pointer to GATT attribute Request structure
 *
 * @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t
app_gatt_attr_request_cb( wiced_bt_gatt_attribute_request_t *p_attr_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    switch (p_attr_req->opcode)
    {

    case GATT_REQ_FIND_TYPE_VALUE:
    case GATT_REQ_READ_BY_TYPE:
        gatt_status = app_gatt_read_by_type_handler(p_attr_req->conn_id,
                                                    p_attr_req->opcode,
                    (wiced_bt_gatt_read_by_type_t*)&p_attr_req->data.read_req,
                                                    p_attr_req->len_requested);
        break;

    case GATT_REQ_READ_BY_GRP_TYPE:
    case GATT_REQ_READ:
    case GATT_REQ_READ_BLOB:
        gatt_status = app_gatt_attr_read_handler( p_attr_req->conn_id,
                                                p_attr_req->opcode,
                            (wiced_bt_gatt_read_t*)&p_attr_req->data.read_req,
                                                p_attr_req->len_requested );
        break;

    case GATT_REQ_WRITE:
    case GATT_CMD_WRITE:
        gatt_status = app_gatt_attr_write_handler( p_attr_req->conn_id,
                                                  p_attr_req->opcode,
                                                  &p_attr_req->data.write_req,
                                                  p_attr_req->len_requested );
        if ((p_attr_req->opcode == GATT_REQ_WRITE) && (gatt_status == WICED_BT_GATT_SUCCESS))
        {
            wiced_bt_gatt_server_send_write_rsp(p_attr_req->conn_id,
                                                p_attr_req->opcode,
                                            p_attr_req->data.write_req.handle);
        }
        break;

    case GATT_REQ_MTU:
        printf("\rClient MTU Req: %d\r\n", p_attr_req->data.remote_mtu);
        preferred_mtu_size = CY_BT_MTU_SIZE <= (p_attr_req->data.remote_mtu) ?
                            CY_BT_MTU_SIZE :
                            (p_attr_req->data.remote_mtu);
        gatt_status = wiced_bt_gatt_server_send_mtu_rsp(  p_attr_req->conn_id,
                                            p_attr_req->data.remote_mtu,
                                            preferred_mtu_size);
        printf("MTU Response status %d\r\n", gatt_status);
        break;

    case GATT_HANDLE_VALUE_NOTIF:
        break;

    case GATT_RSP_ERROR:
        printf("GATT Response Error\r\n");
        break;

    default:
        printf("ERROR: Unhandled GATT Connection Request case: %d\r\n", p_attr_req->opcode);
        break;
    }

    return gatt_status;
}

/**
 * Function Name:
 * app_gatt_attr_write_handler
 *
 * Function Description:
 * @brief  The function is invoked when GATTS_REQ_TYPE_WRITE is received from the
 *         client device and is invoked GATT Server Event Callback function. This
 *         handles "Write Requests" received from Client device.
 *
 * @param p_write_req   Pointer to BLE GATT write request
 *
 * @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t
app_gatt_attr_write_handler(uint16_t conn_id,
                            wiced_bt_gatt_opcode_t opcode,
                            wiced_bt_gatt_write_req_t *p_write_req,
                            uint16_t len_req)
{
    (void)conn_id;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    uint8_t index = 0;

    /* Binary search of handles is done; Make sure the handles are sorted */
    index = app_get_attr_index_by_handle(p_write_req->handle);
    if( INVALID_ATT_TBL_INDEX != index )
    {
        /* Validate the length of the attribute and write to the attribute */
        if( WICED_BT_GATT_SUCCESS != app_set_gatt_attr_value(index,
                                                    p_write_req->p_val,
                                                    p_write_req->val_len,
                                                    opcode))
        {
            printf("WARNING: GATT set attr status 0x%x\r\n", gatt_status);
        }

    }
    else
    {
        gatt_status = WICED_BT_GATT_INVALID_HANDLE;
        printf("Invalid ATT TBL Index : %d\r\n", index);
    }

    return (gatt_status);

}

/**
 * Function Name:
 * app_gatt_attr_read_handler
 *
 * Function Description:
 * @brief  The function is invoked when GATTS_REQ_TYPE_READ is received from the
 *         client device and is invoked by GATT Server Event Callback function.
 *         This handles "Read Requests" received from Client device
 *
 * @param p_read_req   Pointer to BLE GATT read request
 * @param len_req Length of the attribute requested by the Peer device
 *
 * @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t
app_gatt_attr_read_handler( uint16_t conn_id,
                            wiced_bt_gatt_opcode_t opcode,
                            wiced_bt_gatt_read_t *p_read_req,
                            uint16_t len_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    uint16_t valid_end_handle = HDLD_HIDS_REPORT_MAP_EXTERNAL_REPORT_REFERENCE;
    uint8_t index = 0;
    uint16_t len_to_send = 0;

    index = app_get_attr_index_by_handle((p_read_req->handle));
    if (INVALID_ATT_TBL_INDEX != index)
    {
        /* Validate the length of the attribute */
        len_to_send = (len_req < app_gatt_db_ext_attr_tbl[index].cur_len) ?
                        len_req :
                        app_gatt_db_ext_attr_tbl[index].cur_len ;
        /* Send the attribute response */
        gatt_status = wiced_bt_gatt_server_send_read_handle_rsp(conn_id,
                                                                opcode,
                                                                len_to_send,
                                    app_gatt_db_ext_attr_tbl[index].p_data,
                                                                NULL);
    }
    else if(HDLS_GAP == p_read_req->handle)
    {
        gatt_status = wiced_bt_gatt_server_send_read_handle_rsp(conn_id,
                                                                opcode,
                                                                0,
                                                                NULL,
                                                                NULL);
        gatt_status = WICED_BT_GATT_SUCCESS;
    }
    else if( valid_end_handle > p_read_req->handle )
    {
        printf("Invalid ATT TBL Index : %d\r\n", index);
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        gatt_status = WICED_BT_GATT_INVALID_HANDLE;
    }

    return (gatt_status);

}

/**
 * Function Name:
 * app_gatt_read_by_type_handler
 *
 *  Function Description:
 * @brief This function handles the GATT read by type request events from the stack
 *
 * @param uint16_t conn_id: Connection ID
 * @param wiced_bt_gatt_opcode_t opcode: GATT opcode
 * @param wiced_bt_gatt_read_t * p_read_data: Read data structure
 * @param uint16_t len_requested: Length requested
 *
 * @return wiced_bt_gatt_status_t: GATT result
 *
 */
wiced_bt_gatt_status_t
app_gatt_read_by_type_handler(  uint16_t conn_id,
                                wiced_bt_gatt_opcode_t opcode,
                                wiced_bt_gatt_read_by_type_t *p_read_data,
                                uint16_t len_requested )
{
    uint16_t    attr_handle = p_read_data->s_handle;
    uint8_t     *p_rsp = app_alloc_buffer(len_requested);
    uint8_t     pair_len = 0;
    uint8_t     index = 0;
    int         used = 0;
    int         filled = 0;

    if (p_rsp == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type,
     * between the start and end handles */
    while (TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                                                        p_read_data->e_handle,
                                                        &p_read_data->uuid);

        if (attr_handle == 0)
            break;

        index = app_get_attr_index_by_handle(attr_handle);
        if (INVALID_ATT_TBL_INDEX != index)
        {
            filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream( p_rsp + used,
                                                        len_requested - used,
                                                        &pair_len,
                                                        attr_handle,
                                        app_gatt_db_ext_attr_tbl[index].cur_len,
                                        app_gatt_db_ext_attr_tbl[index].p_data);
            if (filled == 0)
            {
                printf("No data is filled\r\n");
                break;
            }
            used += filled;
        }
        else
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id,
                                                opcode,
                                                p_read_data->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            app_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }
        /* Increment starting handle for next search to one past current */
        attr_handle++;
    } // End of adding the data to the stream

    if (used == 0)
    {
        printf("free buffer resp 1 %x \r\n", p_read_data->s_handle );
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            p_read_data->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp( conn_id,
                                                opcode,
                                                pair_len,
                                                used,
                                                p_rsp,
                            (wiced_bt_gatt_app_context_t)app_free_buffer);

    return WICED_BT_GATT_SUCCESS;

}

/**
 * Function Name:
 * app_get_gatt_attr_value
 *
 * Function Description:
 * @brief  The function is invoked by app_gatt_attr_read_handler to get a Value from
 *         GATT DB.
 *
 * @param attr_handle_index  Attribute handle's index in Attribute table.
 * @param p_val        Pointer to BLE GATT read request value
 * @param len          Maximum length of GATT read request
 * @param p_len        Pointer to BLE GATT read request length
 *
 * @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t
app_get_gatt_attr_value(uint8_t attr_handle_index,
                     uint8_t *p_val,
                     uint16_t len,
                     uint16_t *p_len)
{

    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    /* Detected a matching handle in the external lookup table */
    if (len >= app_gatt_db_ext_attr_tbl[attr_handle_index].max_len )
    {
        /* Value fits within the supplied buffer; copy over the value */
        *p_len = app_gatt_db_ext_attr_tbl[attr_handle_index].cur_len;
        memcpy(p_val, app_gatt_db_ext_attr_tbl[attr_handle_index].p_data,
                app_gatt_db_ext_attr_tbl[attr_handle_index].cur_len);

        gatt_status = WICED_BT_GATT_SUCCESS;
    }
    else if(len >= CY_BT_MAX_ATTR_LEN)
    {
        printf("GATT read requested attr len of %d for the handle 0x%x\r\n" ,
                len, app_gatt_db_ext_attr_tbl[attr_handle_index].handle);
        /* Value fits within the supplied buffer; copy over the value */
        *p_len = app_gatt_db_ext_attr_tbl[attr_handle_index].cur_len;
        memcpy( p_val, app_gatt_db_ext_attr_tbl[attr_handle_index].p_data,
                app_gatt_db_ext_attr_tbl[attr_handle_index].cur_len);

        gatt_status = WICED_BT_GATT_SUCCESS;
    }
    else
    {
        printf("GATT read requested more than the max attr len\r\n");
        *p_len = app_gatt_db_ext_attr_tbl[attr_handle_index].cur_len;
        memcpy( p_val, app_gatt_db_ext_attr_tbl[attr_handle_index].p_data,
                len);

        gatt_status = WICED_BT_GATT_SUCCESS;
    }

    return (gatt_status);
}

/**
 * Function Name:
 * app_set_gatt_attr_value
 *
 * Function Description:
 * @brief  The function is invoked by app_gatt_attr_write_handler to set a value
 *         to GATT DB.
 *
 * @param attr_handle_index  Attribute handle's index in Attribute table.
 * @param p_val Pointer to BLE GATT write request value
 * @param len   length of GATT write request
 * @param opcode Opcode from the peer device
 *
 * @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t app_set_gatt_attr_value(uint8_t attr_handle_index,
                                        uint8_t *p_val,
                                        uint16_t len,
                                        wiced_bt_gatt_opcode_t opcode)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    if ((app_gatt_db_ext_attr_tbl[attr_handle_index].max_len) >= len)
    {
        if(NULL != memcpy(  app_gatt_db_ext_attr_tbl[attr_handle_index].p_data,
                            p_val,
                            len))
        {
            if( WICED_BT_GATT_SUCCESS != wiced_bt_gatt_server_send_write_rsp(
                                        app_bt_conn_id,
                                        opcode,
                app_gatt_db_ext_attr_tbl[attr_handle_index].handle))
            {
                printf("WARNING: GATT Write resp status 0x%x\r\n", gatt_status);
            }

            switch(app_gatt_db_ext_attr_tbl[attr_handle_index].handle)
            {
                case HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG :
                    printf("Battery IN report ");
                    app_modify_cccd_in_nv_storage(
                        app_gatt_db_ext_attr_tbl[attr_handle_index].handle,
                        p_val);
                    break;

                case HDLC_HIDS_HID_CONTROL_POINT_VALUE:
                    printf("HID Control point att value : 0x%x\r\n", (unsigned int)app_hids_hid_control_point);
                    break;

                case HDLD_HIDS_KBD_IN_REPORT_CLIENT_CHAR_CONFIG:
                    printf("KBD IN report ");
                    app_modify_cccd_in_nv_storage(
                        app_gatt_db_ext_attr_tbl[attr_handle_index].handle,
                        p_val);
                    break;

                case HDLD_HIDS_CC_IN_REPORT_CLIENT_CHAR_CONFIG:
                    printf("CC IN report ");
                    app_modify_cccd_in_nv_storage(
                        app_gatt_db_ext_attr_tbl[attr_handle_index].handle,
                        p_val);
                    break;
/* BSA Opus */
#ifdef BSA_OPUS
                case HDLD_HIDS_VOICE_IN_REPORT_CLIENT_CHAR_CONFIG:
                    printf("Voice IN report ");
                    app_modify_cccd_in_nv_storage(
                        app_gatt_db_ext_attr_tbl[attr_handle_index].handle,
                        p_val);
                    break;

                case HDLD_HIDS_VOICE_CTL_IN_REPORT_CLIENT_CHAR_CONFIG:
                    printf("Voice CTL IN report ");
                    app_modify_cccd_in_nv_storage(
                        app_gatt_db_ext_attr_tbl[attr_handle_index].handle,
                        p_val);
                    break;

                case HDLC_HIDS_VOICE_CTL_FEAT_REPORT_VALUE:
                    printf("Voice CTL FEAT report ");
                    app_bsa_audio_event_handler(app_gatt_db_ext_attr_tbl[attr_handle_index].p_data);
                    break;
#endif

/* For ADPCM  */
#ifdef ATV_ADPCM
                case HDLD_ATVS_ATV_CONTROL_CHAR_ATV_CONTROL_CCCD:
                    printf("ATV Control Char report ");
                    app_modify_cccd_in_nv_storage(
                        app_gatt_db_ext_attr_tbl[attr_handle_index].handle,
                        p_val);
                     printf("send conn param request\r\n");
                    if (0 == wiced_bt_l2cap_update_ble_conn_params(
                            peer_bd_addr,
                            MIN_CI,
                            MAX_CI,
                            SLAVE_LATENCY,
                            SUPERVISION_TO))
                    {
                        printf("Connection parameter update failed\r\n");
                    }
                    else
                    {
                        printf("Connection parameter update successful\r\n");
                    }
                    break;

                case HDLD_ATVS_ATV_READ_CHAR_ATV_READ_CCCD:
                    printf("ATV Read Char report ");
                    app_modify_cccd_in_nv_storage(
                        app_gatt_db_ext_attr_tbl[attr_handle_index].handle,
                        p_val);
                    break;

                case HDLC_ATVS_ATV_WRITE_CHAR_VALUE:
                    app_atv_audio_event_handler(app_gatt_db_ext_attr_tbl[attr_handle_index].p_data);
                    break;
#endif
                default:
                    break;

            } // Switch case for different ATT handles


            gatt_status = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            printf("memcpy failed\r\n");
        }
    }
    else
    {
        // Check the len of the attribute
        printf("Length to be written %d\r\n", len);
    }

  return (gatt_status);
}


/**
 * Function Name:
 * app_get_attr_index_by_handle
 *
 * Function Description:
 * @brief This function returns the corresponding index for the respective
 * attribute handle from the attribute table.
 *
 * @param attr_handle 16-bit attribute handle for the characteristics and descriptors
 * @return uint8_t The index of the valid attribute handle otherwise
 *         INVALID_ATT_TBL_INDEX
 */
uint8_t app_get_attr_index_by_handle(uint16_t attr_handle)
{

    uint16_t left = 0;
    uint16_t right = app_gatt_db_ext_attr_tbl_size;

    while(left <= right)
    {
        uint16_t mid = left + (right - left)/2;

        if(app_gatt_db_ext_attr_tbl[mid].handle == attr_handle)
        {
            return mid;
        }

        if(app_gatt_db_ext_attr_tbl[mid].handle < attr_handle)
        {
            left = mid + 1;
        }
        else
        {
            right = mid - 1;
        }
    }

    return INVALID_ATT_TBL_INDEX;

}

/**
 * Function Name:
 * app_enable_all_cccds
 *
 * Function Description:
 * @brief This is a temporary function for testing purposes to validate CCCD
 * enablement if the Host is not trying to write to CCCDs. Will be removed once
 * the application is stable.
 *
 * @param void
 *
 * @return void
 */
void app_enable_all_cccds(void)
{

    app_hids_kbd_in_report_client_char_config[0] = 0x01;
    app_hids_cc_in_report_client_char_config[0] = 0x01;
    app_bas_battery_level_client_char_config[0] = 0x01;
#ifdef BSA_OPUS
    app_hids_voice_in_report_client_char_config[0] = 0x01;
    app_hids_voice_ctl_in_report_client_char_config[0] = 0x01;
#endif
#ifdef ATV_ADPCM
    app_atvs_atv_read_char_atv_read_cccd[0] = 0x01;
    app_atvs_atv_control_char_atv_control_cccd[0] =0x01;
#endif
    printf("All Notifications are Enabled\r\n");
}

/**
 * Function Name:
 * app_disable_all_cccds
 *
 * Function Description:
 * @brief This is a temporary function for testing purposes to validate CCCD
 * disablement if the Host is not trying to write to CCCDs. Will be removed once
 * the application is stable.
 *
 * @param void
 *
 * @return void
 */
void app_disable_all_cccds(void)
{

    app_hids_kbd_in_report_client_char_config[0] = 0x00;
    app_hids_cc_in_report_client_char_config[0] = 0x00;
    app_bas_battery_level_client_char_config[0] = 0x00;
#ifdef BSA_OPUS
    app_hids_voice_in_report_client_char_config[0] = 0x00;
    app_hids_voice_ctl_in_report_client_char_config[0] = 0x00;
#endif
#ifdef ATV_ADPCM
    app_atvs_atv_read_char_atv_read_cccd[0] = 0x00;
    app_atvs_atv_control_char_atv_control_cccd[0] =0x00;
#endif
    printf("All Notifications are disabled\r\n");
}

/* [] END OF FILE */
