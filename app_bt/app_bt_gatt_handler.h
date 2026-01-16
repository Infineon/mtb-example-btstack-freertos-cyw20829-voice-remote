/*******************************************************************************
* File Name: app_bt_gatt_handler.h
*
* Description: This file consists of the function prototypes that are
*              necessary for developing the BLE applications with GATT Server
*              callbacks.
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


#ifndef __APP_BT_GATT_HANDLER_H__
#define __APP_BT_GATT_HANDLER_H__

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "wiced_bt_gatt.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
/* The error code for invalid  attribute index in attribute table */
#define INVALID_ATT_TBL_INDEX                   (255u)

/* Connection Parameters */
/* Minimum Connection Interval */
#define MIN_CI                                  (24u)
/* Maximum Connection Interval */
#define MAX_CI                                  (24u)
/* Slave Latency */
#define SLAVE_LATENCY                           (33u)
/* Supervision Timeout */
#define SUPERVISION_TO                          (500u)

/*******************************************************************************
 *                              VARIABLES
 ******************************************************************************/

/* A Global variable to check the status of this device if it is
 * connected to any peer devices*/
extern uint16_t app_bt_conn_id;

/* Number of GATT congestions during audio transfer */
extern uint16_t num_of_congestions;

/* connection parameter updated flag to avoid requesting the connection
 * parameters more than once
 */
extern uint8_t conn_param_updated_flag;

/* Negotiated MTU Size */
extern uint16_t preferred_mtu_size;

/* BD Address of the connected Peer device */
extern wiced_bt_device_address_t peer_bd_addr;
/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
wiced_bt_gatt_status_t
app_gatt_attr_write_handler(uint16_t conn_id,
                            wiced_bt_gatt_opcode_t opcode,
                            wiced_bt_gatt_write_req_t *p_write_req,
                            uint16_t len_req,
                            uint16_t *p_error_handle);

wiced_bt_gatt_status_t
app_gatt_attr_read_handler(uint16_t conn_id,
                            wiced_bt_gatt_opcode_t opcode,
                            wiced_bt_gatt_read_t *p_read_req,
                            uint16_t len_req,
                            uint16_t *p_error_handle);

wiced_bt_gatt_status_t
app_gatt_read_by_type_handler(uint16_t conn_id,
                                wiced_bt_gatt_opcode_t opcode,
                                wiced_bt_gatt_read_by_type_t *p_read_data,
                                uint16_t len_requested,
                                uint16_t *p_error_handle);

wiced_bt_gatt_status_t
app_gatt_connection_status_change_cb(wiced_bt_gatt_connection_status_t *p_conn_status);

wiced_bt_gatt_status_t
app_gatt_attr_request_cb( wiced_bt_gatt_attribute_request_t *p_attr_req, uint16_t *p_error_handle);

wiced_bt_gatt_status_t
app_bt_gatt_event_handler(wiced_bt_gatt_evt_t event,
                          wiced_bt_gatt_event_data_t *p_event_data);

wiced_bt_gatt_status_t app_get_gatt_attr_value( uint8_t attr_handle_index,
                                                uint8_t *p_val,
                                                uint16_t len,
                                                uint16_t *p_len);

wiced_bt_gatt_status_t app_set_gatt_attr_value( uint8_t attr_handle_index,
                                                uint8_t *p_val,
                                                uint16_t len,
                                                wiced_bt_gatt_opcode_t opcode);

uint8_t app_get_attr_index_by_handle(uint16_t attr_handle);

void app_enable_all_cccds(void);

void app_disable_all_cccds(void);

typedef void (*pfn_free_buffer_t)(uint8_t *);

#endif      /* __APP_BT_GATT_HANDLER_H__ */

/* [] END OF FILE */
