/***************************************************************************//**
* File Name: cycfg_gap.h
*
* Description:
* Definitions for GAP configuration.
* This file should not be modified. It was automatically generated by
* Bluetooth Configurator 2.60.0.1460
*
********************************************************************************
* Copyright 2023 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#if !defined(CYCFG_GAP_H)
#define CYCFG_GAP_H

#include "stdint.h"
#include "wiced_bt_ble.h"
#include "cycfg_gatt_db.h"

/* Silicon generated 'Company assigned' part of device address */
#define CY_BT_SILICON_DEVICE_ADDRESS_EN                       0

/* Appearance */
#define CY_BT_APPEARANCE                                      384

/* TX Power Level */
#define CY_BT_TX_POWER                                        10

/* Interval of random address refreshing */
#define CY_BT_RPA_TIMEOUT                                     0

/* Maximum attribute length */
#define CY_BT_MAX_ATTR_LEN                                    507
/* Maximum attribute MTU size */
#define CY_BT_MTU_SIZE                                        512

/* RX PDU size */
#define CY_BT_RX_PDU_SIZE                                     512

/* Maximum connections */
#define CY_BT_SERVER_MAX_LINKS                                1
#define CY_BT_CLIENT_MAX_LINKS                                0

/* LE Filter accept list size */
#define CY_BT_FILTER_ACCEPT_LIST_SIZE                         2
/* This define is obsolete and kept for backward compatibility only. */
#define CY_BT_WHITE_LIST_SIZE                                 CY_BT_FILTER_ACCEPT_LIST_SIZE

/* L2CAP configuration */
#define CY_BT_L2CAP_MAX_LE_PSM                                1
#define CY_BT_L2CAP_MAX_LE_CHANNELS                           1
#define CY_BT_L2CAP_MAX_LE_FIXED_CHANNELS                     0
#define CY_BT_L2CAP_MAX_BR_ERTM_CHANNELS                      0
#define CY_BT_L2CAP_MAX_BR_ERTM_TX_WIN                        0
#define CY_BT_L2CAP_MTU_SIZE                                  512

/* Security level */
#define CY_BT_SECURITY_LEVEL                                  BTM_SEC_BEST_EFFORT

/* Scan configuration */
#define CY_BT_SCAN_MODE                                       BTM_BLE_SCAN_MODE_PASSIVE

#define CY_BT_HIGH_DUTY_SCAN_INTERVAL                         WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL
#define CY_BT_HIGH_DUTY_SCAN_WINDOW                           WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW
#define CY_BT_HIGH_DUTY_SCAN_DURATION                         5

#define CY_BT_LOW_DUTY_SCAN_INTERVAL                          WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_INTERVAL
#define CY_BT_LOW_DUTY_SCAN_WINDOW                            WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW
#define CY_BT_LOW_DUTY_SCAN_DURATION                          60

#define CY_BT_HIGH_DUTY_CONN_SCAN_INTERVAL                    WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL
#define CY_BT_HIGH_DUTY_CONN_SCAN_WINDOW                      WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW
#define CY_BT_HIGH_DUTY_CONN_SCAN_DURATION                    30

#define CY_BT_LOW_DUTY_CONN_SCAN_INTERVAL                     WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL
#define CY_BT_LOW_DUTY_CONN_SCAN_WINDOW                       WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW
#define CY_BT_LOW_DUTY_CONN_SCAN_DURATION                     30

/* Connection configuration */
#define CY_BT_CONN_MIN_INTERVAL                               WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL
#define CY_BT_CONN_MAX_INTERVAL                               WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL
#define CY_BT_CONN_LATENCY                                    WICED_BT_CFG_DEFAULT_CONN_LATENCY
#define CY_BT_CONN_SUPERVISION_TIMEOUT                        WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT

/* Advertisement settings */
#define CY_BT_CHANNEL_MAP                                     (BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39)

#define CY_BT_HIGH_DUTY_ADV_MIN_INTERVAL                      48
#define CY_BT_HIGH_DUTY_ADV_MAX_INTERVAL                      48
#define CY_BT_HIGH_DUTY_ADV_DURATION                          60

#define CY_BT_LOW_DUTY_ADV_MIN_INTERVAL                       2048
#define CY_BT_LOW_DUTY_ADV_MAX_INTERVAL                       2048
#define CY_BT_LOW_DUTY_ADV_DURATION                           30

#define CY_BT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL             400
#define CY_BT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL             800

#define CY_BT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL              48
#define CY_BT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL              48
#define CY_BT_LOW_DUTY_DIRECTED_ADV_DURATION                  60

#define CY_BT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL              160
#define CY_BT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL              160
#define CY_BT_HIGH_DUTY_NONCONN_ADV_DURATION                  30

#define CY_BT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL               2048
#define CY_BT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL               2048
#define CY_BT_LOW_DUTY_NONCONN_ADV_DURATION                   30


/* Advertisement and scan response packets defines */
#define CY_BT_ADV_PACKET_ELEM_COUNT                           4
/* CY_BT_ADV_PACKET_DATA_SIZE is obsolete. Use CY_BT_ADV_PACKET_ELEM_COUNT instead */
#define CY_BT_ADV_PACKET_DATA_SIZE                            CY_BT_ADV_PACKET_ELEM_COUNT

/* cy_bt_device_name is obsolete. Use app_gap_device_name instead. */
#define cy_bt_device_name                                     app_gap_device_name

/* External definitions */
extern wiced_bt_device_address_t cy_bt_device_address;
extern wiced_bt_ble_advert_elem_t cy_bt_adv_packet_data[];

#endif /* CYCFG_GAP_H */
