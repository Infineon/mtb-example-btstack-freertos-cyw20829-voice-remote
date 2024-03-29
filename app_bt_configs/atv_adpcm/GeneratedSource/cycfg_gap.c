/***************************************************************************//**
* File Name: cycfg_gap.c
*
* Description:
* BLE device's GAP configuration.
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

#include "cycfg_gap.h"

/* Device address */
wiced_bt_device_address_t cy_bt_device_address = {0x00, 0xA0, 0x50, 0x00, 0x00, 0x00};

const uint8_t cy_bt_adv_packet_elem_0[1] = { 0x05 };
const uint8_t cy_bt_adv_packet_elem_1[10] = { 0x48, 0x49, 0x44, 0x2D, 0x52, 0x65, 0x6D, 0x6F, 0x74, 0x65 };
const uint8_t cy_bt_adv_packet_elem_2[2] = { 0x12, 0x18 };
const uint8_t cy_bt_adv_packet_elem_3[2] = { 0x80, 0x01 };
wiced_bt_ble_advert_elem_t cy_bt_adv_packet_data[] = 
{
    /* Flags */
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_FLAG, 
        .len = 1, 
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_0, 
    },
    /* Complete local name */
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, 
        .len = 10, 
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_1, 
    },
    /* 16-bit Service Data */
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_SERVICE_DATA, 
        .len = 2, 
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_2, 
    },
    /* Appearance */
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE, 
        .len = 2, 
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_3, 
    },
};
