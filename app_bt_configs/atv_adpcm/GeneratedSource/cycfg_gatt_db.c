/***************************************************************************//**
* File Name: cycfg_gatt_db.c
*
* Description:
* BLE device's GATT database and device configuration.
* This file should not be modified. It was automatically generated by
* Bluetooth Configurator 2.50.0.6117
*
********************************************************************************
* Copyright 2022 Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cycfg_gatt_db.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"

/*************************************************************************************
* GATT server definitions
*************************************************************************************/

const uint8_t gatt_database[] = 
{
    /* Primary Service: Generic Access */
    PRIMARY_SERVICE_UUID16 (HDLS_GAP, __UUID_SERVICE_GENERIC_ACCESS),
        /* Characteristic: Device Name */
        CHARACTERISTIC_UUID16 (HDLC_GAP_DEVICE_NAME, HDLC_GAP_DEVICE_NAME_VALUE, __UUID_CHARACTERISTIC_DEVICE_NAME, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),
        /* Characteristic: Appearance */
        CHARACTERISTIC_UUID16 (HDLC_GAP_APPEARANCE, HDLC_GAP_APPEARANCE_VALUE, __UUID_CHARACTERISTIC_APPEARANCE, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),
        /* Characteristic: Peripheral Preferred Connection Parameters */
        CHARACTERISTIC_UUID16 (HDLC_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS, HDLC_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS_VALUE, __UUID_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    /* Primary Service: Generic Attribute */
    PRIMARY_SERVICE_UUID16 (HDLS_GATT, __UUID_SERVICE_GENERIC_ATTRIBUTE),

    /* Primary Service: Device Information */
    PRIMARY_SERVICE_UUID16 (HDLS_DIS, __UUID_SERVICE_DEVICE_INFORMATION),
        /* Characteristic: PnP ID */
        CHARACTERISTIC_UUID16 (HDLC_DIS_PNP_ID, HDLC_DIS_PNP_ID_VALUE, __UUID_CHARACTERISTIC_PNP_ID, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),
        /* Characteristic: Firmware Revision String */
        CHARACTERISTIC_UUID16 (HDLC_DIS_FIRMWARE_REVISION_STRING, HDLC_DIS_FIRMWARE_REVISION_STRING_VALUE, __UUID_CHARACTERISTIC_FIRMWARE_REVISION_STRING, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    /* Primary Service: Scan Parameters */
    PRIMARY_SERVICE_UUID16 (HDLS_SCPS, __UUID_SERVICE_SCAN_PARAMETERS),
        /* Characteristic: Scan Interval Window */
        CHARACTERISTIC_UUID16_WRITABLE (HDLC_SCPS_SCAN_INTERVAL_WINDOW, HDLC_SCPS_SCAN_INTERVAL_WINDOW_VALUE, __UUID_CHARACTERISTIC_SCAN_INTERVAL_WINDOW, GATTDB_CHAR_PROP_WRITE_NO_RESPONSE, GATTDB_PERM_WRITE_CMD),

    /* Primary Service: Battery */
    PRIMARY_SERVICE_UUID16 (HDLS_BAS, __UUID_SERVICE_BATTERY),
        /* Characteristic: Battery Level */
        CHARACTERISTIC_UUID16 (HDLC_BAS_BATTERY_LEVEL, HDLC_BAS_BATTERY_LEVEL_VALUE, __UUID_CHARACTERISTIC_BATTERY_LEVEL, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY, GATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG, __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),
            /* Descriptor: Report Reference */
            CHAR_DESCRIPTOR_UUID16 (HDLD_BAS_BATTERY_LEVEL_REPORT_REFERENCE, __UUID_DESCRIPTOR_REPORT_REFERENCE, GATTDB_PERM_READABLE),

    /* Primary Service: ATVS */
    PRIMARY_SERVICE_UUID128 (HDLS_ATVS, __UUID_SERVICE_ATVS),
        /* Characteristic: ATV_Write_Char */
        CHARACTERISTIC_UUID128_WRITABLE (HDLC_ATVS_ATV_WRITE_CHAR, HDLC_ATVS_ATV_WRITE_CHAR_VALUE, __UUID_CHARACTERISTIC_ATVS_ATV_WRITE_CHAR, GATTDB_CHAR_PROP_WRITE, GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),
        /* Characteristic: ATV_Read_Char */
        CHARACTERISTIC_UUID128 (HDLC_ATVS_ATV_READ_CHAR, HDLC_ATVS_ATV_READ_CHAR_VALUE, __UUID_CHARACTERISTIC_ATVS_ATV_READ_CHAR, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY, GATTDB_PERM_READABLE),
            /* Descriptor: ATV_Read_CCCD */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_ATVS_ATV_READ_CHAR_ATV_READ_CCCD, __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),
        /* Characteristic: ATV_Control_Char */
        CHARACTERISTIC_UUID128 (HDLC_ATVS_ATV_CONTROL_CHAR, HDLC_ATVS_ATV_CONTROL_CHAR_VALUE, __UUID_CHARACTERISTIC_ATVS_ATV_CONTROL_CHAR, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY, GATTDB_PERM_READABLE),
            /* Descriptor: ATV_Control_CCCD */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_ATVS_ATV_CONTROL_CHAR_ATV_CONTROL_CCCD, __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    /* Primary Service: Human Interface Device */
    PRIMARY_SERVICE_UUID16 (HDLS_HIDS, __UUID_SERVICE_HUMAN_INTERFACE_DEVICE),
        /* Included Service: Battery */
        INCLUDE_SERVICE_UUID16 (HDLI_HIDS_INCLUDE_BAS, HDLS_BAS, HDLD_BAS_BATTERY_LEVEL_REPORT_REFERENCE, __UUID_SERVICE_BATTERY),
        /* Characteristic: Protocol Mode */
        CHARACTERISTIC_UUID16_WRITABLE (HDLC_HIDS_PROTOCOL_MODE, HDLC_HIDS_PROTOCOL_MODE_VALUE, __UUID_CHARACTERISTIC_PROTOCOL_MODE, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE, GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD),
        /* Characteristic: HID Information */
        CHARACTERISTIC_UUID16 (HDLC_HIDS_HID_INFORMATION, HDLC_HIDS_HID_INFORMATION_VALUE, __UUID_CHARACTERISTIC_HID_INFORMATION, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),
        /* Characteristic: HID Control Point */
        CHARACTERISTIC_UUID16_WRITABLE (HDLC_HIDS_HID_CONTROL_POINT, HDLC_HIDS_HID_CONTROL_POINT_VALUE, __UUID_CHARACTERISTIC_HID_CONTROL_POINT, GATTDB_CHAR_PROP_WRITE_NO_RESPONSE, GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD | GATTDB_PERM_RELIABLE_WRITE),
        /* Characteristic: kbd_in_report */
        CHARACTERISTIC_UUID16 (HDLC_HIDS_KBD_IN_REPORT, HDLC_HIDS_KBD_IN_REPORT_VALUE, __UUID_CHARACTERISTIC_REPORT, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY, GATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_HIDS_KBD_IN_REPORT_CLIENT_CHAR_CONFIG, __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),
            /* Descriptor: Report Reference */
            CHAR_DESCRIPTOR_UUID16 (HDLD_HIDS_KBD_IN_REPORT_REPORT_REFERENCE, __UUID_DESCRIPTOR_REPORT_REFERENCE, GATTDB_PERM_READABLE),
        /* Characteristic: cc_in_report */
        CHARACTERISTIC_UUID16 (HDLC_HIDS_CC_IN_REPORT, HDLC_HIDS_CC_IN_REPORT_VALUE, __UUID_CHARACTERISTIC_REPORT, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY, GATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_HIDS_CC_IN_REPORT_CLIENT_CHAR_CONFIG, __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),
            /* Descriptor: Report Reference */
            CHAR_DESCRIPTOR_UUID16 (HDLD_HIDS_CC_IN_REPORT_REPORT_REFERENCE, __UUID_DESCRIPTOR_REPORT_REFERENCE, GATTDB_PERM_READABLE),
        /* Characteristic: Report Map */
        CHARACTERISTIC_UUID16 (HDLC_HIDS_REPORT_MAP, HDLC_HIDS_REPORT_MAP_VALUE, __UUID_CHARACTERISTIC_REPORT_MAP, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),
            /* Descriptor: External Report Reference */
            CHAR_DESCRIPTOR_UUID16 (HDLD_HIDS_REPORT_MAP_EXTERNAL_REPORT_REFERENCE, __UUID_DESCRIPTOR_EXTERNAL_REPORT_REFERENCE, GATTDB_PERM_READABLE),
};

/* Length of the GATT database */
const uint16_t gatt_database_len = sizeof(gatt_database);

/*************************************************************************************
 * GATT Initial Value Arrays
 ************************************************************************************/
 
uint8_t app_gap_device_name[]                                = {'H', 'I', 'D', '-', 'R', 'e', 'm', 'o', 't', 'e', '\0', };
uint8_t app_gap_appearance[]                                 = {0x80, 0x01, };
uint8_t app_gap_peripheral_preferred_connection_parameters[] = {0x18, 0x00, 0x18, 0x00, 0x09, 0x00, 0xF4, 0x01, };
uint8_t app_dis_pnp_id[]                                     = {0x01, 0x31, 0x01, 0x01, 0x00, 0x01, 0x00, };
uint8_t app_dis_firmware_revision_string[]                   = {'B', 'e', 't', 'a', };
uint8_t app_scps_scan_interval_window[]                      = {0x60, 0x00, 0x14, 0x00, };
uint8_t app_bas_battery_level[]                              = {0x64, };
uint8_t app_bas_battery_level_client_char_config[]           = {0x00, 0x00, };
uint8_t app_bas_battery_level_report_reference[]             = {0x03, 0x01, };
uint8_t app_atvs_atv_write_char[]                            = {0x00, 0x00, 0x00, 0x00, 0x00, };
uint8_t app_atvs_atv_read_char[]                             = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
uint8_t app_atvs_atv_read_char_atv_read_cccd[]               = {0x00, 0x00, };
uint8_t app_atvs_atv_control_char[]                          = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
uint8_t app_atvs_atv_control_char_atv_control_cccd[]         = {0x00, 0x00, };
uint8_t app_hids_protocol_mode[]                             = {0x01, };
uint8_t app_hids_hid_information[]                           = {0x11, 0x01, 0x0D, 0x02, };
uint8_t app_hids_hid_control_point[]                         = {0x00, };
uint8_t app_hids_kbd_in_report[]                             = {0x00, 0x00, 0x00, };
uint8_t app_hids_kbd_in_report_client_char_config[]          = {0x00, 0x00, };
uint8_t app_hids_kbd_in_report_report_reference[]            = {0x00, 0x01, };
uint8_t app_hids_cc_in_report[]                              = {0x00, 0x00, };
uint8_t app_hids_cc_in_report_client_char_config[]           = {0x00, 0x00, };
uint8_t app_hids_cc_in_report_report_reference[]             = {0x02, 0x01, };
uint8_t app_hids_report_map[]                                = {
    0x05, 0x01,       /* USAGE_PAGE         */
    0x09, 0x06,       /* USAGE              */
    0xA1, 0x01,       /* COLLECTION         */
    0x05, 0x07,       /* USAGE_PAGE         */
    0x85, 0x01,       /* REPORT_ID          */
    0x75, 0x08,       /* REPORT_SIZE        */
    0x95, 0x03,       /* REPORT_COUNT       */
    0x15, 0x00,       /* LOGICAL_MINIMUM    */
    0x25, 0xFF,       /* LOGICAL_MAXIMUM    */
    0x19, 0x00,       /* USAGE_MINIMUM      */
    0x29, 0xFF,       /* USAGE_MAXIMUM      */
    0x81, 0x00,       /* INPUT              */
    0xC0,             /* END_COLLECTION     */
    0x05, 0x0C,       /* USAGE_PAGE         */
    0x09, 0x01,       /* USAGE              */
    0xA1, 0x01,       /* COLLECTION         */
    0x85, 0x02,       /* REPORT_ID          */
    0x75, 0x0C,       /* REPORT_SIZE        */
    0x95, 0x02,       /* REPORT_COUNT       */
    0x15, 0x00,       /* LOGICAL_MINIMUM    */
    0x26, 0xFF, 0x07, /* LOGICAL_MAXIMUM    */
    0x19, 0x00,       /* USAGE_MINIMUM      */
    0x2A, 0xFF, 0x07, /* USAGE_MAXIMUM      */
    0x81, 0x00,       /* INPUT              */
    0xC0,             /* END_COLLECTION     */
    0x05, 0x0C,       /* USAGE_PAGE         */
    0x09, 0x01,       /* USAGE              */
    0xA1, 0x01,       /* COLLECTION         */
    0x85, 0x03,       /* REPORT_ID          */
    0x05, 0x06,       /* USAGE_PAGE         */
    0x09, 0x20,       /* USAGE              */
    0x15, 0x00,       /* LOGICAL_MINIMUM    */
    0x25, 0x64,       /* LOGICAL_MAXIMUM    */
    0x75, 0x08,       /* REPORT_SIZE        */
    0x95, 0x01,       /* REPORT_COUNT       */
    0x81, 0x02,       /* INPUT              */
    0xC0,             /* END_COLLECTION     */};
uint8_t app_hids_report_map_external_report_reference[]      = {0x19, 0x2A, };
 
 /************************************************************************************
 * GATT Lookup Table
 ************************************************************************************/
 
gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl[] =
{
    /* { attribute handle,                                 maxlen, curlen, attribute data } */
    { HDLC_GAP_DEVICE_NAME_VALUE,                          10,     10,     app_gap_device_name },
    { HDLC_GAP_APPEARANCE_VALUE,                           2,      2,      app_gap_appearance },
    { HDLC_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS_VALUE, 8,      8,      app_gap_peripheral_preferred_connection_parameters },
    { HDLC_DIS_PNP_ID_VALUE,                               7,      7,      app_dis_pnp_id },
    { HDLC_DIS_FIRMWARE_REVISION_STRING_VALUE,             4,      4,      app_dis_firmware_revision_string },
    { HDLC_SCPS_SCAN_INTERVAL_WINDOW_VALUE,                4,      4,      app_scps_scan_interval_window },
    { HDLC_BAS_BATTERY_LEVEL_VALUE,                        1,      1,      app_bas_battery_level },
    { HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG,           2,      2,      app_bas_battery_level_client_char_config },
    { HDLD_BAS_BATTERY_LEVEL_REPORT_REFERENCE,             2,      2,      app_bas_battery_level_report_reference },
    { HDLC_ATVS_ATV_WRITE_CHAR_VALUE,                      5,      5,      app_atvs_atv_write_char },
    { HDLC_ATVS_ATV_READ_CHAR_VALUE,                       134,    134,    app_atvs_atv_read_char },
    { HDLD_ATVS_ATV_READ_CHAR_ATV_READ_CCCD,               2,      2,      app_atvs_atv_read_char_atv_read_cccd },
    { HDLC_ATVS_ATV_CONTROL_CHAR_VALUE,                    9,      9,      app_atvs_atv_control_char },
    { HDLD_ATVS_ATV_CONTROL_CHAR_ATV_CONTROL_CCCD,         2,      2,      app_atvs_atv_control_char_atv_control_cccd },
    { HDLC_HIDS_PROTOCOL_MODE_VALUE,                       1,      1,      app_hids_protocol_mode },
    { HDLC_HIDS_HID_INFORMATION_VALUE,                     4,      4,      app_hids_hid_information },
    { HDLC_HIDS_HID_CONTROL_POINT_VALUE,                   1,      1,      app_hids_hid_control_point },
    { HDLC_HIDS_KBD_IN_REPORT_VALUE,                       3,      3,      app_hids_kbd_in_report },
    { HDLD_HIDS_KBD_IN_REPORT_CLIENT_CHAR_CONFIG,          2,      2,      app_hids_kbd_in_report_client_char_config },
    { HDLD_HIDS_KBD_IN_REPORT_REPORT_REFERENCE,            2,      2,      app_hids_kbd_in_report_report_reference },
    { HDLC_HIDS_CC_IN_REPORT_VALUE,                        2,      2,      app_hids_cc_in_report },
    { HDLD_HIDS_CC_IN_REPORT_CLIENT_CHAR_CONFIG,           2,      2,      app_hids_cc_in_report_client_char_config },
    { HDLD_HIDS_CC_IN_REPORT_REPORT_REFERENCE,             2,      2,      app_hids_cc_in_report_report_reference },
    { HDLC_HIDS_REPORT_MAP_VALUE,                          73,     73,     app_hids_report_map },
    { HDLD_HIDS_REPORT_MAP_EXTERNAL_REPORT_REFERENCE,      2,      2,      app_hids_report_map_external_report_reference },
};

/* Number of Lookup Table entries */
const uint16_t app_gatt_db_ext_attr_tbl_size = (sizeof(app_gatt_db_ext_attr_tbl) / sizeof(gatt_db_lookup_table_t));

/* Number of GATT initial value arrays entries */
const uint16_t app_gap_device_name_len = 10;
const uint16_t app_gap_appearance_len = (sizeof(app_gap_appearance));
const uint16_t app_gap_peripheral_preferred_connection_parameters_len = (sizeof(app_gap_peripheral_preferred_connection_parameters));
const uint16_t app_dis_pnp_id_len = (sizeof(app_dis_pnp_id));
const uint16_t app_dis_firmware_revision_string_len = (sizeof(app_dis_firmware_revision_string));
const uint16_t app_scps_scan_interval_window_len = (sizeof(app_scps_scan_interval_window));
const uint16_t app_bas_battery_level_len = (sizeof(app_bas_battery_level));
const uint16_t app_bas_battery_level_client_char_config_len = (sizeof(app_bas_battery_level_client_char_config));
const uint16_t app_bas_battery_level_report_reference_len = (sizeof(app_bas_battery_level_report_reference));
const uint16_t app_atvs_atv_write_char_len = (sizeof(app_atvs_atv_write_char));
const uint16_t app_atvs_atv_read_char_len = (sizeof(app_atvs_atv_read_char));
const uint16_t app_atvs_atv_read_char_atv_read_cccd_len = (sizeof(app_atvs_atv_read_char_atv_read_cccd));
const uint16_t app_atvs_atv_control_char_len = (sizeof(app_atvs_atv_control_char));
const uint16_t app_atvs_atv_control_char_atv_control_cccd_len = (sizeof(app_atvs_atv_control_char_atv_control_cccd));
const uint16_t app_hids_protocol_mode_len = (sizeof(app_hids_protocol_mode));
const uint16_t app_hids_hid_information_len = (sizeof(app_hids_hid_information));
const uint16_t app_hids_hid_control_point_len = (sizeof(app_hids_hid_control_point));
const uint16_t app_hids_kbd_in_report_len = (sizeof(app_hids_kbd_in_report));
const uint16_t app_hids_kbd_in_report_client_char_config_len = (sizeof(app_hids_kbd_in_report_client_char_config));
const uint16_t app_hids_kbd_in_report_report_reference_len = (sizeof(app_hids_kbd_in_report_report_reference));
const uint16_t app_hids_cc_in_report_len = (sizeof(app_hids_cc_in_report));
const uint16_t app_hids_cc_in_report_client_char_config_len = (sizeof(app_hids_cc_in_report_client_char_config));
const uint16_t app_hids_cc_in_report_report_reference_len = (sizeof(app_hids_cc_in_report_report_reference));
const uint16_t app_hids_report_map_len = (sizeof(app_hids_report_map));
const uint16_t app_hids_report_map_external_report_reference_len = (sizeof(app_hids_report_map_external_report_reference));
