/***************************************************************************//**
* File Name: cycfg_gatt_db.h
*
* Description:
* Definitions for constants used in the device's GATT database and function
* prototypes.
* This file should not be modified. It was automatically generated by
* Bluetooth Configurator 2.50.0.6066
*
********************************************************************************
* Copyright 2021 Cypress Semiconductor Corporation (an Infineon company) or
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

#if !defined(CYCFG_GATT_DB_H)
#define CYCFG_GATT_DB_H

#include "stdint.h"

#define __UUID_SERVICE_GENERIC_ACCESS                          0x1800
#define __UUID_CHARACTERISTIC_DEVICE_NAME                      0x2A00
#define __UUID_CHARACTERISTIC_APPEARANCE                       0x2A01
#define __UUID_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS    0x2A04
#define __UUID_SERVICE_GENERIC_ATTRIBUTE                       0x1801
#define __UUID_SERVICE_DEVICE_INFORMATION                      0x180A
#define __UUID_CHARACTERISTIC_PNP_ID                           0x2A50
#define __UUID_CHARACTERISTIC_FIRMWARE_REVISION_STRING         0x2A26
#define __UUID_SERVICE_SCAN_PARAMETERS                         0x1813
#define __UUID_CHARACTERISTIC_SCAN_INTERVAL_WINDOW             0x2A4F
#define __UUID_SERVICE_BATTERY                                 0x180F
#define __UUID_CHARACTERISTIC_BATTERY_LEVEL                    0x2A19
#define __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION    0x2902
#define __UUID_DESCRIPTOR_REPORT_REFERENCE                     0x2908
#define __UUID_SERVICE_HUMAN_INTERFACE_DEVICE                  0x1812
#define __UUID_CHARACTERISTIC_HID_INFORMATION                  0x2A4A
#define __UUID_CHARACTERISTIC_HID_CONTROL_POINT                0x2A4C
#define __UUID_CHARACTERISTIC_REPORT                           0x2A4D
#define __UUID_CHARACTERISTIC_REPORT_MAP                       0x2A4B
#define __UUID_DESCRIPTOR_EXTERNAL_REPORT_REFERENCE            0x2907

/* Service Generic Access */
#define HDLS_GAP                                               0x0001
/* Characteristic Device Name */
#define HDLC_GAP_DEVICE_NAME                                   0x0002
#define HDLC_GAP_DEVICE_NAME_VALUE                             0x0003
/* Characteristic Appearance */
#define HDLC_GAP_APPEARANCE                                    0x0004
#define HDLC_GAP_APPEARANCE_VALUE                              0x0005
/* Characteristic Peripheral Preferred Connection Parameters */
#define HDLC_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS    0x0006
#define HDLC_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS_VALUE    0x0007

/* Service Generic Attribute */
#define HDLS_GATT                                              0x0008

/* Service Device Information */
#define HDLS_DIS                                               0x0009
/* Characteristic PnP ID */
#define HDLC_DIS_PNP_ID                                        0x000A
#define HDLC_DIS_PNP_ID_VALUE                                  0x000B
/* Characteristic Firmware Revision String */
#define HDLC_DIS_FIRMWARE_REVISION_STRING                      0x000C
#define HDLC_DIS_FIRMWARE_REVISION_STRING_VALUE                0x000D

/* Service Scan Parameters */
#define HDLS_SCPS                                              0x000E
/* Characteristic Scan Interval Window */
#define HDLC_SCPS_SCAN_INTERVAL_WINDOW                         0x000F
#define HDLC_SCPS_SCAN_INTERVAL_WINDOW_VALUE                   0x0010

/* Service Battery */
#define HDLS_BAS                                               0x0011
/* Characteristic Battery Level */
#define HDLC_BAS_BATTERY_LEVEL                                 0x0012
#define HDLC_BAS_BATTERY_LEVEL_VALUE                           0x0013
/* Descriptor Client Characteristic Configuration */
#define HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG              0x0014
/* Descriptor Report Reference */
#define HDLD_BAS_BATTERY_LEVEL_REPORT_REFERENCE                0x0015

/* Service Human Interface Device */
#define HDLS_HIDS                                              0x0016
/* Service Battery */
#define HDLI_HIDS_INCLUDE_BAS                                  0x0017
/* Characteristic HID Information */
#define HDLC_HIDS_HID_INFORMATION                              0x0018
#define HDLC_HIDS_HID_INFORMATION_VALUE                        0x0019
/* Characteristic HID Control Point */
#define HDLC_HIDS_HID_CONTROL_POINT                            0x001A
#define HDLC_HIDS_HID_CONTROL_POINT_VALUE                      0x001B
/* Characteristic kbd_in_report */
#define HDLC_HIDS_KBD_IN_REPORT                                0x001C
#define HDLC_HIDS_KBD_IN_REPORT_VALUE                          0x001D
/* Descriptor Client Characteristic Configuration */
#define HDLD_HIDS_KBD_IN_REPORT_CLIENT_CHAR_CONFIG             0x001E
/* Descriptor Report Reference */
#define HDLD_HIDS_KBD_IN_REPORT_REPORT_REFERENCE               0x001F
/* Characteristic cc_in_report */
#define HDLC_HIDS_CC_IN_REPORT                                 0x0020
#define HDLC_HIDS_CC_IN_REPORT_VALUE                           0x0021
/* Descriptor Client Characteristic Configuration */
#define HDLD_HIDS_CC_IN_REPORT_CLIENT_CHAR_CONFIG              0x0022
/* Descriptor Report Reference */
#define HDLD_HIDS_CC_IN_REPORT_REPORT_REFERENCE                0x0023
/* Characteristic voice_in_report */
#define HDLC_HIDS_VOICE_IN_REPORT                              0x0024
#define HDLC_HIDS_VOICE_IN_REPORT_VALUE                        0x0025
/* Descriptor Client Characteristic Configuration */
#define HDLD_HIDS_VOICE_IN_REPORT_CLIENT_CHAR_CONFIG           0x0026
/* Descriptor Report Reference */
#define HDLD_HIDS_VOICE_IN_REPORT_REPORT_REFERENCE             0x0027
/* Characteristic voice_ctl_in_report */
#define HDLC_HIDS_VOICE_CTL_IN_REPORT                          0x0028
#define HDLC_HIDS_VOICE_CTL_IN_REPORT_VALUE                    0x0029
/* Descriptor Client Characteristic Configuration */
#define HDLD_HIDS_VOICE_CTL_IN_REPORT_CLIENT_CHAR_CONFIG       0x002A
/* Descriptor Report Reference */
#define HDLD_HIDS_VOICE_CTL_IN_REPORT_REPORT_REFERENCE         0x002B
/* Characteristic voice_ctl_feat_report */
#define HDLC_HIDS_VOICE_CTL_FEAT_REPORT                        0x002C
#define HDLC_HIDS_VOICE_CTL_FEAT_REPORT_VALUE                  0x002D
/* Descriptor Report Reference */
#define HDLD_HIDS_VOICE_CTL_FEAT_REPORT_REPORT_REFERENCE       0x002E
/* Characteristic Report Map */
#define HDLC_HIDS_REPORT_MAP                                   0x002F
#define HDLC_HIDS_REPORT_MAP_VALUE                             0x0030
/* Descriptor External Report Reference */
#define HDLD_HIDS_REPORT_MAP_EXTERNAL_REPORT_REFERENCE         0x0031

/* External Lookup Table Entry */
typedef struct
{
    uint16_t handle;
    uint16_t max_len;
    uint16_t cur_len;
    uint8_t  *p_data;
} gatt_db_lookup_table_t;

/* External definitions */
extern const uint8_t  gatt_database[];
extern const uint16_t gatt_database_len;
extern gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl[];
extern const uint16_t app_gatt_db_ext_attr_tbl_size;
extern uint8_t app_gap_device_name[];
extern const uint16_t app_gap_device_name_len;
extern uint8_t app_gap_appearance[];
extern const uint16_t app_gap_appearance_len;
extern uint8_t app_gap_peripheral_preferred_connection_parameters[];
extern const uint16_t app_gap_peripheral_preferred_connection_parameters_len;
extern uint8_t app_dis_pnp_id[];
extern const uint16_t app_dis_pnp_id_len;
extern uint8_t app_dis_firmware_revision_string[];
extern const uint16_t app_dis_firmware_revision_string_len;
extern uint8_t app_scps_scan_interval_window[];
extern const uint16_t app_scps_scan_interval_window_len;
extern uint8_t app_bas_battery_level[];
extern const uint16_t app_bas_battery_level_len;
extern uint8_t app_bas_battery_level_client_char_config[];
extern const uint16_t app_bas_battery_level_client_char_config_len;
extern uint8_t app_bas_battery_level_report_reference[];
extern const uint16_t app_bas_battery_level_report_reference_len;
extern uint8_t app_hids_hid_information[];
extern const uint16_t app_hids_hid_information_len;
extern uint8_t app_hids_hid_control_point[];
extern const uint16_t app_hids_hid_control_point_len;
extern uint8_t app_hids_kbd_in_report[];
extern const uint16_t app_hids_kbd_in_report_len;
extern uint8_t app_hids_kbd_in_report_client_char_config[];
extern const uint16_t app_hids_kbd_in_report_client_char_config_len;
extern uint8_t app_hids_kbd_in_report_report_reference[];
extern const uint16_t app_hids_kbd_in_report_report_reference_len;
extern uint8_t app_hids_cc_in_report[];
extern const uint16_t app_hids_cc_in_report_len;
extern uint8_t app_hids_cc_in_report_client_char_config[];
extern const uint16_t app_hids_cc_in_report_client_char_config_len;
extern uint8_t app_hids_cc_in_report_report_reference[];
extern const uint16_t app_hids_cc_in_report_report_reference_len;
extern uint8_t app_hids_voice_in_report[];
extern const uint16_t app_hids_voice_in_report_len;
extern uint8_t app_hids_voice_in_report_client_char_config[];
extern const uint16_t app_hids_voice_in_report_client_char_config_len;
extern uint8_t app_hids_voice_in_report_report_reference[];
extern const uint16_t app_hids_voice_in_report_report_reference_len;
extern uint8_t app_hids_voice_ctl_in_report[];
extern const uint16_t app_hids_voice_ctl_in_report_len;
extern uint8_t app_hids_voice_ctl_in_report_client_char_config[];
extern const uint16_t app_hids_voice_ctl_in_report_client_char_config_len;
extern uint8_t app_hids_voice_ctl_in_report_report_reference[];
extern const uint16_t app_hids_voice_ctl_in_report_report_reference_len;
extern uint8_t app_hids_voice_ctl_feat_report[];
extern const uint16_t app_hids_voice_ctl_feat_report_len;
extern uint8_t app_hids_voice_ctl_feat_report_report_reference[];
extern const uint16_t app_hids_voice_ctl_feat_report_report_reference_len;
extern uint8_t app_hids_report_map[];
extern const uint16_t app_hids_report_map_len;
extern uint8_t app_hids_report_map_external_report_reference[];
extern const uint16_t app_hids_report_map_external_report_reference_len;

#endif /* CYCFG_GATT_DB_H */