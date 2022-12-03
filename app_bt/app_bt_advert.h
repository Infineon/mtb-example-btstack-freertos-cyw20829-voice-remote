/*******************************************************************************
* File Name: app_bt_advert.h
*
* Description: This File provides the interfaces necessary for Bluetooth
* Advertisements.
*
* Related Document: See README.md
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

#ifndef __APP_BT_ADVERT_H__
#define __APP_BT_ADVERT_H__
/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "cycfg_gap.h"

/*******************************************************************************
 *                      MACROS / VARIABLE DEFINITIONS
 ******************************************************************************/
/* Number of advertisement frames */
#define NUM_ADV_ELEM            (CY_BT_ADV_PACKET_DATA_SIZE)

/* State of the remote application */
typedef enum
{
    UNPAIRED_ON,                    /* Unpaired state when the Device is just powered up */
    PAIRED_ON,                      /* Paired state when the Device is just powered up */
    UNPAIRED_ADVERTISING,           /* Unpaired Undirected advertisement state in which the device is not bonded and making it discoverable for the hosts */
    PAIRED_ADVERTISING_KNOWN_HOST,  /* Paired Directed advertisement state in which the device has the bond information to connect again to the host */
    PAIRED_ADVERTISING_ANY_HOST,    /* Paired Undirected advertisement state in which the device is bonded but making it discoverable for the other new hosts */
    UNPAIRED_IDLE,                  /* Unpaired Sleep state in which advertisement has timed out due to no interest from any new hosts */
    PAIRED_IDLE,                    /* Paired Sleep state in which advertisement has timed out due to no interest from any new hosts or connected host */
    CONNECTED_NON_ADVERTISING,      /* Paired Connected state in which the device is not interested in advertising */
    CONNECTED_ADVERTISING,          /* Paired Connected state in which the device is advertising to get paired to a new host */
}remote_app_ble_state_t;

extern remote_app_ble_state_t current_remote_state;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
/* This function initializes advertisement data and pairable mode */
void app_bt_adv_start(void);

/* This Function does advertising only for the bonded devices */
void app_bt_set_adv_filter(wiced_bt_device_address_t bdaddr);

/* This function handles the remote state based on the advert state change */
void app_bt_advert_state_handler(wiced_bt_ble_advert_mode_t current_adv_mode);

/* This Function starts directed Bluetooth advertisement */
void app_bt_start_adv_known_host(void);

/* This Function starts undirected Bluetooth advertisement */
void app_bt_start_adv_any_host(void);

/* This Function stops ongoing Bluetooth advertisement */
void app_bt_stop_adv(void);

#endif // __APP_BT_ADVERT_H__