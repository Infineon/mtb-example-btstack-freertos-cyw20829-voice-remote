/*******************************************************************************
* File Name: app_bt_advert.h
*
* Description: This File provides the interfaces necessary for Bluetooth
* Advertisements.
*
* Related Document: See README.md
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

/* This Function starts undirected Bluetooth advertisement */
void app_bt_start_adv_any_host(void);

/* This Function stops ongoing Bluetooth advertisement */
void app_bt_stop_adv(void);

#endif // __APP_BT_ADVERT_H__