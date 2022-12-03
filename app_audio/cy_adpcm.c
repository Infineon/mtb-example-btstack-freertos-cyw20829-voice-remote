/******************************************************************************
* File Name:   cy_adpcm.c
*
* Description: This files contains the function definition of ADPCM Codec
* Initialization, configuration and Encoding
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2022, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "adpcm.h"
#include "audio.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/


/*******************************************************************************
*        Variable Definitions
*******************************************************************************/

/* APPCM Packet Format */
/*
 *---------------------------------------------------
 *  2 bytes    |   1 byte  |    128 Bytes data      |
 *---------------------------------------------------
 *             |           |
 * PredictVal  |index      | Encoded Samples        |
 *             |           |                        |
 *---------------------------------------------------
 */

dvi_adpcm_state_t g_adpcm_state;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/**
 * Function Name:
 * cy_encoder_init
 *
 * @brief  This function invokes ADPCM Encoder Initialization
 *
 * @param void
 *
 * @return void
 */
void cy_encoder_init(void)
{
    dvi_adpcm_init(&g_adpcm_state);
}

/**
 * Function Name:
 * cy_encode
 *
 * @brief  This Function invokes the ADPCM Encoder api to encode the audio data.
 *
 * @param ip_samples  Input Audio Samples to encoder
 *
 * @param op_frame  Encoded Audio frame given by ADPCM Encoder
 *
 * @return uint16_t cy_encode length of Encoded Audio Packet
 */
uint16_t cy_encode(int16_t *ip_samples, uint8_t *op_frame)
{
    int adpcm_pkt_len;
    dvi_adpcm_encode(ip_samples, AUDIO_FRAME_SIZE*2, op_frame+3 ,
                     &adpcm_pkt_len, (void *)&g_adpcm_state, 1);

    return (int16_t)adpcm_pkt_len;
}

/**
 * Function Name:
 * cy_encoder_reset
 *
 * @brief  This function resets the Encoder State.
 *
 * @param void
 *
 * @return void
 */
void cy_encoder_reset(void)
{
    dvi_adpcm_init(&g_adpcm_state);
}

/* [] END OF FILE */
