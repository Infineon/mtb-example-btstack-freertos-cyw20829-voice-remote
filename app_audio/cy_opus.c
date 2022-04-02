/******************************************************************************
* File Name:   cy_opus.c
*
* Description: This files contains the function definition of OPUS Codec
* Initialization, configuration and Encoding
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "opus.h"
#include "debug.h"
#include "opus_types.h"
#include "opus_private.h"
#include "audio.h"


/*******************************************************************************
*        Variable Definitions
*******************************************************************************/

/* Opus Packet Format */
/*
 *---------------------------------------------------
 *  4 bytes  |   4 bytes      |     80 Bytes data    |
 *---------------------------------------------------
 *           |                |                      |
 * Length    |Enc Final Range | Encoded Samples      |
 *           |                |                      |
 *---------------------------------------------------
 */

/*
                             0 1 2 3 4 5 6 7
                             +-+-+-+-+-+-+-+-+
                             | config  |s| c |
                             +-+-+-+-+-+-+-+-+
 */

/* Opus Encoder initialization configuration */
typedef struct
{
    uint16_t sampling_rate;         /* Sampling frequency */
    uint16_t bitrate;               /* bits per second */
    uint16_t pcm_frame_size;        /* Audio Frame per samples */
    uint16_t bit_depth;             /* 8 - 24*/
    uint8_t  complexity;            /* 0 - 10 */
    uint8_t  vbr;                   /* variable bit rate, 1:vbr, 0:cbr */
    uint8_t  cvbr;                  /* constrained vbr, 1:disable, 0:enable */
    uint8_t  *p_encoded_buffer;     /* Pointer to Output Buffer */
    int16_t  *p_pcm_buffer;         /* Pointer to Input Buffer */
    uint8_t  channels;              /* 1 - mono, 2- stereo */
    uint8_t  status;                /* Store Encoder Creation result */
    void     *enc_h;                /* Opus Encoder Handle */
}cy_opus_enc_cfg_t;


/* Opus encoder configuration parameters */
cy_opus_enc_cfg_t opus_enc_cfg =
{
    .sampling_rate = AUDIO_SAMPLING_RATE,
    .bitrate = OPUS_ENC_BITRATE,
    .pcm_frame_size = AUDIO_FRAME_SIZE,
    .bit_depth = 16,
    .complexity = 3,
    .vbr = 0,
    .cvbr = 0,
    .channels = 1
};


/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/


/*******************************************************************************
* Function Name: int_to_char
********************************************************************************
* Summary:
*  Converts integer to character
*
* Parameters:
*  i:  The input integer
*  ch: Stores the converted character
*
*******************************************************************************/
static void int_to_char(uint32_t i, unsigned char ch[4])
{
    ch[0] = i>>24;
    ch[1] = (i>>16)&0xFF;
    ch[2] = (i>>8)&0xFF;
    ch[3] = i&0xFF;
}


/*******************************************************************************
* Function Name: cy_opus_encode
********************************************************************************
* Summary:
*  Reads the PCM buffer and runs Opus encoder to compress
*
* Parameters:
*  config: Pointer to Encoder Configuration
*
* Return:
*   length of encoded Samples
*
*******************************************************************************/
static uint16_t cy_opus_encode(cy_opus_enc_cfg_t *config)
{
    uint16_t len;
    uint32_t enc_final_range;
    unsigned char int_field[4];
    uint8_t *p_opus_buffer = config->p_encoded_buffer;
    OpusEncoder *enc = (OpusEncoder *)config->enc_h;

    /* Skip the OPUS packet header, point to the encoded data */
    p_opus_buffer = p_opus_buffer + 8;

#ifdef MEASURE_CYCLES

    uint8_t interruptState = 0;
    if (cnt < NO_OF_ENC_BUFF)
    {
        interruptState = Cy_SysLib_EnterCriticalSection();
        prev_cnt = DWT_CCNT;
        DWT_CTRL = 0x40000001; // Enable Cycle Count
    }
#endif
    /* Encodes the PCM frame and returns the no of encoded samples */
    len = opus_encode(enc, config->p_pcm_buffer, config->pcm_frame_size,
                      p_opus_buffer, AUDIO_FRAME_SIZE);

#ifdef MEASURE_CYCLES

    if (cnt < NO_OF_ENC_BUFF)
      {
        DWT_CTRL = 0x40000000; // Disable Cycle Count
        cycle_cnt[cnt] = DWT_CCNT - prev_cnt;
        total_cnt = total_cnt + cycle_cnt[cnt] ;
        cnt++;
        Cy_SysLib_ExitCriticalSection(interruptState);
      }
#endif

    /* Store the len in header of packet */
    p_opus_buffer = config->p_encoded_buffer;
    int_to_char(len, int_field);
    memcpy(p_opus_buffer, &int_field, 4);

    /* store the encoder final range in header */
    p_opus_buffer = p_opus_buffer + 4;
    opus_encoder_ctl(enc, OPUS_GET_FINAL_RANGE(&enc_final_range));
    int_to_char(enc_final_range, int_field);
    memcpy(p_opus_buffer, &int_field, 4);

    /* Total len of pkt is len + header */
    return (len + OPUS_ENC_HEADER_SIZE);
}

/*******************************************************************************
* Function Name: cy_opus_enc_init
********************************************************************************
* Summary:
*  This function creates Opus encoder with the given Encoder configuration and
*  configures the encoder for CELT codec only
*
* Parameters:
*  config: Pointer to Encoder Configuration
*
*******************************************************************************/
static void cy_opus_enc_init(cy_opus_enc_cfg_t *config)
{
    int err;
    OpusEncoder *enc;

    /* Opus Encoder parameters check, support limited configuration */
    if ((config->sampling_rate != 16000) ||
        (config->channels != 1) ||
        (config->bitrate != 32000) ||
        ((config->complexity > 10) || (config->complexity < 0)) ||
        (config->vbr != 0) ||
        (config->cvbr != 0) ||
        (config->pcm_frame_size != AUDIO_FRAME_SIZE))
    {
        printf("Encoder params check Failed! \r\n");
        CY_ASSERT(0);
    }

    /* Allocates Opus encoder instance and initializes */
    config->enc_h = (void *) opus_encoder_create(config->sampling_rate, config->channels,
                                                 OPUS_APPLICATION_AUDIO, &err);

    config->status =  err;

    /* check for Successful creation of encoder */
    if(OPUS_OK != config->status)
    {
        printf("Encoder creation Failed\r\n");
        CY_ASSERT(0);
    }
    enc = (OpusEncoder *)config->enc_h;

    /* Configures the Encoder for CELT codec Mode */
    opus_encoder_ctl(enc, OPUS_SET_BITRATE(config->bitrate));
    opus_encoder_ctl(enc, OPUS_SET_BANDWIDTH(OPUS_AUTO));
    opus_encoder_ctl(enc, OPUS_SET_VBR(config->vbr));
    opus_encoder_ctl(enc, OPUS_SET_VBR_CONSTRAINT(config->cvbr));
    opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(config->complexity));
    opus_encoder_ctl(enc, OPUS_SET_FORCE_CHANNELS(OPUS_AUTO));
    opus_encoder_ctl(enc, OPUS_SET_LSB_DEPTH(config->bit_depth));
    opus_encoder_ctl(enc, OPUS_SET_PACKET_LOSS_PERC(0));
    opus_encoder_ctl(enc, OPUS_SET_DTX(0));
    opus_encoder_ctl(enc, OPUS_SET_INBAND_FEC(0));
    opus_encoder_ctl(enc, OPUS_SET_FORCE_MODE(MODE_CELT_ONLY));
    opus_encoder_ctl(enc, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_ARG));

}


/*******************************************************************************
* Function Name: cy_encoder_init
********************************************************************************
* Summary:
*  This function invokes Opus Encoder Initialization
*
*******************************************************************************/
void cy_encoder_init(void)
{
    cy_opus_enc_init(&opus_enc_cfg);
}

/*******************************************************************************
* Function Name: cy_encode
********************************************************************************
* Summary:
* This Function invokes the Opus Encoder api to encode the audio data
*
* Parameters:
*  ip_samples: Input Audio Samples to encoder
*  op_frame: Encoded Audio frame given by Opus Encoder
*
* Return:
*   length of Encoded Audio Packet
*******************************************************************************/
uint16_t cy_encode(int16_t *ip_samples, uint8_t *op_frame)
{
    uint16_t opus_pkt_len;
    opus_enc_cfg.p_pcm_buffer = ip_samples;

    opus_enc_cfg.p_encoded_buffer = op_frame+1;

    opus_pkt_len = cy_opus_encode(&opus_enc_cfg);

    return opus_pkt_len;
}

/*******************************************************************************
* Function Name: cy_encoder_reset
********************************************************************************
* Summary:
*  This function resets the Encoder State
*
*******************************************************************************/
void cy_encoder_reset(void)
{
    return ;
}

/* [] END OF FILE */
