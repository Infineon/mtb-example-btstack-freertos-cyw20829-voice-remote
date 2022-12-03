/******************************************************************************
* File Name:   audio.h
*
* Description: This file includes the structs, function declaration, macros and
*  enumerations used by audio module
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

#ifndef __AUDIO_H
#define __AUDIO_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
//#define USE_WAVE_FILE

/* Audio sampling frequency in kHz */
#define AUDIO_SAMPLING_RATE     (16000)

/* Samples per second*/
#define AUDIO_SAMPLES_1_MS      (AUDIO_SAMPLING_RATE/1000)

/* Number of audio frame buffers */
#define NO_AUDIO_DMA_BUF        (2)


#ifdef CONFIG_ADPCM_CODEC
/* Size of audio frame in samples*/
#define AUDIO_FRAME_SIZE        (AUDIO_SAMPLES_1_MS * 16)

#define ENC_PKT_SIZE             (AUDIO_FRAME_SIZE/2 + 6)

#else
/* Size of audio frame in samples*/
#define AUDIO_FRAME_SIZE        (AUDIO_SAMPLES_1_MS * 20)

/* OPUS frame size in ms */
#define OPUS_FRAME_IN_MS        (20)

/* Opus Encoder bitrate in bps(bits per second) */
#define OPUS_ENC_BITRATE        (32000)

/* Length of Opus frame in bytes */
#define OPUS_FRAME_SIZE         (OPUS_ENC_BITRATE/8/(1000/OPUS_FRAME_IN_MS))

/* OPUS Packet header size */
#define OPUS_ENC_HEADER_SIZE    (9)

/* OPUS encoded packet size */
#define ENC_PKT_SIZE           (OPUS_FRAME_SIZE + OPUS_ENC_HEADER_SIZE)

#endif



#ifdef USE_WAVE_FILE
/* No of Encoded Packets in buffer pool */
#define NO_OF_ENC_BUFF         (50)

//#define MEASURE_CYCLES
#else
#define NO_OF_ENC_BUFF         (5)
#endif

#define TICKS_TO_WAIT          (10u)

/*******************************************************************************
 * Data structure and enumeration
 ******************************************************************************/

/* PCM buff state */
typedef enum
{
    STATE_FREE = 0,
    STATE_ENCODING
}pcm_buff_state_t;

/* Audio Capture State */
typedef enum
{
    STREAM_ON = 0,
    STREAM_OFF
}audio_state_t;

/* Message Types for Audio msg queue */
typedef enum
{
    AUDIO_CAP_START,
    AUDIO_DATA_ENCODE,
    AUDIO_CAP_STOP,
}audio_cmd_t;

/* Message format of Audio msg */
typedef struct
{
    uint8_t cmd;
    uint8_t data;
}audio_msg_q_pkt_t;


/*******************************************************************************
 * Global variable
 ******************************************************************************/

extern int16_t pcm_buff[NO_AUDIO_DMA_BUF][AUDIO_FRAME_SIZE];
extern uint8_t enc_buff_pool[NO_OF_ENC_BUFF][ENC_PKT_SIZE];
extern volatile uint16_t pcm_buff_state[NO_AUDIO_DMA_BUF];
extern int16_t pdm_pcm_garbage_buff[AUDIO_FRAME_SIZE];

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void audio_task_init(void);
void xmit_task_init(void);
void send_msg_to_xmit_q(uint8_t idx, uint16_t len);
void send_msg_to_audio_q(uint8_t idx);
void send_start_streaming_msg(void);
void send_stop_streaming_msg(void);
void audio_mic_cap_stop(void);
void audio_streaming_stop(void);

#ifdef MEASURE_CYCLES

extern unsigned int cycle_cnt[NO_OF_ENC_BUFF];
extern unsigned int prev_cnt, total_cnt;
extern int cnt;

#define DWT_CCNT  (*(volatile unsigned int *)0xE0001004)
#define DWT_CTRL  (*(volatile unsigned int *)0xE0001000)

#endif

#endif      /* __OPUS_APP_H */


/* [] END OF FILE */
