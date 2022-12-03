/******************************************************************************
* File Name:   audio.c
*
* Description: This files contains the task that initializes the encoder
* configures the audio interface i.e. PDM/Analog and starts audio capture,
* encoding of audio data.
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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "audio.h"
#include "cy_encoder.h"
#ifdef PDM_MIC
#include "pdm_mic.h"
#else
#include "adc_mic.h"
#endif
#include "cyhal_syspm.h"
#include "app_hw_handler.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
#define AUDIO_CAPTURE_START         (1)
#define AUDIO_CAPTURE_STOP          (0)
#ifdef CONFIG_ADPCM_CODEC
#define AUDIO_TASK_STACK_SZ         (1024)
#else
#define AUDIO_TASK_STACK_SZ         (5*1024)
#endif
#define AUDIO_TASK_PRIORITY         (5)
#define AUDIO_QUEUE_SIZE            (2)


/*******************************************************************************
*        Variable Definitions
*******************************************************************************/

uint8_t enc_buff_pool[NO_OF_ENC_BUFF][ENC_PKT_SIZE];

/* Pcm buffer */
int16_t pcm_buff[NO_AUDIO_DMA_BUF][AUDIO_FRAME_SIZE];

int16_t pdm_pcm_garbage_buff[AUDIO_FRAME_SIZE];

/* State of the Buffer */
volatile uint16_t pcm_buff_state[NO_AUDIO_DMA_BUF];

#ifdef MEASURE_CYCLES

unsigned int cycle_cnt[NO_OF_ENC_BUFF];
unsigned int prev_cnt, total_cnt;
int cnt = 0;

#endif

static audio_state_t audio_cap_state = STREAM_OFF;
static QueueHandle_t audio_q;
static TaskHandle_t audio_task_h;


/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/**
 * Function Name:
 * send_msg_to_audio_q
 *
 * @brief  Sends the index(0/1) of the ping pong PCM buffer to the audio queue
 *         to process.
 *
 * @param idx Index of the ping pong PCM buffer
 *
 * @return void
 */
void send_msg_to_audio_q(uint8_t idx)
{
    audio_msg_q_pkt_t msg;
    BaseType_t xHigherPriorityTaskWoken;
    msg.cmd = AUDIO_DATA_ENCODE;
    msg.data = idx ;
    if(xQueueSendFromISR(audio_q, &msg, &xHigherPriorityTaskWoken) != pdPASS)
    {
        printf("Sending msg to audio queue failed! \r\n");
        CY_ASSERT(0);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * Function Name:
 * send_start_streaming_msg
 *
 * @brief Sends the message to audio queue to start the audio capture from the
 *        MIC.
 *
 * @param void
 *
 * @return void
 */
void send_start_streaming_msg(void)
{
    audio_msg_q_pkt_t msg;
    msg.cmd = AUDIO_CAP_START ;
    if(xQueueSend(audio_q, &msg, TICKS_TO_WAIT) != pdPASS)
    {
        printf("Sending msg to audio queue failed! \r\n");
        CY_ASSERT(0);
    }
}

/**
 * Function Name:
 * send_stop_streaming_msg
 *
 * @brief Sends the message to audio queue to stop the audio capture from the
 *        MIC.
 *
 * @param void
 *
 * @return void
 */
void send_stop_streaming_msg(void)
{
    audio_msg_q_pkt_t msg;
    msg.cmd = AUDIO_CAP_STOP ;
    if(xQueueSend(audio_q, &msg, TICKS_TO_WAIT) != pdPASS)
    {
        printf("Sending msg to audio queue failed! \r\n");
        CY_ASSERT(0);
    }
}

/**
 * Function Name:
 * audio_mic_interface_init
 *
 * @brief  This function initializes the audio mic interface.
 *
 * @param void
 *
 * @return void
 */
static void audio_mic_interface_init(void)
{
#if defined (PDM_MIC)
    pdm_pcm_init();
#else
    analog_mic_interface_init();
#endif

}

/**
 * Function Name:
 * audio_mic_cap_start
 *
 * @brief  This function starts the audio capture from the Analog/PDM Mic.
 *
 * @param void
 *
 * @return void
 */
static void audio_mic_cap_start(void)
{
    if(audio_cap_state == STREAM_ON)
        return;
    /* Don't allow Device to go to DS while audio streaming */
    cyhal_syspm_lock_deepsleep();

#if defined (PDM_MIC)
    pdm_pcm_capture_start(AUDIO_CAPTURE_START);
#else
    analog_mic_capture_start(AUDIO_CAPTURE_START);
#endif
    audio_cap_state = STREAM_ON;
}

/**
 * Function Name:
 * audio_mic_cap_stop
 *
 * @brief  This function stops the audio capture from the Analog/PDM Mic.
 *
 * @param void
 *
 * @return void
 */
void audio_mic_cap_stop(void)
{

    if(audio_cap_state == STREAM_OFF)
        return;
#if defined (PDM_MIC)
    pdm_pcm_capture_start(AUDIO_CAPTURE_STOP);
#else
    analog_mic_capture_start(AUDIO_CAPTURE_STOP);
#endif
    audio_cap_state = STREAM_OFF;

    /* Allow Device to go to DS after audio streaming is done */
    cyhal_syspm_unlock_deepsleep();
}

/**
 * Function Name:
 * audio_streaming_stop
 *
 * @brief  This function stops the audio stream.
 *
 * @param void
 *
 * @return void
 */
void audio_streaming_stop(void)
{
    audio_mic_cap_stop();
    cy_encoder_reset();
}

#ifdef USE_WAVE_FILE
void encode_wave_file()
{
    uint16_t  enc_buf_cnt = 0;

    int pkt_len;
    int16_t *p_pcm_buffer;
    uint8_t *p_encoded_buffer;

    for(enc_buf_cnt = 0; enc_buf_cnt < NO_OF_ENC_BUFF; enc_buf_cnt++)
    {
        p_pcm_buffer = (int16_t*)(&rawData[enc_buf_cnt*640]);

         /* Pointer to the free buffer in encoder pool */
        p_encoded_buffer = (uint8_t *)&enc_buff_pool[enc_buf_cnt];


        /* Encodes the PCM frame and returns encoded buffer length */
        pkt_len = cy_encode(p_pcm_buffer, p_encoded_buffer);

        /* check for size of encoded buffer */
         if(pkt_len <= 8)
         {
             printf("Encoded Buffer length differed! \r\n");
              CY_ASSERT(0);
         }

     }
}
#endif

/**
 * Function Name:
 * audio_task
 *
 * @brief  Encoder Task Handler: This task function initializes the encoder and
*          processes the audio data for encoding.
 *
 * @param arg  Not used
 *
 * @return void
 */
static void audio_task(void *arg)
{
    BaseType_t xResult = pdFAIL;
    uint16_t  enc_buf_cnt = 0;
    uint16_t  pkt_len = 0;
    uint8_t idx;
    int16_t *p_pcm_buffer;
    uint8_t *p_encoded_buffer;

    audio_msg_q_pkt_t msg;


#ifndef USE_WAVE_FILE
    /* Initializes PDM/PCM block and configures DMA channel for same */
    audio_mic_interface_init();
#endif

    /* Initializes the Encoder defined */
    cy_encoder_init();

    while(true)
    {
        /* Block until a command is received */
        xResult = xQueueReceive(audio_q, &(msg), portMAX_DELAY);
        if(xResult != pdPASS)
        {
            continue;
        }

        switch(msg.cmd)
        {
        case AUDIO_CAP_START :

#ifdef MEASURE_CYCLES
             cnt = 0;
             prev_cnt = 0;
             total_cnt = 0;
             DWT_CCNT = 0;
#endif

#ifndef USE_WAVE_FILE
             /* Start audio capture */
             audio_mic_cap_start();
#else
             encode_wave_file();
#endif

             break;

        case AUDIO_DATA_ENCODE :
            /* Buffer Index received by queue on Completion of PDM/PCM Dma done
             * Interrupt on every Audio Frame defined */
            idx = msg.data;

            if(enc_buf_cnt < NO_OF_ENC_BUFF)
            {
                /* Pointer to current pcm buffer */
                p_pcm_buffer = (int16_t *)&pcm_buff[idx];
                /* Pointer to the free buffer in encoder pool */
                p_encoded_buffer = (uint8_t *)&enc_buff_pool[enc_buf_cnt];
                /* Change the state of PCM Buffer to Encoded*/
                pcm_buff_state[idx] = STATE_ENCODING;
                /* Encodes the PCM frame and returns encoded buffer length */
                pkt_len = cy_encode(p_pcm_buffer, p_encoded_buffer);
                /* Change the state of PCM Buffer to free State*/
                pcm_buff_state[idx] = STATE_FREE;


                /* check for size of encoded buffer */
                if(pkt_len <= 8)
                {
                    printf("Encoded Buffer length differed! \r\n");
                    CY_ASSERT(0);
                }

                send_msg_to_xmit_q(enc_buf_cnt, pkt_len);
                enc_buf_cnt++;

                /* Check for Encoder Buffer Pool size, reset to start when full */
                if(NO_OF_ENC_BUFF == enc_buf_cnt)
                {
                    enc_buf_cnt = 0;
                }

            }
            break;

        case AUDIO_CAP_STOP :

            /* Stop audio capture */
            audio_mic_cap_stop();

            /* reset to start */
            enc_buf_cnt = 0;

            /* reset the encoder state */
            cy_encoder_reset();

            break;

        default:

            break;
        }

    }

}

/**
 * Function Name:
 * audio_task_init
 *
 * @brief  This Function creates Encoder task and queue.
 *
 * @param void
 *
 * @return void
 */
void audio_task_init(void)
{
    /* Queue Create for Encoder */
    audio_q = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(audio_msg_q_pkt_t));

    if(NULL == audio_q)
    {
        printf("Audio Queue creation Failed! \r\n");
        CY_ASSERT(0);
    }
    //vQueueAddToRegistry(audio_q,"Audio_Q");

    /* Task Create of Encoder */
    xTaskCreate(audio_task, "Audio Encoder Task", AUDIO_TASK_STACK_SZ, 0,
                AUDIO_TASK_PRIORITY, &audio_task_h);

    if(NULL == audio_task_h)
    {
        printf("Audio Encoder task creation Failed! \r\n");
        CY_ASSERT(0);
    }

}


/* [] END OF FILE */
