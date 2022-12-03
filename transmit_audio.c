/******************************************************************************
* File Name:   transmit_audio.c
*
* Description: This files contains the task that sends the audio Encoded Data to
* the Host via  Bluetooth LE
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

#include "audio.h"
#include "stdio.h"
#include "app_bt_hid.h"
#include "app_hw_handler.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/

#define XMIT_TASK_STACK_SZ          (512u)
#define XMIT_TASK_PRIORITY          (4)
#define XMIT_MSG_QUEUE_SIZE         (5)

//#define DROP_ENC_PKTS_ON_CONG


/*******************************************************************************
*        Variable Definitions
*******************************************************************************/
TaskHandle_t    xmit_task_h;
QueueHandle_t   xmit_q;

/* Message format of xmit msg */
typedef struct
{
    uint8_t cmd;
    uint8_t idx;
    uint16_t len;
}xmit_msg_q_pkt_t;

/* Message Types for xmit msg queue */
typedef enum
{
    AUDIO_ENC_DATA_READY,
    AUDIO_XMIT_STOP
}xmit_cmd_t;

#ifdef DROP_ENC_PKTS_ON_CONG
uint16_t  xmit_drop_pkt_cnt;
#endif

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/**
 * Function name:
 * send_msg_to_xmit_q
 *
 * Function Description:
 * @brief  Sends the index of the buffer from encoded buffer pool to the xmit queue to
 *         further processing.
 *
 * @param idx Index of the buffer
 * @param len Length of the Encoded packet
 *
 *
 * @return void
 */
void send_msg_to_xmit_q(uint8_t idx, uint16_t len)
{
    xmit_msg_q_pkt_t msg;
    msg.cmd = AUDIO_ENC_DATA_READY ;
    msg.idx = idx ;
    msg.len = len ;

#ifdef DROP_ENC_PKTS_ON_CONG
    if(uxQueueMessagesWaiting(xmit_q) == XMIT_MSG_QUEUE_SIZE)
    {
        xmit_drop_pkt_cnt++;
        return;
    }
#endif

    if(xQueueSend(xmit_q, &msg, TICKS_TO_WAIT) != pdPASS)
      {
          audio_streaming_stop();

          /* Flush the Queue when XMIT Queue gets full */
          xQueueReset(xmit_q);
          printf("failed to send message to xmit q\r\n");
      }

}

/**
 * Function Name:
 * send_stop_msg_to_xmit_q
 *
 * Function Description:
 * @brief  Sends the stop message to xmit queue to stop sending.
 *
 * @param void
 *
 * @return void
 */
void send_stop_msg_to_xmit_q(void)
{
    xmit_msg_q_pkt_t msg;
    msg.cmd = AUDIO_XMIT_STOP ;
    if(xQueueSend(xmit_q, &msg, TICKS_TO_WAIT) != pdPASS)
      {
          CY_ASSERT(0);
      }
}

/**
 * Function Name:
 * xmit_task
 *
 * Function Description:
 * @brief  Encoder Task Handler: This task function receives the encoded audio
 *         packet and sends the packet over air using defined transport
 *
 * @param arg Not used
 *
 * @return void
 */
static void xmit_task(void *arg)
{
    BaseType_t xResult = pdFAIL;
    uint16_t idx;
    uint8_t *p_encoded_buffer;

    xmit_msg_q_pkt_t msg;

    while(1)
    {
        /* Block until a command is received */
        xResult = xQueueReceive(xmit_q, &(msg), portMAX_DELAY);
        if(xResult != pdPASS)
        {
            continue;
        }

        switch(msg.cmd)
        {

        case AUDIO_ENC_DATA_READY :

            idx = msg.idx;
            p_encoded_buffer=enc_buff_pool[idx];

            /* Send Encoded Data for streaming*/
            app_send_audio_data(p_encoded_buffer);

            break;

        default:

            break;
        }

    }

}

/**
 * Function Name:
 * xmit_task_init
 *
 * Function Description:
 * @brief  This Function creates Encoder task and queue
 *
 * @param void
 *
 * @return void
 */
void xmit_task_init(void)
{
    /* Queue Create for Transmit Task */
    xmit_q = xQueueCreate(XMIT_MSG_QUEUE_SIZE, 4);

    if(NULL == xmit_q)
    {
        printf("Transmit Queue creation failed! \r\n");
        CY_ASSERT(0);
    }

    /* Task Create of Transmit */
    xTaskCreate(xmit_task, "Transmit Audio Task", XMIT_TASK_STACK_SZ, 0,
                XMIT_TASK_PRIORITY, &xmit_task_h);

    if(NULL == xmit_task_h)
    {
        printf("Transmit task creation failed! \r\n");
        CY_ASSERT(0);
    }

}


/* [] END OF FILE */
