/******************************************************************************
* File Name:   pdm_mic.c
*
* Description: This files contains the function definition of PDM Mic interface
* and DMA channel configuration for the audio capture
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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "audio.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
#define PDM_DMA_INTR_PRIORITY       (7)


/*******************************************************************************
*        Variable Definitions
*******************************************************************************/

/* PDM/PCM dma interrupt configuration parameters */
const cy_stc_sysint_t PDM_DMA_IRQ_cfg = {
    .intrSrc = (IRQn_Type)CYBSP_DMA_PDM_IRQ,
    .intrPriority = PDM_DMA_INTR_PRIORITY
};

/* PDM/PCM interrupt configuration parameters */
const cy_stc_sysint_t PDM_IRQ_cfg = {
    .intrSrc = (IRQn_Type)CYBSP_PDM_CHANNEL_1_IRQ,
    .intrPriority = 7
};

static volatile int pdm_dma_buf_cnt = 0;

/* Debug Variables */
static uint32_t pdm_dbg_intr_cnt;
static uint32_t overflow_dbg_cnt;
/******************************************************************************
 *                          Function Definitions
 *****************************************************************************/

/**
 * Function Name:
 * pdm_overflow_intr_handler
 *
 * @brief  PDM Overflow ISR handler. Sets up the descriptor for junk buffer to
 *         consume data from the PDM fifo.
 *
 * @param void
 *
 * return void
 */
static void pdm_overflow_intr_handler(void)
{
    /* Write to the PDM interrupt register and read from it to clear overflow
       Interrupt */
    Cy_PDM_PCM_Channel_GetInterruptStatus(PDM0, 1);
    Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, 1, CY_PDM_PCM_INTR_MASK);

    /* Enable the DMA to junk the Overflown data from FIFO to garbage Buffer */
    Cy_DMA_Channel_SetDescriptor(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL, &CYBSP_DMA_PDM_Descriptor_2);
    Cy_DMA_Channel_Enable(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL);
    overflow_dbg_cnt++;
}

/**
 * Function Name:
 * pdm_dma_intr_handler
 *
 * @brief  PDM DMA ISR handler. An audio frame(320 samples) is received on this
 *         interrupt Sends a message to encoder queue to encode the current pcm
 *  buffer (audio frame received) and sets up the DMA to copy next frame from
 *  PDM mic to free pcm buffer.
 *
 * @param void
 *
 * @return void
 */
static void pdm_dma_intr_handler(void)
{
    uint8_t idx;
    /* Clear the PDM/PCM dma done interrupt */
    Cy_DMA_Channel_ClearInterrupt(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL);
    NVIC_ClearPendingIRQ(PDM_DMA_IRQ_cfg.intrSrc);

    /* Check if the next buffer is free, don't setup DMA for buffer if not free */
    if(pcm_buff_state[pdm_dma_buf_cnt])
    {
        return;
    }

    idx = pdm_dma_buf_cnt;
    pdm_dma_buf_cnt++;

    /* Check if PCM buffer is full, set it to start of PCM buffer */
    if(pdm_dma_buf_cnt >= NO_AUDIO_DMA_BUF)
        pdm_dma_buf_cnt = 0;

    send_msg_to_audio_q(idx);

    /* Sets up the Descriptor for next free buffer and enable the DMA Channel*/
    if(idx)
    {
        Cy_DMA_Channel_SetDescriptor(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL, &CYBSP_DMA_PDM_Descriptor_0);
    }
    else
    {
        Cy_DMA_Channel_SetDescriptor(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL, &CYBSP_DMA_PDM_Descriptor_1);
    }
    Cy_DMA_Channel_Enable(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL);

    /* PDM/PCM dma interrupt count for debug */
    pdm_dbg_intr_cnt++;
}

/**
 * Function Name:
 * configure_pdm_pcm_dma
 *
 * @brief  It setups dma descriptor for ping pong buffer and initializes PDM DMA
 *         channel.
 *
 * @param void
 *
 * @return void
 */
static void configure_pdm_pcm_dma(void)
{
    cy_en_dma_status_t status;
    /* Initializes the DMA descriptor for 1st buffer*/
    status = Cy_DMA_Descriptor_Init(&CYBSP_DMA_PDM_Descriptor_0, &CYBSP_DMA_PDM_Descriptor_0_config);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA descriptor for 1st buffer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    /*Sets the Various parameters of Descriptor Like Y loop count, Src and Dest Addr */
    Cy_DMA_Descriptor_SetXloopDataCount(&CYBSP_DMA_PDM_Descriptor_0, AUDIO_FRAME_SIZE/2);
    Cy_DMA_Descriptor_SetYloopDstIncrement(&CYBSP_DMA_PDM_Descriptor_0, AUDIO_FRAME_SIZE/2);
    Cy_DMA_Descriptor_SetYloopDataCount(&CYBSP_DMA_PDM_Descriptor_0, 2);
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_PDM_Descriptor_0, (void *) (&CYBSP_PDM_HW->CH[1].RX_FIFO_RD));
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_PDM_Descriptor_0, (void *) pcm_buff[0]);

    /* Initializes the DMA descriptor for 2nd buffer*/
    status = Cy_DMA_Descriptor_Init(&CYBSP_DMA_PDM_Descriptor_1, &CYBSP_DMA_PDM_Descriptor_1_config);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA descriptor for 2nd buffer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    Cy_DMA_Descriptor_SetXloopDataCount(&CYBSP_DMA_PDM_Descriptor_1, AUDIO_FRAME_SIZE/2);
    Cy_DMA_Descriptor_SetYloopDstIncrement(&CYBSP_DMA_PDM_Descriptor_1, AUDIO_FRAME_SIZE/2);
    Cy_DMA_Descriptor_SetYloopDataCount(&CYBSP_DMA_PDM_Descriptor_1, 2);
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_PDM_Descriptor_1, (void *) (&CYBSP_PDM_HW->CH[1].RX_FIFO_RD));
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_PDM_Descriptor_1, (void *) pcm_buff[1]);

    /* Initializes the DMA descriptor for Garbage buffer*/
    status = Cy_DMA_Descriptor_Init(&CYBSP_DMA_PDM_Descriptor_2, &CYBSP_DMA_PDM_Descriptor_2_config);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA descriptor for garbage buffer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
    //Cy_DMA_Descriptor_SetYloopDataCount(&CYBSP_DMA_PDM_Descriptor_2, 1);
    Cy_DMA_Descriptor_SetXloopDataCount(&CYBSP_DMA_PDM_Descriptor_2, AUDIO_FRAME_SIZE/2);
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_PDM_Descriptor_2, (void *) (&CYBSP_PDM_HW->CH[1].RX_FIFO_RD));
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_PDM_Descriptor_2, (void *) pdm_pcm_garbage_buff);

    /* Sets up descriptor and other parameters for DMA channel */
    status = Cy_DMA_Channel_Init(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL, &CYBSP_DMA_PDM_channelConfig);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA param Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
    Cy_DMA_Enable(CYBSP_DMA_PDM_HW);

    /* Register the interrupt handler of PDM DMA Done Irq */
    if(CY_SYSINT_SUCCESS != Cy_SysInt_Init(&PDM_DMA_IRQ_cfg, &pdm_dma_intr_handler))
    {
        printf("PDM DMA IRQ Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    NVIC_ClearPendingIRQ(PDM_DMA_IRQ_cfg.intrSrc);
    NVIC_EnableIRQ(PDM_DMA_IRQ_cfg.intrSrc);
}

/**
 * Function Name:
 * pdm_pcm_capture_start
 *
 * @brief Start/Stops PDM/PCM to capture audio data
 *
 * @param en
 *
 * @return void
 */
 void pdm_pcm_capture_start(uint8_t en)
{
    /* Starts PDM/PCM DMA to capture audio data and unmask irq for dma channel */
    if(en)
    {
        pdm_dma_buf_cnt = 0;
        Cy_DMA_Channel_SetInterruptMask(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL, en);
        Cy_PDM_PCM_Channel_SetInterruptMask(CYBSP_PDM_HW, 1, CY_PDM_PCM_INTR_RX_OVERFLOW);
        Cy_PDM_PCM_Channel_ClearInterrupt(CYBSP_PDM_HW, 1, CY_PDM_PCM_INTR_MASK);
        Cy_PDM_PCM_Activate_Channel(CYBSP_PDM_HW, 1);
        Cy_DMA_Channel_Enable(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL);
    }
    /* Stops PDM/PCM DMA to capture audio data and mask irq for DMA channel */
    else
    {
        Cy_DMA_Channel_SetInterruptMask(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL, en);
        Cy_PDM_PCM_Channel_SetInterruptMask(CYBSP_PDM_HW, 1, 0);
        Cy_PDM_PCM_DeActivate_Channel(CYBSP_PDM_HW, 1);
        Cy_DMA_Channel_Disable(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL);
    }
}

/**
 * Function Name:
 * pdm_pcm_init
 *
 * @brief Initializes the PDM/PCM block
 *
 * @param void
 *
 * @return void
 */
void pdm_pcm_init(void)
{

    Cy_PDM_PCM_Init(CYBSP_PDM_HW, &CYBSP_PDM_config);
    Cy_PDM_PCM_Channel_Enable(CYBSP_PDM_HW, 1);

    Cy_PDM_PCM_Channel_Init(CYBSP_PDM_HW, &channel_1_config, 1);
    Cy_PDM_PCM_Channel_ClearInterrupt(CYBSP_PDM_HW, 1, CY_PDM_PCM_INTR_MASK);

    Cy_PDM_PCM_Channel_SetInterruptMask(CYBSP_PDM_HW, 1, 0);

    /* Register the PDM/PCM hardware block IRQ handler */
    if(CY_SYSINT_SUCCESS != Cy_SysInt_Init(&PDM_IRQ_cfg, &pdm_overflow_intr_handler))
    {
        printf("PDM/PCM Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
    NVIC_ClearPendingIRQ(PDM_IRQ_cfg.intrSrc);
    NVIC_EnableIRQ(PDM_IRQ_cfg.intrSrc);

    /* DMA channel for PDM/PCM is configured  */
    configure_pdm_pcm_dma();

}


/* [] END OF FILE */
