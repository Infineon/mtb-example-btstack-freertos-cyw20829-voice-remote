/******************************************************************************
* File Name:   adc_mic.c
*
* Description: This files contains the function definition of Analog Mic
* interface and DMA channel configuration for the audio capture
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
#include "cy_adcmic.h"
#include "audio.h"
#include "app_hw_batmon.h"
#include "app_hw_handler.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
#define ADC_DMA_INTR_PRIORITY       (7)
#define CYBSP_DMA_ADC_HW            DW0
#define CYBSP_DMA_ADC_CHANNEL       0U
#define CYBSP_DMA_ADC_TRG_HW        DW0
#define CYBSP_DMA_ADC_TRG_CHANNEL   1U
/*******************************************************************************
*        Variable Definitions
*******************************************************************************/

/* ADC dma interrupt configuration parameters */
const cy_stc_sysint_t ADC_DMA_IRQ_cfg = {
    .intrSrc = (IRQn_Type)cpuss_interrupts_dw0_0_IRQn,
    .intrPriority = ADC_DMA_INTR_PRIORITY
};

static volatile int adc_dma_buf_cnt = 0;

/* Debug Variables */
static uint32_t adc_dma_dbg_intr_cnt;
static uint32_t adc_overflow_dbg_cnt;
static uint32_t pcm_buff_overflow_dbg_cnt;

/* Fifo Data Trigger Clear */
int trigger_clr = CY_ADCMIC_TRIG_DATA;

/* ADC DMA Descriptor 0 */
cy_stc_dma_descriptor_t CYBSP_DMA_ADC_Descriptor_0 =
{
    .ctl = 0UL,
    .src = 0UL,
    .dst = 0UL,
    .xCtl = 0UL,
    .yCtl = 0UL,
    .nextPtr = 0UL,
};

/* ADC DMA Descriptor 0 configuration */
const cy_stc_dma_descriptor_config_t CYBSP_DMA_ADC_Descriptor_0_config =
{
    .retrigger = CY_DMA_RETRIG_16CYC,
    .interruptType = CY_DMA_X_LOOP,
    .triggerOutType = CY_DMA_1ELEMENT,
    .channelState = CY_DMA_CHANNEL_ENABLED,
    .triggerInType = CY_DMA_1ELEMENT,
    .dataSize = CY_DMA_WORD,
    .srcTransferSize = CY_DMA_TRANSFER_SIZE_WORD,
    .dstTransferSize = CY_DMA_TRANSFER_SIZE_WORD,
    .descriptorType = CY_DMA_2D_TRANSFER,
    .srcAddress = NULL,
    .dstAddress = NULL,
    .srcXincrement = 0,
    .dstXincrement = 1,
    .xCount = AUDIO_FRAME_SIZE/2,
    .srcYincrement = 0,
    .dstYincrement =  AUDIO_FRAME_SIZE/2,
    .yCount = 2,
    .nextDescriptor = &CYBSP_DMA_ADC_Descriptor_0,
};

/* ADC DMA Channel configuration */
const cy_stc_dma_channel_config_t CYBSP_DMA_ADC_channelConfig =
{
    .descriptor = &CYBSP_DMA_ADC_Descriptor_0,
    .preemptable = false,
    .priority = 3,
    .enable = false,
    .bufferable = false,
};

/* ADC Trigger DMA Descriptor 0 */
cy_stc_dma_descriptor_t CYBSP_DMA_ADC_TRG_Descriptor_0 =
{
    .ctl = 0UL,
    .src = 0UL,
    .dst = 0UL,
    .xCtl = 0UL,
    .yCtl = 0UL,
    .nextPtr = 0UL,
};

/* ADC Trigger DMA Descriptor 0 configuration */
const cy_stc_dma_descriptor_config_t CYBSP_DMA_ADC_TRG_Descriptor_0_config =
{
    .retrigger = CY_DMA_RETRIG_16CYC,
    .interruptType = CY_DMA_DESCR,
    .triggerOutType = CY_DMA_1ELEMENT,
    .channelState = CY_DMA_CHANNEL_ENABLED,
    .triggerInType = CY_DMA_1ELEMENT,
    .dataSize = CY_DMA_WORD,
    .srcTransferSize = CY_DMA_TRANSFER_SIZE_WORD,
    .dstTransferSize = CY_DMA_TRANSFER_SIZE_WORD,
    .descriptorType = CY_DMA_1D_TRANSFER,
    .srcAddress = NULL,
    .dstAddress = NULL,
    .srcXincrement = 0,
    .dstXincrement = 0,
    .xCount = 1,
    .srcYincrement = 0,
    .dstYincrement = 0,
    .yCount = 1,
    .nextDescriptor = &CYBSP_DMA_ADC_TRG_Descriptor_0,
};

/* ADC Trigger DMA Channel configuration */
const cy_stc_dma_channel_config_t CYBSP_DMA_ADC_TRG_channelConfig =
{
    .descriptor = &CYBSP_DMA_ADC_TRG_Descriptor_0,
    .preemptable = false,
    .priority = 3,
    .enable = false,
    .bufferable = false,
};

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/**
 * Function Name:
 * adc_dma_intr_handler
 *
 * @brief  Analog mic DMA ISR handler. An audio frame(AUDIO_FRAME_SIZE samples) is received
 *         on this interrupt.Sends a message to encoder queue to encode the current pcm
 *         buffer (audio frame received) and sets up the DMA to copy next frame from
 *         Analog mic to free pcm buffer.
 *
 * @param void
 *
 * @return void
 */
CY_SECTION_RAMFUNC_BEGIN
static void adc_dma_intr_handler(void)
{
    uint8_t idx;
    /* Clear the PDM/PCM dma done interrupt */
    Cy_DMA_Channel_ClearInterrupt(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL);
    NVIC_ClearPendingIRQ(ADC_DMA_IRQ_cfg.intrSrc);

    if(Cy_ADCMic_GetFifoStatus(adcmic_0_HW) & CY_ADCMIC_FIFO_OVERFLOW)
    {
        adc_overflow_dbg_cnt++;
    }

    Cy_ADCMic_ClearTrigger(adcmic_0_HW, CY_ADCMIC_TRIG_DATA);

    if(pcm_buff_state[adc_dma_buf_cnt])
    {
        pcm_buff_overflow_dbg_cnt++;
    }

    idx = adc_dma_buf_cnt;
    adc_dma_buf_cnt++;

    /* Check if PCM buffer is full, set it to start of PCM buffer */
    if(adc_dma_buf_cnt >= NO_AUDIO_DMA_BUF)
        adc_dma_buf_cnt = 0;

    send_msg_to_audio_q(idx);

     /* ADC mic dma interrupt count for debug */
    adc_dma_dbg_intr_cnt++;
}
CY_SECTION_RAMFUNC_END

/**
 * Function Name:
 * adcmic_dma_trg_configure
 *
 * @brief  It setups dma descriptor for ADC Data trigger, This is setup to Clear
 *         the trigger on every DMA element transfer for Analog Mic.
 *
 * @param void
 *
 * @return void
 */
static void adcmic_dma_trg_configure(void)
{
    cy_en_dma_status_t status;
    /* Initializes the DMA descriptor for ADC Trigger*/
    status = Cy_DMA_Descriptor_Init(&CYBSP_DMA_ADC_TRG_Descriptor_0, &CYBSP_DMA_ADC_TRG_Descriptor_0_config);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA descriptor for ADC trigger has failed! \r\n");
        CY_ASSERT(0);
    }

    /*Sets the Various parameters of Descriptor Like Y loop count, Src and Dest Addr */
    Cy_DMA_Descriptor_SetXloopDataCount(&CYBSP_DMA_ADC_TRG_Descriptor_0, AUDIO_FRAME_SIZE/2);
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_ADC_TRG_Descriptor_0, (void *) (&trigger_clr));
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_ADC_TRG_Descriptor_0, (void *) (CY_ADCMIC_TRIGGER_CLR_REG_PTR(adcmic_0_HW)));

    status = Cy_DMA_Channel_Init(CYBSP_DMA_ADC_TRG_HW, CYBSP_DMA_ADC_TRG_CHANNEL, &CYBSP_DMA_ADC_TRG_channelConfig);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA channel Initialization for ADC trigger has failed! \r\n");
        CY_ASSERT(0);
    }
    Cy_DMA_Enable(CYBSP_DMA_ADC_TRG_HW);
}

/**
 * Function Name:
 * adcmic_dma_configure
 *
 * @brief  It setups dma descriptor for ping pong buffer and initializes PDM
 *         DMA channel.
 * @param void
 *
 * @return void
 */
static void adcmic_dma_configure(void)
{
    cy_en_dma_status_t status;
    /* Initializes the DMA descriptor for 1st buffer*/
    status = Cy_DMA_Descriptor_Init(&CYBSP_DMA_ADC_Descriptor_0, &CYBSP_DMA_ADC_Descriptor_0_config);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA descriptor for 1st buffer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    /*Sets the Various parameters of Descriptor Like Y loop count, Src and Dest Addr */
    Cy_DMA_Descriptor_SetXloopDataCount(&CYBSP_DMA_ADC_Descriptor_0, AUDIO_FRAME_SIZE/2);
    Cy_DMA_Descriptor_SetYloopDstIncrement(&CYBSP_DMA_ADC_Descriptor_0, AUDIO_FRAME_SIZE/2);
    Cy_DMA_Descriptor_SetYloopDataCount(&CYBSP_DMA_ADC_Descriptor_0, 2);
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_ADC_Descriptor_0, (void *) (CY_ADCMIC_FIFO_DATA_REG_PTR(adcmic_0_HW)));
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_ADC_Descriptor_0, (void *) pcm_buff[0]);

    /* Sets up descriptor and other parameters for DMA channel */
    status = Cy_DMA_Channel_Init(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL, &CYBSP_DMA_ADC_channelConfig);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA param Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
    Cy_DMA_Enable(CYBSP_DMA_ADC_HW);
}

/**
 * Function Name:
 * analog_mic_capture_start
 *
 * @brief  Start/Stops capture audio data from Analog mic.
 *
 * @param en
 *
 * @return void
 */
 void analog_mic_capture_start(uint8_t en)
{
    /* Starts ADC DMA to capture audio data and unmask irq for dma channel */
    if(en)
    {
        adc_dma_buf_cnt = 0;
        /* Disable DC monitoring and Init the ADCMic for Analog Mic */
        adc_dc_monitoring_enable(0);

        adcmic_dma_configure();

        /* DMA channel for ADC Trigger is configured  */
        adcmic_dma_trg_configure();

        if (CY_ADCMIC_SUCCESS != Cy_ADCMic_Init(adcmic_0_HW, &adcmic_0_config, CY_ADCMIC_MIC))
        {
            CY_ASSERT(0);
        }
#ifdef CONFIG_ADPCM_CODEC
        Cy_ADCMic_SetSampleRate(adcmic_0_HW, CY_ADCMIC_8KSPS);
#endif
        /* Set ADC BIQUAD filter settings to default */
        Cy_ADCMic_BiquadBypass(adcmic_0_HW, 0);

        Cy_DMA_Channel_SetInterruptMask(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL, en);

        /* Clear the ADC DATA Trigger  */
        Cy_ADCMic_ClearTrigger(adcmic_0_HW, CY_ADCMIC_TRIG_DATA);
        Cy_ADCMic_Enable (adcmic_0_HW);
        Cy_DMA_Channel_Enable(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL);
        Cy_DMA_Channel_Enable(CYBSP_DMA_ADC_TRG_HW, CYBSP_DMA_ADC_TRG_CHANNEL);

    }
    /* Stops ADC DMA to capture audio data and mask irq for DMA channel */
    else
    {
        Cy_DMA_Channel_SetInterruptMask(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL, en);
        Cy_DMA_Channel_Disable(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL);
        Cy_DMA_Channel_Disable(CYBSP_DMA_ADC_TRG_HW, CYBSP_DMA_ADC_TRG_CHANNEL);

        /* Clear the ADC DATA Trigger  */
        Cy_ADCMic_ClearTrigger(adcmic_0_HW, CY_ADCMIC_TRIG_DATA);
        adcmic_0_HW->ADCMIC_FIFO_CTRL &= 0x3FFF;
        Cy_ADCMic_Disable(adcmic_0_HW);

        /* Enable DC monitoring after Streaming is finished */
        adc_dc_monitoring_enable(1);
    }
}

/**
 * Function Name:
 * analog_mic_interface_init
 *
 * @brief  Initializes the PDM/PCM block
 *
 * @param void
 *
 * @return void
 */
void analog_mic_interface_init(void)
{
    /*TriggerMux Connections for ADC Mic DMA and ADC Trgigger DMA */
    Cy_TrigMux_Connect(TRIG_IN_MUX_0_ADCMIC_DATA_AVAIL, TRIG_OUT_MUX_0_PDMA0_TR_IN0, false, TRIGGER_TYPE_EDGE);
    Cy_TrigMux_Connect(TRIG_IN_MUX_0_PDMA0_TR_OUT0, TRIG_OUT_MUX_0_PDMA0_TR_IN1, false, TRIGGER_TYPE_EDGE);

   /* DMA channel for Analog Mic is configured  */
    adcmic_dma_configure();

    /* DMA channel for ADC Trigger is configured  */
    adcmic_dma_trg_configure();

    /* Register the interrupt handler of PDM DMA Done Irq */
    if(CY_SYSINT_SUCCESS != Cy_SysInt_Init(&ADC_DMA_IRQ_cfg, &adc_dma_intr_handler))
    {
        printf("Analog MIC DMA IRQ Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    NVIC_ClearPendingIRQ(ADC_DMA_IRQ_cfg.intrSrc);
    NVIC_EnableIRQ(ADC_DMA_IRQ_cfg.intrSrc);
}


/* [] END OF FILE */
