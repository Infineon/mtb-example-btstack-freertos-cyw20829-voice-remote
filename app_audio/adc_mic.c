/******************************************************************************
* File Name:   adc_mic.c
*
* Description: This files contains the function definition of Analog Mic
* interface and DMA channel configuration for the audio capture
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
#include "audio.h"
#include "app_hw_batmon.h"

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

/* Fifo Data Trigger Clear */
int trigger_clr = CY_ADCMIC_TRIG_DATA;

/* ADC DMA Descriptor 0 configuration */
const cy_stc_dma_descriptor_config_t CYBSP_DMA_ADC_Descriptor_0_config = 
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
    .dstXincrement = 1,
    .xCount = AUDIO_FRAME_SIZE/2,
    .srcYincrement = 0,
    .dstYincrement =  AUDIO_FRAME_SIZE/2,
    .yCount = 1,
    .nextDescriptor = NULL,
};

/* ADC DMA Descriptor 1 configuration */
const cy_stc_dma_descriptor_config_t CYBSP_DMA_ADC_Descriptor_1_config = 
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
    .dstXincrement = 1,
    .xCount =  AUDIO_FRAME_SIZE/2,
    .srcYincrement = 0,
    .dstYincrement =  AUDIO_FRAME_SIZE/2,
    .yCount = 1,
    .nextDescriptor = NULL,
};

/* ADC DMA Descriptor 2 configuration */
const cy_stc_dma_descriptor_config_t CYBSP_DMA_ADC_Descriptor_2_config = 
{
    .retrigger = CY_DMA_RETRIG_16CYC,
    .interruptType = CY_DMA_1ELEMENT,
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
    .dstXincrement = 1,
    .xCount =  AUDIO_FRAME_SIZE/2,
    .srcYincrement = 0,
    .dstYincrement =  AUDIO_FRAME_SIZE/2,
    .yCount = 1,
    .nextDescriptor = NULL,
};

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

/* ADC DMA Descriptor 1 */
cy_stc_dma_descriptor_t CYBSP_DMA_ADC_Descriptor_1 = 
{
    .ctl = 0UL,
    .src = 0UL,
    .dst = 0UL,
    .xCtl = 0UL,
    .yCtl = 0UL,
    .nextPtr = 0UL,
};

/* ADC DMA Descriptor 2 */
cy_stc_dma_descriptor_t CYBSP_DMA_ADC_Descriptor_2 = 
{

    .ctl = 0UL,
    .src = 0UL,
    .dst = 0UL,
    .xCtl = 0UL,
    .yCtl = 0UL,
    .nextPtr = 0UL,
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

/* ADC Analog Mic path configuration */
cy_stc_adcmic_audio_analog_path_config_t adcmic_analog_mic_config =
{
    .micBias = CY_ADCMIC_BIAS_1_14_REF,
    .micBiasLz = false,
    .micPd=false,
    .micClamp=false,
    .pgaGain = CY_ADCMIC_PGA_GAIN_20, /* 20X */
    .pgaInCm = CY_ADCMIC_INCM_0_4,
    .pgaOutCm = CY_ADCMIC_OUTCM_0_6
};

/* ADC Analog Mic fifo configuration */
cy_stc_adcmic_fifo_config_t adcmic_fifo_config =
{
    .full = 40,
    .empty = 10
};

/* ADC Analog Mic fifo trigger configuration */
cy_stc_adcmic_timer_trigger_config_t adcmic_fifo_trigger_config =
{
    .timerTrigger = false,
    .fifoTrigger = true,
    .period = 64000UL,
    .input = CY_ADCMIC_TIMER_COUNT_INPUT_CLK_SYS
};

/* ADC Analog Mic configuration */
cy_stc_adcmic_config_t adcmic_config =
{
    .clockDiv=2,
    .source = CY_ADCMIC_MIC,
#ifdef CONFIG_ADPCM_CODEC
    .sampleRate = CY_ADCMIC_8KSPS,
#else
    .sampleRate = CY_ADCMIC_16KSPS,
#endif
    .anaConfig = &adcmic_analog_mic_config,
    .digConfig = NULL,
    .dcConfig = NULL,
    .biQuadConfig = NULL,
    .fifoConfig = &adcmic_fifo_config,
    .tmrTrgConfig = &adcmic_fifo_trigger_config
    };

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*******************************************************************************
* Function Name: adc_dma_intr_handler
********************************************************************************
* Summary:
*  Analog mic DMA ISR handler. An audio frame(AUDIO_FRAME_SIZE samples) is received
*  on this interrupt.Sends a message to encoder queue to encode the current pcm
*  buffer (audio frame received) and sets up the DMA to copy next frame from
*  Analog mic to free pcm buffer.
*
*******************************************************************************/
static void adc_dma_intr_handler(void)
{
    uint8_t idx;
    /* Clear the PDM/PCM dma done interrupt */
    Cy_DMA_Channel_ClearInterrupt(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL);
    NVIC_ClearPendingIRQ(ADC_DMA_IRQ_cfg.intrSrc);

    Cy_ADCMic_ClearTrigger(adcmic_0_HW, CY_ADCMIC_TRIG_DATA);
    /* Check if the next buffer is free, setup DMA for Junk buffer if not free */
    if(pcm_buff_state[adc_dma_buf_cnt])
    {
        Cy_DMA_Channel_SetDescriptor(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL, &CYBSP_DMA_ADC_Descriptor_2);
        Cy_DMA_Channel_Enable(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL);
        adc_overflow_dbg_cnt++;
        return;
    }

    idx = adc_dma_buf_cnt;
    adc_dma_buf_cnt++;

    /* Check if PCM buffer is full, set it to start of PCM buffer */
    if(adc_dma_buf_cnt >= NO_AUDIO_DMA_BUF)
        adc_dma_buf_cnt = 0;

    send_msg_to_audio_q(idx);

    /* Sets up the Descriptor for next free buffer and enable the DMA Channel*/
    if(idx)
    {
        Cy_DMA_Channel_SetDescriptor(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL, &CYBSP_DMA_ADC_Descriptor_0);
    }
    else
    {
        Cy_DMA_Channel_SetDescriptor(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL, &CYBSP_DMA_ADC_Descriptor_1);
    }
   Cy_DMA_Channel_Enable(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL);
   Cy_DMA_Channel_Enable(CYBSP_DMA_ADC_TRG_HW, CYBSP_DMA_ADC_TRG_CHANNEL);

    /* ADC mic dma interrupt count for debug */
    adc_dma_dbg_intr_cnt++;
}


/*******************************************************************************
* Function Name: adcmic_dma_trg_configure
********************************************************************************
* Summary:
*  It setups dma descriptor for ADC Data trigger, This is setup to Clear the
*  trigger on every DMA element transfer for Analog Mic
*
*******************************************************************************/
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

/*******************************************************************************
* Function Name: adcmic_dma_configure
********************************************************************************
* Summary:
*  It setups dma descriptor for ping pong buffer and initializes PDM DMA channel
*
*******************************************************************************/
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
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_ADC_Descriptor_0, (void *) (CY_ADCMIC_FIFO_DATA_REG_PTR(adcmic_0_HW)));
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_ADC_Descriptor_0, (void *) pcm_buff[0]);

    /* Initializes the DMA descriptor for 2nd buffer*/
    status = Cy_DMA_Descriptor_Init(&CYBSP_DMA_ADC_Descriptor_1, &CYBSP_DMA_ADC_Descriptor_1_config);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA descriptor for 2nd buffer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    Cy_DMA_Descriptor_SetXloopDataCount(&CYBSP_DMA_ADC_Descriptor_1, AUDIO_FRAME_SIZE/2);
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_ADC_Descriptor_1, (void *) (CY_ADCMIC_FIFO_DATA_REG_PTR(adcmic_0_HW)));
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_ADC_Descriptor_1, (void *) pcm_buff[1]);

    /* Initializes the DMA descriptor for Garbage buffer*/
    status = Cy_DMA_Descriptor_Init(&CYBSP_DMA_ADC_Descriptor_2, &CYBSP_DMA_ADC_Descriptor_2_config);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA descriptor for garbage buffer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    Cy_DMA_Descriptor_SetXloopDataCount(&CYBSP_DMA_ADC_Descriptor_2, AUDIO_FRAME_SIZE/4);
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_ADC_Descriptor_2, (void *) (CY_ADCMIC_FIFO_DATA_REG_PTR(adcmic_0_HW)));
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_ADC_Descriptor_2, (void *) pdm_pcm_garbage_buff);

    /* Sets up descriptor and other parameters for DMA channel */
    status = Cy_DMA_Channel_Init(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL, &CYBSP_DMA_ADC_channelConfig);
    if (CY_DMA_SUCCESS != status)
    {
        printf("DMA param Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
    Cy_DMA_Enable(CYBSP_DMA_ADC_HW);

    /* Register the interrupt handler of PDM DMA Done Irq */
    if(CY_SYSINT_SUCCESS != Cy_SysInt_Init(&ADC_DMA_IRQ_cfg, &adc_dma_intr_handler))
    {
        printf("PDM DMA IRQ Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    NVIC_ClearPendingIRQ(ADC_DMA_IRQ_cfg.intrSrc);
    NVIC_EnableIRQ(ADC_DMA_IRQ_cfg.intrSrc);
}

/*******************************************************************************
* Function Name: analog_mic_capture_start
********************************************************************************
* Summary:
*  Start/Stops capture audio data from Analog mic
*
*******************************************************************************/
 void analog_mic_capture_start(uint8_t en)
{
    /* Starts ADC DMA to capture audio data and unmask irq for dma channel */
    if(en)
    {
        adc_dma_buf_cnt = 0;
        /* Disable DC monitoring and Init the ADCMic for Analog Mic */
        adc_dc_monitoring_enable(0);
        if (CY_ADCMIC_SUCCESS != Cy_ADCMic_Init(adcmic_0_HW, &adcmic_config))
        {
            CY_ASSERT(0);
        }

        /* Set ADC BIQUAD filter settings to default */
        Cy_ADCMic_BiquadBypass(adcmic_0_HW, 0);

        Cy_DMA_Channel_SetInterruptMask(CYBSP_DMA_ADC_HW, CYBSP_DMA_ADC_CHANNEL, en);

        /* Clear the ADC DATA Trigger  */
        Cy_ADCMic_ClearTrigger(adcmic_0_HW, CY_ADCMIC_TRIG_DATA);
        Cy_ADCMic_WakeUpMic(adcmic_0_HW);
        Cy_ADCMic_Enable (adcmic_0_HW);
        Cy_ADCMic_StartConvert(adcmic_0_HW);
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
        Cy_ADCMic_SleepMic(adcmic_0_HW);
        Cy_ADCMic_StopConvert (adcmic_0_HW);
        Cy_ADCMic_Disable(adcmic_0_HW);

        /* Enable DC monitoring after Streaming is finished */
        adc_dc_monitoring_enable(1);
    }
}

/*******************************************************************************
* Function Name: analog_mic_interface_init
********************************************************************************
* Summary:
*  Initializes the PDM/PCM block
*
*******************************************************************************/
void analog_mic_interface_init(void)
{
    /*TriggerMux Connections for ADC Mic DMA and ADC Trgigger DMA */
    Cy_TrigMux_Connect(TRIG_IN_MUX_0_ADCMIC_DATA_AVAIL, TRIG_OUT_MUX_0_PDMA0_TR_IN0, false, TRIGGER_TYPE_EDGE);
    Cy_TrigMux_Connect(TRIG_IN_MUX_0_PDMA0_TR_OUT0, TRIG_OUT_MUX_0_PDMA0_TR_IN1, false, TRIGGER_TYPE_EDGE);

   /* DMA channel for Analog Mic is configured  */
    adcmic_dma_configure();

    /* DMA channel for ADC Trigger is configured  */
    adcmic_dma_trg_configure();
}


/* [] END OF FILE */
