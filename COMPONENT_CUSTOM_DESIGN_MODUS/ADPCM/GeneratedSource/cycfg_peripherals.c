/*******************************************************************************
* File Name: cycfg_peripherals.c
*
* Description:
* Peripheral Hardware Block configuration
* This file was automatically generated and should not be modified.
* Tools Package 2.4.0.5972
* mtb-pdl-cat1 2.4.0.14662
* personalities 6.0.0.0
* udd 3.0.0.2024
*
********************************************************************************
* Copyright 2022 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/

#include "cycfg_peripherals.h"

const cy_stc_adcmic_dc_path_config_t adcmic_0_dc_path_config = 
{
    .range = CY_ADCMIC_DC_RANGE_3_6V,
    .input = CY_ADCMIC_VDDIO,
    .tmrLatch = false,
};
const cy_stc_adcmic_timer_trigger_config_t adcmic_0_timer_trigger_config = 
{
    .timerTrigger = false,
    .fifoTrigger = false,
    .period = 10000,
    .input = CY_ADCMIC_TIMER_COUNT_INPUT_CLK_SYS,
};
const cy_stc_adcmic_config_t adcmic_0_config = 
{
    .clockDiv = 2U,
    .source = CY_ADCMIC_DC,
    .sampleRate = CY_ADCMIC_480KSPS,
    .anaConfig = NULL,
    .digConfig = NULL,
    .dcConfig = &adcmic_0_dc_path_config,
    .biQuadConfig = NULL,
    .fifoConfig = NULL,
    .tmrTrgConfig = &adcmic_0_timer_trigger_config,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t adcmic_0_obj = 
    {
        .type = CYHAL_RSC_ADC,
        .block_num = 0,
        .channel_num = 0,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_ks_config_t keyscan_0_config = 
{
    .macroDownDebCnt = 3UL,
    .macroUpDebCnt = 3UL,
    .microDebCnt = 3UL,
    .noofRows = 3UL,
    .noofColumns = 10UL,
    .ghostEnable = true,
    .cpuWakeupEnable = true,
    .clkStayOn = true,
};
const cy_stc_pdm_pcm_config_v2_t CYBSP_PDM_config = 
{
    .clkDiv = 11,
    .clksel = CY_PDM_PCM_SEL_SRSS_CLOCK,
    .halverate = CY_PDM_PCM_RATE_FULL,
    .route = 0,
    .fir0_coeff_user_value = true,
    .fir1_coeff_user_value = true,
    .fir0_coeff = {{1, 26}, {57, 15}, {-139, -239}, {-14, 491}, {665, -110}, {-1425, -1650}, {636, 4820}, {8191, 8191} },
    .fir1_coeff = {{190, 258}, {-106, -87}, {16, 169}, {-35, -182}, {-5, 226}, {40, -261}, {-97, 300}, {173, -336}, {-276, 369}, {418, -398}, {-630, 422}, {986, -439}, {-1764, 450}, {5480, 8191} },
};
const cy_stc_pdm_pcm_channel_config_t channel_1_config = 
{
    .sampledelay = 2,
    .wordSize = CY_PDM_PCM_WSIZE_16_BIT,
    .signExtension = true,
    .rxFifoTriggerLevel = 10,
    .fir0_enable = true,
    .cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_16,
    .fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_2,
    .fir0_scale = 10,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2,
    .fir1_scale = 10,
    .dc_block_disable = false,
    .dc_block_code = CY_PDM_PCM_CHAN_DCBLOCK_CODE_2,
};
const cy_stc_smif_config_t smif_0_config = 
{
    .mode = (uint32_t)CY_SMIF_NORMAL,
    .deselectDelay = smif_0_DESELECT_DELAY,
    .rxClockSel = (uint32_t)CY_SMIF_SEL_INVERTED_FEEDBACK_CLK,
    .blockEvent = (uint32_t)CY_SMIF_BUS_ERROR,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t smif_0_obj = 
    {
        .type = CYHAL_RSC_SMIF,
        .block_num = 0U,
        .channel_num = 0U,
    };
#endif //defined (CY_USING_HAL)


void init_cycfg_peripherals(void)
{
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&adcmic_0_obj);
#endif //defined (CY_USING_HAL)

    Cy_SysClk_PeriPclkAssignDivider(PCLK_PDM0_CLK_IF_SRSS, CY_SYSCLK_DIV_16_5_BIT, 0U);

#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&smif_0_obj);
#endif //defined (CY_USING_HAL)
}
