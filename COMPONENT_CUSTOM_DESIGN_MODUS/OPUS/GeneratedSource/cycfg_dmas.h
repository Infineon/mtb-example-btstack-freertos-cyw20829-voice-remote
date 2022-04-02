/*******************************************************************************
* File Name: cycfg_dmas.h
*
* Description:
* DMA configuration
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

#if !defined(CYCFG_DMAS_H)
#define CYCFG_DMAS_H

#include "cycfg_notices.h"
#include "cy_dma.h"
#if defined (CY_USING_HAL)
    #include "cyhal_hwmgr.h"
#endif //defined (CY_USING_HAL)

#if defined(__cplusplus)
extern "C" {
#endif

#define CYBSP_DMA_PDM_ENABLED 1U
#define CYBSP_DMA_PDM_HW DW0
#define CYBSP_DMA_PDM_CHANNEL 13U
#define CYBSP_DMA_PDM_IRQ cpuss_interrupts_dw0_13_IRQn

extern const cy_stc_dma_descriptor_config_t CYBSP_DMA_PDM_Descriptor_0_config;
extern const cy_stc_dma_descriptor_config_t CYBSP_DMA_PDM_Descriptor_1_config;
extern const cy_stc_dma_descriptor_config_t CYBSP_DMA_PDM_Descriptor_2_config;
extern cy_stc_dma_descriptor_t CYBSP_DMA_PDM_Descriptor_0;
extern cy_stc_dma_descriptor_t CYBSP_DMA_PDM_Descriptor_1;
extern cy_stc_dma_descriptor_t CYBSP_DMA_PDM_Descriptor_2;
extern const cy_stc_dma_channel_config_t CYBSP_DMA_PDM_channelConfig;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t CYBSP_DMA_PDM_obj;
#endif //defined (CY_USING_HAL)

void init_cycfg_dmas(void);

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_DMAS_H */
