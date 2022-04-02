/*******************************************************************************
* File Name: cycfg_peripherals.h
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

#if !defined(CYCFG_PERIPHERALS_H)
#define CYCFG_PERIPHERALS_H

#include "cycfg_notices.h"
#include "cy_adcmic.h"
#if defined (CY_USING_HAL)
    #include "cyhal_hwmgr.h"
#endif //defined (CY_USING_HAL)
#include "cy_keyscan.h"
#include "cy_pdm_pcm_v2.h"
#include "cy_sysclk.h"
#include "cy_smif.h"
#include "cycfg_qspi_memslot.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define adcmic_0_ENABLED 1U
#define adcmic_0_HW MXS40ADCMIC0
#define adcmic_0_IRQ adcmic_interrupt_adcmic_IRQn
#define adcmic_0_FIFO_DATA_REG_PTR CY_ADCMIC_FIFO_DATA_REG_PTR(MXS40ADCMIC0)
#define adcmic_0_TRIGGER_CLR_REG_PTR CY_ADCMIC_TRIGGER_CLR_REG_PTR(MXS40ADCMIC0)
#define keyscan_0_ENABLED 1U
#define keyscan_0_HW MXKEYSCAN
#define keyscan_0_IRQ keyscan_interrupt_IRQn
#define CYBSP_PDM_ENABLED 1U
#define CYBSP_PDM_HW PDM0
#define CYBSP_PDM_CHANNEL_1_IRQ pdm_0_interrupts_1_IRQn
#define smif_0_ENABLED 1U
#define smif_0_HW SMIF0
#define smif_0_IRQ smif_interrupt_normal_IRQn
#define smif_0_MEMORY_MODE_ALIGMENT_ERROR (0UL)
#define smif_0_RX_DATA_FIFO_UNDERFLOW (0UL)
#define smif_0_TX_COMMAND_FIFO_OVERFLOW (0UL)
#define smif_0_TX_DATA_FIFO_OVERFLOW (0UL)
#define smif_0_RX_FIFO_TRIGEER_LEVEL (0UL)
#define smif_0_TX_FIFO_TRIGEER_LEVEL (0UL)
#define smif_0_DATALINES0_1 (1UL)
#define smif_0_DATALINES2_3 (1UL)
#define smif_0_DATALINES4_5 (0UL)
#define smif_0_DATALINES6_7 (0UL)
#define smif_0_SS0 (1UL)
#define smif_0_SS1 (0UL)
#define smif_0_SS2 (0UL)
#define smif_0_SS3 (0UL)
#define smif_0_DESELECT_DELAY 7

extern const cy_stc_adcmic_dc_path_config_t adcmic_0_dc_path_config;
extern const cy_stc_adcmic_timer_trigger_config_t adcmic_0_timer_trigger_config;
extern const cy_stc_adcmic_config_t adcmic_0_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t adcmic_0_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_ks_config_t keyscan_0_config;
extern const cy_stc_pdm_pcm_config_v2_t CYBSP_PDM_config;
extern const cy_stc_pdm_pcm_channel_config_t channel_1_config;
extern const cy_stc_smif_config_t smif_0_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t smif_0_obj;
#endif //defined (CY_USING_HAL)

void init_cycfg_peripherals(void);

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_PERIPHERALS_H */
