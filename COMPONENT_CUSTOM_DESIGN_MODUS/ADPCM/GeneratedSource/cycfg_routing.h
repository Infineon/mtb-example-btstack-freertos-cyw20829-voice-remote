/*******************************************************************************
* File Name: cycfg_routing.h
*
* Description:
* Establishes all necessary connections between hardware elements.
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

#if !defined(CYCFG_ROUTING_H)
#define CYCFG_ROUTING_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "cycfg_notices.h"
void init_cycfg_routing(void);
#define init_cycfg_connectivity() init_cycfg_routing()
#define ioss_0_port_0_pin_0_HSIOM P0_0_PDM_PDM_CLK1
#define ioss_0_port_0_pin_1_HSIOM P0_1_PDM_PDM_DATA1
#define ioss_0_port_0_pin_4_HSIOM P0_4_KEYSCAN_KS_ROW0
#define ioss_0_port_0_pin_5_HSIOM P0_5_KEYSCAN_KS_ROW1
#define ioss_0_port_1_pin_5_HSIOM P1_5_KEYSCAN_KS_COL5
#define ioss_0_port_1_pin_6_HSIOM P1_6_KEYSCAN_KS_COL6
#define ioss_0_port_2_pin_0_HSIOM P2_0_SMIF_SPIHB_SELECT0
#define ioss_0_port_2_pin_1_HSIOM P2_1_SMIF_SPIHB_DATA3
#define ioss_0_port_2_pin_2_HSIOM P2_2_SMIF_SPIHB_DATA2
#define ioss_0_port_2_pin_3_HSIOM P2_3_SMIF_SPIHB_DATA1
#define ioss_0_port_2_pin_4_HSIOM P2_4_SMIF_SPIHB_DATA0
#define ioss_0_port_2_pin_5_HSIOM P2_5_SMIF_SPIHB_CLK
#define ioss_0_port_3_pin_4_HSIOM P3_4_KEYSCAN_KS_COL7
#define ioss_0_port_3_pin_5_HSIOM P3_5_KEYSCAN_KS_COL8
#define ioss_0_port_3_pin_6_HSIOM P3_6_KEYSCAN_KS_COL9
#define ioss_0_port_4_pin_0_HSIOM P4_0_KEYSCAN_KS_ROW2
#define ioss_0_port_5_pin_2_HSIOM P5_2_KEYSCAN_KS_COL2

#define CYBSP_DMA_PDM_tr_in_0_TRIGGER_OUT TRIG_OUT_1TO1_1_PDM0_RX_TO_PDMA0_TR_IN13
#define CYBSP_PDM_tr_rx_req_1_TRIGGER_IN TRIG_IN_1TO1_1_PDM0_RX_TO_PDMA0_TR_IN13

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_ROUTING_H */
