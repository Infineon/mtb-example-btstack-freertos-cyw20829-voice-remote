/*******************************************************************************
* File Name: cycfg_pins.c
*
* Description:
* Pin configuration
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

#include "cycfg_pins.h"

const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_0_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = ioss_0_port_0_pin_0_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_0_pin_0_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_0_pin_0_PORT_NUM,
        .channel_num = ioss_0_port_0_pin_0_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_1_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom = ioss_0_port_0_pin_1_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_0_pin_1_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_0_pin_1_PORT_NUM,
        .channel_num = ioss_0_port_0_pin_1_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_4_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_PULLUP,
    .hsiom = ioss_0_port_0_pin_4_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_0_pin_4_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_0_pin_4_PORT_NUM,
        .channel_num = ioss_0_port_0_pin_4_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_5_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_PULLUP,
    .hsiom = ioss_0_port_0_pin_5_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_0_pin_5_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_0_pin_5_PORT_NUM,
        .channel_num = ioss_0_port_0_pin_5_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_1_pin_5_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = ioss_0_port_1_pin_5_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_1_pin_5_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_1_pin_5_PORT_NUM,
        .channel_num = ioss_0_port_1_pin_5_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_1_pin_6_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = ioss_0_port_1_pin_6_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_1_pin_6_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_1_pin_6_PORT_NUM,
        .channel_num = ioss_0_port_1_pin_6_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_0_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = ioss_0_port_2_pin_0_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_2_pin_0_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_2_pin_0_PORT_NUM,
        .channel_num = ioss_0_port_2_pin_0_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_1_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom = ioss_0_port_2_pin_1_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_2_pin_1_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_2_pin_1_PORT_NUM,
        .channel_num = ioss_0_port_2_pin_1_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_2_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom = ioss_0_port_2_pin_2_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_2_pin_2_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_2_pin_2_PORT_NUM,
        .channel_num = ioss_0_port_2_pin_2_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_3_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom = ioss_0_port_2_pin_3_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_2_pin_3_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_2_pin_3_PORT_NUM,
        .channel_num = ioss_0_port_2_pin_3_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_4_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom = ioss_0_port_2_pin_4_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_2_pin_4_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_2_pin_4_PORT_NUM,
        .channel_num = ioss_0_port_2_pin_4_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_5_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom = ioss_0_port_2_pin_5_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_2_pin_5_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_2_pin_5_PORT_NUM,
        .channel_num = ioss_0_port_2_pin_5_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_3_pin_4_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = ioss_0_port_3_pin_4_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_3_pin_4_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_3_pin_4_PORT_NUM,
        .channel_num = ioss_0_port_3_pin_4_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_3_pin_5_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = ioss_0_port_3_pin_5_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_3_pin_5_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_3_pin_5_PORT_NUM,
        .channel_num = ioss_0_port_3_pin_5_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_3_pin_6_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = ioss_0_port_3_pin_6_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_3_pin_6_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_3_pin_6_PORT_NUM,
        .channel_num = ioss_0_port_3_pin_6_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_4_pin_0_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_PULLUP,
    .hsiom = ioss_0_port_4_pin_0_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_4_pin_0_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_4_pin_0_PORT_NUM,
        .channel_num = ioss_0_port_4_pin_0_PIN,
    };
#endif //defined (CY_USING_HAL)
const cy_stc_gpio_pin_config_t ioss_0_port_5_pin_2_config = 
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = ioss_0_port_5_pin_2_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
    .nonSec = 1UL,
};
#if defined (CY_USING_HAL)
    const cyhal_resource_inst_t ioss_0_port_5_pin_2_obj = 
    {
        .type = CYHAL_RSC_GPIO,
        .block_num = ioss_0_port_5_pin_2_PORT_NUM,
        .channel_num = ioss_0_port_5_pin_2_PIN,
    };
#endif //defined (CY_USING_HAL)


void init_cycfg_pins(void)
{
    Cy_GPIO_Pin_Init(ioss_0_port_0_pin_0_PORT, ioss_0_port_0_pin_0_PIN, &ioss_0_port_0_pin_0_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_0_pin_0_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_0_pin_1_PORT, ioss_0_port_0_pin_1_PIN, &ioss_0_port_0_pin_1_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_0_pin_1_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_0_pin_4_PORT, ioss_0_port_0_pin_4_PIN, &ioss_0_port_0_pin_4_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_0_pin_4_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_0_pin_5_PORT, ioss_0_port_0_pin_5_PIN, &ioss_0_port_0_pin_5_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_0_pin_5_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_1_pin_5_PORT, ioss_0_port_1_pin_5_PIN, &ioss_0_port_1_pin_5_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_1_pin_5_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_1_pin_6_PORT, ioss_0_port_1_pin_6_PIN, &ioss_0_port_1_pin_6_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_1_pin_6_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_2_pin_0_PORT, ioss_0_port_2_pin_0_PIN, &ioss_0_port_2_pin_0_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_2_pin_0_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_2_pin_1_PORT, ioss_0_port_2_pin_1_PIN, &ioss_0_port_2_pin_1_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_2_pin_1_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_2_pin_2_PORT, ioss_0_port_2_pin_2_PIN, &ioss_0_port_2_pin_2_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_2_pin_2_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_2_pin_3_PORT, ioss_0_port_2_pin_3_PIN, &ioss_0_port_2_pin_3_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_2_pin_3_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_2_pin_4_PORT, ioss_0_port_2_pin_4_PIN, &ioss_0_port_2_pin_4_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_2_pin_4_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_2_pin_5_PORT, ioss_0_port_2_pin_5_PIN, &ioss_0_port_2_pin_5_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_2_pin_5_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_3_pin_4_PORT, ioss_0_port_3_pin_4_PIN, &ioss_0_port_3_pin_4_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_3_pin_4_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_3_pin_5_PORT, ioss_0_port_3_pin_5_PIN, &ioss_0_port_3_pin_5_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_3_pin_5_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_3_pin_6_PORT, ioss_0_port_3_pin_6_PIN, &ioss_0_port_3_pin_6_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_3_pin_6_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_4_pin_0_PORT, ioss_0_port_4_pin_0_PIN, &ioss_0_port_4_pin_0_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_4_pin_0_obj);
#endif //defined (CY_USING_HAL)

    Cy_GPIO_Pin_Init(ioss_0_port_5_pin_2_PORT, ioss_0_port_5_pin_2_PIN, &ioss_0_port_5_pin_2_config);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&ioss_0_port_5_pin_2_obj);
#endif //defined (CY_USING_HAL)
}
