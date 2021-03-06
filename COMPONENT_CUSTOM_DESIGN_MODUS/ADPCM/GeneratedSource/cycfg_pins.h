/*******************************************************************************
* File Name: cycfg_pins.h
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

#if !defined(CYCFG_PINS_H)
#define CYCFG_PINS_H

#include "cycfg_notices.h"
#include "cy_gpio.h"
#include "cycfg_routing.h"
#if defined (CY_USING_HAL)
    #include "cyhal_hwmgr.h"
#endif //defined (CY_USING_HAL)

#if defined(__cplusplus)
extern "C" {
#endif

#define ioss_0_port_0_pin_0_ENABLED 1U
#define ioss_0_port_0_pin_0_PORT GPIO_PRT0
#define ioss_0_port_0_pin_0_PORT_NUM 0U
#define ioss_0_port_0_pin_0_PIN 0U
#define ioss_0_port_0_pin_0_NUM 0U
#define ioss_0_port_0_pin_0_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define ioss_0_port_0_pin_0_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_0_HSIOM
    #define ioss_0_port_0_pin_0_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_0_pin_0_IRQ ioss_interrupts_gpio_0_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_0_HAL_PORT_PIN P0_0
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_0 P0_0
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_0_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_0_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_0_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_0_pin_1_ENABLED 1U
#define ioss_0_port_0_pin_1_PORT GPIO_PRT0
#define ioss_0_port_0_pin_1_PORT_NUM 0U
#define ioss_0_port_0_pin_1_PIN 1U
#define ioss_0_port_0_pin_1_NUM 1U
#define ioss_0_port_0_pin_1_DRIVEMODE CY_GPIO_DM_HIGHZ
#define ioss_0_port_0_pin_1_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_1_HSIOM
    #define ioss_0_port_0_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_0_pin_1_IRQ ioss_interrupts_gpio_0_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_1_HAL_PORT_PIN P0_1
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_1 P0_1
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_1_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_1_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_1_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_NONE
#endif //defined (CY_USING_HAL)
#define ioss_0_port_0_pin_4_ENABLED 1U
#define ioss_0_port_0_pin_4_PORT GPIO_PRT0
#define ioss_0_port_0_pin_4_PORT_NUM 0U
#define ioss_0_port_0_pin_4_PIN 4U
#define ioss_0_port_0_pin_4_NUM 4U
#define ioss_0_port_0_pin_4_DRIVEMODE CY_GPIO_DM_PULLUP
#define ioss_0_port_0_pin_4_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_4_HSIOM
    #define ioss_0_port_0_pin_4_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_0_pin_4_IRQ ioss_interrupts_gpio_0_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_4_HAL_PORT_PIN P0_4
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_4 P0_4
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_4_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_4_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_4_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_PULLUP
#endif //defined (CY_USING_HAL)
#define ioss_0_port_0_pin_5_ENABLED 1U
#define ioss_0_port_0_pin_5_PORT GPIO_PRT0
#define ioss_0_port_0_pin_5_PORT_NUM 0U
#define ioss_0_port_0_pin_5_PIN 5U
#define ioss_0_port_0_pin_5_NUM 5U
#define ioss_0_port_0_pin_5_DRIVEMODE CY_GPIO_DM_PULLUP
#define ioss_0_port_0_pin_5_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_5_HSIOM
    #define ioss_0_port_0_pin_5_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_0_pin_5_IRQ ioss_interrupts_gpio_0_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_5_HAL_PORT_PIN P0_5
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_5 P0_5
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_5_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_5_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_0_pin_5_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_PULLUP
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define CYBSP_SWO (P1_0)
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define CYBSP_SWDIO (P1_2)
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define CYBSP_SWDCK (P1_3)
#endif //defined (CY_USING_HAL)
#define ioss_0_port_1_pin_5_ENABLED 1U
#define ioss_0_port_1_pin_5_PORT GPIO_PRT1
#define ioss_0_port_1_pin_5_PORT_NUM 1U
#define ioss_0_port_1_pin_5_PIN 5U
#define ioss_0_port_1_pin_5_NUM 5U
#define ioss_0_port_1_pin_5_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define ioss_0_port_1_pin_5_INIT_DRIVESTATE 1
#ifndef ioss_0_port_1_pin_5_HSIOM
    #define ioss_0_port_1_pin_5_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_1_pin_5_IRQ ioss_interrupts_gpio_1_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_1_pin_5_HAL_PORT_PIN P1_5
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_1_pin_5 P1_5
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_1_pin_5_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_1_pin_5_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_1_pin_5_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_1_pin_6_ENABLED 1U
#define ioss_0_port_1_pin_6_PORT GPIO_PRT1
#define ioss_0_port_1_pin_6_PORT_NUM 1U
#define ioss_0_port_1_pin_6_PIN 6U
#define ioss_0_port_1_pin_6_NUM 6U
#define ioss_0_port_1_pin_6_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define ioss_0_port_1_pin_6_INIT_DRIVESTATE 1
#ifndef ioss_0_port_1_pin_6_HSIOM
    #define ioss_0_port_1_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_1_pin_6_IRQ ioss_interrupts_gpio_1_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_1_pin_6_HAL_PORT_PIN P1_6
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_1_pin_6 P1_6
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_1_pin_6_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_1_pin_6_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_1_pin_6_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_2_pin_0_ENABLED 1U
#define ioss_0_port_2_pin_0_PORT GPIO_PRT2
#define ioss_0_port_2_pin_0_PORT_NUM 2U
#define ioss_0_port_2_pin_0_PIN 0U
#define ioss_0_port_2_pin_0_NUM 0U
#define ioss_0_port_2_pin_0_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define ioss_0_port_2_pin_0_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_0_HSIOM
    #define ioss_0_port_2_pin_0_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_2_pin_0_IRQ ioss_interrupts_gpio_2_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_0_HAL_PORT_PIN P2_0
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_0 P2_0
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_0_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_0_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_0_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_2_pin_1_ENABLED 1U
#define ioss_0_port_2_pin_1_PORT GPIO_PRT2
#define ioss_0_port_2_pin_1_PORT_NUM 2U
#define ioss_0_port_2_pin_1_PIN 1U
#define ioss_0_port_2_pin_1_NUM 1U
#define ioss_0_port_2_pin_1_DRIVEMODE CY_GPIO_DM_STRONG
#define ioss_0_port_2_pin_1_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_1_HSIOM
    #define ioss_0_port_2_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_2_pin_1_IRQ ioss_interrupts_gpio_2_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_1_HAL_PORT_PIN P2_1
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_1 P2_1
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_1_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_1_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_1_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_2_pin_2_ENABLED 1U
#define ioss_0_port_2_pin_2_PORT GPIO_PRT2
#define ioss_0_port_2_pin_2_PORT_NUM 2U
#define ioss_0_port_2_pin_2_PIN 2U
#define ioss_0_port_2_pin_2_NUM 2U
#define ioss_0_port_2_pin_2_DRIVEMODE CY_GPIO_DM_STRONG
#define ioss_0_port_2_pin_2_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_2_HSIOM
    #define ioss_0_port_2_pin_2_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_2_pin_2_IRQ ioss_interrupts_gpio_2_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_2_HAL_PORT_PIN P2_2
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_2 P2_2
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_2_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_2_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_2_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_2_pin_3_ENABLED 1U
#define ioss_0_port_2_pin_3_PORT GPIO_PRT2
#define ioss_0_port_2_pin_3_PORT_NUM 2U
#define ioss_0_port_2_pin_3_PIN 3U
#define ioss_0_port_2_pin_3_NUM 3U
#define ioss_0_port_2_pin_3_DRIVEMODE CY_GPIO_DM_STRONG
#define ioss_0_port_2_pin_3_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_3_HSIOM
    #define ioss_0_port_2_pin_3_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_2_pin_3_IRQ ioss_interrupts_gpio_2_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_3_HAL_PORT_PIN P2_3
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_3 P2_3
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_3_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_3_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_3_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_2_pin_4_ENABLED 1U
#define ioss_0_port_2_pin_4_PORT GPIO_PRT2
#define ioss_0_port_2_pin_4_PORT_NUM 2U
#define ioss_0_port_2_pin_4_PIN 4U
#define ioss_0_port_2_pin_4_NUM 4U
#define ioss_0_port_2_pin_4_DRIVEMODE CY_GPIO_DM_STRONG
#define ioss_0_port_2_pin_4_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_4_HSIOM
    #define ioss_0_port_2_pin_4_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_2_pin_4_IRQ ioss_interrupts_gpio_2_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_4_HAL_PORT_PIN P2_4
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_4 P2_4
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_4_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_4_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_4_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_2_pin_5_ENABLED 1U
#define ioss_0_port_2_pin_5_PORT GPIO_PRT2
#define ioss_0_port_2_pin_5_PORT_NUM 2U
#define ioss_0_port_2_pin_5_PIN 5U
#define ioss_0_port_2_pin_5_NUM 5U
#define ioss_0_port_2_pin_5_DRIVEMODE CY_GPIO_DM_STRONG
#define ioss_0_port_2_pin_5_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_5_HSIOM
    #define ioss_0_port_2_pin_5_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_2_pin_5_IRQ ioss_interrupts_gpio_2_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_5_HAL_PORT_PIN P2_5
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_5 P2_5
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_5_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_5_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_2_pin_5_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define CYBSP_DEBUG_UART_CTS (P3_0)
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define CYBSP_DEBUG_UART_RTS (P3_1)
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define CYBSP_DEBUG_UART_RX (P3_2)
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define CYBSP_DEBUG_UART_TX (P3_3)
#endif //defined (CY_USING_HAL)
#define ioss_0_port_3_pin_4_ENABLED 1U
#define ioss_0_port_3_pin_4_PORT GPIO_PRT3
#define ioss_0_port_3_pin_4_PORT_NUM 3U
#define ioss_0_port_3_pin_4_PIN 4U
#define ioss_0_port_3_pin_4_NUM 4U
#define ioss_0_port_3_pin_4_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define ioss_0_port_3_pin_4_INIT_DRIVESTATE 1
#ifndef ioss_0_port_3_pin_4_HSIOM
    #define ioss_0_port_3_pin_4_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_3_pin_4_IRQ ioss_interrupts_gpio_3_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_4_HAL_PORT_PIN P3_4
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_4 P3_4
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_4_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_4_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_4_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_3_pin_5_ENABLED 1U
#define ioss_0_port_3_pin_5_PORT GPIO_PRT3
#define ioss_0_port_3_pin_5_PORT_NUM 3U
#define ioss_0_port_3_pin_5_PIN 5U
#define ioss_0_port_3_pin_5_NUM 5U
#define ioss_0_port_3_pin_5_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define ioss_0_port_3_pin_5_INIT_DRIVESTATE 1
#ifndef ioss_0_port_3_pin_5_HSIOM
    #define ioss_0_port_3_pin_5_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_3_pin_5_IRQ ioss_interrupts_gpio_3_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_5_HAL_PORT_PIN P3_5
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_5 P3_5
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_5_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_5_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_5_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_3_pin_6_ENABLED 1U
#define ioss_0_port_3_pin_6_PORT GPIO_PRT3
#define ioss_0_port_3_pin_6_PORT_NUM 3U
#define ioss_0_port_3_pin_6_PIN 6U
#define ioss_0_port_3_pin_6_NUM 6U
#define ioss_0_port_3_pin_6_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define ioss_0_port_3_pin_6_INIT_DRIVESTATE 1
#ifndef ioss_0_port_3_pin_6_HSIOM
    #define ioss_0_port_3_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_3_pin_6_IRQ ioss_interrupts_gpio_3_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_6_HAL_PORT_PIN P3_6
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_6 P3_6
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_6_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_6_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_3_pin_6_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)
#define ioss_0_port_4_pin_0_ENABLED 1U
#define ioss_0_port_4_pin_0_PORT GPIO_PRT4
#define ioss_0_port_4_pin_0_PORT_NUM 4U
#define ioss_0_port_4_pin_0_PIN 0U
#define ioss_0_port_4_pin_0_NUM 0U
#define ioss_0_port_4_pin_0_DRIVEMODE CY_GPIO_DM_PULLUP
#define ioss_0_port_4_pin_0_INIT_DRIVESTATE 1
#ifndef ioss_0_port_4_pin_0_HSIOM
    #define ioss_0_port_4_pin_0_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_4_pin_0_IRQ ioss_interrupts_gpio_4_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_4_pin_0_HAL_PORT_PIN P4_0
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_4_pin_0 P4_0
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_4_pin_0_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_4_pin_0_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_4_pin_0_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_PULLUP
#endif //defined (CY_USING_HAL)
#define ioss_0_port_5_pin_2_ENABLED 1U
#define ioss_0_port_5_pin_2_PORT GPIO_PRT5
#define ioss_0_port_5_pin_2_PORT_NUM 5U
#define ioss_0_port_5_pin_2_PIN 2U
#define ioss_0_port_5_pin_2_NUM 2U
#define ioss_0_port_5_pin_2_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define ioss_0_port_5_pin_2_INIT_DRIVESTATE 1
#ifndef ioss_0_port_5_pin_2_HSIOM
    #define ioss_0_port_5_pin_2_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_5_pin_2_IRQ ioss_interrupts_gpio_5_IRQn
#if defined (CY_USING_HAL)
    #define ioss_0_port_5_pin_2_HAL_PORT_PIN P5_2
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_5_pin_2 P5_2
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_5_pin_2_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_5_pin_2_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#endif //defined (CY_USING_HAL)
#if defined (CY_USING_HAL)
    #define ioss_0_port_5_pin_2_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif //defined (CY_USING_HAL)

extern const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_0_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_0_pin_0_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_1_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_0_pin_1_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_4_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_0_pin_4_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_5_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_0_pin_5_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_1_pin_5_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_1_pin_5_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_1_pin_6_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_1_pin_6_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_0_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_2_pin_0_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_1_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_2_pin_1_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_2_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_2_pin_2_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_3_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_2_pin_3_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_4_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_2_pin_4_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_5_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_2_pin_5_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_3_pin_4_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_3_pin_4_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_3_pin_5_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_3_pin_5_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_3_pin_6_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_3_pin_6_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_4_pin_0_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_4_pin_0_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_gpio_pin_config_t ioss_0_port_5_pin_2_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t ioss_0_port_5_pin_2_obj;
#endif //defined (CY_USING_HAL)

void init_cycfg_pins(void);

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_PINS_H */
