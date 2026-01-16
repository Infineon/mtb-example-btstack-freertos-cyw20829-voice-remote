/*******************************************************************************
* File Name: main.c
*
* Description:
* This file contains the starting point of Bluetooth LE Voice Remote application.
* The wiced_bt_stack_init() registers for Bluetooth events in this main function.
* The Bluetooth Management callback manages the Bluetooth events and the
* application developer can customize the functionality and behavior depending on
* the Bluetooth events. The Bluetooth Management callback acts like a
* Finite State Machine (FSM) for the SoC.
*
* Related Document: See README.md
*
*******************************************************************************
 * (c) 2022-2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
*                               Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cybsp_bt_config.h"
#include "cyhal.h"
#include "cy_sysclk.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#if (defined(RED_LED_ENABLE)) || (defined(GREEN_LED_ENABLE))
#include "app_hw_gpio.h"
#endif
#include "app_bt_event_handler.h"
#include "cycfg_gap.h"
#include "cycfg_bt_settings.h"
#include "wiced_memory.h"
#include "app_bt_utils.h"
#include "app_hw_handler.h"
#ifdef VOICE_REMOTE
#include "app_hw_keyscan.h"
#else
#include "app_hw_button.h"
#endif
#include "app_hw_serial_flash.h"
#include "app_bt_hid.h"
#include "audio.h"
#include "timers.h"
#include "app_hw_batmon.h"
#include "cybt_platform_hci.h"
#include "cybt_platform_trace.h"

#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#else
#include "cy_retarget_io.h"
#endif

#ifdef ENABLE_BT_SPY_LOG
#define printf               WICED_BT_TRACE
#endif
/*******************************************************************************
*                               Macro Definitions
*******************************************************************************/

/* Sufficient Heap size for Bluetooth activities */
#define BT_HEAP_SIZE                        (0x1000)

/*******************************************************************************
*                           Global Variables
*******************************************************************************/
/* Queue Handles of  Bluetooth LE Voice Remote Application  */
extern QueueHandle_t hid_rpt_q;

/*Kvstore block device*/
mtb_kvstore_bd_t                    block_device;

const cybt_platform_config_t bt_platform_cfg_settings =
{
    .hci_config =
    {
        .hci_transport = CYBT_HCI_IPC,
    },

    .controller_config =
    {
        #if defined(CY_CFG_PWR_SYS_IDLE_MODE) && \
                    ((CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_SLEEP) || \
                    (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)|| \
                    (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP_RAM))
            .sleep_mode = { .sleep_mode_enabled = 1 },
        #else
            .sleep_mode = { .sleep_mode_enabled = 0 },
        #endif
    },
};

/*******************************************************************************
*                           Function Prototypes
*******************************************************************************/

/* Function to initialize the various tasks for the Bluetooth LE Remote application */
static void remote_tasks_init(void);

#ifdef ENABLE_BT_SPY_LOG
void app_enable_bt_spy()
{
    cybt_debug_uart_config_t config = {
        .uart_tx_pin    = CYBSP_DEBUG_UART_TX,
        .uart_rx_pin    = CYBSP_DEBUG_UART_RX,
        .uart_cts_pin   = CYBSP_DEBUG_UART_CTS,
        .uart_rts_pin   = CYBSP_DEBUG_UART_RTS,
        .baud_rate      = DEBUG_UART_BAUDRATE,
        .flow_control   = TRUE };
    cybt_debug_uart_init(&config, NULL);
}
#endif

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/**
 *  @brief Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
int main(void)
 {

    /* Initialise the BSP and Verify the BSP initialization */
    CY_ASSERT(CY_RSLT_SUCCESS == cybsp_init());

#ifndef PDM_MIC
    Cy_GPIO_Pin_FastInit(GPIO_PRT0, 1U, 0x00U, 0x00U, HSIOM_SEL_GPIO);
    Cy_GPIO_Pin_FastInit(GPIO_PRT0, 2U, 0x00U, 0x00U, HSIOM_SEL_GPIO);
    Cy_SysClk_PeriPclkDisableDivider((en_clk_dst_t)PERI_0_GROUP_3_DIV_16_5_0_GRP_NUM, CY_SYSCLK_DIV_16_5_BIT, 0U);


    /* Disable the Unused Peripheral IP Slaves */
    (void)Cy_SysClk_PeriGroupSetSlaveCtl(0, CY_SYSCLK_PERI_GROUP_SL_CTL2, 0xFFFFFF80U);
    (void)Cy_SysClk_PeriGroupSetSlaveCtl(1, CY_SYSCLK_PERI_GROUP_SL_CTL2, 0xFFFFFDBCU);
    (void)Cy_SysClk_PeriGroupSetSlaveCtl(2, CY_SYSCLK_PERI_GROUP_SL_CTL2, 0xFFFFFF19U);


    (void)Cy_SysClk_PeriGroupSetSlaveCtl(0, CY_SYSCLK_PERI_GROUP_SL_CTL, 0x0000007FU);
    (void)Cy_SysClk_PeriGroupSetSlaveCtl(1, CY_SYSCLK_PERI_GROUP_SL_CTL, 0x00000243U);
    (void)Cy_SysClk_PeriGroupSetSlaveCtl(2, CY_SYSCLK_PERI_GROUP_SL_CTL, 0x000000E6U);

#endif
    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the tasks */
    remote_tasks_init();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    printf("Scheduler exited unexpectedly\r\n");
    CY_ASSERT(0);
}


/**
 *  Function name:
 *  remote_tasks_init
 *
 *  Function Description:
 *  @brief Initializes all the FreeRTOS tasks needed for the voice remote
 *  application
 *
 *  @param void
 *
 *  @return void
 *
 */
static void remote_tasks_init(void)
{

    // TODO Separate HAL/PDL and RTOS Task/Queue Initialisation

#ifdef ENABLE_BT_SPY_LOG
    app_enable_bt_spy();
#else
#ifndef NO_LOGGING
    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,\
                        CY_RETARGET_IO_BAUDRATE);
    Cy_GPIO_SetHSIOM(GPIO_PRT3, 2, HSIOM_SEL_GPIO);
#endif
#endif
    cybt_platform_set_trace_level(CYBT_TRACE_ID_STACK, CYBT_TRACE_ID_MAX);

    /* Initialising the HCI UART for Host contol */
    cybt_platform_config_init(&bt_platform_cfg_settings);

    /* Debug logs on UART port */
    printf("****** Bluetooth LE Voice Remote Application ******\r\n");

#ifdef BSA_OPUS
    printf("Bluetooth LE Voice Transport : BSA Opus\r\n");
#endif
#ifdef ATV_ADPCM
    printf("Bluetooth LE Voice Transport : Google's Android TV\r\n");
#endif

    printf("GAP Peripheral preffered Connection Parameters\r\n");
    app_print_byte_array(app_gap_peripheral_preferred_connection_parameters,
                         app_gap_peripheral_preferred_connection_parameters_len);

    printf("\r\nThis application implements HoGP and sends HID reports on "
            "key press over BLE \r\n");

    printf("\r\nDiscover this device with the name:%s\r\n", app_gap_device_name);

    // TODO Initialize the serial flash for bonding
    /* Initialize the block device used by kv-store for perfroming read/write
     * operations to the flash
     */
    app_flash_bd_init(&block_device);

    /* Configure Bluetooth LE configuration & registers Bluetooth LE event callback function
     * with the BT stack
     */
    if( WICED_BT_SUCCESS != wiced_bt_stack_init(app_bt_management_callback,   \
                                                &wiced_bt_cfg_settings))
    {
        /* Check if stack initialization was successful */
        printf("Bluetooth Stack Initialization failed!!\r\n");
    }

    /* Create a buffer heap, make it the default heap.  */
    if( NULL == wiced_bt_create_heap("app", NULL, BT_HEAP_SIZE, NULL, WICED_TRUE))
    {
        printf("Heap create Failed");
    }

    ble_task_init();

    /* Creates Task and Queue for Encoder */
    audio_task_init();

    xmit_task_init();

    batmon_task_init();

    /* Initialize the RED and GREEN LED for connection, advertisement and notifications */
#if defined (RED_LED_ENABLE)
    app_status_led_init(RED_LED);
#endif

#if defined (GREEN_LED_ENABLE)
    cyhal_gpio_init(GREEN_LED,
                    CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_PULLDOWN,
                    CYBSP_LED_STATE_OFF);
#endif

    // Keyscan Queue to send msgs to Bluetooth LE Task
    hid_rpt_q =  xQueueCreate(HID_MSG_Q_SZ, HID_MSG_Q_ITEM_SZ);
    if(NULL == hid_rpt_q)
    {
        printf("HID Report Queue creation failed! \r\n");
        CY_ASSERT(0);
    };

    #ifdef VOICE_REMOTE
        keyscan_task_init();
    #else
        button_task_init();
    #endif
    /* TODO Future Task: Initialize the IR task */


}




/* [] END OF FILE */
