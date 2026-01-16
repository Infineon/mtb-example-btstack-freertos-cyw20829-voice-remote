/*******************************************************************************
* File Name: app_hw_keyscan.c
*
* Description: This file consists of the function defintions that are
*              necessary for developing Keyscan use cases.
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
 *                              INCLUDES
 ******************************************************************************/
#include "app_hw_handler.h"
#include "app_hw_serial_flash.h"
#include "app_hw_keyscan.h"
#include "app_hw_gpio.h"
#include "app_bt_hid.h"
#include "cyabs_rtos_dsram.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#else
#include "cy_retarget_io.h"
#endif
/*******************************************************************************
 *                              Macro Definitions
 ******************************************************************************/
#define MAX_NOOF_ROWS                       (3U)
#define MAX_NOOF_COLUMNS                    (10U)
/* Keyscan Task Priority of  Bluetooth LE Voice Remote */
#define KEYSCAN_TASK_PRIORITY               (configMAX_PRIORITIES - 2)
/* Keyscan Stack size for Bluetooth LE Voice Remote */
#define KEYSCAN_TASK_STACK_SIZE             (512u)
/* Keyscan task names for Bluetooth LE Voice Remote */
#define KEYSCAN_TASK_NAME                   "Keyscan Task"

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
/* Task handle to send Keyscan notifications */
TaskHandle_t keyscan_task_h;

cy_stc_keyscan_context_t context;

cy_stc_syspm_callback_params_t syspm_ks_ds_params;
cy_stc_syspm_callback_params_t syspm_ks_dsram_params;
cy_stc_syspm_callback_params_t syspm_uart_dsram_params;
uint8_t key_state_cnt;
uint8_t deepsleep_hold;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

static void app_keyscan_evt_notif_enable(uint32_t keyNotify);

static void send_msg_to_hid_msg_q(uint8_t keyCode, uint8_t upDownFlag);

static void app_keyscan_intHandler(void);

static app_keyscan_status_t app_keyscan_interrupt_init(void);

static void app_keyscan_handler_init(void);

static int app_configure_keyscan(void);

static void app_key_detected_callback(void);

#ifdef ENABLE_BT_SPY_LOG
extern void app_enable_bt_spy();
#endif

cy_en_syspm_status_t
app_syspm_ks_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
                              cy_en_syspm_callback_mode_t mode);

cy_en_syspm_status_t
app_syspm_ks_dsram_cb(cy_stc_syspm_callback_params_t *callbackParams,
                              cy_en_syspm_callback_mode_t mode);
cy_en_syspm_status_t
app_syspm_uart_dsram_cb(cy_stc_syspm_callback_params_t *callbackParams,
                              cy_en_syspm_callback_mode_t mode);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

cy_stc_syspm_callback_t syspm_ks_ds_cb_handler =
{
    app_syspm_ks_ds_cb,
    CY_SYSPM_DEEPSLEEP,
    0u,
    &syspm_ks_ds_params,
    NULL,
    NULL,
    250
};
cy_stc_syspm_callback_t syspm_ks_dsram_cb_handler =
{
    app_syspm_ks_dsram_cb,
    CY_SYSPM_DEEPSLEEP_RAM,
    0u,
    &syspm_ks_dsram_params,
    NULL,
    NULL,
    250
};
cy_stc_syspm_callback_t syspm_uart_dsram_cb_handler =
{
    app_syspm_uart_dsram_cb,
    CY_SYSPM_DEEPSLEEP_RAM,
    0u,
    &syspm_uart_dsram_params,
    NULL,
    NULL,
    0
};

/* Populate the Sysint configuration structure for Keyscan */
const cy_stc_sysint_t keyscan_irq_cfg =
{
    /* .intrSrc */ keyscan_interrupt_IRQn, /* Keyscan interrupt number */
    /* .intrPriority */ 7UL
};

/**
 * Function Name:
 * app_syspm_ks_ds_cb
 *
 * Function Description:
 * @brief DeepSleep Callback Function
 *
 * @param callbackParams Pointer to cy_stc_syspm_callback_params_t
 * @param mode cy_en_syspm_callback_mode_t
 *
 * @return cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
app_syspm_ks_ds_cb( cy_stc_syspm_callback_params_t *callbackParams,
                cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    switch(mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
            if(key_state_cnt == 0 && !deepsleep_hold)
            {
                Cy_Keyscan_SetInterruptMask(MXKEYSCAN,  MXKEYSCAN_INTR_KEY_EDGE_DONE);
                Cy_SysClk_MfoEnable(false);
            }
            else
            {
                Cy_SysClk_MfoEnable(true);
            }

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {

            //Enable SM
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        /* Performs the actions to be done after entering the low power mode */
        {
            Cy_Keyscan_SetInterruptMask(MXKEYSCAN, MXKEYSCAN_INTR_FIFO_THRESH_DONE);
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END

/**
 * Function Name:
 * app_syspm_ks_ds_cb
 *
 * Function Description:
 * @brief DeepSleep Callback Function
 *
 * @param callbackParams Pointer to cy_stc_syspm_callback_params_t
 * @param mode cy_en_syspm_callback_mode_t
 *
 * @return cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
app_syspm_ks_dsram_cb( cy_stc_syspm_callback_params_t *callbackParams,
                cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    switch(mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
            if(key_state_cnt == 0 && !deepsleep_hold)
            {
                Cy_Keyscan_SetInterruptMask(MXKEYSCAN,  MXKEYSCAN_INTR_KEY_EDGE_DONE);
                Cy_SysClk_MfoEnable(false);
            }
            else
            {
                Cy_SysClk_MfoEnable(true);
            }

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        /* Performs the actions to be done after entering the low power mode */
        {
            Cy_Keyscan_SetInterruptMask(MXKEYSCAN, MXKEYSCAN_INTR_FIFO_THRESH_DONE);

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END


/**
 * Function Name:
 * app_syspm_ks_ds_cb
 *
 * Function Description:
 * @brief DeepSleep Callback Function
 *
 * @param callbackParams Pointer to cy_stc_syspm_callback_params_t
 * @param mode cy_en_syspm_callback_mode_t
 *
 * @return cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
app_syspm_uart_dsram_cb( cy_stc_syspm_callback_params_t *callbackParams,
                cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    switch(mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        /* Performs the actions to be done after entering the low power mode */
        {
            if(Cy_SysLib_IsDSRAMWarmBootEntry())
            {
#ifndef ENABLE_BT_SPY_LOG
#ifndef NO_LOGGING
            cy_retarget_io_deinit();
            cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,115200);
            Cy_GPIO_SetHSIOM(GPIO_PRT3, 2, HSIOM_SEL_GPIO);
#endif
#else
#ifndef NO_LOGGING
            cybt_debug_uart_deinit();
            app_enable_bt_spy();

#endif
#endif

            }

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END
/**
 * Function Name: keyscan_task
 *
 * Function Description:
 * @brief
 * Task that prints the keycode, row and columns of the keyscan activity
 *
 * @param void *args - Task parameter defined during task creation (unused)
 *
 * @return void
 *
 */
void keyscan_task(void *args)
{

    bool events_pending;
    cy_en_ks_status_t app_ks_result;
    cy_stc_key_event kevent;
    cy_en_ks_status_t status;
    uint32_t ulNotifiedValue;
    uint8_t keyCode;
    uint8_t upDownFlag;

    /* Initialize NVIC interrupt for Keyscan */
    app_ks_result = app_keyscan_interrupt_init();
    if( 0 != app_ks_result)
    {
        printf("Keyscan Interrupt init failed \r\n");
    }

    /* Initialize and Handle Keyscan Hardware configuration */
    app_keyscan_handler_init();

    Cy_SysPm_RegisterCallback(&syspm_ks_ds_cb_handler);
    Cy_SysPm_RegisterCallback(&syspm_ks_dsram_cb_handler);
    Cy_SysPm_RegisterCallback(&syspm_uart_dsram_cb_handler);

    while(1)
    {
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);

        Cy_Keyscan_EventsPending(MXKEYSCAN, &events_pending, &context);

        while (events_pending)
        {
            status = Cy_Keyscan_GetNextEvent(MXKEYSCAN, &kevent, &context);
            if(status != CY_KEYSCAN_SUCCESS)
            {
                printf("Keyscan get next event failed \r\n");
            }
            /*Check value of keycode for the corresponding key being pressed*/
            keyCode = kevent.keyCode;
            upDownFlag = kevent.upDownFlag;

            if(keyCode == KEYSCAN_KEYCODE_ROLLOVER)
            {
                printf(" \r\n ROLLOVER KEY DETECTED \r\n");
            }
            else if(keyCode == KEYSCAN_KEYCODE_END_OF_SCAN_CYCLE)
            {
                Cy_Keyscan_EventsPending(MXKEYSCAN, &events_pending, &context);
                continue;
            }
            else
            {
                if(upDownFlag)
                {
                    key_state_cnt-- ;
                }
                else
                {
                    key_state_cnt++ ;
                }
                /* Keycode is calculated as ((no of rows * column num) + row num) */
                printf("Keycode detected is :%u \r\n", keyCode);
                // Send the keycode to Bluetooth LE task
                send_msg_to_hid_msg_q(keyCode, upDownFlag);

            }
            Cy_Keyscan_EventsPending(MXKEYSCAN, &events_pending, &context);
        }
    }

}


/**
 * Function Name:
 * app_keyscan_intHandler
 *
 * Function Description:
 * @brief This is an interrupt callback which will be executed based on the
 *        Keyscan interrupt triggers.
 *
 * @param void
 *
 * @return void
 */
static void app_keyscan_intHandler(void)
{
    cy_en_ks_status_t status;
    uint32_t int_status;
    Cy_Keyscan_GetInterruptMaskedStatus(MXKEYSCAN, &int_status);
    if(MXKEYSCAN_INTR_FIFO_THRESH_DONE == int_status)
    {
       status = Cy_Keyscan_Interrupt_Handler(MXKEYSCAN, &context);
       if(CY_KEYSCAN_SUCCESS != status)
       {
            printf("Keyscan interrupt handler failed \r\n");
        }
        deepsleep_hold = 0;
    }
    else
    {
        deepsleep_hold = 1;
        Cy_Keyscan_ClearInterrupt(MXKEYSCAN, MXKEYSCAN_INTR_KEY_EDGE_DONE);
    }

}


/**
 * Function Name:
 * app_key_detected_callback
 *
 * Function Description:
 * @brief This is an interrupt callback which will be executed based on the
 * Keyscan interrupt triggers.
 *
 * @param void
 *
 * @return void
 */
static void app_key_detected_callback(void)
{
    // Send Event Notification to Keyscan task
     app_keyscan_evt_notif_enable(0);
}

/**
 * Function Name: app_configure_keyscan
 *
 * @brief Configures the Keyscan Hardware Block with
 * 3 rows and 10 columns
 * macroDownDebCnt = 6u,
 * macroUpDebCnt = 4u,
 * microDebCnt = 1u,
 *
 * @param void
 *
 * @return int app_keyscan_status_t
 */
static int app_configure_keyscan(void)
{
    cy_en_ks_status_t ks_status;
    ks_status = Cy_Keyscan_Init(MXKEYSCAN, &keyscan_0_config, &context);
    if(ks_status != CY_KEYSCAN_SUCCESS)
    {
        printf("Keyscan Initialization failed \r\n");
        return KEYSCAN_FAILURE;
    }

    ks_status = Cy_Keyscan_Register_Callback(app_key_detected_callback, &context);
    if(ks_status != CY_KEYSCAN_SUCCESS)
    {
        printf("Keyscan register event notification failed. \r\n");
        return KEYSCAN_FAILURE;
    }

    Cy_Keyscan_ClearInterrupt(MXKEYSCAN, MXKEYSCAN_INTR_ALL);
    ks_status = Cy_Keyscan_SetInterruptMask(MXKEYSCAN, MXKEYSCAN_INTR_FIFO_THRESH_DONE);
    if(ks_status != CY_KEYSCAN_SUCCESS)
    {
        printf("Keyscan set interrupt mask failed. \r\n");
        return KEYSCAN_FAILURE;
    }
    return KEYSCAN_SUCCESS;
}

/**
 * Function Name:
 * app_keyscan_handler_init
 *
 * Function Description:
 * @brief Configures Keymatrix size and debounce filters
 *
 * @param void
 *
 * @return void
 */
static void app_keyscan_handler_init(void)
{

    /* Configure Keymatrix size and debounce filters */
    if ( KEYSCAN_SUCCESS != app_configure_keyscan() )
    {
        printf("Keyscan Config failed \r\n");
    }
    else
    {
        printf("Keyscan Config is successful \r\n");
    }

}

/**
 * Function Name:
 * app_keyscan_interrupt_init
 *
 * Function Description:
 * @brief Function to configure Keyscan interrupt
 *
 * @param void
 *
 * @return app_keyscan_status_t KEYSCAN_FAILURE or KEYSCAN_SUCCESS
 */
static app_keyscan_status_t app_keyscan_interrupt_init(void)
{

    cy_en_sysint_status_t sysStatus;

    /* Hook the interrupt service routine and enable the interrupt */
    sysStatus = Cy_SysInt_Init(&keyscan_irq_cfg, app_keyscan_intHandler);
    if(CY_SYSINT_SUCCESS != sysStatus)
    {
        return KEYSCAN_FAILURE;
    }

    /* Enable interrupt in NVIC. */
    NVIC_EnableIRQ(keyscan_irq_cfg.intrSrc);

    return KEYSCAN_SUCCESS;
}

/**
 * Function Name:
 * app_keyscan_evt_notif_enable
 *
 * Function Description:
 * @brief This function is invoked when a Keyscan button interrupt is triggered
 *
 * @param keyNotify uint32_t
 *
 * @return void
 */
static void app_keyscan_evt_notif_enable(uint32_t keyNotify)
{
    BaseType_t xHigherPriorityTaskWoken;

    if (pdTRUE != xTaskNotifyFromISR(
                    keyscan_task_h,              // Handle of the task being notified
                    keyNotify,                   // Used to update the notification value of the target task
                    eSetValueWithOverwrite ,     // The Target task receives the event and is unconditionally set to ulvalue.
                    &xHigherPriorityTaskWoken )) // This value will be set to pdTRUE when sending)
    {
        printf("Key evt not Notified to task\r\n");
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
 * Function Name:
 * send_msg_to_hid_msg_q
 *
 * Function Description:
 * @brief This function is invoked when a Keyscan button interrupt is triggered
 *
 * @param keyCode KeyCode from Keyscan driver
 * @param upDownFlag KEY_PRESSED or KEY_RELEASED
 *
 * @return void
 */
static void send_msg_to_hid_msg_q(uint8_t keyCode, uint8_t upDownFlag)
{
    struct hid_rpt_msg ks_msg;
    ks_msg.msg_type = KS_MSG_TYPE;
    ks_msg.data.ks.keycode = keyCode;
    ks_msg.data.ks.upDownFlag = upDownFlag;
#if defined (GREEN_LED_ENABLE)
    if(upDownFlag == KEY_PRESSED)
    {
        app_status_led_on(GREEN_LED);
    }
    else
    {
        app_status_led_off(GREEN_LED);
    }
#endif
    if( pdPASS != xQueueSend( hid_rpt_q, &ks_msg, TICKS_TO_WAIT) )
    {
        printf("Failed to send msg from KS to HID rpt Queue\r\n");
        CY_ASSERT(0);
    }
}

/**
 * Function Name:
 * button_task_init
 *
 * @brief  This Function creates Button task.
 *
 * @param void
 *
 * @return void
 */
void keyscan_task_init(void)
{
    /* Initialize the Keyscan task */
    if( pdPASS != xTaskCreate(keyscan_task,
                              KEYSCAN_TASK_NAME,
                              KEYSCAN_TASK_STACK_SIZE,
                              NULL,                     /* (void*) &xBatmonTaskParam */
                              KEYSCAN_TASK_PRIORITY,
                              &keyscan_task_h))
    {
        /* Task is not created due to insufficient Heap memory.
         * Use vApplicationMallocFailedHook() callback to trap.
         * And xPortGetFreeHeapSize() to query unallocated heap memory
         */
        printf("Keyscan Task creation failed");
    }
}
