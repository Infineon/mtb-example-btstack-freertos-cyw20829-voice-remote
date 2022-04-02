/*******************************************************************************
* File Name: app_hw_keyscan.h
*
* Description: This file consists of the function defintions that are
*              necessary for developing Keyscan use cases.
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "app_hw_keyscan.h"

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
/* Task handle to send Bluetooth LE GATT notifications */

TaskHandle_t keyscan_task_h;

cy_stc_keyscan_context_t context;

bool events_pending;
uint32_t g_PORT_SEL0;
uint32_t g_PORT_SEL1;
uint32_t g_CFG;
uint32_t g_OUT;
cy_stc_syspm_callback_params_t syspm_ks_ds_params;
uint8_t key_state_cnt;
uint8_t deepsleep_hold;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

static void app_keyscan_evt_notif_enable(uint32_t keyNotify);

static void send_msg_to_hid_msg_q(uint8_t keyCode, uint8_t upDownFlag);

static void keyscan_intHandler(void);

cy_en_syspm_status_t
syspm_ks_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
                              cy_en_syspm_callback_mode_t mode);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

cy_stc_syspm_callback_t syspm_ks_ds_cb_handler =
{
    syspm_ks_ds_cb,
    CY_SYSPM_DEEPSLEEP,
    0u,
    &syspm_ks_ds_params,
    NULL,
    NULL,
    255
};

/* Populate the Sysint configuration structure for Keyscan */
const cy_stc_sysint_t keyscan_irq_cfg =
{
    /* .intrSrc */ keyscan_interrupt_IRQn, /* Keyscan interrupt number */
    /* .intrPriority */ 7UL
};

/**
 * @brief DeepSleep Callback Function
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
syspm_ks_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
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

            // Disable SMIF
            SMIF0->CTL = SMIF0->CTL & ~SMIF_CTL_ENABLED_Msk;

            g_PORT_SEL0 = HSIOM_PRT2->PORT_SEL0;
            g_PORT_SEL1 = HSIOM_PRT2->PORT_SEL1;
            g_CFG = GPIO_PRT2->CFG;
            g_OUT = GPIO_PRT2->OUT;

            HSIOM_PRT2->PORT_SEL0 = 0x00;
            HSIOM_PRT2->PORT_SEL1 = 0x00;

            GPIO_PRT2->CFG = 0x600006;

            GPIO_PRT2->OUT = 0x1;
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {

            //Enable SMIF
            SMIF0->CTL = SMIF0->CTL | SMIF_CTL_ENABLED_Msk;

            HSIOM_PRT2->PORT_SEL0 = g_PORT_SEL0;
            HSIOM_PRT2->PORT_SEL1 = g_PORT_SEL1;

            GPIO_PRT2->CFG = g_CFG;

            GPIO_PRT2->OUT = g_OUT;

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


/*******************************************************************************
 * Function Name: keyscan_task
 *******************************************************************************
 * Summary:
 *  Task that prints the keycode, row and columns of the keyscan activity
 *
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void keyscan_task(void *args)
{

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
    app_ks_result = app_keyscan_handler_init();

    Cy_SysPm_RegisterCallback(&syspm_ks_ds_cb_handler);

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
            //Check value of keycode for the corresponding key being pressed//
            keyCode = kevent.keyCode;
            // scanCycle = kevent.scanCycleFlag;
            upDownFlag = kevent.upDownFlag;

            if(keyCode == KEYSCAN_KEYCODE_ROLLOVER)
            {
                printf(" \r\n ROLLOVER KEY DETECTED \r\n");
            }
            else if(keyCode == KEYSCAN_KEYCODE_END_OF_SCAN_CYCLE)
            {
                // printf("\r\n End of Scan Cycle event. Added by driver when an event"
                //         "is occured in different scan cycle. \r\n");
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
                // printf("Row %u Column %u\r\n",
                //         (keyCode % keyscan_0_config.noofRows),
                //         (keyCode / keyscan_0_config.noofRows));
                // printf("scanCycle  is :%u \r\n", scanCycle);
                // printf("upDownFlag is :%u \r\n", upDownFlag);

                // Send the keycode to Bluetooth LE task
                send_msg_to_hid_msg_q(keyCode, upDownFlag);

            }
            Cy_Keyscan_EventsPending(MXKEYSCAN, &events_pending, &context);
        }
    }

}


/**
 * @brief This is an interrupt callback which will be executed based on the
 * Keyscan interrupt triggers.
 *
 */
static void keyscan_intHandler(void)
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
 * @brief This is an interrupt callback which will be executed based on the
 * Keyscan interrupt triggers.
 *
 */
void key_detected_callback(void)
{
    // Send Event Notification to Keyscan task
     app_keyscan_evt_notif_enable(0);
}


/**
 *****************************************************************************
 **  app_configure_keyscan
 **  Configures the keyscan ip with
 **  3 rows and 10 columns
 **  macroDownDebCnt = 6u,
 **  macroUpDebCnt = 4u,
 **  microDebCnt = 1u,
 **
 *****************************************************************************/
int app_configure_keyscan(void)
{
    cy_en_ks_status_t ks_status;
    ks_status = Cy_Keyscan_Init(MXKEYSCAN, &keyscan_0_config, &context);
    if(ks_status != CY_KEYSCAN_SUCCESS)
    {
        printf("Keyscan Initialization failed \r\n");
        return KEYSCAN_FAILURE;
    }

    ks_status = Cy_Keyscan_Register_Callback(key_detected_callback, &context);
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

app_keyscan_status_t app_keyscan_handler_init(void)
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

    return KEYSCAN_SUCCESS;
}


app_keyscan_status_t app_keyscan_interrupt_init(void)
{

    cy_en_sysint_status_t sysStatus;

    /* Hook the interrupt service routine and enable the interrupt */
    sysStatus = Cy_SysInt_Init(&keyscan_irq_cfg, keyscan_intHandler);
    if(CY_SYSINT_SUCCESS != sysStatus)
    {
        return KEYSCAN_FAILURE;
    }

    /* Enable interrupt in NVIC. */
    NVIC_EnableIRQ(keyscan_irq_cfg.intrSrc);

    return KEYSCAN_SUCCESS;
}

/**
 * @brief This function is invoked when a Keyscan button interrupt is triggered
 *
 */
static void app_keyscan_evt_notif_enable(uint32_t keyNotify)
{
    BaseType_t xHigherPriorityTaskWoken;

    if (pdTRUE == xTaskNotifyFromISR(
                    keyscan_task_h,         // Handle of the task being notified
                    keyNotify,                   // Used to update the notification value of the target task
                    eSetValueWithOverwrite ,     // The Target task receives the event and is unconditionally set to ulvalue.
                    &xHigherPriorityTaskWoken ))  // This value will be set to pdTRUE when sending)
    {
        // printf("key evt Notified to task\r\n");
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
 * @brief This function is invoked when a Keyscan button interrupt is triggered
 *
 */
static void send_msg_to_hid_msg_q(uint8_t keyCode, uint8_t upDownFlag)
{
    struct hid_rpt_msg ks_msg;
    ks_msg.msg_type = KS_MSG_TYPE;
    ks_msg.data.ks.keycode = keyCode;
    ks_msg.data.ks.upDownFlag = upDownFlag;
    if( pdPASS != xQueueSend( hid_rpt_q, &ks_msg, TICKS_TO_WAIT) )
    {
        printf("Failed to send msg from KS to HID rpt Queue\r\n");
        CY_ASSERT(0);
    }
}
