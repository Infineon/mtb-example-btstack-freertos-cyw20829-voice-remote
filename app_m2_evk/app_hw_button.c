/*******************************************************************************
* File Name: app_hw_button.c
*
* Description: This file consists of the function definitions that are
*              necessary for developing button use cases.
*
*******************************************************************************
 * (c) 2021-2025, Infineon Technologies AG, or an affiliate of Infineon
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
#include "app_hw_button.h"
#include "app_hw_gpio.h"
#include "app_bt_hid.h"
#include "app_bt_gatt_handler.h"
#include "timers.h"
#include "app_bt_advert.h"
#include "app_bt_bonding.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
/* Interrupt priority for GPIO connected to button */
#define BUTTON_INT_PRIORITY                 (7)
/* Button press interval timer of 10 seconds */
#define BUTTON_PRESS_INTERVAL               (10000)
/* Button Task Priority of  Bluetooth LE Voice Remote */
#define BUTTON_TASK_PRIORITY                (configMAX_PRIORITIES - 2)
/* Button Stack size for Bluetooth LE Voice Remote */
#define BUTTON_TASK_STACK_SIZE              (512u)
/* Button task names for Bluetooth LE Voice Remote */
#define BUTTON_TASK_NAME                    "Button Task"

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
/* Task handle to send Button notifications */
TaskHandle_t button_task_h;

TimerHandle_t button_timer_handle;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

static void send_to_hid_msg_q(uint8_t keyCode, uint8_t upDownFlag);
static void button_timer_callback(TimerHandle_t xTimer);
void button_intHandler(void *handler_arg, cyhal_gpio_event_t event);

/* For button press interrupt */
cyhal_gpio_callback_data_t btn_cb_data =
{
    .callback     = button_intHandler,
    .callback_arg = NULL
};

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/**
 * Function Name: configure_button
 *
 * Function Description:
 *   @brief This function initializes a button as input that triggers interrupt on
 *   rising edge.
 *
 *   @param None
 *
 *   @return None
 *
 */
void configure_button(void)
{
    /* Initialize the user button */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLDOWN, CYBSP_BTN_OFF);

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &btn_cb_data);

    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_BOTH,
                            BUTTON_INT_PRIORITY, true);
}

/**
 * Function Name: button_intHandler
 *
 * Function Description:
 *   @brief Button interrupt handler.
 *
 *   @param void *handler_arg (unused)
 *   @param cyhal_gpio_irq_event_t (unused)
 *
 *   @return None
 *
 */
void button_intHandler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken;

    if(CYHAL_GPIO_IRQ_FALL == event)
    {
        vTaskNotifyGiveFromISR(button_task_h, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        xTimerStartFromISR(button_timer_handle, 0u);
    }
    else if(CYHAL_GPIO_IRQ_RISE == event)
    {
        vTaskNotifyGiveFromISR(button_task_h, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        xTimerStopFromISR(button_timer_handle, 0u);
    }
}

/*******************************************************************************
 * Function Name: button_timer_callback
 ********************************************************************************
 * Summary:
 *  Button timer callback. This function increment the steps level on button
 *  press and hold.
 *
 * Parameters:
 *  TimerHandle_t xTimer (unused)
 *
 *******************************************************************************/
void button_timer_callback(TimerHandle_t xTimer)
{
    app_bt_delete_bond_info();
    wiced_bt_set_pairable_mode(TRUE, FALSE);
    app_bt_conn_id = 0;
    app_bt_start_adv_any_host();
    printf("Bond Info removed \r\n");
    current_remote_state = UNPAIRED_ADVERTISING;
}

/**
 * @brief This function is invoked when a button interrupt is triggered
 *
 * @param keyCode KeyCode for MIC
 * @param upDownFlag KEY_PRESSED or KEY_RELEASED
 *
 * @return void
 */
static void send_to_hid_msg_q(uint8_t keyCode, uint8_t upDownFlag)
{
    struct hid_rpt_msg ks_msg;
    ks_msg.msg_type = KS_MSG_TYPE;
    ks_msg.data.ks.keycode = keyCode;
    ks_msg.data.ks.upDownFlag = upDownFlag;

    if(pdPASS != xQueueSend(hid_rpt_q, &ks_msg, TICKS_TO_WAIT))
    {
        printf("Failed to send msg from Button to HID rpt Queue\r\n");
        CY_ASSERT(0);
    }
}

/**
 * Function Name: button_task
 *
 * @brief
 *  Task handles the button activity
 *
 * @param void *args - Task parameter defined during task creation (unused)
 *
 * @return void
 *
 */
void button_task(void *args)
{
    uint8_t keyCode;
    uint8_t upDownFlag;

    /* Button configuration */
    configure_button();

    /* Initialize timer for button long press detection */
    button_timer_handle = xTimerCreate ("button timer", BUTTON_PRESS_INTERVAL,
            pdFALSE, NULL, button_timer_callback);

    if(NULL == button_timer_handle)
    {
        printf("Button timer initialization failed!\r\n");
        CY_ASSERT(0u);
    }

    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(KEY_PRESSED == cyhal_gpio_read(CYBSP_USER_BTN))
        {
            keyCode=KC_MIC;
            upDownFlag=KEY_PRESSED;
            printf("Button pressed \r\n");
            // Send the keycode to Bluetooth LE task
            send_to_hid_msg_q(keyCode, upDownFlag);
        }
        else if(KEY_RELEASED == cyhal_gpio_read(CYBSP_USER_BTN))
        {
            keyCode=0;
            upDownFlag=KEY_RELEASED;
            printf("Button released \r\n");
            // Send the keycode to Bluetooth LE task
            send_to_hid_msg_q(keyCode, upDownFlag);
        }
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
void button_task_init(void)
{
    /* Initialize the Button task */
    if( pdPASS != xTaskCreate(button_task,
                              BUTTON_TASK_NAME,
                              BUTTON_TASK_STACK_SIZE,
                              NULL,                     /* (void*) &xBatmonTaskParam */
                              BUTTON_TASK_PRIORITY,
                              &button_task_h))
    {
        /* Task is not created due to insufficient Heap memory.
         * Use vApplicationMallocFailedHook() callback to trap.
         * And xPortGetFreeHeapSize() to query unallocated heap memory
         */
        printf("Button Task creation failed");
    }
}

