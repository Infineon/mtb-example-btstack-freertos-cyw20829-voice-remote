/*******************************************************************************
* File Name: app_hw_gpio.c
*
* Description: This file consists of the function definitions that are
*              necessary for developing LED use cases.
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
#include "app_hw_gpio.h"
#include "cybsp.h"
#include "stdio.h"
#include "cyhal.h"
#include "FreeRTOS.h"
#include "timers.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
/* Constants for LED timeouts */
#define LED_ON_TIME          (500u)
#define LED_OFF_TIME         (500u)

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

/*                  LED and User Button Macros and variables                  */
/* FreeRTOS timer handles for LEDs */
#ifdef RED_LED_ENABLE
static TimerHandle_t red_led_timer_h;
#endif

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

static void app_led_timer_init(cyhal_gpio_t led_pin);

#ifdef RED_LED_ENABLE
static void app_red_led_toggle_cb(TimerHandle_t cb_parms);
#endif

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/*                      Status LED APIs                                       */

/**
 * Function Name:
 * app_status_led_init
 *
 * Function Description:
 * @brief This function initializes the LED Pins with OFF state.
 *
 * @param cyhal_gpio_t led_pin the GPIO pin number
 *
 * @return void
 */
void app_status_led_init(cyhal_gpio_t led_pin)
{

    cyhal_gpio_init(led_pin,
                    CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_PULLDOWN,
                    CYBSP_LED_STATE_OFF);
    app_led_timer_init(led_pin);
}

/**
 * Function Name:
 * app_led_timer_init
 *
 * Function Description:
 * @brief This function initializes the timer and remains in the state until
 *        it is altered.
 *
 * @param cyhal_gpio_t led_pin  The Led pin
 *
 * @return void
 */
static void app_led_timer_init(cyhal_gpio_t led_pin)
{
#ifdef RED_LED_ENABLE
    if(RED_LED == led_pin)
    {
        red_led_timer_h = xTimerCreate("RED_LED_Timer",
                                        LED_ON_TIME,
                                        pdTRUE,
                                        NULL,
                                        app_red_led_toggle_cb);
        if(NULL == red_led_timer_h)
        {
            printf("Red Led Timer initialization failed \r\n");
        }
    }
    else
#endif
    {
        printf("Invalid Colour \r\n");
    }

}

/**
 * Function Name:
 * app_red_led_toggle_cb
 *
 * @brief This function toggles Red LED during advertising
 *
 * @param cb_parms parameters for timer callback
 *
 * @return void
 */
#ifdef RED_LED_ENABLE
static void app_red_led_toggle_cb(TimerHandle_t cb_parms)
{
    (void)cb_parms;
    cyhal_gpio_toggle(RED_LED);
}
#endif

/**
 * Function Name:
 * app_status_led_on
 *
 * @brief This function turns on the status LED and remains in the state until
 *        it is altered.
 *
 * @param cyhal_gpio_t led_pin The GPIO pin number
 *
 * @return void
 */
void app_status_led_on(cyhal_gpio_t led_pin)
{
    /* Turn On Status LED */
    cyhal_gpio_write(led_pin, CYBSP_LED_STATE_ON);

}

/**
 * Function Name:
 * app_status_led_off
 *
 * @brief This function turns off the status LED  and remains in the state until
 *        it is altered.
 *
 * @param cyhal_gpio_t led_pin  The GPIO pin number
 *
 * @return void
 */
void app_status_led_off(cyhal_gpio_t led_pin)
{
    cyhal_gpio_write(led_pin, CYBSP_LED_STATE_OFF);
}

/**
 * Function Name:
 * app_status_led_blinky_on
 *
 * @brief This function turn on the blinking of status LED and remains in the
 *        state until it is altered.
 *
 * @param t_led_colour colour  The colour of the LED
 *
 * @return void
 */
void app_status_led_blinky_on(t_led_colour colour)
{
#ifdef RED_LED_ENABLE
     if(RED == colour)
    {
         if(xTimerIsTimerActive(red_led_timer_h))
         {
             if(pdPASS != xTimerStop(red_led_timer_h , 10u))
             {
                printf("Red Led timer failed to stop \r\n");
             }
         }
        if (pdPASS != xTimerStart(red_led_timer_h, 10u))
        {
            printf("Red LED timer failed to start  \r\n");
        }

    }
    else
#endif
    {
        printf("Invalid colour\r\n");
    }
}

/**
 * Function Name:
 * app_status_led_blinky_off
 *
 * @brief This function turns off blinking of status LED and remains in the
 *        state until it is altered.
 *
 * @param t_led_colour colour  The colour of the LED
 *
 * @return void
 */
void app_status_led_blinky_off(t_led_colour colour)
{
#ifdef RED_LED_ENABLE
    if(RED == colour)
    {
        if (pdPASS != xTimerStop(red_led_timer_h, 10u))
        {
            printf("Failed to stop LED timer!\r\n");
        }

        cyhal_gpio_write(RED_LED, CYBSP_LED_STATE_OFF);

    }
    else
#endif
    {
        printf("Invalid colour \r\n");
    }
}


