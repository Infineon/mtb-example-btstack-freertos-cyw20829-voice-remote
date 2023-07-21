/*******************************************************************************
* File Name: app_hw_gpio.c
*
* Description: This file consists of the function definitions that are
*              necessary for developing LED use cases.
*
*******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
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


