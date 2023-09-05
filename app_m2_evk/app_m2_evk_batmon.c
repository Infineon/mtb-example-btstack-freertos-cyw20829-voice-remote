/******************************************************************************
* File Name:   batmon.c
*
* Description: This files contains the function definition of DC monitoring
* of the battery
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2022, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
/*******************************************************************************
*                               Includes
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "app_hw_button.h"
#include "app_m2_evk_batmon.h"
#include "app_bt_hid.h"
#include "cyhal_syspm.h"
#include "app_hw_serial_flash.h"
#include "app_hw_handler.h"

/*******************************************************************************
*                               Macro Definitions
*******************************************************************************/
#define NO_OF_DC_SAMPLES                    (8)
#define BATMON_TIMEOUT_IN_MS                (300000)
#define BATMON_TASK_STACK_SIZE              (256u)
#define BATMON_TASK_PRIORITY                (2)
#define BATT_LVL_0_MV                       (1800)
#define BATT_LVL_100_MV                     (3000)
#define BATT_CAP_MV                         (BATT_LVL_100_MV - BATT_LVL_0_MV)
#define BATT_MV_1_CAP                       (BATT_CAP_MV/100)   //MV for 1 percent capacity of Battery Level

/*******************************************************************************
*                               Global Variables
*******************************************************************************/
int16_t batmon_samples[NO_OF_DC_SAMPLES];
uint8_t dc_sample_cnt = 0;
uint32_t batmon_dc_avg = 0;

/* Initial State of ADC driver set to IDLE */
volatile adc_driver_state_t adc_drv_state = ADC_IDLE;

static TimerHandle_t batmon_timer_h;
static TaskHandle_t batmon_task_h;
static SemaphoreHandle_t adc_sem_h = NULL;

/* ADCMIC interrupt configuration parameters */
const cy_stc_sysint_t ADCMIC_IRQ_cfg = {
    .intrSrc = (IRQn_Type)adcmic_0_IRQ,
    .intrPriority = 7
};

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/


/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/**
 * Function Name:
 * adc_dc_cap_start
 *
 * Function Description:
 * @brief It enable the ADC DC monitoring.
 *
 * @param void
 *
 * @return void
 */
static void adc_dc_cap_start(void)
{
    /* Acquire Semaphore to change the ADC Driver state to DC_MON_ON */
    if(xSemaphoreTake(adc_sem_h, ( TickType_t ) 20 ) == pdTRUE)
    {
        if(adc_drv_state == ADC_IDLE)
        {
            /* Initialize the ADCMic for for DC monitoring */
            if (CY_ADCMIC_SUCCESS != Cy_ADCMic_Init(adcmic_0_HW, &adcmic_0_config,CY_ADCMIC_DC))
            {
               CY_ASSERT(0);
            }

            Cy_ADCMic_SetInterruptMask(adcmic_0_HW, CY_ADCMIC_INTR_DC);
            Cy_ADCMic_ClearInterrupt(adcmic_0_HW,CY_ADCMIC_INTR);
            Cy_ADCMic_Enable(adcmic_0_HW);
            Cy_ADCMic_EnableTimer(adcmic_0_HW);
            adc_drv_state = ADC_DC_MON_ON;

            /* Do not allow the device to go to DS */
            cyhal_syspm_lock_deepsleep();
        }
        xSemaphoreGive(adc_sem_h);
    }
    else
    {
        printf("Failed to acquire adc mutex!\r\n");
    }

}

/**
 * Function Name:
 * batmon_timer_cb
 *
 * Function Description:
 * @brief Timer cb to start ADC DC monitoring.
 *
 * @param cb_param: Argument to cb
 *
 * @return void
 */
static void batmon_timer_cb(TimerHandle_t cb_params)
{

   (void)cb_params;
   adc_dc_cap_start();
}

#ifndef PDM_MIC

/**
 * Function Name:
 * adc_dc_cap_stop
 *
 * Function Description:
 * @brief Stops the ADC DC monitoring.
 *
 * @param void
 *
 * @return void
 *
 */
static void adc_dc_cap_stop(void)
{
    Cy_ADCMic_SetInterruptMask(adcmic_0_HW, 0);
    Cy_ADCMic_DisableTimer(adcmic_0_HW);
    Cy_ADCMic_Disable(adcmic_0_HW);
    adc_drv_state = ADC_IDLE;
    dc_sample_cnt = 0 ;

    /* Allow the device to go to DS */
    cyhal_syspm_unlock_deepsleep();
}

/**
 * Function Name:
 * adc_dc_monitoring_enable
 *
 * Function Description:
 * @brief Function to Enable/Disable ADC DC monitoring
 *
 * @param en Enable/Disable
 *
 * @return void
 */
void adc_dc_monitoring_enable(uint8_t en)
{
    if(en)
    {
        /* Set the state ADC driver State to IDLE */
        adc_drv_state = ADC_IDLE;

        /* Start the batmon timer */
        if (pdPASS != xTimerStart(batmon_timer_h, 10u))
        {
            printf("Failed to start batmon timer!\r\n");
            CY_ASSERT(0);
        }
    }
    else
    {
        /* Stop the batmon timer if in active state */
        if(xTimerIsTimerActive(batmon_timer_h))
        {
            if (pdPASS != xTimerStop(batmon_timer_h, 10u))
            {
                printf("Failed to stop batmon timer!\r\n");
                CY_ASSERT(0);
            }
        }

        /* Acquire Semaphore to change the state of ADC Driver to Streaming_ON
         * and Stop the Ongoing DC Measurement  */
        if(xSemaphoreTake(adc_sem_h, ( TickType_t ) 20 ) == pdTRUE)
        {
            if(adc_drv_state == ADC_DC_MON_ON)
            {
                adc_dc_cap_stop();
            }
            adc_drv_state = ADC_AUDIO_STREAMING_ON;
            xSemaphoreGive(adc_sem_h);
        }
    }
}
#endif

/**
 * Function Name:
 * adcmic_dc_intr_handler
 *
 * Function Description:
 * @brief
 *  ADC DC Measurement ISR handler. On Every ISR, one DC measurement is read and
 *  stored. Once the Number of DC samples are read for averaging, It sends
 *  notification battery monitoring task for further processing.
 *
 * @param void
 *
 * @return void
 */
static void adcmic_dc_intr_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    uint32_t intr_status;
    Cy_ADCMic_DisableTimer(adcmic_0_HW);

    /* Clear the DC interrupt */
    intr_status = Cy_ADCMic_GetInterruptStatusMasked(adcmic_0_HW);
    Cy_ADCMic_ClearInterrupt(adcmic_0_HW, intr_status);

    /* Don't start the timer for Next measurment if ADC driver state is changed
     * to IDLE */
    if(adc_drv_state == ADC_IDLE)
        return ;

    batmon_samples[dc_sample_cnt] = Cy_ADCMic_GetDcResult(adcmic_0_HW);
    dc_sample_cnt++;

    /* Stop Measurement on receiving the number of Defined ADC DC Samples */
    if(dc_sample_cnt >= NO_OF_DC_SAMPLES)
    {
        dc_sample_cnt =0 ;

        /* Stop the conversion */
        Cy_ADCMic_Disable(adcmic_0_HW);
        Cy_ADCMic_SetInterruptMask(adcmic_0_HW, 0);


        /* Set the ADC Driver state to IDLE once DC measurement is done */
        adc_drv_state = ADC_IDLE;

        xTaskNotifyFromISR(batmon_task_h, 0, eNoAction, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
    else
        Cy_ADCMic_EnableTimer(adcmic_0_HW);
}

/**
 * Function Name:
 * pdm_pcm_capture_start
 *
 * Function Description:
 * @brief
 *  Start/Stops PDM/PCM to capture audio data
 *
 * @param void
 *
 * @return void
 *
 */
static void batmon_init(void)
{
    /* Register the interrupt handler of ADCMIC Irq */
    Cy_SysInt_Init(&ADCMIC_IRQ_cfg,adcmic_dc_intr_handler);
    NVIC_ClearPendingIRQ(ADCMIC_IRQ_cfg.intrSrc);
    NVIC_EnableIRQ(ADCMIC_IRQ_cfg.intrSrc);

    /* Create a Periodic timer for Battery Monitoring */
    batmon_timer_h = xTimerCreate("Battery Monitoring Timer",
                                BATMON_TIMEOUT_IN_MS,
                                pdTRUE,
                                NULL ,
                                batmon_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL ==  batmon_timer_h)
    {
        printf("Battery Monitoring Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    /* Create Mutex for switching the ADC Driver State to different states */
    adc_sem_h = xSemaphoreCreateMutex();
    if (NULL ==  adc_sem_h)
    {
        printf("Adc mutex creation has failed! \r\n");
        CY_ASSERT(0);
    }

    if (pdPASS != xTimerStart(batmon_timer_h, 10u))
    {
        printf("Failed to start batmon timer!\r\n");
        CY_ASSERT(0);
    }
    printf("batmon timer started!\r\n");
}

/**
 * Function Name:
 * send_batmon_msg_to_hid_msg_q
 *
 * Function Description:
 * @brief
 *  Sends battery capacity level to HID queue
 *
 * @param batt_level Battery capacity level
 *
 * @return void
 */
static void send_batmon_msg_to_hid_msg_q(uint8_t batt_level)
{
    struct hid_rpt_msg batmon_msg;
    batmon_msg.msg_type = BATT_MSG_TYPE;
    batmon_msg.data.batt_level = batt_level;
    printf("The battery level percentage is %u \r\n",batt_level);
    if( pdPASS != xQueueSend( hid_rpt_q, &batmon_msg, TICKS_TO_WAIT) )
    {
        printf("Failed to send msg from BM to HID rpt Queue\r\n");
        CY_ASSERT(0);
    }
}

/**
 * Function Name:
 * batmon_task
 *
 * Function Description:
 * @brief
 *  Battery Monitoring Task Handler: This task function creates timer, mutex and
 *  processes the ADC measurement data, converts it to the battery capacity level
 *
 * @param arg Not used
 *
 * @return void
 */
static void batmon_task(void *arg)
{
    uint32_t ulNotifiedValue;
    int i;
    int16_t batt_level_mv;
    int16_t batt_cap, batt_cap_prev = 100;

    batmon_init();

    while(1)
    {
        /* Block until a command is received */
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
        batmon_dc_avg = 0;

        /* Averaging of DC measurement data */
        for (i = 0; i< NO_OF_DC_SAMPLES; i++)
        {
            batmon_dc_avg = batmon_dc_avg + batmon_samples[i];
        }
        batmon_dc_avg = batmon_dc_avg/NO_OF_DC_SAMPLES;

        /* get the DC data to MV and convert it ot battery capacity */
        batt_level_mv = Cy_ADCMic_CountsTo_mVolts((uint32_t)batmon_dc_avg, adcmic_0_config.dcConfig->context );
        if(batt_level_mv >= BATT_LVL_100_MV)
        {
            batt_cap = 100;
        }
        else
        {
            if(batt_level_mv <= BATT_LVL_0_MV)
            {
                batt_cap = 0;
            }
            else
            {
                batt_level_mv = BATT_LVL_100_MV - batt_level_mv;
                batt_cap = 100 - (batt_level_mv / BATT_MV_1_CAP);
            }
        }

        if(batt_cap <= batt_cap_prev || batt_cap <= 5)
        {
            batt_cap_prev = batt_cap;
            if(batt_cap <= 5)
            {
                send_batmon_msg_to_hid_msg_q(batt_cap);
                vTaskDelay(3000);
                Cy_SysPm_SystemEnterHibernate();
            }
            else
            {
                send_batmon_msg_to_hid_msg_q(batt_cap);
            }
        }

        /* Allow the device to go to DS */
        cyhal_syspm_unlock_deepsleep();
    }
}

/**
 * Function Name:
 * batmon_task_init
 *
 * Function Description:
 * @brief This function creates task for Battery Monitoring
 *
 * @param void
 *
 * @return void
 */

void batmon_task_init(void)
{
     /* Initialize the Battery Monitoring task */
    xTaskCreate(batmon_task, "Battery Monitoring Task", BATMON_TASK_STACK_SIZE, 0,
                BATMON_TASK_PRIORITY, &batmon_task_h);

    if(NULL == batmon_task_h)
    {
        printf("Battery Monitoring Task creation Failed! \r\n");
        CY_ASSERT(0);
    }

}


/* [] END OF FILE */
