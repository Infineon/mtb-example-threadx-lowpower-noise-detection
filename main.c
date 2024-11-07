/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Lowpower Noise Detection
* 			   and Wakeup Example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company)
* oran affiliate of Cypress Semiconductor Corporation.  All rights reserved.
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
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cyhal_syspm.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define ADCCOMP_BASE ADCCOMP0

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
bool pm_callback(cyhal_syspm_callback_state_t state, \
		        cyhal_syspm_callback_mode_t mode, void* callback_arg);

void adccomp_interrupt_callback(void *callback_arg, cyhal_comp_event_t event);

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* ADC Comparator Configuration */
cyhal_comp_config_t adccomp_cfg = {
		.power = CYHAL_POWER_LEVEL_DEFAULT,
		.hysteresis = TRUE,
};

/* SysPM Callback Data */
cyhal_syspm_callback_data_t pm_callback_data = {
		.callback = pm_callback,
		.states = CYHAL_SYSPM_CALLBACK_STATE_ALL,
		.ignore_modes = (CYHAL_SYSPM_CHECK_READY | 
                CYHAL_SYSPM_CHECK_FAIL |
				CYHAL_SYSPM_BEFORE_TRANSITION | 
                CYHAL_SYSPM_AFTER_DS_WFI_TRANSITION),
	    .args = NULL,
	    .next = NULL
};

/* Variable for ThreadX thread */
TX_THREAD *main_thread = NULL;

/* Flag for ADCCOMP interrupt */
volatile bool adccomp_interrupt_occured = false;

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
 * Function Name: pm_callback()
 *******************************************************************************
 * Summary:
 *  This function receives the low power transition callbacks from SysPM Module
 * and allows user to take necessary action if needed.
 *
 ******************************************************************************/
bool pm_callback(cyhal_syspm_callback_state_t state, \
		        cyhal_syspm_callback_mode_t mode, void* callback_arg)
{
	bool allow_transition = true;

	if ((CYHAL_SYSPM_AFTER_TRANSITION == mode) && (main_thread != NULL))
	{
		tx_thread_resume(main_thread);
    	cyhal_gpio_write(CYBSP_USER_LED2, CYBSP_LED_STATE_ON);
		main_thread = NULL;
	}
	return allow_transition;
}

/*******************************************************************************
* Function Name: adccomp_isr_callback()
********************************************************************************
* Summary:
* This is the callback function for ADC Comparator Interrupt.
*
* Parameters:
*  void *callback_arg
*  cyhal_comp_event_t event: Event type for interrupt trigger
*
* Return:
*  void
*
*******************************************************************************/
void adccomp_interrupt_callback(void *callback_arg, cyhal_comp_event_t event)
{
    Cy_ADCCOMP_ClearInterrupt(ADCCOMP_BASE, CY_ADCCOMP_INTR_LPCOMP1);

}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM33 CPU. It does the following:
*    1. Initializes the GPIO, ADCCOMP and SysPM modules
*    2. Suspends main thread to put device to Deepsleep
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    int comparator_settled_poll_count = 0;
    cyhal_comp_t obj;
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

   /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("Device is setting up. Please wait for 20 seconds...\r\n");
    /* Wait till local sleep/wake pattern to complete */
    Cy_SysLib_Delay(20000);

    /* Application Starts */
    printf("\x1b[2J\x1b[;H");
    printf("************************************************************\n"
           "        Lowpower Noise Detection and Wakeup Example         \n"
           "************************************************************\n");

    /* Initialize the User LED 2 for checking deepsleep and wakeup*/
    cyhal_gpio_init(CYBSP_USER_LED2, CYHAL_GPIO_DIR_OUTPUT, \
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Register SysPM Callback */
    cyhal_syspm_register_callback(&pm_callback_data);

    /* Initialise the ADC Comparator (ADCCOMP) in Noise Threshold
     * Detection (NTD) mode */
    cyhal_comp_init(&obj, MIC_P, NC, NC, &adccomp_cfg);

    /* ADCCOMP Settling */
    do
    {
        /* Wait for 10 ms */
    	Cy_SysLib_Delay(10);

        /* Poll for ADCCOMP latch status and clear the latch to reset */
        if (cyhal_comp_read(&obj))
        {
            Cy_ADCCOMP_LPCOMP_ClearLatch(ADCCOMP_BASE, CY_ADCCOMP_LPCOMP_1);
        }

        /* If latch status clear for 20 consecutive polls,
         * ADCCOMP is ready to use */
        else
        {
            comparator_settled_poll_count++;
            if (comparator_settled_poll_count == 20)
            {
                break;
            }
        }
    } while(1);

    /* Register ADCCOMP callback */
    cyhal_comp_register_callback(&obj, &adccomp_interrupt_callback, &obj);

    /* Enable ADCCOMP in NTD mode as Wake-up Source*/
    cyhal_comp_enable_event(&obj, CYHAL_COMP_RISING_EDGE, \
    		                CYHAL_ISR_PRIORITY_DEFAULT, true);

    printf("ADC Comparator initialised in Noise Threshold Detection mode.\r\n\n");
    printf("Generate any noise near the Analog MIC to wakeup!\r\n\n");

    /* Prepare and trigger the Deepsleep */
    while(1)
    {
    	/* Suspend the main thread */
    	tx_thread_suspend(main_thread = tx_thread_identify());

    	/* Lock the device from entering deepsleep */
    	cyhal_syspm_lock_deepsleep();

    	printf("Device Wakeup Indication!\r\n\n");

    	/* Turn the User LED 2 OFF */
    	Cy_SysLib_Delay(1000);
    	cyhal_gpio_write(CYBSP_USER_LED2, CYBSP_LED_STATE_OFF);

    	/* Unlock the device to allow deepsleep */
    	cyhal_syspm_unlock_deepsleep();

    }
}

/* [] END OF FILE */
