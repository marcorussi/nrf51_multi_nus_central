/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2015] [Marco Russi]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/


/* ---------- Inclusions ---------- */

/* Compiler libraries */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/* Nordic common library */
#include "nordic_common.h"

/* CONNECTION component */
#include "conn_manager.h"

/* UART component */
#include "uart_manager.h"

/* APP components */
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"




/* ------------ Local defines ------------- */

/* Value of the RTC1 PRESCALER register. */   
#define APP_TIMER_PRESCALER             0   

/* Maximum number of simultaneously created timers. */                                                   
#define APP_TIMER_MAX_TIMERS            2	

/* Size of timer operation queues. */				    
#define APP_TIMER_OP_QUEUE_SIZE         4   




/* ---------- Local functions implementation ------------ */

/* Function for asserts in the SoftDevice.
   This function will be called in case of an assert in the SoftDevice.
   Warning: This handler is an example only and does not fit a final product. 
   	    You need to analyze how your product is supposed to react in case of Assert.
 	    On assert from the SoftDevice, the system can only recover on reset.
   Parameters: line_num     Line number of the failing ASSERT call.
 	       p_file_name  File name of the failing ASSERT call.
*/
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/* Function for putting the chip into sleep mode.
   Handling events from the ble_nus_c module
   This function will not return.
*/
static void sleep_mode_enter(void)
{
    uint32_t err_code;

    /* Go to system-off mode (this function will not return; wakeup will cause a reset). */
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/* Function for the Power manager. */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/* Main function */
int main(void)
{
	/* Initialize timers */
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

#if 0
    /* init pins */
    nrf_gpio_pin_dir_set(21, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(22, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_write(21, 1);
    nrf_gpio_pin_write(22, 1);
	nrf_gpio_pin_write(23, 1);
    nrf_gpio_pin_write(24, 1);
#endif

    /* init UART */
    uart_init();
    
    /* init connection manager */
    conn_init(false);
    
    uart_send_string((uint8_t *)"CIAO", 4);

    for (;;)
    {
        power_manage();
    }
}
