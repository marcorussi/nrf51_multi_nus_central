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


// TODO: consider to implement UART directly on top of the nrf_uart driver


/* ---------- Inclusions ---------- */

/* Compiler libraries */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

/* Nordic common library */
#include "nordic_common.h"

/* nrf drivers */
#include "nrf.h"

/* CONNECTION component */
#include "conn_manager.h"

/* APP components */
#include "app_error.h"
#include "app_uart.h"

/* header file */
#include "uart_manager.h"




/* ---------- Local defines ---------- */

/* UART TX, RX, CTS, RTS pin numbers */
#define UART_TX_PIN        		12				
#define UART_RX_PIN        		13				
#define UART_RTS_PIN        	14
#define UART_CTS_PIN        	15		

/* UART TX and RX buffers sizes */
#define UART_TX_BUF_SIZE        256                             	
#define UART_RX_BUF_SIZE        1     

/* UART received command buffer size */
#define UART_CMD_BUFFER_LENGTH	16 




/* ---------- Local variables declarations ----------- */

/* UART received command buffer length */
static uint8_t uart_cmd_buff[UART_CMD_BUFFER_LENGTH];

/* UART received command buffer index */
static uint8_t uart_cmd_buff_index = 0;

/* Flag to indicate if online command mode is active */
/* Init in command mode */
static bool is_command_mode = true;




/* ---------- Local functions prototypes ----------- */

static void parse_uart_data(uint8_t *);
static void uart_event_handler(app_uart_evt_t *p_event);




/* ------------ Exported functions implementation --------------- */

/* Function for initializing the UART. */
void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
        .rx_pin_no    = UART_RX_PIN,
        .tx_pin_no    = UART_TX_PIN,
        .rts_pin_no   = UART_RTS_PIN,
        .cts_pin_no   = UART_CTS_PIN,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud38400
      };

    APP_UART_FIFO_INIT(&comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_event_handler,
                        APP_IRQ_PRIORITY_LOW,
                        err_code);

    APP_ERROR_CHECK(err_code);
}


/* Function for send a string */
extern void uart_send_string(uint8_t *data_string, uint8_t data_length)
{
	uint8_t index;

	for(index = 0; index < data_length; index++)
	{
		app_uart_put(data_string[index]);
	}
}


/* Function for resetting buffer index */
extern void uart_reset(void)
{
	/* reset buffer index */
	uart_cmd_buff_index = 0;
}




/* ------------- Local functions implementation --------------- */

/* Function to parse received data from uart */
static void parse_uart_data(uint8_t *data_buff)
{
	/* check a command string */
	if(0 == strncmp((const char *)data_buff, (const char *)"AT+", (size_t)3))
    {
		data_buff += 3;
		if(0 == strncmp((const char *)data_buff, (const char *)"SCAN+", (size_t)5))
		{
			/* Start scanning for peripherals and initiate connection
			   with devices that advertise NUS UUID. */
			conn_start_scan();

			uart_send_string((uint8_t *)"OK.", 3);
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"SCAN-", (size_t)5))
		{
			/* stop scanning */
			conn_stop_scan();
			/* send number of found devices */
			conn_send_num_found_devices();	
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"FOUND=", (size_t)6))
		{
			data_buff += 6;
			/* send a specific found device */
			conn_send_found_device((*data_buff & 0x0F));	

			/* response is sent in called function */
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"CONN=", (size_t)5))
		{
			data_buff += 5;
			/* request a connection to a previously found device */
			if (true == conn_request_connection((*data_buff & 0x0F)))
			{
				/* enter into data mode */
				is_command_mode = false;

				uart_send_string((uint8_t *)"WAIT.", 5);
			}
			else
			{
				uart_send_string((uint8_t *)"ERROR.", 6);
			}
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"SWITCH=", (size_t)7))
		{
			data_buff += 7;
			/* switch connection index */
			if(true == conn_switch_conn((*data_buff & 0x0F)))
			{
				/* enter into data mode */
				is_command_mode = false;

				uart_send_string((uint8_t *)"OK.", 3);
			}
			else
			{
				uart_send_string((uint8_t *)"ERROR.", 6);
			}
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"DROP=", (size_t)5))
		{
			data_buff += 5;
			/* drop an ongoing connection */
            if (true == conn_drop_connection((*data_buff & 0x0F)))
            {
				uart_send_string((uint8_t *)"WAIT.", 5);
            }
			else
			{
				uart_send_string((uint8_t *)"ERROR.", 6);
			}
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"AUTO", (size_t)4))
		{
			/* enter into data mode */
			is_command_mode = false;

			uart_send_string((uint8_t *)"OK.", 3);
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"RESET", (size_t)5))
		{
			uart_send_string((uint8_t *)"OK.", 3);
			/* system reset */
			sd_nvic_SystemReset();
		}
		else
		{
			/* command is not supported */
			uart_send_string((uint8_t *)"ERROR.", 6);
		}
    }
	else if(0 == strncmp((const char *)data_buff, (const char *)"AT?", (size_t)3))
	{
		uart_send_string((uint8_t *)"OK.", 3);
	}
    else
    {
		/* invalid command string */
		uart_send_string((uint8_t *)"ERROR.", 6);
    }
}


/* Function for handling app_uart events.
   This function will receive a single character from the app_uart module and append it to 
   a string. The string will be be sent over BLE when the last character received is a 
   '.' character or if the string has reached a length of NUS_MAX_DATA_LENGTH.
*/
void uart_event_handler(app_uart_evt_t *p_event)
{
    /* manage event type */
    switch (p_event->evt_type)
    {
		/* manage data from UART */
        case APP_UART_DATA_READY:
		{
			/* TODO: consider to check uart_cmd_buff_index validity and clear it if invalid */

			UNUSED_VARIABLE(app_uart_get(&uart_cmd_buff[uart_cmd_buff_index]));

			/* if command mode is enabled */
			if(is_command_mode == true)
			{
				/* consider oncoming data commands */
				if (uart_cmd_buff[uart_cmd_buff_index] == '.')
				{
					parse_uart_data(uart_cmd_buff);
					uart_cmd_buff_index = 0;
				}
				else
				{
					/* increment buffer index */
					uart_cmd_buff_index++;
					/* if buffer overflow */
					if(uart_cmd_buff_index >= UART_CMD_BUFFER_LENGTH)
					{
						/* clear buffer index */
						uart_cmd_buff_index = 0;
					}
				}
			}
			/* else if data mode */
			else
			{
				/* send data through NUS service if termination char has been received */
				if (uart_cmd_buff[uart_cmd_buff_index] == '.') 
			    {
					
					/* send data through NUS service */
					conn_send_data_nus(uart_cmd_buff, uart_cmd_buff_index);
					/* clear buffer index */
				    uart_cmd_buff_index = 0;
			    }
				else if (uart_cmd_buff[uart_cmd_buff_index] == '*') 
				{
					/* clear buffer index */
			        uart_cmd_buff_index = 0;
					/* enter into configuration mode */
					is_command_mode = true;

					uart_send_string((uint8_t *)"OK.", 3); 
				}
				else
				{
					/* increment buffer index */
					uart_cmd_buff_index++;
					/* if buffer overflow */
					if(uart_cmd_buff_index >= UART_CMD_BUFFER_LENGTH)
					{
						/* clear buffer index */
						uart_cmd_buff_index = 0;
					}
				}
			}
            break;
		}
        case APP_UART_COMMUNICATION_ERROR:
		{
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
		}
        case APP_UART_FIFO_ERROR:
		{
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;
		}
        default:
            break;
    }
}




/* End of file */




