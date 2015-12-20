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
#include <string.h>

/* Nordic common library */
#include "nordic_common.h"

/* nrf drivers */
#include "nrf.h"
#include "nrf_gpio.h"

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
static bool online_command_mode = false;




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

	//app_uart_put(0x0D);
	//app_uart_put(0x0A);
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

	/* online data mode is not active */
	online_command_mode = false;
}




/* ------------- Local functions implementation --------------- */

/* Function to parse received data from uart */
static void parse_uart_data(uint8_t *data_buff)
{
	/* check a command string */
	if(0 == strncmp((const char *)data_buff, (const char *)"AT+", (size_t)3))
    {
		data_buff += 3;
		if(0 == strncmp((const char *)data_buff, (const char *)"BLESCAN+", (size_t)8))
		{
			data_buff += 8;

			/* Start scanning for peripherals and initiate connection
			   with devices that advertise NUS UUID. */
			conn_start_scan();

			uart_send_string((uint8_t *)"OK", 2);

			//nrf_gpio_pin_write(22, 0);	
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"BLESCAN-", (size_t)8))
		{
			/* stop scanning */
			conn_stop_scan();
			/* send found devices */
			conn_send_found_devices();

			//nrf_gpio_pin_write(21, 0);	
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"BLECONN=", (size_t)8))
		{
			data_buff += 8;
			/* request a connection to a previously found device */
			if (true == conn_request_connection((*data_buff & 0x0F)))
			{
				uart_send_string((uint8_t *)"WAIT", 4);
			}
			else
			{
				uart_send_string((uint8_t *)"ERROR", 5);
			}
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"BLEDROP", (size_t)7))
		{
			/* drop an ongoing connection */
            if (true == conn_drop_connection())
            {
				uart_send_string((uint8_t *)"WAIT", 4);
            }
			else
			{
				uart_send_string((uint8_t *)"ERROR", 5);
			}
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"BLEAUTO", (size_t)7))
		{
			/* get back into online data mode */
			online_command_mode = false;
			uart_send_string((uint8_t *)"OK", 2);
		}
		else if(0 == strncmp((const char *)data_buff, (const char *)"EU", (size_t)2))
		{
			uart_send_string((uint8_t *)"CHE", 3);
		}
		else
		{
			/* command is not supported */
			uart_send_string((uint8_t *)"ERROR", 5);
		}
    }
    else
    {
		/* invalid command string */
		uart_send_string((uint8_t *)"ERROR", 5);
    }
}


/* Function for handling app_uart events.
   This function will receive a single character from the app_uart module and append it to 
   a string. The string will be be sent over BLE when the last character received was a 
   'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of NUS_MAX_DATA_LENGTH.
*/
void uart_event_handler(app_uart_evt_t *p_event)
{
    /* manage event type */
    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */ 
        case APP_UART_DATA_READY:

			/* TODO: consider to check uart_cmd_buff_index validity and clear it if invalid */

			UNUSED_VARIABLE(app_uart_get(&uart_cmd_buff[uart_cmd_buff_index]));
			//app_uart_put(uart_cmd_buff[uart_cmd_buff_index]);

			/* get connection state */
			conn_ke_state connection_state = conn_get_state();

			/* if device is connected as central */
			if((CONN_KE_CONNECTED_C == connection_state)
			&& (online_command_mode == false))
			{
				//nrf_gpio_pin_write(24, 0);

				/* send data through NUS service if termination char has been received */
				if (uart_cmd_buff[uart_cmd_buff_index] == '.') 
		        {
					/* send data through NUS service */
					conn_send_data_c_nus(uart_cmd_buff, uart_cmd_buff_index);
					/* clear buffer index */
		            uart_cmd_buff_index = 0;

					uart_send_string((uint8_t *)"SENT", 4); 
		        }
				else if (uart_cmd_buff[uart_cmd_buff_index] == '*') 
				{
					/* clear buffer index */
		            uart_cmd_buff_index = 0;
					/* enter in online command mode */
					online_command_mode = true;

					uart_send_string((uint8_t *)"OK", 2); 
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
			/* else if device is not connected */
			else if((CONN_KE_INIT_C == connection_state)
				 || (online_command_mode == true))
			{
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
			/* else if device is connected as peripheral */
			else if(CONN_KE_CONNECTED_P == connection_state)
			{

				nrf_gpio_pin_write(23, 0);

				/* send data through NUS service if termination char has been received */
				if (uart_cmd_buff[uart_cmd_buff_index] == '.') 
		        {
					/* send data through NUS service */
					conn_send_data_p_nus(uart_cmd_buff, uart_cmd_buff_index);
					/* clear buffer index */
		            uart_cmd_buff_index = 0;

					uart_send_string((uint8_t *)"SENT", 4); 
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
			else
			{
				/* it should never arrive here, invalid/unknown connection state */
				/* do nothing */
			}
            break;

        /**@snippet [Handling data from UART] */ 
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}




/* End of file */




