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

/* softdevice handler */
#include "softdevice_handler.h"

/* nrf drivers */
#include "nrf.h"
#include "nrf_gpio.h"

/* UTIL component */
#include "util.h"

/* BLE components */
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_nus_c.h"
#include "ble_db_discovery.h"

/* UART component */
#include "uart_manager.h"

/* APP components */
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"

/* header file */
#include "conn_manager.h"




/* ---------- Local definitions ---------- */

/* NUS service UUID type */                                                                
#define NUS_SERVICE_UUID_TYPE   		BLE_UUID_TYPE_VENDOR_BEGIN      /* UUID type for the Nordic UART Service (vendor specific). */
                                                                
/* Scanning parameters */                                                            
#define SCAN_INTERVAL           		0x00A0                          /* Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             		0x0050                          /* Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             		1                               /* If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          		0                               /* If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            		0x0000                          /* */
       
/* GAP connection parameters */                                                         
#define MIN_CONNECTION_INTERVAL 		MSEC_TO_UNITS(20, UNIT_1_25_MS) /* Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL 		MSEC_TO_UNITS(75, UNIT_1_25_MS) /* Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           		0                               /* Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     		MSEC_TO_UNITS(4000, UNIT_10_MS)	/* Determines supervision time-out in units of 10 millisecond. */

/* UUID fields sizes */                                                 
#define UUID16_SIZE             		2                            
#define UUID32_SIZE	        			4                              
#define UUID128_SIZE            		16                          

/* Device list length */
#define DEVICE_LIST_LENGTH				CONN_MAX_NUM_DEVICES

/* Number of maximum simultaneous connections */
#define NUM_OF_CONNECTIONS				DEVICE_LIST_LENGTH	/* ATTENTION: num of simultaneous connection is the same of device list length */

/* Max data length through NUS service */
#define MAX_DATA_LENGTH					BLE_NUS_MAX_DATA_LEN

/* connection pin number */
#define CONN_PIN_NUMBER 				21	/* P0.21 */

/* connection pin state as connected */
#define CONNECTED_PIN_STATE				1	/* high */

/* connection pin state as disconnected */
#define DISCONNECTED_PIN_STATE			0	/* low */




/* ---------- Local structures definition ---------- */

/* Device list length */
typedef struct
{
	ble_gap_addr_t gap_addr;
	uint8_t name[21];
	uint8_t name_length;
} device_info;




/* ---------- Local variables ---------- */

/* Structure to identify the Nordic UART central service. */
static ble_nus_c_t              		m_ble_nus_c;

/* Database discovery structure */
static ble_db_discovery_t       		m_ble_db_discovery;

/* Connection parameters requested for connection. */
static const ble_gap_conn_params_t m_connection_param =
{
	(uint16_t)MIN_CONNECTION_INTERVAL,   /* Minimum connection */
	(uint16_t)MAX_CONNECTION_INTERVAL,   /* Maximum connection */
	0,                                   /* Slave latency */
	(uint16_t)SUPERVISION_TIMEOUT        /* Supervision time-out */
};

/* Parameters used when scanning. */
static const ble_gap_scan_params_t m_scan_params = 
{
	.active      = SCAN_ACTIVE,
	.selective   = SCAN_SELECTIVE,
	.p_whitelist = NULL,
	.interval    = SCAN_INTERVAL,
	.window      = SCAN_WINDOW,
	.timeout     = SCAN_TIMEOUT
};

/* NUS uuid */
static const ble_uuid_t m_nus_uuid = 
{
	.uuid = BLE_UUID_NUS_SERVICE,
	.type = NUS_SERVICE_UUID_TYPE	
};

/* Found devices list */
static device_info found_devices[DEVICE_LIST_LENGTH];

/* Found devices list index */
static uint8_t devices_list_index = 0;

/* Index of pending NUS connection in the list */
static uint8_t pending_nus_conn_index = 0xFF;

static uint16_t active_conn_handles[NUM_OF_CONNECTIONS];




/* ---------- Local functions prototypes ---------- */
static void target_name_if_present(const ble_gap_evt_adv_report_t *, uint8_t *, uint8_t *);
static bool is_uuid_present(const ble_uuid_t *, const ble_gap_evt_adv_report_t *);
static uint8_t get_devices_list_id(ble_gap_addr_t);
static void on_ble_evt(ble_evt_t *);
static void ble_nus_c_evt_handler(ble_nus_c_t *, const ble_nus_c_evt_t *);
static void ble_c_evt_dispatch(ble_evt_t *);
static void ble_stack_init(bool);
static void nus_c_init(void);




/* ------------ Exported functions implementation --------------- */

/* Function to init the connection manager */
void conn_init(void)
{
    uint32_t err_code;
	int i;

	/* init "connection" pin */
	nrf_gpio_pin_dir_set(CONN_PIN_NUMBER, NRF_GPIO_PIN_DIR_OUTPUT);
   
	/* init BLE stack */
	ble_stack_init(true);

	/* init database discovery */
	err_code = ble_db_discovery_init();
	APP_ERROR_CHECK(err_code);
	
	/* init NUS client service */
	nus_c_init();

	/* init connection handle array */
	for(i=0; i<NUM_OF_CONNECTIONS; i++)
	{
		active_conn_handles[i] = BLE_CONN_HANDLE_INVALID;
	}
}


/* Function to start scanning devices */
void conn_start_scan(void)
{
	uint32_t err_code;

	/* reset device list index */
	devices_list_index = 0;

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
}


/* Function to start scanning devices */
void conn_stop_scan(void)
{
	/* stop scanning */
	sd_ble_gap_scan_stop();
}


/* Function to switch connection index */
bool conn_switch_conn(uint8_t active_conn_index)
{
	bool success = false; 

	/* if connection index is valid */
	if(active_conn_index < NUM_OF_CONNECTIONS)
	{
		/* if selected connection handle is valid */
		if(active_conn_handles[active_conn_index] != BLE_CONN_HANDLE_INVALID)
		{
			/* set related connection handle to the selected one */
			m_ble_nus_c.conn_handle = active_conn_handles[active_conn_index];
			/* valid operation */
			success = true;
		}
		else
		{
			/* invalid handle: keep the current one */
		}
	}
	else
	{
		/* invalid parameter */
	}

	return success;
}


/* Function to request a connection to a previously found device */
bool conn_request_connection(uint8_t device_index)
{
	uint32_t err_code;
	bool success = false;

	/* if requested index (from ascii to int) is lower than num of devices */
	if(device_index < devices_list_index)
	{
		/* store pending connection index */
		pending_nus_conn_index = device_index;

		/* request a connection */
		err_code = sd_ble_gap_connect(&found_devices[device_index].gap_addr,
		                              &m_scan_params,
		                              &m_connection_param);
		if (err_code == NRF_SUCCESS)
		{	
			/* success */
			success = true;
		}
		else
		{
			/* connection fail */
		}
	}
	else
	{
		/* invalid device index */
	}

	return success;
}


/* Function to drop an ongoing connection */
bool conn_drop_connection(uint8_t nus_conn_index)
{
	uint32_t err_code;
	bool success = false;

	/* check connection index validity */
	if(nus_conn_index < NUM_OF_CONNECTIONS)
	{
		/* store pending disconnection index */
		pending_nus_conn_index = nus_conn_index;

		/* get related connection handle */
		m_ble_nus_c.conn_handle = active_conn_handles[nus_conn_index];

		/* try to disconnect */
		err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
		    APP_ERROR_CHECK(err_code);
			/* success */
			success = true;
		}
		else
		{
			/* fail to disconnect */ 
		}
	}
	else
	{
		/* invalid connection index: do nothing */
	}

	return success;
}


/* Function to send data through NUS central service. */
// TODO: consider to implement a timeout
void conn_send_data_nus(uint8_t *p_data, uint16_t data_length)
{
	/* if data length is greater the max allowed value */
	if(data_length > MAX_DATA_LENGTH)
	{
		/* limit data length and send anyway */
		data_length = MAX_DATA_LENGTH;
	}
	else
	{
		/* length is valid, do nothing */
	}

	/* if current connection handle is valid */
	if(m_ble_nus_c.conn_handle != BLE_CONN_HANDLE_INVALID)
	{
		/* send data but do not send termination char */
		while (ble_nus_c_string_send(&m_ble_nus_c, p_data, data_length) != NRF_SUCCESS)			
		{
			/* repeat until sent */
		}
	}
	else
	{
		/* do not send anything. TODO: consider to return a fail flag value */
	}
}


/* send a number of found devices */
void conn_send_num_found_devices(void)
{
	char string[4];
	/* build the string */
	string[0] = 'O';
	string[1] = 'K';
	string[2] = '-';
	string[3] = (char)(0x30 | devices_list_index);	/* ascii conversion */
	/* send the string */
	uart_send_string((uint8_t *)string, 4);
	app_uart_put('.');
}


/* send a found device info according to passed index */
void conn_send_found_device(uint8_t found_dev_index)
{
	char ascii_address[12];

	uart_send_string((uint8_t *)"OK-", 3);
	/* check required device index */
	if(found_dev_index < devices_list_index)
	{
		/* connvert and send the device address */
		util_address_to_string(found_devices[found_dev_index].gap_addr.addr, ascii_address);
		uart_send_string((uint8_t *)ascii_address, 12);
		app_uart_put('-');
		/* send device name */
		if(found_devices[found_dev_index].name_length > 0)
		{
			uart_send_string(found_devices[found_dev_index].name, found_devices[found_dev_index].name_length-1);
		}
		else
		{
			uart_send_string((uint8_t *)"Unknown", 7);
		}
	}
	else
	{
		uart_send_string((uint8_t *)"ERROR", 5);
	}
	app_uart_put('.');
}




/* ---------- Local functions implementation ---------- */


/* CENTRAL: Reads an advertising report and checks if the target name is present.
   Parameters: p_adv_report  Pointer to the advertisement report.
   Return: true if the target name is present in the advertisement report. Otherwise false  
*/
static void target_name_if_present(const ble_gap_evt_adv_report_t *p_adv_report, uint8_t *name_string, uint8_t *name_length)
{
    uint32_t index = 0;
    uint8_t *p_data = (uint8_t *)p_adv_report->data;
    
    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];

		/* check target name */
		if ( field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME )
		{
			/* copy name string */
			strncpy((char *)name_string, (char *)(&p_data[index + 2]), (size_t)field_length);

			/* copy name length */
			*name_length = field_length;
			
			/* Name found */
		  	return;
		}

		/* increment index */
        index += field_length + 1;
    }
}


/* CENTRAL: Reads an advertising report and checks if a uuid is present in the service list.
   The function is able to search for 16-bit, 32-bit and 128-bit service uuids. 
   Parameters: p_target_uuid The uuid to search fir
               p_adv_report  Pointer to the advertisement report.
   Return: true if the UUID is present in the advertisement report. Otherwise false  
*/
static bool is_uuid_present(const ble_uuid_t *p_target_uuid, 
                            const ble_gap_evt_adv_report_t *p_adv_report)
{
    uint32_t err_code;
    uint32_t index = 0;
    uint8_t *p_data = (uint8_t *)p_adv_report->data;
    ble_uuid_t extracted_uuid;
    
    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];

		/* check UUID field */
        if ( (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
           || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE) )
        {
            for (uint32_t u_index = 0; u_index < (field_length/UUID16_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(  UUID16_SIZE, 
                                                &p_data[u_index * UUID16_SIZE + index + 2], 
                                                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }
        else if ( (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE) )
        {
            for (uint32_t u_index = 0; u_index < (field_length/UUID32_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(UUID16_SIZE, 
                &p_data[u_index * UUID32_SIZE + index + 2], 
                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }
        else if ( (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE) )
        {
            err_code = sd_ble_uuid_decode(UUID128_SIZE, 
                                          &p_data[index + 2], 
                                          &extracted_uuid);
            if (err_code == NRF_SUCCESS)
            {
                if ((extracted_uuid.uuid == p_target_uuid->uuid)
                    && (extracted_uuid.type == p_target_uuid->type))
                {
                    return true;
                }
            }
        }
		else
		{
			/* do nothing */
		}

		/* increment index */
        index += field_length + 1;
    }

    return false;
}


/* CENTRAL: Function for getting the device id of the found devices list.
   Parameters: gap_addr   Bluetooth low energy address.
*/
static uint8_t get_devices_list_id(ble_gap_addr_t gap_addr)
{
	uint8_t device_index = 0;

	/* check address type - TODO */
	//if(gap_addr.addr_type == BLE_GAP_ADDR_TYPE_PUBLIC)

	/* seek address in found device list */
	while( (0 != strncmp((const char *)(gap_addr.addr), (const char *)(found_devices[device_index].gap_addr.addr), (size_t)6))
	&&	   (device_index < devices_list_index) )
	{
		device_index++;
	}

	/* if the address has not been found in the list */
	if(device_index >= devices_list_index)
	{
		/* return "not found" value */
		device_index = 0xFF;
	}

	return device_index;
}


/* Function for handling the Application's BLE Stack events.
   Parameters: p_ble_evt   Bluetooth stack event.
*/
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
	uint8_t index = 0;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;	
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t *p_adv_report = &p_gap_evt->params.adv_report;

			index = get_devices_list_id(p_adv_report->peer_addr);
			/* id device has been already found or list is full */
			if( index != 0xFF)
			{
				/* device already found */
				/* new adv update */
				target_name_if_present(p_adv_report, found_devices[index].name, &found_devices[index].name_length);
			}
			else
			{
				/* if UUID is present */
				if (is_uuid_present(&m_nus_uuid, p_adv_report))
			    {
					/* get last free index */
					index = devices_list_index;
					/* increment last free index */
					devices_list_index++;
					/* insert the new device into the list: copy address */
					strncpy((char *)(found_devices[index].gap_addr.addr), (char *)(p_adv_report->peer_addr.addr), (size_t)6);
					/* copy address type */
					found_devices[index].gap_addr.addr_type = p_adv_report->peer_addr.addr_type;
				}
			}
            break;
        }
        case BLE_GAP_EVT_CONNECTED:
		{
			/* if pending connection index of central role is valid */
			if(pending_nus_conn_index < NUM_OF_CONNECTIONS)
			{
				/* store related connection handle */
				active_conn_handles[pending_nus_conn_index] = p_ble_evt->evt.gap_evt.conn_handle;
				/* set current handle as this one */
				m_ble_nus_c.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

				/* reset pending NUS connection index */
				pending_nus_conn_index = 0xFF;
				/* reset uart */
				uart_reset();
				/* set "connection" pin as connected */
				nrf_gpio_pin_write(CONN_PIN_NUMBER, CONNECTED_PIN_STATE);
				/* send confirmation string */
				uart_send_string((uint8_t *)"OK.", 3);

				/* start discovery of services. The NUS Client waits for a discovery result */
				err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
				APP_ERROR_CHECK(err_code);					
			}
			else
			{
				/* internal error: do nothing */
			}
			
            break;
		}
		case BLE_GAP_EVT_DISCONNECTED:
		{
			/* it should not pass here */
            break;
    	}
        case BLE_GAP_EVT_TIMEOUT:
		{
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                /* scan timed out */
				uart_send_string((uint8_t *)"TIMEOUT.", 8);
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                /* connection request timed out: do nothing */
            }
			else
			{
				/* do nothing */
			}
            break;
        }
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		{
            /* ATTENTION: Pairing not supported at the moment */
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
    	}
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
		{
            /* Accepting parameters requested by peer. */
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;
		}
        default:
            break;
    }
}


/* CENTRAL: Callback handling NUS Client events.
   Handling events from the ble_nus_c module.
   This function is called to notify the application of NUS client events.
   Parameters: p_ble_nus_c   NUS Client Handle. This identifies the NUS client
               p_ble_nus_evt Pointer to the NUS Client event.
*/
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
{
    uint32_t err_code;
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_FOUND_NUS_TX_CHARACTERISTIC:
		{
            /* TX characteristic found */
            break;
        }
        case BLE_NUS_C_EVT_FOUND_NUS_RX_CHARACTERISTIC:
		{
	    	/* RX characteristic found: enable notification on that */
            err_code = ble_nus_c_rx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            break;
        }
        case BLE_NUS_C_EVT_NUS_RX_EVT:
		{
			/* send received data from NUS to uart interface */
            for (uint32_t i = 0; i < p_ble_nus_evt->data_len; i++)
            {
                while(app_uart_put( p_ble_nus_evt->p_data[i]) != NRF_SUCCESS);
            }
			app_uart_put('.');
            break;
        }
        case BLE_NUS_C_EVT_DISCONNECTED:
		{
			/* clear related connection handle */
			active_conn_handles[pending_nus_conn_index] = BLE_CONN_HANDLE_INVALID;

			/* reset pending NUS connection index */
			pending_nus_conn_index = 0xFF;
			/* reset uart */
			uart_reset();
			/* set "connection" pin as disconnected */
			nrf_gpio_pin_write(CONN_PIN_NUMBER, DISCONNECTED_PIN_STATE);
			/* send confirmation string */
			uart_send_string((uint8_t *)"OK.", 3);
		
            break;
		}
    }
}


/* CENTRAL: Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
   This function is called from the scheduler in the main loop after a BLE stack event has
   been received.
   Parameters: p_ble_evt   Bluetooth stack event.
*/
static void ble_c_evt_dispatch(ble_evt_t * p_ble_evt)
{
	uint8_t i = 0;

    on_ble_evt(p_ble_evt);  
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);

	/* seek connection index from the received connection handle */
	while((active_conn_handles[i] != p_ble_evt->evt.gap_evt.conn_handle)
	&&    (i < NUM_OF_CONNECTIONS))
	{
		i++;
	}
	if(i < NUM_OF_CONNECTIONS)
	{
		/* get related connection handle */
		m_ble_nus_c.conn_handle = active_conn_handles[i];
		/* connection index found: manage NUS client event */
		ble_nus_c_on_ble_evt(&m_ble_nus_c, p_ble_evt);
	}
	else
	{
		/* connection index not found: do nothing */
	}
}


/* Function for initializing the BLE stack.
   Initializes the SoftDevice and the BLE event interrupt.
*/
static void ble_stack_init(bool is_central)
{
    uint32_t err_code;

    /* Initialize the SoftDevice handler module. */
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    /* Enable BLE stack. */
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
	ble_enable_params.gatts_enable_params.service_changed = false;

	/* enable BLE */
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

	/* Register with the SoftDevice handler module for BLE events in central role */
	err_code = softdevice_ble_evt_handler_set(ble_c_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}


/* Function for initializing the NUS Client. */
static void nus_c_init(void)
{
    uint32_t err_code;
    ble_nus_c_init_t nus_c_init_t;
    
	/* same init structure for all clients */
    nus_c_init_t.evt_handler = ble_nus_c_evt_handler;

	/* init NUS client */
	err_code = ble_nus_c_init(&m_ble_nus_c, &nus_c_init_t);
	APP_ERROR_CHECK(err_code);
}




/* End of file */


