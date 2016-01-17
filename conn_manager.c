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


/* TODO: check passed drop connection index -> remember connected indexes */


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
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_nus_c.h"	/* NUS central */
#include "ble_nus.h"	/* NUS peripheral */
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

/* Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */
#define IS_SRVC_CHANGED_CHARACT_PRESENT	0  
                                         
/* Name of device. Will be included in the advertising data. */
#define DEVICE_NAME            	"ble_led_cube"     

/* Default target device name and length */
#define TARGET_DEV_NAME      	"ble_led_ctrl_01"  
#define TARGET_DEV_LENGTH      	15 

/* NUS service UUID type */                                                                
#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN      /**< UUID type for the Nordic UART Service (vendor specific). */
                                                                
/* Scanning parameters */                                                            
#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< */
       
/* GAP connection parameters */                                                         
#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                               /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Determines supervision time-out in units of 10 millisecond. */

/* The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_INTERVAL                64      

/* The advertising timeout (in units of seconds). */                                    
#define APP_ADV_TIMEOUT_IN_SECONDS      10    
             
/* Value of the RTC1 PRESCALER register. */   
#define APP_TIMER_PRESCALER             0   

/* Maximum number of simultaneously created timers. */                                                   
#define APP_TIMER_MAX_TIMERS            2	

/* Size of timer operation queues. */				    
#define APP_TIMER_OP_QUEUE_SIZE         4   
  
/* Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */          
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) 

/* Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */ 
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) 

/* Number of attempts before giving up the connection parameter negotiation. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3    

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
	uint8_t valid_target;
} device_info;




/* ---------- Local variables ---------- */

/* Structure to identify the Nordic UART peripheral service. */
static ble_nus_t                        m_ble_nus_p;    

/* Handle of the current connection. */
static uint16_t                         m_p_conn_handle = BLE_CONN_HANDLE_INVALID;    

/* Universally unique service identifier. */
static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  

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

/* Flag to indicate if device is connected */
static conn_ke_state connection_state = CONN_KE_NOT_INIT;




/* ---------- Local functions prototypes ---------- */

static void gap_params_init(void);
static void nus_p_init(void);
static void on_conn_params_evt(ble_conn_params_evt_t *);
static void conn_params_error_handler(uint32_t);
static void conn_params_init(void);
static void on_adv_evt(ble_adv_evt_t);
static void advertising_init(void);
static void ble_nus_p_data_handler(ble_nus_t *, uint8_t *, uint16_t);
static void ble_p_evt_dispatch(ble_evt_t *);

static bool is_target_name_present(const ble_gap_evt_adv_report_t *, uint8_t *, uint8_t *);
static bool is_uuid_present(const ble_uuid_t *, const ble_gap_evt_adv_report_t *);
static uint8_t get_devices_list_id(ble_gap_addr_t);
static void on_ble_evt(ble_evt_t *);
static void ble_nus_c_evt_handler(ble_nus_c_t *, const ble_nus_c_evt_t *);
static void ble_c_evt_dispatch(ble_evt_t *);
static void ble_stack_init(bool);
static void nus_c_init(void);




/* ------------ Exported functions implementation --------------- */

/* Function to init the connection manager */
void conn_init(bool is_central)
{
    uint32_t err_code;

	/* init "connection" pin */
	nrf_gpio_pin_dir_set(CONN_PIN_NUMBER, NRF_GPIO_PIN_DIR_OUTPUT);
   
	/* if central role */
	if(is_central == true)
	{
		/* init BLE stack */
		ble_stack_init(true);

		/* init database discovery */
		err_code = ble_db_discovery_init();
		APP_ERROR_CHECK(err_code);
		
		/* init NUS client service */
		nus_c_init();

		/* connection manager initialised as central and not connected */
		connection_state = CONN_KE_INIT_C;
	}
	/* else if peripheral or any other invalid boolean values */
	else
	{
		/* init BLE stack */
		ble_stack_init(false);
		/* init GAP parameters */
		gap_params_init();
		/* init NUS peripheral service */
		nus_p_init();
		/* init advertising */
		advertising_init();
		/* init connection parameters */
		conn_params_init();

		/* start advertising */
		err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
		APP_ERROR_CHECK(err_code);

		/* connection manager initialised as peripheral and not connected */
		connection_state = CONN_KE_INIT_P;
	}
}


/* Function to get the current connection status */
conn_ke_state conn_get_state(void)
{
	return connection_state;
}


/* Function to start scanning devices */
void conn_start_scan(void)
{
	uint32_t err_code;

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);

	/* reset device list index */
	devices_list_index = 0;
}


/* Function to start scanning devices */
void conn_stop_scan(void)
{
	/* stop scanning */
	sd_ble_gap_scan_stop();
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


/* Function to send data through NUS central service. 
   TODO: consider to implement a timeout */
void conn_send_data_c_nus(uint8_t nus_conn_index, uint8_t *p_data, uint16_t data_length)
{
	/* check connection index validity */
	if(nus_conn_index < NUM_OF_CONNECTIONS)
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

		/* get related connection handle */
		m_ble_nus_c.conn_handle = active_conn_handles[nus_conn_index];

		/* send data but do not send termination char */
		while (ble_nus_c_string_send(&m_ble_nus_c, p_data, data_length) != NRF_SUCCESS)			
		{
			/* repeat until sent */
		}
	}
	else
	{
		/* invalid connection index: do nothing */
	}
}


/* Function to send data through NUS peripheral service. 
   TODO: consider to implement a timeout */
void conn_send_data_p_nus(uint8_t *p_data, uint16_t data_length)
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

	/* send data but do not send termination char */
	while (ble_nus_string_send(&m_ble_nus_p, p_data, data_length) != NRF_SUCCESS)			
	{
		/* repeat until sent */
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


/* PERIPHERAL: Function for the GAP initialization.
   This function will set up all the necessary GAP (Generic Access Profile) parameters of 
   the device. It also sets the permissions and appearance */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONNECTION_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONNECTION_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = SUPERVISION_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/* PERIPHERAL: Function for handling an event from the Connection Parameters Module.
   This function will be called for all events in the Connection Parameters Module
   which are passed to the application.
   All this function does is to disconnect. This could have been done by simply setting
   the disconnect_on_fail config parameter, but instead we use the event handler
   mechanism to demonstrate its use */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_p_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/* PERIPHERAL: Function for handling errors from the Connection Parameters module.
   Error code containing information about what went wrong */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/* PERIPHERAL: Function for initializing the Connection Parameters module */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/* PERIPHERAL: Function for handling advertising events.
   This function will be called for advertising events which are passed to the application */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
	    	/* do nothing */
            break;
        case BLE_ADV_EVT_IDLE:
	    	/* start advertising again */
	    	err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}


/* PERIPHERAL: Function for initializing the Advertising functionality */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    /* prepare advertising data struct to pass into ble_advertising_init */
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

	/* prepare scan response data struct to pass into ble_advertising_init */
    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

	/* prepare scan response data struct to pass into ble_advertising_init */
    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

	/* init advertising */
    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/* CENTRAL: Reads an advertising report and checks if the target name is present.
   Parameters: p_adv_report  Pointer to the advertisement report.
   Return: true if the target name is present in the advertisement report. Otherwise false  
*/
static bool is_target_name_present(const ble_gap_evt_adv_report_t *p_adv_report, uint8_t *name_string, uint8_t *name_length)
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
		  	return true;
	   	    
		}

        index += field_length + 1;
    }

    return false;
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

uint8_t pippo = 0;
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
			}
			else
			{
				/* get last free index */
				index = devices_list_index;
				/* increment last free index */
				devices_list_index++;
				/* insert the new device into the list */
				/* copy address */
				strncpy((char *)(found_devices[index].gap_addr.addr), (char *)(p_adv_report->peer_addr.addr), (size_t)6);
				/* copy address type */
				found_devices[index].gap_addr.addr_type = p_adv_report->peer_addr.addr_type;
			
				if (is_target_name_present(p_adv_report, found_devices[index].name, &found_devices[index].name_length))
				{
					found_devices[index].valid_target |= 1;
				}	
			
				if (is_uuid_present(&m_nus_uuid, p_adv_report))
			    {
					found_devices[index].valid_target |= 2;
				}
			}
            break;
        }
        case BLE_GAP_EVT_CONNECTED:
		{
			/* if role is central */
			if((connection_state == CONN_KE_INIT_C)
			|| (connection_state == CONN_KE_CONNECTED_C))
			{
				/* if pending connection index is valid */
				if(pending_nus_conn_index < NUM_OF_CONNECTIONS)
				{
					uart_send_string((uint8_t *)"CONNECTED.", 10);
					/* device is connected as central to a peripheral */
					connection_state = CONN_KE_CONNECTED_C;
					/* set "connection" pin as connected */
					nrf_gpio_pin_write(CONN_PIN_NUMBER, CONNECTED_PIN_STATE);
					/* reset uart */
					uart_reset();

					/* store related connection handle */
					active_conn_handles[pending_nus_conn_index] = p_ble_evt->evt.gap_evt.conn_handle;
					/* set current handle as this one */
				    m_ble_nus_c.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

					/* reset pending NUS connection index */
					pending_nus_conn_index = 0xFF;
				    /* start discovery of services. The NUS Client waits for a discovery result */
				    err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
				    APP_ERROR_CHECK(err_code);
				}
				else
				{
					/* internal error: do nothing */
				}
			}
			/* else if role is peripheral */
			else if(connection_state == CONN_KE_INIT_P)
			{
				/* device is connected as peripheral to a central */
				connection_state = CONN_KE_CONNECTED_P;
				/* store connection handle */
				m_p_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;;
				/* set "connection" pin as connected */
				nrf_gpio_pin_write(CONN_PIN_NUMBER, CONNECTED_PIN_STATE);
			}
			else
			{
				/* do nothing */
			}
            break;
		}
		case BLE_GAP_EVT_DISCONNECTED:
		{
			/* if role is peripheral */
			if(connection_state == CONN_KE_CONNECTED_P)
			{
				/* device is not connected now */
				connection_state = CONN_KE_INIT_P;
				/* restore connection handle to invalid */
				m_p_conn_handle = BLE_CONN_HANDLE_INVALID;
				/* set "connection" pin as disconnected */
				nrf_gpio_pin_write(CONN_PIN_NUMBER, DISCONNECTED_PIN_STATE);
			}
			else
			{
				/* do nothing at the moment */
			}
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
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		{
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_p_conn_handle, NULL, 0, 0);
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
            /* TX characteristic found */
            break;
        
        case BLE_NUS_C_EVT_FOUND_NUS_RX_CHARACTERISTIC:
	    	/* RX characteristic found: enable notification on that */
            err_code = ble_nus_c_rx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            break;
        
        case BLE_NUS_C_EVT_NUS_RX_EVT:
			/* send received data from NUS to uart interface */
            for (uint32_t i = 0; i < p_ble_nus_evt->data_len; i++)
            {
                while(app_uart_put( p_ble_nus_evt->p_data[i]) != NRF_SUCCESS);
            }
			app_uart_put('.');
            break;
        
        case BLE_NUS_C_EVT_DISCONNECTED:
			/* TODO: consider to clear related connection handle */
			/* device is not connected as central anymore */
			connection_state = CONN_KE_INIT_C;
			/* reset uart */
			uart_reset();
			uart_send_string((uint8_t *)"DISCONNECTED.", 13);
			/* set "connection" pin as disconnected */
			nrf_gpio_pin_write(CONN_PIN_NUMBER, DISCONNECTED_PIN_STATE);
            break;
    }
}


/* Function for handling the data from the Nordic UART Service.
   This function will process the data received from the Nordic UART BLE Service and send
   it to the UART module */
static void ble_nus_p_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	/* send received data from NUS to uart interface */
	for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
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


/* PERIPHERAL: Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
   event handler.
   This function is called from the S110 SoftDevice event interrupt handler after a S110 
   SoftDevice event has been received */
static void ble_p_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_ble_nus_p, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);  
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

	/* if central role is required */
	if(is_central == true)
	{
		ble_enable_params.gatts_enable_params.service_changed = false;
	}
	/* else if peripheral role or other invalid values */
	else
	{
		ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
	}

	/* enable BLE */
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

	/* if central role is required */
	if(is_central == true)
	{
		/* Register with the SoftDevice handler module for BLE events in central role */
		err_code = softdevice_ble_evt_handler_set(ble_c_evt_dispatch);
		APP_ERROR_CHECK(err_code);
	}
	/* else if peripheral role or other invalid values */
	else
	{
		/* Register with the SoftDevice handler module for BLE events in peripheral role */
		err_code = softdevice_ble_evt_handler_set(ble_p_evt_dispatch);
		APP_ERROR_CHECK(err_code);
	}
}


/* PERIPHERAL: Function for initializing services that will be used by the application */
static void nus_p_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_p_init_t;
    
    memset(&nus_p_init_t, 0, sizeof(nus_p_init_t));

    nus_p_init_t.data_handler = ble_nus_p_data_handler;
    
    err_code = ble_nus_init(&m_ble_nus_p, &nus_p_init_t);
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



