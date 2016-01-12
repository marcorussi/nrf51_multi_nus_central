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




/* ---------- Exported typedefs -------------- */

/* Connection status typedef */
typedef enum
{
	CONN_KE_NOT_INIT,
	CONN_KE_INIT_P,
	CONN_KE_INIT_C,
	CONN_KE_CONNECTED_P,
	CONN_KE_CONNECTED_C,
	CONN_KE_UNKNOWN
} conn_ke_state;




/* ---------- Exported functions prototypes ----------- */

extern void conn_init(bool);
extern conn_ke_state conn_get_state(void);
extern void conn_start_scan(void);
extern void conn_stop_scan(void);
extern bool conn_request_connection(uint8_t);
extern bool conn_drop_connection(void);
extern void conn_send_data_c_nus(uint8_t *, uint16_t);
extern void conn_send_data_p_nus(uint8_t *, uint16_t);
extern void conn_send_num_found_devices(void);
extern void conn_send_found_device(uint8_t);
extern void conn_send_found_devices(void);





/* End of file */




