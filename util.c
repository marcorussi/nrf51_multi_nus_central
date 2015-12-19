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

/* header file */
#include "util.h"




/* ------------ Exported functions implementation --------------- */

/* Function to convert a Bluetooth address to a string */
void util_address_to_string(uint8_t *uint8_addr, char *ascii_addr)
{
	uint8_t index;
	char ascii_value;

	/* convert all digits */
	for(index = 0; index < 12; index++)
	{
		/* get next uint8_t digit from the numeric address */
		ascii_value = (char)(((uint8_addr[(index / 2)] >> ((index % 2) * 4))) & 0x0F);

		/* convert it to the corresponding ascii digit */
		if( (ascii_value >= 0x00)
		&&  (ascii_value <= 0x09) )
		{
			ascii_value |= 0x30;
		}
		else if( (ascii_value >= 0x0A)
			 &&  (ascii_value <= 0x0F) )
		{
			ascii_value += 0x37;
		}
		else
		{
			/* invalid bluetooth address digit */
			/* TODO - consider to return an error code */
			/* put an 'X' at the moment */
			ascii_value = 'X';
		}

		/* store it into the ascii address array */
		/* the address is in little indian format */
		ascii_addr[(11 - index)] = ascii_value;
	}
}




/* End of file */



