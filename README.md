# nrf51_nus_central
A multilink NUS central role firmware with Nordic NUS service through UART AT commands. Developed under Ubuntu environment using a nrf51 PCA10028 development kit. The firmware is based on S130 from Nordic SDK 10.x.x.

This software allows to connect to several devices simultaneously. It is possible to switch from an established connection to another one for sending data to any device at any time. In fact, once an established connection can be considered as a data link between the two devices where data pass through. A specific command allows to switch to any of the existing data links and so next oncoming data will be sent through the last switched data link.

The module manages oncoming serial data according to two modes:
- **configuration mode**: the module parses received data for detecting an eventual valid command and then executes it if found;
- **data mode**: any received data are sent through the last indexed device with an established connection.

For easily detecting an end of string in the firmware and on the host side, **commands and data must be terminated by "." character in order to be managed by the module according to current mode. Similarly, all responses are terminated by the same "." character.**.

In configuration mode the following commands are valid:
- "AT?.": query commands parser. Responses:
  - "OK.": commands parser is running properly.
- "AT+SCAN+.": start a scan of devices with UUID service. Responses:
  - "OK.": scan started successfully.
- "AT+SCAN-.": stop an eventual ongoing scan. Responses:
  - "OK-*n*.": scan stopped successfully. The *n* value represents the number of found devices with UUID service.
- "AT+FOUND=*i*.": get address and name of found device at index *i*. The index *i* must be between 0 and *n*-1 (number of found devices menus 1) previously obtained by AT+SCAN- command. Responses:
  - "OK-*address*-*name*.": a valid device index has been requested. The *address* field is the 12 characters bluetooth address and the *name* field is the device name. If name was not found then *name* field is substituted by "Unknown" string;
  - "ERROR.": a not valid index has been requested. This means that 0 devices have been found or requested index is greater than *n*-1 (number of found devices menus 1).
- "AT+CONN=*i*.": request a connection of device at index *i*. Responses:
  - "WAIT.": required connection has been successfully requested. In case of success a "OK." response will follow;
  - "OK.": a valid connection has been established. This response follows a "WAIT." response. After this reponse the module enters into data mode and so no more commands will be parsed. Any received data will be sent to the just connected device;
  - "ERROR.": required connection request failed. This meand that the index *i* is not valid or related connection request failed.
- "AT+SWITCH=*i*.": request to switch next data to device at index *i*. This implies that a previous successfull connection is established with that device. Responses:
  - "OK.": switch request performed successfully;
  - "ERROR.": switch request failed.
- "AT+DROP=*i*.": drop an established connection with device at index *i*. This implies that a previous successfull connection is established with that device. Responses:
  - "WAIT.": a connection drop request has been successfully requested. In case of success a "OK." response will follow;
  - "OK.": connection has been dropped successfully. This response follows a "WAIT." response;
  - "ERROR.": required connection drop failed. This meand that the index *i* is not valid or related drop request failed.
- "AT+AUTO.": enter into data mode with the last valid device index. This implies that an escape character has been previously sent during the data mode of an established connection. Responses:
  - "OK.": entered successfully in data mode.
- "AT+RESET.": perform a software reset of the module. Responses:
- "OK.": reset request accepted. This response is immediately followed by a software reset.

In data mode any received data unless the "." terminator is sent to the last indexed connected device. The special character for escaping from data mode and so entering into configuration mode is "*".

**The UART TX and RX lines are respectively P0.12 and P0.13.**

**In addition, find my related Arduino host application here:** https://github.com/marcorussi/arduino_nus_host



**Install**

Download Segger JLink tool from https://www.segger.com/jlink-software.html. Unpack it and move it to /opt directory.
Download the Nordic SDK from http://developer.nordicsemi.com/nRF5_SDK/nRF51_SDK_v10.x.x/. Unpack it and move it to /opt directory.
Clone my nrfjprog.sh repo in /opt directory by running:

    $ cd [your path]/opt
    $ git clone https://github.com/marcorussi/nrfjprog.git

Clone this repo in your projects directory:

    $ git clone https://github.com/marcorussi/nrf51_nus_central.git
    $ cd nrf51_nus_central
    $ gedit Makefile

Verify and modify following names and paths as required according to your ARM GCC toolchain:

```
PROJECT_NAME := nrf51_multi_nus_central
NRFJPROG_PATH := /opt/nrfjprog
SDK_PATH := /opt/nRF51_SDK_10.0.0_dc26b5e
LINKER_SCRIPT := multi_nus_c_nrf51.ld
GNU_INSTALL_ROOT := /home/marco/ARMToolchain/gcc-arm-none-eabi-4_9-2015q2
GNU_VERSION := 4.9.3
GNU_PREFIX := arm-none-eabi
```



**Flash**

Connect your nrf51 Dev. Kit, make and flash it:
 
    $ make
    $ make flash_softdevice (for the first time only)
    $ make flash

You can erase the whole flash memory by running:

    $ make erase



