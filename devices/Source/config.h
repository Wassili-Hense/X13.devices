/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// Global configuration settings

#ifndef _CONFIG_H
#define _CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// MQTT-SN Section
#define MQTTSN_MSG_SIZE         48      // Size of payload(base for all buffers)
#define MQTTSN_USE_DHCP         1       // Use Automatic address resolution

// Object Dictionary
#define OD_MAX_INDEX_LIST       16      // Size of identificators list
#define OD_DEV_SWVERSH          '3'     // Software Version
#define OD_DEV_SWVERSM          '0'
#define OD_DEV_SWVERSL          '1'

#define POLL_TMR_FREQ           100     // System Tick Period

#define UART_BaudRate           38400
#define RF_BASE_FREQ            868300000UL

//Optional modules
//#define EXTDIO_USED             1       // Use DIO
//#define EXTPWM_USED             1       // Use PWM
//#define EXTAIN_USED             1       // Use Analog inputs
//#define EXTTWI_USED             1       // Use TWI/I2C Devices
//#define EXTSER_USED             1       // Use Serial Port

//#define DIAG_USED               1       // Enable diagnostic messages

// Include Hardware definitions
// Atmel
// ATM328P
#if     (defined CFG_A1Cn12)
#include "HAL/HWconfig_A1Cn12.h"
#elif   (defined CFG_A1En12)
#include "HAL/HWconfig_A1En12.h"
#elif   (defined CFG_A1ES12)
#include "HAL/HWconfig_A1ES12.h"
#elif   (defined CFG_A1SC12)
#include "HAL/HWconfig_A1SC12.h"
#elif   (defined CFG_A1Sn12)
#include "HAL/HWconfig_A1Sn12.h"
// ATM1284P
#elif   (defined CFG_A3SC12)
#include "HAL/HWconfig_A3SC12.h"
#elif   (defined CFG_A3Sn12)
#include "HAL/HWconfig_A3Sn12.h"
// ATM2560
#elif   (defined CFG_A4En12)
#include "HAL/HWconfig_A4En12.h"
#elif   (defined CFG_A4ES12)
#include "HAL/HWconfig_A4ES12.h"
#elif   (defined CFG_A4Sn12)
#include "HAL/HWconfig_A4Sn12.h"
// STM32F051
#elif   (defined CFG_S2En12)
#include "HAL/HWconfig_S2En12.h"
#elif   (defined CFG_S2SC12)
#include "HAL/HWconfig_S2SC12.h"
#elif   (defined CFG_S2Sn12)
#include "HAL/HWconfig_S2Sn12.h"
// STM32F10x
#elif   (defined CFG_S3En12)
#include "HAL/HWconfig_S3En12.h"
#elif   (defined CFG_S3Sn12)
#include "HAL/HWconfig_S3Sn12.h"
#else
#error Undefined configuration
#endif  //  Configuration

#include "mqMEM.h"
#include "mqTypes.h"
#include "mqttsn.h"
#include "ObjDict.h"

// Time Counters
uint32_t GetTickCounter();


// Global Definitions
// GPIO Types
typedef enum
{
    DIO_MODE_IN_FLOAT = 0,
    DIO_MODE_IN_PD,
    DIO_MODE_IN_PU,
    DIO_MODE_OUT,
    DIO_MODE_PWM,
    DIO_MODE_AIN,
    // ARM Specific definition
    DIO_MODE_OUT_HS,
    DIO_MODE_AF0,
    DIO_MODE_AF1
}eDIOmode_t;

#ifdef __cplusplus
}
#endif

#endif  //  _CONFIG_H
