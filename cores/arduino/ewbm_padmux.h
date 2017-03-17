/**
  ******************************************************************************
  * @file    ewbm_padmux.h
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   Header file of padmux setting
  ******************************************************************************
  * @attention
  *
  * Copyright(c) 2015 ~ 2020 eWBM Korea , Ltd.
  * All rights reserved.
  * This software contains confidential information of eWBM Korea Co.,Ltd.
  * and unauthorized distribution of this software, or any portion of it, are
  * prohibited.
  *
  ******************************************************************************
  */

#ifndef __EWBM_PADMUX_H__
#define __EWBM_PADMUX_H__

#include "ewbm_device.h"


typedef enum 
{
    PA0 = 0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
    PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
    PIN_MAX, NON
} ePADMUX_PIN;

typedef enum 
{
    PADMUX_MODE_SYS     = 0,
    PADMUX_MODE_SPI     = 1,
    PADMUX_MODE_UART    = 2,
    PADMUX_MODE_I2C     = 3,
    PADMUX_MODE_TIMER   = 4,
    PADMUX_MODE_SDIO    = 5
} ePADMUX_MODE;

typedef enum 
{
    PADMUX_PORT_A = 0,
    PADMUX_PORT_B,
    PADMUX_PORT_C,
    PADMUX_PORT_MAX
} ePADMUX_PORT;

void padmux_init(void);

#endif //__EWBM_PADMUX_H__

