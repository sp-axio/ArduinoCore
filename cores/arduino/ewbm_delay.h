/**
  ******************************************************************************
  * @file    ewbm_delay.h
  * @author  eWBM
  * @version v0.1
  * @date    06-06-2016
  * @brief   Header file of io delay
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

#ifndef __DELAY_H__
#define __DELAY_H__

#include "ewbm_device.h"

void __io_delay(uint32_t delay);
void __m_delay(uint32_t ms);

#define millis get_system_jitter 

#endif //__DELAY_H__


