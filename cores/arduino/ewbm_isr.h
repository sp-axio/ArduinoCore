/**
  ******************************************************************************
  * @file    ewbm_isr.h
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   Header file of interrupt service
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

#ifndef __EWBM_ISR_H__
#define __EWBM_ISR_H__

#include "ewbm_device.h"

typedef void (*ISRCB)(void*);

void external_isr_init(void);
int  external_isr_set(IRQn_Type idx, ISRCB isrCb, void* priv);
int  external_isr_clear(IRQn_Type idx);
uint32_t get_system_jitter(void);

#endif // __EWBM_ISR_H__
