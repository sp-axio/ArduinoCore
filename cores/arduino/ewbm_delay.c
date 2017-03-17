/**
  ******************************************************************************
  * @file    ewbm_delay.c
  * @author  eWBM
  * @version v0.1
  * @date    20-10-2016
  * @brief   Delay driver
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

#include "ewbm_delay.h"

#ifdef __FREE_RTOS__
#include "FreeRTOS.h"
#include "task.h"
#endif

void __io_delay(uint32_t delay)
{
    uint32_t i;
    for ( i=0; i<=delay; i++ )
    {
        __NOP();
    }
}

void __m_delay(uint32_t ms)
{
#ifdef __FREE_RTOS__
    vTaskDelay(ms);
#else
    extern uint32_t get_system_jitter(void);
    uint32_t start  = get_system_jitter();
    while ((get_system_jitter() - start) < (uint32_t)ms);
#endif
}