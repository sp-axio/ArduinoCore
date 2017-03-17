/**
  ******************************************************************************
  * @file    ewbm_device.h
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   CMSIS Core Peripheral Access Layer Header File for ARMCM0 Device Series
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

/* Copyright (c) 2011 - 2015 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


#ifndef __EWBM_DEVICE_H__
#define __EWBM_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------  Interrupt Number Definition  ------------------------ */
typedef enum IRQn {
/* -------------------  Cortex-M0 Processor Exceptions Numbers  ------------------- */
    NonMaskableInt_IRQn           = -14,      /*  2 Non Maskable Interrupt */
    HardFault_IRQn                = -13,      /*  3 HardFault Interrupt */
    SVCall_IRQn                   =  -5,      /* 11 SV Call Interrupt */
    PendSV_IRQn                   =  -2,      /* 14 Pend SV Interrupt */
    SysTick_IRQn                  =  -1,      /* 15 System Tick Interrupt */
/* ----------------------  ARMCM0 Specific Interrupt Numbers  --------------------- */
    SystemInit_IRQn               =   0,      /* System Int (Clock Status, WAKEUP pin), plus event which can be used as NMI */
    RTC_IRQn                      =   1,      /* RTC (alarm or wakeup) */
    DMA_IRQn                      =   2,      /* DMA IRQ */
    DMA_ERR_IRQn                  =   3,      /* DMA Error IRQ */
    SDIO_IRQn                     =   4,      /* SDIO global interrupt */
    SDMMC_IRQn                    =   5,      /* SDMMC global interrupt */
    TRNG_IRQn                     =   6,      /* Security Module (TRNG) */
    ARIA_IRQn                     =   7,      /* ARIA interrupt */
    SPAcc_IRQn                    =   8,      /* Security Engine (SPAcc) */
    SKP_IRQn                      =   9,      /* SecureKey Policy interrupt */
    PKA_IRQn                      =  10,      /* PKA global interrupt */
    CRM_IRQn                      =  11,      /* Clock reset manager interrupt */
    WDT_IRQn                      =  12,      /* WDT interrupt */
    GPIO1_IRQn                    =  13,      /* GPIO1 global interrupt */
    GPIO2_IRQn                    =  14,      /* GPIO2 global interrupt */
    GPIO3_IRQn                    =  15,      /* GPIO3 global interrupt */
    I2C1_IRQn                     =  16,      /* I2C1 global interrupt */
    I2C2_IRQn                     =  17,      /* I2C2 global interrupt */
    I2C3_IRQn                     =  18,      /* I2C3 global interrupt */
    I2C4_IRQn                     =  19,      /* I2C4 global interrupt */
    SPI1_IRQn                     =  20,      /* SPI1 global interrupt */
    SPI2_IRQn                     =  21,      /* SPI2 global interrupt */
    SPI3_IRQn                     =  22,      /* SPI3 global interrupt */
    UART1_IRQn                    =  23,      /* UART3 global interrupt */
    UART2_IRQn                    =  24,      /* UART1 global interrupt */
    UART3_IRQn                    =  25,      /* UART2 global interrupt */
    TIMER1_IRQn                   =  26,      /* Timer1 global interrupt */
    TIMER2_IRQn                   =  27,      /* Timer2 global interrupt */
    INTR_MAX_IRQn
} IRQn_Type;

/* -------  Start of section using anonymous unions and disabling warnings  ------- */
#if defined (__CC_ARM)
    #pragma push
    #pragma anon_unions
#elif defined (__ICCARM__)
    #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wc11-extensions"
    #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
    /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
    /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
    #pragma warning 586
#elif defined (__CSMC__)
    /* anonymous unions are enabled by default */
#else
    #warning Not supported compiler type
#endif


/* --------  Configuration of the Cortex-M0 Processor and Core Peripherals  ------- */
#define __CM0_REV                 0x0000U   /* Core revision r0p0 */
#define __MPU_PRESENT             0         /* MPU present or not */
#define __VTOR_PRESENT            0         /* no VTOR present*/
#define __NVIC_PRIO_BITS          2         /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0         /* Set to 1 if different SysTick Config is used */

#include "core_cm0.h"                       /* Processor and core peripherals */
#include "system_ewbm.h"               /* System Header */

/* --------  End of section using anonymous unions and disabling warnings  -------- */
#if   defined (__CC_ARM)
    #pragma pop
#elif defined (__ICCARM__)
    /* leave anonymous unions enabled */
#elif (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
#elif defined (__GNUC__)
    /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
    /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
    #pragma warning restore
#elif defined (__CSMC__)
    /* anonymous unions are enabled by default */
#else
    #warning Not supported compiler type
#endif

#ifdef __cplusplus
}
#endif

#endif  /* __EWBM_DEVICE_H__ */
