/**
  ******************************************************************************
  * @file    startup_ewbm.c
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief
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

/* Copyright (c) 2011 - 2014 ARM LIMITED

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
#include "ewbm_device.h"
#include "ewbm_isr.h"
#include "startup_config.h"

/*----------------------------------------------------------------------------
  Linker generated Symbols
 *----------------------------------------------------------------------------*/
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler Function Prototype
 *----------------------------------------------------------------------------*/
typedef void( *pFunc )( void );


/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
#ifndef __START
    extern void  _start(void) __attribute__((noreturn));    /* PreeMain (C library entry point) */
#else
    extern int  __START(void) __attribute__((noreturn));    /* main entry point */
#endif

#ifndef __NO_SYSTEM_INIT
    extern void SystemInit (void);            /* CMSIS System Initialization      */
#endif

#ifdef __FREE_RTOS__
__attribute__((weak)) void software_init_hook (void);   /* NEWLIB! */
#endif

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
void External_Handler(void);
void Reset_Handler(void);                            /* Reset Handler */

/*----------------------------------------------------------------------------
  User Initial Stack & Heap
 *----------------------------------------------------------------------------*/
#ifndef __STACK_SIZE
    #define   __STACK_SIZE  0x00000800
#endif

#ifndef __HEAP_SIZE
    #define   __HEAP_SIZE   0x00000800
#endif

#if __STACK_SIZE > 0
    static uint8_t stack[__STACK_SIZE] __attribute__ ((aligned(8), used, section(".stack")));
#endif

#if __HEAP_SIZE > 0
    static uint8_t heap[__HEAP_SIZE]   __attribute__ ((aligned(8), used, section(".heap")));
#endif


#ifdef __FREE_RTOS__
    #define SVC_Handler vPortSVCHandler
    #define PendSV_Handler xPortPendSVHandler
    #define SysTick_Handler xPortSysTickHandler
#endif

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Cortex-M0 Processor Exceptions */
void NMI_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler   (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler   (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler    (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler  (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler    (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));


/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
const pFunc __Vectors[] __attribute__ ((section(".vectors"))) = {
    (pFunc)&__StackTop,                       /*      Initial Stack Pointer     */
    Reset_Handler,                            /*      Reset Handler             */
    NMI_Handler,                              /*      NMI Handler               */
    HardFault_Handler,                        /*      Hard Fault Handler        */
    MemManage_Handler,                        /*      MPU Fault Handler         */
    BusFault_Handler,                         /*      Bus Fault Handler         */
    UsageFault_Handler,                       /*      Usage Fault Handler       */
    0,                                        /*      Reserved                  */
    0,                                        /*      Reserved                  */
    0,                                        /*      Reserved                  */
    0,                                        /*      Reserved                  */
    SVC_Handler,                              /*      SVCall Handler            */
    DebugMon_Handler,                         /*      Debug Monitor Handler     */
    0,                                        /*      Reserved                  */
    PendSV_Handler,                           /*      PendSV Handler            */
    SysTick_Handler,                          /*      SysTick Handler           */

    /* External interrupts */
    External_Handler,                              /* 00 : System Init ( Clock Status, WAKEUP pin), plus event which can be used as NMI */
    External_Handler,                              /* 01 : RTC ( alarm or wakeup )   */
    External_Handler,                              /* 02 : DMA IRQ */
    External_Handler,                              /* 03 : DMA Error IRQ */
    External_Handler,                              /* 04 : SDIO global interrupt */
    External_Handler,                              /* 05 : SDMMC global interrupt */
    External_Handler,                              /* 06 : Security Module (TRNG) */
    External_Handler,                              /* 07 : ARIA interrupt */
    External_Handler,                              /* 08 : Security Engine (SPAcc) */
    External_Handler,                              /* 09 : SecureKey Policy interrupt */
    External_Handler,                              /* 10 : PKA global interrupt */
    External_Handler,                              /* 11 : Clock reset manager interrupt */
    External_Handler,                              /* 12 : WDT interrupt */
    External_Handler,                              /* 13 : GPIO1 global interrupt */
    External_Handler,                              /* 14 : GPIO2 global interrupt */
    External_Handler,                              /* 15 : GPIO3 global interrupt */
    External_Handler,                              /* 16 : I2C1 global interrupt */
    External_Handler,                              /* 17 : I2C2 global interrupt */
    External_Handler,                              /* 18 : I2C3 global interrupt */
    External_Handler,                              /* 19 : I2C4 global interrupt */
    External_Handler,                              /* 20 : SPI1 global interrupt */
    External_Handler,                              /* 21 : SPI2 global interrupt */
    External_Handler,                              /* 22 : SPI3 global interrupt */
    External_Handler,                              /* 23 : UART1 global interrupt */
    External_Handler,                              /* 24 : UART2 global interrupt */
    External_Handler,                              /* 25 : UART3 global interrupt */
    External_Handler,                              /* 26 : Timer1 global interrupt */
    External_Handler,                              /* 27 : Timer2 global interrupt */
    0
};

#define __STARTUP_CLEAR_BSS
//#define __STARTUP_COPY_MULTIPLE

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
void Reset_Handler(void)
{
#ifndef __JTAG__
    uint32_t *pSrc;
#endif
    uint32_t *pDest;
    uint32_t *pTable __attribute__((unused));

#ifdef __STARTUP_COPY_MULTIPLE
    pTable = &__copy_table_start__;

    for ( ; pTable<&__copy_table_end__; pTable=pTable+3 )
    {
        pSrc  = (uint32_t*)*(pTable + 0);
        pDest = (uint32_t*)*(pTable + 1);
        for ( ; pDest<(uint32_t*)(*(pTable+1)+*(pTable+2)); )
            *pDest++ = *pSrc++;
    }
#elif !defined (__JTAG__)    
    pSrc  = &__etext;
    pDest = &__data_start__;

    for ( ; pDest<&__data_end__; )
        *pDest++ = *pSrc++;    
#endif /*__STARTUP_COPY_MULTIPLE */

#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
    pTable = &__zero_table_start__;

    for ( ; pTable<&__zero_table_end__; pTable=pTable+2 )
    {
        pDest = (uint32_t*)*(pTable + 0);
        for ( ; pDest<(uint32_t*)(*(pTable + 0)+*(pTable + 1)); )
        {
            *pDest++ = 0;
        }
    }
#elif defined (__STARTUP_CLEAR_BSS)
    pDest = &__bss_start__;
    for ( ; pDest<&__bss_end__; )
        *pDest++ = 0ul;
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
    SystemInit();
#endif

#ifdef __FREE_RTOS__
/* Call a NEWLIB hook if defined (used by RTX to start main() as a task) */
if (software_init_hook) {
    software_init_hook();
    return;
}
#endif

#ifndef __START
    #define __START _start
#endif

    __START();
}


/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
}

