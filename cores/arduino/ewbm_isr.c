/**
  ******************************************************************************
  * @file    ewbm_isr.c
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   Source file of interrupt service
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

/* Header File */
#include "ewbm_isr.h"
#include "ewbm_uart.h"

typedef struct _tISR_INFO 
{
    ISRCB   isrCb;
    void*   priv;
} tISR_INFO;

static tISR_INFO isrInfoVector[INTR_MAX_IRQn];

int external_isr_clear(IRQn_Type idx)
{
    if ( idx >= INTR_MAX_IRQn )
        return -1;

    isrInfoVector[idx].isrCb = (void*)0;
    isrInfoVector[idx].priv = (void*)0;

    NVIC_DisableIRQ(idx);
    return 0;
}

int external_isr_set(IRQn_Type idx, ISRCB isrCb, void* priv)
{
    if ( idx >= INTR_MAX_IRQn )
        return -1;

    if ( isrCb == (void *)0 )
        return -1;

    isrInfoVector[idx].isrCb = isrCb;
    isrInfoVector[idx].priv  = priv;

    NVIC_ClearPendingIRQ(idx);
    NVIC_EnableIRQ(idx);

    return 0;
}

void external_isr_init(void)
{
    IRQn_Type irq;

    __enable_irq();

    for ( irq=SystemInit_IRQn; irq<INTR_MAX_IRQn; irq++ )
    {
        isrInfoVector[irq].isrCb = (void*)0;
        isrInfoVector[irq].priv = (void*)0;

        NVIC_DisableIRQ(irq);
    }
}

#ifndef __FREE_RTOS__
static uint32_t jitter = 0;

void SysTick_Handler(void)
{
    jitter++;

    if ( jitter >= 0xFFFFFFFF )
        jitter = 0;
}

uint32_t get_system_jitter(void)
{
    return jitter;
}
#endif

void __dump_stacks(uint32_t type, uint32_t *addr)
{
	volatile uint32_t r0, r1, r2, r3, r12, lr, pc, psr, stack[16];
	volatile uint32_t cpuid, icsr, aircr, scr, ccr, shp[2], shcsr;
	const char *handler[] = {"NMI", "HardFault", "MemManage",
		"BusFault", "UsageFault", "SVC", "DebugMon", "PendSV"};

	r0        = addr[0];  r1        = addr[1];  r2        = addr[2];  r3        = addr[3];
	r12       = addr[4];  lr        = addr[5];  pc        = addr[6];  psr       = addr[7];
	stack[0]  = addr[8];  stack[1]  = addr[9];  stack[2]  = addr[10]; stack[3]  = addr[11];
	stack[4]  = addr[12]; stack[5]  = addr[13]; stack[6]  = addr[14]; stack[7]  = addr[15];
	stack[8]  = addr[16]; stack[9]  = addr[17]; stack[10] = addr[18]; stack[11] = addr[19];
	stack[12] = addr[20]; stack[13] = addr[21]; stack[14] = addr[22]; stack[15] = addr[23];
	cpuid     = SCB->CPUID;
	icsr      = SCB->ICSR;   // Interrupt Control and State
	aircr     = SCB->AIRCR;  // Application Interrupt and Reset Control
	scr       = SCB->SCR;    // System Control
	ccr       = SCB->CCR;    // Configuration Control
	shp[0]    = SCB->SHP[0]; // System Handlers Priority
	shp[1]    = SCB->SHP[1]; // System handlers priority
	shcsr     = SCB->SHCSR;  // System handler Control and State

	if (type < 8) {
		UART_Printf("** %s exception\n", handler[type]);
	} else {
		UART_Printf("** Other exception(%d)\n", type);
	}

	UART_Printf("** Dump Stack (%08x)\n", addr);
	UART_Printf("**   R0      %08x\n", r0);
	UART_Printf("**   R1      %08x\n", r1);
	UART_Printf("**   R2      %08x\n", r2);
	UART_Printf("**   R3      %08x\n", r3);
	UART_Printf("**   R12     %08x\n", r12);
	UART_Printf("**   LR      %08x\n", lr);
	UART_Printf("**   PC      %08x\n", pc);
	UART_Printf("**   PSR     %08x\n", psr);
	UART_Printf("**   Stack   %08x %08x %08x %08x\n", stack[0], stack[1], stack[2], stack[3]);
	UART_Printf("**   Stack   %08x %08x %08x %08x\n", stack[4], stack[5], stack[6], stack[7]);
	UART_Printf("**   Stack   %08x %08x %08x %08x\n", stack[8], stack[9], stack[10], stack[11]);
	UART_Printf("**   Stack   %08x %08x %08x %08x\n", stack[12], stack[13], stack[14], stack[15]);
	UART_Printf("** Dump SCB\n");
	UART_Printf("**   CPUID   %08x\n", cpuid);
	UART_Printf("**   ICSR    %08x\n", icsr);
	UART_Printf("**   AIRCR   %08x\n", aircr);
	UART_Printf("**   SCR     %08x\n", scr);
	UART_Printf("**   CCR     %08x\n", ccr);
	UART_Printf("**   SHP0    %08x\n", shp[0]);
	UART_Printf("**   SHP1    %08x\n", shp[1]);
	UART_Printf("**   SHCSR   %08x\n", shcsr);
	UART_Printf("** end of dump\n");
	//__asm volatile("bkpt #0\n"); // Break into the debugger
	while(1);
}

__attribute__((naked)) void dump_stacks(int type)
{
	__asm volatile (
			".syntax unified\n"
			"movs r4, #4 \n"
			"mov r5, lr \n"
			"tst r4, r5 \n"
			"beq . + 8 \n"
			"mrs r1, psp \n"
			"b __dump_stacks\n"
			"mrs r1, msp \n"
			"b __dump_stacks\n"
			".syntax divided\n"
			);
}

void NMI_Handler(void)
{
    uint32_t wdt_sts  = SYSCON->nmi_sts&0x2;
	UART_Printf("NMI_Handler\n");

    if ( wdt_sts == 0x2 )
    {
        if ( isrInfoVector[WDT_IRQn].isrCb )
        {
            isrInfoVector[WDT_IRQn].isrCb(isrInfoVector[WDT_IRQn].priv);
        }
    }
    else
    {
        while(1);
    }
}


__attribute__((naked)) void HardFault_Handler(void)
{
	__asm volatile (
#if 0 //jjhh
			// first verion: this code is not work in thumb mode
			" tst lr, #4                                                \n"
			" ite eq                                                    \n"
			" mrseq r0, msp                                             \n"
			" mrsne r0, psp                                             \n"
			" ldr r1, [r0, #24]                                         \n"
			" ldr r2, handler2_address_const                            \n"
			" bx r2                                                     \n"
			" handler2_address_const: .word prvGetRegistersFromStack    \n"
			// second version: this code works, but need one more parameter and the label is not necessary
			".syntax unified\n"
			"MOVS R0, #4 \n"
			"MOV R1, LR \n"
			"TST R0, R1 \n"
			"BEQ _MSP \n"
			"MRS R0, PSP \n"
			"B HardFault_HandlerC \n"
			"_MSP: \n"
			"MRS R0, MSP \n"
			"B HardFault_HandlerC \n"
			".syntax divided\n"
			// third version: parameter1(r0): fault type, parameter2(r1): stack address
			".syntax unified\n"
			"movs r0, #2 \n"
			"movs r4, #4 \n"
			"mov r5, lr \n"
			"tst r4, r5 \n"
			"beq . + 8 \n"
			"mrs r1, psp \n"
			"b __dump_stacks\n"
			"mrs r1, msp \n"
			"b __dump_stacks\n"
			".syntax divided\n"
#endif
			// final version: split into two parts
			"movs r0, #1 \n"
			"b dump_stacks\n"
			);
}

__attribute__((naked)) void MemManage_Handler(void)
{
	__asm volatile (
			"movs r0, #2 \n"
			"b dump_stacks\n" );
    while(1);
}

__attribute__((naked)) void BusFault_Handler(void)
{
	__asm volatile (
			"movs r0, #3 \n"
			"b dump_stacks\n" );
    while(1);
}

__attribute__((naked)) void  UsageFault_Handler(void)
{
	__asm volatile (
			"movs r0, #4 \n"
			"b dump_stacks\n" );
}

#ifndef __FREE_RTOS__
__attribute__((naked)) void SVC_Handler(void)
{
	__asm volatile (
			"movs r0, #5 \n"
			"b dump_stacks\n" );
    while(1);
}

__attribute__((naked)) void DebugMon_Handler(void)
{
	__asm volatile (
			"movs r0, #6 \n"
			"b dump_stacks\n" );
    while(1);
}

__attribute__((naked)) void PendSV_Handler(void)
{
	__asm volatile (
			"movs r0, #7 \n"
			"b dump_stacks\n" );
    while(1);
}
#endif

void External_Handler(void)
{
    uint32_t extNum = __get_IPSR() - 0x10;

    if ( isrInfoVector[extNum].isrCb )
        isrInfoVector[extNum].isrCb(isrInfoVector[extNum].priv);
}

