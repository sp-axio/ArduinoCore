/**
  ******************************************************************************
  * @file    ewbm_uart.c
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   UART module driver
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
#include "ewbm_uart.h"
#include "ewbm_isr.h"
#include "ewbm_clock.h"
#include "ewbm_delay.h"
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


#ifdef __FREE_RTOS__
    #include "FreeRTOS.h"
    #include "task.h"
    #define malloc(size) pvPortMalloc(size)
    #define free(ptr) vPortFree(ptr)
#endif

#ifdef DBG_ENABLE
    #define UART_DBG(fmt,args...)    UART_Printf(fmt,##args)
#else
    #define UART_DBG(fmt,args...)
#endif

#ifndef __FREE_RTOS__
    extern uint32_t get_system_jitter(void);
#endif

/** @addtogroup EWBM_Peripheral
  * @{
  */

/** @addtogroup EWBM_UART
  * @{
  */

/** @defgroup UART_Internal_Types UART Intenal Types
  * @{
  */

/**
  * @brief  UART buffer max size define
  */
#define PRINT_BUFFER_MAX_SIZE    256

/**
  * @brief  UART clock reset bit define
  */
#define UART_CLK_RESET(x)   (1<<(12+x))

/**
  * @brief  tUART_INTCB structure that use to register interrupt
  */
typedef struct _tUART_INTRCB {
    UARTCB  cb;
    void*   param;
} tUART_INTCB;

/**
  * @brief  Internal global arrangement variables that have a structure of tUART_INTCB
  */
static tUART_INTCB gUartIntCb[UART_BANK_MAX][UART_INTERRUPT_MAX];

#ifdef __ENABLE_UART_P__
static tUART_HANDLE* gPrintHandle;
static uint8_t*      gPrintBuf;
#endif

/**
  * @}
  */

/** @addtogroup UART_Static_Func
  * @{
  */

/** @addtogroup UART_Static_Func_G1
  * @{
  */

/**
  * @brief  Get base address about bank of UART
  * @param  bank: bank number of UART to use
  * @retval pointer to base address
  */
static tUART_REG* uart_get_baseaddr(eUART_BANK bank)
{
    switch ( bank )
    {
        case UART_BANK_1:
               return UART1;

        case UART_BANK_2:
               return UART2;

        case UART_BANK_3:
               return UART3;

        default:
               return NULL;
    }
}

/**
  * @brief  Get masked interrupt status
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  Masked interrupt status
  */
eUART_MODE uart_get_int_mask(tUART_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    // Get current interrupt mask
    if ( handle->reg->mis & UART_MIS_TXMIS )
    {
        return UART_MODE_TX;
    }
    else
    if ( handle->reg->mis & UART_MIS_RXMIS )
    {
        return UART_MODE_RX;
    }

    // Error status
    return UART_MODE_TX_RX;
}

/**
  * @brief  Clear the Interrupt status
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  mode: Masked interrupt status
  * @retval  none
  */
static void uart_int_clear(tUART_HANDLE* handle, eUART_MODE mask)
{
    if ( handle == NULL || handle->reg == NULL )
        return;

    if ( mask == UART_MODE_TX )
    {
        handle->reg->icr |= UART_ICR_TXIC; // Clear TX interrupt
    }
    else
    if ( mask == UART_MODE_RX )
    {
        handle->reg->icr |= UART_ICR_RXIC; // Clear RX interrupt
    }
}

/**
  * @brief  UART interrupt callback, call the registered callback function according to inerrupt type
  * @param  priv: pointer to parameter of interrupt
  * @retval none
  */
static void uart_int_cb(void *priv)
{
    tUART_HANDLE* handle = (tUART_HANDLE*)priv;
    eUART_MODE mask;

    // get interrupt status
    mask = uart_get_int_mask(handle);
    uart_int_clear(handle, mask);

    if ( mask == UART_MODE_TX && gUartIntCb[handle->bank][UART_MODE_TX].cb )
    {
        gUartIntCb[handle->bank][UART_MODE_TX].cb(gUartIntCb[handle->bank][UART_MODE_TX].param);
    }
    else
    if ( mask == UART_MODE_RX && gUartIntCb[handle->bank][UART_MODE_RX].cb )
    {
        gUartIntCb[handle->bank][UART_MODE_RX].cb(gUartIntCb[handle->bank][UART_MODE_RX].param);
    }
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup UART_External_Func
  * @{
  */

/** @addtogroup UART_External_Func_G1
  *  @brief    Initialization and Configuration functions
  * @{
  */

/**
  * @brief  Initializes the UART according to the specified parameters
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  if success, return EWBMOK value
  */
int UART_Init(tUART_HANDLE* handle)
{
    int ret;

    if ( handle == NULL )
        return EWBMINVARG;

    handle->reg = uart_get_baseaddr(handle->bank);
    if ( handle->reg == NULL )
        return EWBMINVARG;

    // Reset UART clk
    CLKRST->peri_rst = UART_CLK_RESET(handle->bank);
    while ( !(CLKRST->peri_rst & UART_CLK_RESET(handle->bank)) );

    // Disable UART, TX, RX
    handle->reg->cr &= ~(UART_CR_UARTEN|UART_CR_TXE|UART_CR_RXE);

    // Disable FIFO
    handle->reg->lcr_h &= ~(UART_LCR_H_FEN);

    // init word length
    handle->reg->lcr_h &= ~(UART_LCR_H_WLEN_MASK);


    // 0-10bit, clear all interrupt
    handle->reg->icr |= UART_ALL_MASK;

    // 0-10bit, clear all interrupt mask
    handle->reg->imsc &= ~UART_ALL_MASK;

    // Clear interrupt service routine
    external_isr_clear((IRQn_Type)(UART1_IRQn+handle->bank));

    // Set word length
    ret = UART_SetWordLength(handle);
    if ( ret != EWBMOK )
    {
        return ret;
    }

    // Set baudrate
    ret = UART_SetBaudRate(handle);
    if ( ret != EWBMOK )
    {
        return ret;
    }

    return EWBMOK;
}

/**
  * @brief  DeInitializes the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  if success, return EWBMOK value
  */
int UART_DeInit(tUART_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    // Disable UART, TX, RX
    handle->reg->cr &= ~(UART_CR_UARTEN|UART_CR_TXE|UART_CR_RXE);
    // Disable FIFO
    handle->reg->lcr_h &= ~(UART_LCR_H_FEN);

    // 0-10bit, clear all interrupt
    handle->reg->icr |= UART_ALL_MASK;
    // 0-10bit, clear all interrupt mask
    handle->reg->imsc &= ~UART_ALL_MASK;

    gUartIntCb[handle->bank][UART_MODE_TX].cb = NULL;
    gUartIntCb[handle->bank][UART_MODE_TX].param = NULL;
    gUartIntCb[handle->bank][UART_MODE_RX].cb = NULL;
    gUartIntCb[handle->bank][UART_MODE_RX].param = NULL;

    // Clear interrupt service routine
    external_isr_clear((IRQn_Type)(UART1_IRQn+handle->bank));

    handle->reg = NULL;

    return EWBMOK;
}

/**
  * @}
  */

/** @addtogroup UART_External_Func_G2
  *  @brief    Operation functions
  * @{
  */

/**
  * @brief  Enable the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  mode: UART mode, that is transmit section or receive section or both
  * @retval  if success, return EWBMOK value
  */
int UART_EnableUart(tUART_HANDLE* handle, eUART_MODE mode)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case UART_MODE_TX:
            handle->reg->cr |= UART_CR_TXE;  // Enable TX
            break;

        case UART_MODE_RX:
            handle->reg->cr |= UART_CR_RXE;  // Enable RX
            break;

        case UART_MODE_TX_RX:
            handle->reg->cr |= (UART_CR_TXE|UART_CR_RXE);  // Enable TX, RX
            break;

        default:
            return EWBMINVARG;
    }

    // Enable UART
    handle->reg->cr |= UART_CR_UARTEN;

    return EWBMOK;
}

/**
  * @brief  Disable the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  mode: UART mode, that is transmit section or receive section or both
  * @retval  if success, return EWBMOK value
  */
int UART_DisableUart(tUART_HANDLE* handle, eUART_MODE mode)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case UART_MODE_TX:
            handle->reg->cr &= ~UART_CR_TXE;  // Disable TX
            break;

        case UART_MODE_RX:
            handle->reg->cr &= ~UART_CR_RXE;  // Disable RX
            break;

        case UART_MODE_TX_RX:
            handle->reg->cr &= ~(UART_CR_TXE|UART_CR_RXE);  // Disable TX, RX
            handle->reg->cr &= ~UART_CR_UARTEN;  // Disable UART
            break;

        default:
            return EWBMINVARG;
    }

    return EWBMOK;
}

/**
  * @brief  Enable FIFO of the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  txLevel: trigger level of transmit FIFO
  * @param  rxLevel: trigger level of receive FIFO
  * @retval  if success, return EWBMOK value
  */
int UART_EnableFifo(tUART_HANDLE* handle, eUART_FIFO_LEVEL txLevel, eUART_FIFO_LEVEL rxLevel)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    // Set TX, RX FIFO level
    handle->reg->ifls = (txLevel&UART_IFLS_TXIFLSEL_MASK)|((rxLevel<<3)&UART_IFLS_RXIFLSEL_MASK);

    // Enable FIFO
    handle->reg->lcr_h |= UART_LCR_H_FEN;

    return EWBMOK;
}

/**
  * @brief  Disable FIFO of the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  if success, return EWBMOK value
  */
int UART_DisableFifo(tUART_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    // Disable FIFO
    handle->reg->lcr_h &= ~UART_LCR_H_FEN;

    return EWBMOK;
}

/**
  * @brief  Enable interrupt of the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  mode: interrupt mode, that is transmit section or receive section or both
  * @param  txIntCb: pointer to callback function when TX interrupt occurs
  * @param  txIntCbParam: pointer to parameter of TX callback function
  * @param  rxIntCb: pointer to callback function when RX interrupt occurs
  * @param  rxIntCbParam: pointer to parameter of RX callback function
  * @retval  if success, return EWBMOK value
  */
int UART_EnableInterrupt(tUART_HANDLE* handle, eUART_MODE mode, UARTCB txIntCb, void* txIntCbParam, UARTCB rxIntCb, void* rxIntCbParam)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case UART_MODE_TX:
            {
                handle->reg->imsc |= UART_IMSC_TXIM;  // Set TX interrupt mask
                if ( txIntCb )
                {
                    gUartIntCb[handle->bank][UART_MODE_TX].cb = txIntCb;
                    gUartIntCb[handle->bank][UART_MODE_TX].param = txIntCbParam;
                }
            }
            break;

        case UART_MODE_RX:
            {
                handle->reg->imsc |= UART_IMSC_RXIM;  // Set RX interrupt mask
                if ( rxIntCb )
                {
                    gUartIntCb[handle->bank][UART_MODE_RX].cb = rxIntCb;
                    gUartIntCb[handle->bank][UART_MODE_RX].param = rxIntCbParam;
                }
            }
            break;

        case UART_MODE_TX_RX:
            {
                handle->reg->imsc |= (UART_IMSC_TXIM|UART_IMSC_RXIM);  // Set TX, RX interrupt mask
                if ( txIntCb )
                {
                    gUartIntCb[handle->bank][UART_MODE_TX].cb = txIntCb;
                    gUartIntCb[handle->bank][UART_MODE_TX].param = txIntCbParam;
                }

                if ( rxIntCb )
                {
                    gUartIntCb[handle->bank][UART_MODE_RX].cb = rxIntCb;
                    gUartIntCb[handle->bank][UART_MODE_RX].param = rxIntCbParam;
                }
            }
            break;

        default:
            return EWBMINVARG;
    }

    external_isr_set((IRQn_Type)(UART1_IRQn+handle->bank), uart_int_cb, (void*)handle);
    return EWBMOK;
}

/**
  * @brief  Disable interrupt of the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  mode: interrupt mode, that is transmit section or receive section or both
  * @retval  if success, return EWBMOK value
  */
int UART_DisableInterrupt(tUART_HANDLE* handle, eUART_MODE mode)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case UART_MODE_TX:
            {
                handle->reg->imsc &= ~UART_IMSC_TXIM;  // Clear TX interrupt mask
                gUartIntCb[handle->bank][UART_MODE_TX].cb = NULL;
                gUartIntCb[handle->bank][UART_MODE_TX].param = NULL;
                if ( gUartIntCb[handle->bank][UART_MODE_RX].cb == NULL )
                {
                    // Clear interrupt service routine
                    external_isr_clear((IRQn_Type)(UART1_IRQn+handle->bank));
                    return EWBMOK;
                }
            }
            break;

        case UART_MODE_RX:
            {
                handle->reg->imsc &= ~UART_IMSC_RXIM;  // Clear RX interrupt mask
                gUartIntCb[handle->bank][UART_MODE_RX].cb = NULL;
                gUartIntCb[handle->bank][UART_MODE_RX].param = NULL;
                if ( gUartIntCb[handle->bank][UART_MODE_TX].cb == NULL )
                {
                    // Clear interrupt service routine
                    external_isr_clear((IRQn_Type)(UART1_IRQn+handle->bank));
                    return EWBMOK;
                }
            }
            break;
        case UART_MODE_TX_RX:
            {
                handle->reg->imsc &= ~(UART_IMSC_TXIM|UART_IMSC_RXIM);  // Clear TX, RX interrupt mask
                gUartIntCb[handle->bank][UART_MODE_TX].cb = NULL;
                gUartIntCb[handle->bank][UART_MODE_TX].param = NULL;
                gUartIntCb[handle->bank][UART_MODE_RX].cb = NULL;
                gUartIntCb[handle->bank][UART_MODE_RX].param = NULL;
                // Clear interrupt service routine
                external_isr_clear((IRQn_Type)(UART1_IRQn+handle->bank));
                return EWBMOK;
            }
        default:
            return EWBMINVARG;
    }

    return EWBMERR;
}

/**
  * @brief  Set baud rate of the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  if success, return EWBMOK value
  */
int UART_SetBaudRate(tUART_HANDLE* handle)
{
    uint32_t ibrd, temp;
    float fbrd;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    // Disable UART
    handle->reg->cr &= ~UART_CR_UARTEN;

    // Set baudrate
    ibrd = UART_INPUT_CLOCK/(16*handle->baudRate);

    handle->reg->ibrd = UART_IBRD_BAUD_DIVINT_MASK&ibrd;
    fbrd = (((float)UART_INPUT_CLOCK/(16*handle->baudRate))-ibrd)*64;

    temp = fbrd*10;
    temp %= 10;

    if ( temp > 5 )
    {
        fbrd +=1;
    }

    handle->reg->fbrd = UART_FBRD_BAUD_DIVFRAC_MASK&(int)fbrd;

    return EWBMOK;
}

/**
  * @brief  Set word length of the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  if success, return EWBMOK value
  */
int UART_SetWordLength(tUART_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    // Set word length
    handle->reg->lcr_h |= (handle->wordLength<<5)&UART_LCR_H_WLEN_MASK;

    return EWBMOK;
}

/**
  * @brief  Transmit data until timeout
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  data: pointer to tx data buffer
  * @param  len: length of tx data buffer
  * @param  timeOut: timeout duration
  * @retval  if success, return EWBMOK value
  */
int UART_Transmit(tUART_HANDLE* handle, uint8_t* data, uint32_t len, uint32_t timeOut)
{
    uint8_t* str = data;
    uint32_t cnt = len;
#ifdef __FREE_RTOS__
    uint32_t curTime = xTaskGetTickCount();
#else
    uint32_t curTime = get_system_jitter();
#endif

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    while ( cnt > 0 )
    {
        while ( handle->reg->fr & UART_FR_TXFF )
        {
#ifdef __RTX_RTOS__
            if ( (timeOut != 0) && (rt_time_get()-curTime > timeOut) )
#elif __FREE_RTOS__
            if ( (timeOut != 0) && (xTaskGetTickCount()-curTime > timeOut) )
#else
            if ( (timeOut != 0) && (get_system_jitter()-curTime > timeOut) )
#endif
            {
                return EWBMTMOUT;
            }
        }
        handle->reg->dr = *str;
        str++;
        cnt--;
    }
    return EWBMOK;
}

/**
  * @brief  Write a char to data register
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  data: a char for writing
  */
void UART_TransmitChar(tUART_HANDLE* handle, uint8_t data)
{
    if ( handle == NULL || handle->reg == NULL )
        return;

    while ( handle->reg->fr & UART_FR_TXFF );
    handle->reg->dr = data;
}

/**
  * @brief  Receive data until timeout
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  data: pointer to rx data buffer
  * @param  len: length of rx data buffer
  * @param  timeOut: timeout duration
  * @retval  if success, return EWBMOK value
  */
int UART_Receive(tUART_HANDLE* handle, uint8_t* data, uint32_t len, uint32_t timeOut)
{
    uint8_t* str = data;
    uint32_t cnt = len;
#ifdef __FREE_RTOS__
    uint32_t curTime = xTaskGetTickCount();
#else
    uint32_t curTime = get_system_jitter();
#endif

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    while ( cnt > 0 )
    {
        while ( handle->reg->fr & UART_FR_RXFE )
        {
#ifdef __RTX_RTOS__
            if ( (timeOut != 0) && (rt_time_get()-curTime > timeOut) )
#elif __FREE_RTOS__
            if ( (timeOut != 0) && (xTaskGetTickCount()-curTime > timeOut) )
#else
            if ( (timeOut != 0) && (get_system_jitter()-curTime > timeOut) )
#endif
            {
                return EWBMTMOUT;
            }
        }

        *str = handle->reg->dr;
        str++;
        cnt--;

        // Error handling
        if ( handle->reg->lcr_h&UART_LCR_H_FEN )   // Enabled FIFO
        {
            if ( handle->reg->dr&UART_DR_BE )
                handle->reg->dr &= ~UART_DR_BE;    // Break error

            if ( handle->reg->dr&UART_DR_OE )
                handle->reg->dr &= ~UART_DR_OE;    // Overrun error

            if ( handle->reg->dr&UART_DR_FE )
                handle->reg->dr &= ~UART_DR_FE;    // Framing error

            if ( handle->reg->dr&UART_DR_PE )
                handle->reg->dr &= ~UART_DR_PE;    // Parity error
        }
        else // Disabled FIFO
        {
            if ( handle->reg->rsr_ecr&UART_RSR_ECR_BE )
                handle->reg->rsr_ecr &= ~UART_RSR_ECR_BE;  // Break error

            if ( handle->reg->rsr_ecr&UART_RSR_ECR_OE )
                handle->reg->rsr_ecr &= ~UART_RSR_ECR_OE;  // Overrun error

            if ( handle->reg->rsr_ecr&UART_RSR_ECR_FE )
                handle->reg->rsr_ecr &= ~UART_RSR_ECR_FE;  // Framing error

            if ( handle->reg->rsr_ecr&UART_RSR_ECR_PE )
                handle->reg->rsr_ecr &= ~UART_RSR_ECR_PE;  // Parity error
        }
    }

    return EWBMOK;
}

/**
  * @brief  Read received a char
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  read data
  */
uint8_t UART_ReceiveChar(tUART_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    while(handle->reg->fr & UART_FR_RXFE);
    return handle->reg->dr;
}

#ifdef __ENABLE_DMA__
/**
  * @brief  Enable DMA of the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  mode: DMA mode, that is transmit section or receive section or both
  * @retval  if success, return EWBMOK value
  */
int UART_EnableDma(tUART_HANDLE* handle, eUART_MODE mode)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case UART_MODE_TX:
            handle->reg->dmacr |= UART_DMACR_TXDMAE;
            break;

        case UART_MODE_RX:
            handle->reg->dmacr |= UART_DMACR_RXDMAE;
            break;

        case UART_MODE_TX_RX:
            handle->reg->dmacr |= (UART_DMACR_TXDMAE|UART_DMACR_RXDMAE);
            break;

        default:
            return EWBMINVARG;
    }

    return EWBMOK;
}

/**
  * @brief  Disable DMA of the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  mode: DMA mode, that is transmit section or receive section or both
  * @retval  if success, return EWBMOK value
  */
int UART_DisableDma(tUART_HANDLE* handle, eUART_MODE mode)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case UART_MODE_TX:
            handle->reg->dmacr &= ~UART_DMACR_TXDMAE;
            break;

        case UART_MODE_RX:
            handle->reg->dmacr &= ~UART_DMACR_RXDMAE;
            break;

        case UART_MODE_TX_RX:
            handle->reg->dmacr &= ~(UART_DMACR_TXDMAE|UART_DMACR_RXDMAE);
            break;

        default:
            return EWBMINVARG;
    }

    return EWBMOK;
}

/**
  * @brief  Transmit data through DMA
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  data: pointer to the transmission data
  * @param  len: transmission data length
  * @param  dmaHandle: pointer to a tDMA_HANDLE structure that contains the configuration information for DMA module
  * @param  dmaCb: pointer to callback function when dma interrupt occurs
  * @param  dmaCbParam: pointer to parameter of dma callback function
  * @retval  if success, return EWBMOK value
  */
int UART_TransmitDma(tUART_HANDLE* handle, uint8_t* data, uint32_t len, tDMA_HANDLE* dmaHandle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    dmaHandle->srcAddr = data;
    dmaHandle->dstAddr = handle->reg;
    dmaHandle->transLength = len;

    return DMA_Start(dmaHandle);
}

/**
  * @brief  Receive data through DMA
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @param  data: pointer to the transmission data
  * @param  len: transmission data length
  * @param  dmaHandle: pointer to a tDMA_HANDLE structure that contains the configuration information for DMA module
  * @retval  if success, return EWBMOK value
  */
int UART_ReceiveDma(tUART_HANDLE* handle, uint8_t* data, uint32_t len, tDMA_HANDLE* dmaHandle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    dmaHandle->srcAddr = handle->reg;
    dmaHandle->dstAddr = data;
    dmaHandle->transLength = len;

    return DMA_Start(dmaHandle);
}
#endif

/**
  * @}
  */

#ifdef __ENABLE_UART_P__
/** @addtogroup UART_External_Func_G3
  *  @brief    Printf functions
  * @{
  */

/**
  * @brief  Initializes the print handle and allocate memory for PRINT_BUFFER_MAX_SIZE
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  none
  */
void UART_PrintInit(tUART_HANDLE* handle)
{
    gPrintHandle = handle;
    gPrintBuf = (uint8_t*)malloc(PRINT_BUFFER_MAX_SIZE);
}

/**
  * @brief  DeInitializes the print handle and free allocated memory
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  none
  */
void UART_PrintDeInit(tUART_HANDLE* handle)
{
    gPrintHandle = NULL;
    free(gPrintBuf);
}

static void add_cr(void)
{
    uint8_t *buf;
    int i, j;
    buf = (uint8_t*)malloc(PRINT_BUFFER_MAX_SIZE*2);
    memset(buf, 0, PRINT_BUFFER_MAX_SIZE*2);

    for (i=0, j=0; j<PRINT_BUFFER_MAX_SIZE; ++i, ++j) {
	buf[i] = gPrintBuf[j];
	if (buf[i] == '\n') {
	    i++;
	    buf[i] = '\r';
	}
    }
    buf[i] = 0;
    memcpy(gPrintBuf, buf, PRINT_BUFFER_MAX_SIZE);
    free(buf);
}

/**
  * @brief  Print debug message according to variable arguments
  * @param  fmt: format of the second paramter
  * @param  variable arguments: variable parameter list
  * @retval  none
  */
int UART_Printf(const char *fmt, ...)
{
    if ( gPrintHandle == NULL || gPrintHandle->reg == NULL ) {
        return 0;
    }

    va_list args;
    int printlen;
    memset(gPrintBuf, 0, sizeof(PRINT_BUFFER_MAX_SIZE));
    va_start(args,fmt);
    vsprintf((char*)gPrintBuf, fmt, args);
    va_end(args);
    add_cr();
    printlen = strlen((const char *)gPrintBuf);
    (printlen > PRINT_BUFFER_MAX_SIZE) ? printlen = PRINT_BUFFER_MAX_SIZE : printlen;
    UART_Transmit(gPrintHandle, gPrintBuf, printlen, 100);
    __io_delay(printlen * 960);
    return printlen;
}

/**
  * @}
  */
#endif

/** @addtogroup UART_External_Func_G4
  *  @brief    Identification functions
  * @{
  */

/**
  * @brief  Get part number of UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  return part number
  */
uint16_t UART_GetPartNum(tUART_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    return ((UART_PERIPH_ID0(handle->reg)&UART_PERIPH_ID0_PARTNUM_MASK)|(UART_PERIPH_ID1(handle->reg)&UART_PERIPH_ID1_PARTNUM_MASK)<<8);
}

/**
  * @brief  Get identification of the designer
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  return desinger
  */
uint8_t UART_GetDesigner(tUART_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    return ((UART_PERIPH_ID1(handle->reg)&UART_PERIPH_ID1_DESIGNER_MASK)>>4|(UART_PERIPH_ID2(handle->reg)&UART_PERIPH_ID2_DESIGNER_MASK)<<4);
}

/**
  * @brief  Get revision number of the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  return revision
  */
uint8_t UART_GetRevision(tUART_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    return ((UART_PERIPH_ID2(handle->reg)&UART_PERIPH_ID2_REVISION_MASK)>>4);
}

/**
  * @brief  Get configuration option of the UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  return configuration
  */
uint8_t UART_GetConfiguration(tUART_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    return (UART_PERIPH_ID3(handle->reg)&UART_PERIPH_ID3_CONFIG_MASK);
}

/**
  * @brief  Get identification of UART
  * @param  handle: pointer to a tUART_HANDLE structure that contains the configuration information for UART module
  * @retval  return identification
  */
uint32_t UART_GetId(tUART_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    return ((UART_PCELL_ID0(handle->reg)&UART_PCELL_ID0_MASK)|((UART_PCELL_ID1(handle->reg)&UART_PCELL_ID1_MASK)<<8)|\
            ((UART_PCELL_ID2(handle->reg)&UART_PCELL_ID2_MASK)<<16)|((UART_PCELL_ID3(handle->reg)&UART_PCELL_ID3_MASK)<<24));
}

int UART_DataAvailable(tUART_HANDLE *h)
{
	if (!(h->reg->fr & UART_FR_RXFE)) {
		return 1;
	}
	return 0;
}

int UART_FifoFull(tUART_HANDLE *h)
{
    return h->reg->fr & UART_FR_TXFF ? 1: 0;
}

#define UART_LCRH_STP2 0x8
void UART_SetStopbit(tUART_HANDLE *h, eUART_STOPBIT b)
{
  if (b == UART_STOPBIT_1) {
    h->reg->lcr_h &= ~UART_LCRH_STP2;
  } else if (b == UART_STOPBIT_1_5 || b == UART_STOPBIT_2) {
    h->reg->lcr_h |= UART_LCRH_STP2;
  }
}

#define UART_LCRH_PARITY_MASK	0x6
#define UART_LCRH_PEN		0x2
#define UART_LCRH_EPS		0x4
void UART_SetParity(tUART_HANDLE *h, eUART_PARITY p)
{
  uint32_t parity = 0;
  if (p == UART_PARITY_NONE) {
    parity = 0;
  } else if (p == UART_PARITY_ODD) {
    parity = UART_LCRH_PEN ;
  } else if (p == UART_PARITY_EVEN) {
    parity = UART_LCRH_PEN | UART_LCRH_EPS;
  }
  h->reg->lcr_h &= ~UART_LCRH_PARITY_MASK;
  h->reg->lcr_h |= parity;
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/* vim: ts=4 sw=4
 */
