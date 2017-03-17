/**
  ******************************************************************************
  * @file    ewbm_uart.h
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   Header file of UART module.
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

#ifndef __EWBM_UART_H__
#define __EWBM_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ewbm_device.h"

#ifdef __ENABLE_DMA__
    #include "ewbm_dma.h"
#endif

/** @addtogroup EWBM_Peripheral
  * @{
  */

/** @addtogroup EWBM_UART
  * @{
  */

/** @defgroup UART_External_Types DMA Exported Types
  * @{
  */

/**
  * @brief  UART input clock definition
  */
#define UART_INPUT_CLOCK    (48*1000*1000)

/**
  * @brief  DMA callback function type
  */
typedef void (*UARTCB)(void*);

/**
  * @brief  UART bank enumeration
  */
typedef enum {
    UART_BANK_1 = 0,
    UART_BANK_2,
    UART_BANK_3,
    UART_BANK_MAX
} eUART_BANK;

/**
  * @brief  UART mode enumeration
  */
typedef enum {
    UART_MODE_TX = 0,
    UART_MODE_RX,
    UART_MODE_TX_RX
} eUART_MODE;

/**
  * @brief  UART baud rate enumeration
  */
typedef enum {
    UART_BAUDRATE_460800 = 460800,
    UART_BAUDRATE_230400 = 230400,
	UART_BAUDRATE_172800 = 172800,
    UART_BAUDRATE_115200 = 115200,
    UART_BAUDRATE_76800 = 76800,
    UART_BAUDRATE_57600 = 57600,
    UART_BAUDRATE_38400 = 38400,
    UART_BAUDRATE_19200 = 19200,
    UART_BAUDRATE_14400 = 14400,
    UART_BAUDRATE_9600 = 9600,
    UART_BAUDRATE_2400 = 2400,
    UART_BAUDRATE_1200 = 1200,
    UART_BAUDRATE_110 = 110
} eUART_BAUDRATE;

/**
  * @brief  UART word length enumeration
  */
typedef enum {
    UART_WORDLENGTH_5BIT = 0,
    UART_WORDLENGTH_6BIT,
    UART_WORDLENGTH_7BIT,
    UART_WORDLENGTH_8BIT
} eUART_WORDLENGTH;

/**
  * @brief  UART stop bit enumeration
  */
typedef enum {
    UART_STOPBIT_NONE = 0,
    UART_STOPBIT_1 = 1,
    UART_STOPBIT_1_5,
    UART_STOPBIT_2,
} eUART_STOPBIT;

/**
  * @brief  UART parity enumeration
  */
typedef enum {
    UART_PARITY_EVEN = 1,
    UART_PARITY_ODD = 2,
    UART_PARITY_NONE = 3,
} eUART_PARITY;

/**
  * @brief  UART FIFO level enumeration
  */
typedef enum {
    UART_FIFO_LEVEL_1_OVER_8 = 0,  // 1/8
    UART_FIFO_LEVEL_1_OVER_4,      // 1/4
    UART_FIFO_LEVEL_1_OVER_2,      // 1/2
    UART_FIFO_LEVEL_3_OVER_4,      // 3/4
    UART_FIFO_LEVEL_7_OVER_8      // 7/8
} eUART_FIFO_LEVEL;

/**
  * @brief  UART interrupt type enumeration
  */
typedef enum {
    UART_INTERRUPT_RX = 0,
    UART_INTERRUPT_TX,
    UART_INTERRUPT_MAX
} eUART_INTTYPE;

/**
  * @brief  DMA handle structure definition
  */
typedef struct _tUART_HANDLE {
    eUART_BANK       bank;		// Uart bank
    tUART_REG*       reg;		// Base Address
    eUART_BAUDRATE   baudRate;  // Baud rate
    eUART_WORDLENGTH wordLength;// Word length
} tUART_HANDLE;

/**
  * @}
  */

/** @addtogroup UART_External_Func
  * @{
  */

/** @addtogroup UART_External_Func_G1
  * @{
  */

// init/deinit functions
int UART_Init(tUART_HANDLE* handle);
int UART_DeInit(tUART_HANDLE* handle);

/**
  * @}
  */

/** @addtogroup UART_External_Func_G2
  * @{
  */

int UART_EnableUart(tUART_HANDLE* handle, eUART_MODE mode);
int UART_DisableUart(tUART_HANDLE* handle, eUART_MODE mode);
int UART_EnableFifo(tUART_HANDLE* handle, eUART_FIFO_LEVEL txLevel, eUART_FIFO_LEVEL rxLevel);
int UART_DisableFifo(tUART_HANDLE* handle);
int UART_EnableInterrupt(tUART_HANDLE* handle, eUART_MODE mode, UARTCB txIntCb, void* txIntCbParam, UARTCB rxIntCb, void* rxIntCbParam);
int UART_DisableInterrupt(tUART_HANDLE* handle, eUART_MODE mode);
int UART_SetBaudRate(tUART_HANDLE* handle);
int UART_SetWordLength(tUART_HANDLE* handle);
void UART_SetStopbit(tUART_HANDLE *h, eUART_STOPBIT b);
void UART_SetParity(tUART_HANDLE *h, eUART_PARITY p);
int UART_Transmit(tUART_HANDLE* handle, uint8_t* data, uint32_t len, uint32_t timeOut);
void UART_TransmitChar(tUART_HANDLE* handle, uint8_t data);
int UART_Receive(tUART_HANDLE* handle, uint8_t* data, uint32_t len, uint32_t timeOut);
uint8_t UART_ReceiveChar(tUART_HANDLE* handle);
int UART_DataAvailable(tUART_HANDLE *h);
int UART_FifoFull(tUART_HANDLE *h);


#ifdef __ENABLE_DMA__
int UART_EnableDma(tUART_HANDLE* handle, eUART_MODE mode);
int UART_DisableDma(tUART_HANDLE* handle, eUART_MODE mode);
int UART_TransmitDma(tUART_HANDLE* handle, uint8_t* data, uint32_t len, tDMA_HANDLE* dmaHandle);
int UART_ReceiveDma(tUART_HANDLE* handle, uint8_t* data, uint32_t len, tDMA_HANDLE* dmaHandle);
#endif
/**
  * @}
  */

/** @addtogroup UART_External_Func_G3
  * @{
  */

// Print functions
#ifdef __ENABLE_UART_P__
void UART_PrintInit(tUART_HANDLE* handle);
void UART_PrintDeInit(tUART_HANDLE* handle);
int UART_Printf(const char *fmt, ...);
#else
#define UART_PrintInit(handle)
#define UART_PrintDeInit(handle)
#define UART_Printf(fmt, ...);
#endif

/**
  * @}
  */

/** @addtogroup UART_External_Func_G4
  * @{
  */
uint16_t UART_GetPartNum(tUART_HANDLE* handle);
uint8_t  UART_GetDesigner(tUART_HANDLE* handle);
uint8_t  UART_GetRevision(tUART_HANDLE* handle);
uint8_t  UART_GetConfiguration(tUART_HANDLE* handle);
uint32_t UART_GetId(tUART_HANDLE* handle);

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
#ifdef __cplusplus
}
#endif

#endif // __EWBM_UART_H__
