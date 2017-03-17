/**
  ******************************************************************************
  * @file    ewbm_spi.h
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   Header file of SPI module.
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

#ifndef __EWBM_SPI_H__
#define __EWBM_SPI_H__

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

/** @addtogroup EWBM_SPI_COMMON
  * @{
  */

/** @defgroup SPI_External_Types Exported Types
  * @{
  */

/**
  * @brief  SPI bank enumeration
  */

typedef enum _eSPI_BANK {
    SPI_BANK_1 = 0,
    SPI_BANK_2,
    SPI_BANK_3,
    SPI_BANK_MAX
}eSPI_BANK;

/**
  * @brief  SPI mode enumeration for Master/Slave mode
  */

typedef enum _eSPI_MODE_SELECT {
    SPI_MASTER_MODE = 0,
    SPI_SLAVE_MODE
} eSPI_MODE_SELECT;

/**
  * @brief  SPI mode enumeration for Frame Format
  */

typedef enum _eSPI_FRAME_FORMAT {
    SPI_MOTOROLA_SPI_FRAME       = 0,
    SPI_TI_SYNC_SERIAL_FRAME,
    SPI_NATIONAL_MICROWIRE_FRAME,
    SPI_RESERVED
} eSPI_FRAME_FORMAT;

/**
  * @brief  SPI mode enumeration for data size
  */

typedef enum _eSPI_DATA_SIZE_SELECT {
    SPI_DSS_04_BIT_DATA = 3,
    SPI_DSS_05_BIT_DATA,
    SPI_DSS_06_BIT_DATA,
    SPI_DSS_07_BIT_DATA,
    SPI_DSS_08_BIT_DATA,
    SPI_DSS_09_BIT_DATA,
    SPI_DSS_10_BIT_DATA,
    SPI_DSS_11_BIT_DATA,
    SPI_DSS_12_BIT_DATA,
    SPI_DSS_13_BIT_DATA,
    SPI_DSS_14_BIT_DATA,
    SPI_DSS_15_BIT_DATA,
    SPI_DSS_16_BIT_DATA
} eSPI_DATA_SIZE_SELECT;

/**
  * @brief  SPI mode enumeration for interrupt mode
  */

typedef enum _eSPI_INTR_MODE {
    SPI_INTR_MODE_ROR = 0,
    SPI_INTR_MODE_RT,
    SPI_INTR_MODE_RX,
    SPI_INTR_MODE_TX,
    SPI_INTR_MODE_TXRX,
    SPI_INTR_MODE_MAX
} eSPI_INTR_MODE;

/**
  * @brief  SPI mode enumeration for DMA mode
  */

typedef enum _eSPI_DMA_MODE {
    SPI_DMA_MODE_TX = 0,
    SPI_DMA_MODE_RX,
    SPI_DMA_MODE_TXRX,
} eSPI_DMA_MODE;

/**
  * @brief  SPI interrupt callback definition
  */

typedef void (*SPICB)(void*);

/**
  * @brief  SPI handle structure definition
  */

typedef struct _tSPI_HANDLE {
    eSPI_BANK             bank;
    eSPI_INTR_MODE        intrMode;
    eSPI_MODE_SELECT      mode;          /* Master, Slave    */
    eSPI_DMA_MODE         dmaMode;       /* DAM Mode         */
    eSPI_DATA_SIZE_SELECT dataFormat;    /* Data size format */
    eSPI_FRAME_FORMAT     frameFormat;   /* Frame Format     */
    uint8_t               prescale;      /* Prescale         */
    uint8_t               clockPolarity; /* Clock polarity   */
    uint8_t               clockPhase;    /* Clock Phase      */
    void*                 reg;
} tSPI_HANDLE;

/**
  * @}
  */

/** @addtogroup SPI_External_Func
  * @{
  */

/** @addtogroup SPI_External_Func_G1
  * @{
  */

int SPI_Init(tSPI_HANDLE* handle);
int SPI_SlaveInit(tSPI_HANDLE* handle);
int SPI_DeInit(tSPI_HANDLE* handle);

/**
  * @}
  */

/** @addtogroup SPI_External_Func_G2
  * @{
  */

int SPI_PreScale(tSPI_HANDLE* handle);
int SPI_ClockPolarity(tSPI_HANDLE* handle);
int SPI_ClockPhase(tSPI_HANDLE* handle);
int SPI_DataSize(tSPI_HANDLE* handle);
int SPI_FrameFormat(tSPI_HANDLE* handle);
int SPI_ModeSelect(tSPI_HANDLE* handle);
int SPI_Enable(tSPI_HANDLE* handle);
int SPI_Disable(tSPI_HANDLE* handle);
int SPI_EnableInterrupt(tSPI_HANDLE* handle, SPICB txCb, void* txIntCbParam, SPICB rxCb, void* rxIntCbParam);
int SPI_DisableInterrupt(tSPI_HANDLE* handle);
int SPI_IntrClear(tSPI_HANDLE* handle, eSPI_INTR_MODE mode);
#ifdef __ENABLE_DMA__
int SPI_TransmitDma(tSPI_HANDLE* handle, uint16_t* data, uint32_t len, tDMA_HANDLE* dmaHandle);
int SPI_ReceiveDma(tSPI_HANDLE* handle, uint16_t* data, uint32_t len, tDMA_HANDLE* dmaHandle);
int SPI_DmaEnable(tSPI_HANDLE* handle);
int SPI_DmaDisable(tSPI_HANDLE* handle);
#endif
int SPI_Write(tSPI_HANDLE* handle, uint16_t data);
uint16_t     SPI_Read(tSPI_HANDLE* handle);
uint8_t      SPI_Status(tSPI_HANDLE* handle, uint8_t data);

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

#endif /*__EWBM_SPI_H__*/
