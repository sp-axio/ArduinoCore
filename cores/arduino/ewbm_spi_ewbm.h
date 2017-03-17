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

#ifndef __EWBM_SPI_EWBM_H__
#define __EWBM_SPI_EWBM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ewbm_device.h"
#include "ewbm_spi.h"

/** @addtogroup EWBM_Peripheral
  * @{
  */

/** @addtogroup EWBM_SPI_EWBM
  * @{
  */


/** @addtogroup SPI_External_Func
  * @{
  */

/** @addtogroup SPI_External_Func_G1
  * @{
  */

int SPI_EWBM_Init(tSPI3_REG* reg);
int SPI_EWBM_SlaveInit(tSPI3_REG* reg);
int SPI_EWBM_DeInit(tSPI3_REG* reg);

/**
  * @}
  */

/** @addtogroup SPI_External_Func_G2
  * @{
  */

int    SPI_EWBM_PreScale(tSPI3_REG* reg, uint8_t prescale);
int    SPI_EWBM_ClockPolarity(tSPI3_REG* reg, uint8_t clockPolarity);
int    SPI_EWBM_ClockPhase(tSPI3_REG* reg, uint8_t clockPhase);
int    SPI_EWBM_DataSize(tSPI3_REG* reg, eSPI_DATA_SIZE_SELECT dataSize);
int    SPI_EWBM_FrameFormat(tSPI3_REG* reg, eSPI_FRAME_FORMAT frameFormat);
int    SPI_EWBM_ModeSelect(tSPI3_REG* reg, eSPI_MODE_SELECT mode);
int    SPI_EWBM_Enable(tSPI3_REG* reg);
int    SPI_EWBM_Disable(tSPI3_REG* reg);
int    SPI_EWBM_EnableInterrupt(tSPI3_REG* reg, eSPI_INTR_MODE intrMode);
int    SPI_EWBM_DisableInterrupt(tSPI3_REG* reg, eSPI_INTR_MODE intrMode);
eSPI_INTR_MODE  SPI_EWBM_GetIntrMask(tSPI3_REG* reg);
int    SPI_EWBM_IntrClear(tSPI3_REG* reg, eSPI_INTR_MODE mode);
int    SPI_EWBM_DmaEnable(tSPI3_REG* reg, eSPI_DMA_MODE dmaMode);
int    SPI_EWBM_DmaDisable(tSPI3_REG* reg, eSPI_DMA_MODE dmaMode);
int    SPI_EWBM_Write(tSPI3_REG* reg, uint16_t data);
uint16_t        SPI_EWBM_Read(tSPI3_REG* reg);
uint8_t         SPI_EWBM_Status(tSPI3_REG* reg, uint8_t data);

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

#endif /*__EWBM_SPI_EWBM_H__*/
