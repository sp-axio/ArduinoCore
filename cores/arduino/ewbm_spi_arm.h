/**
  ******************************************************************************
  * @file    ewbm_spi_arm.h
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

#ifndef __EWBM_SPI_ARM_H__
#define __EWBM_SPI_ARM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ewbm_device.h"
#include "ewbm_spi.h"

/** @addtogroup EWBM_Peripheral
  * @{
  */

/** @addtogroup EWBM_SPI_ARM
  * @{
  */

/** @addtogroup SPI_External_Func
  * @{
  */

/** @addtogroup SPI_External_Func_G1
  * @{
  */

int SPI_ARM_Init(tSPI_REG* reg);
int SPI_ARM_SlaveInit(tSPI_REG* reg);
int SPI_ARM_DeInit(tSPI_REG* reg);

/**
  * @}
  */

/** @addtogroup SPI_External_Func_G2
  * @{
  */

int    SPI_ARM_PreScale(tSPI_REG* reg, uint8_t prescale);
int    SPI_ARM_ClockPolarity(tSPI_REG* reg, uint8_t clockPolarity);
int    SPI_ARM_ClockPhase(tSPI_REG* reg, uint8_t clockPhase);
int    SPI_ARM_DataSize(tSPI_REG* reg, eSPI_DATA_SIZE_SELECT dataSize);
int    SPI_ARM_FrameFormat(tSPI_REG* reg, eSPI_FRAME_FORMAT frameFormat);
int    SPI_ARM_ModeSelect(tSPI_REG* reg, eSPI_MODE_SELECT mode);
int    SPI_ARM_Enable(tSPI_REG* reg);
int    SPI_ARM_Disable(tSPI_REG* reg);
int    SPI_ARM_EnableInterrupt(tSPI_REG* reg, eSPI_INTR_MODE intrMode);
int    SPI_ARM_DisableInterrupt(tSPI_REG* reg, eSPI_INTR_MODE intrMode);
eSPI_INTR_MODE  SPI_ARM_GetIntrMask(tSPI_REG* reg);
int    SPI_ARM_IntrClear(tSPI_REG* reg, eSPI_INTR_MODE mode);
int    SPI_ARM_DmaEnable(tSPI_REG* reg, eSPI_DMA_MODE dmaMode);
int    SPI_ARM_DmaDisable(tSPI_REG* reg, eSPI_DMA_MODE dmaMode);
int    SPI_ARM_Write(tSPI_REG* reg, uint16_t data);
uint16_t        SPI_ARM_Read(tSPI_REG* reg);
uint8_t         SPI_ARM_Status(tSPI_REG* reg, uint8_t data);

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

#endif /*__EWBM_SPI_ARM_H__*/
