/**
  ******************************************************************************
  * @file    ewbm_spi.c
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   SPI module driver
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
#include "ewbm_spi.h"
#include "ewbm_isr.h"
#include "ewbm_clock.h"
#include "ewbm_delay.h"
#include "ewbm_spi_arm.h"
#include "ewbm_spi_ewbm.h"

#ifdef SPI_DBG_ENABLE
    #include "ewbm_uart.h"
    #define SPI_DBG(fmt,args...)        UART_Printf(fmt,##args)
#else
    #define SPI_DBG(fmt,args...)
#endif

/** @addtogroup EWBM_Peripheral
  * @{
  */

/** @addtogroup EWBM_SPI_COMMON
  * @{
  */

/** @defgroup SPI_Internal_Types SPI Intenal Types
  * @{
  */

/**
  * @brief  SPI clock reset bit defines
  */
#define SPI_CLK_RESET(x)       (1<<(16+x))

/**
  * @brief  tSPI_INTRCB structure that use to register interrupt
  */
typedef struct _tSPI_INTRCB {
    SPICB  cb;
    void*  param;
} tSPI_INTRCB;

/**
  * @brief  Internal global arrangement variables that have a structure of tSPI_INTRCB
  */
static tSPI_INTRCB gSpiIntrCb[SPI_BANK_MAX][SPI_INTR_MODE_MAX];
/**
  * @}
  */

/** @addtogroup SPI_Static_Func
  * @{
  */

/** @addtogroup SPI_Static_Func_G1
  * @{
  */

/**
  * @brief  Get base address about banc of SPI
  * @param  bank: bank number of spi to use
  * @retval pointer to base address
  */
static void* spi_get_baseaddr(eSPI_BANK bank)
{
    switch ( bank )
    {
        case SPI_BANK_1:
            return SPI1;

        case SPI_BANK_2:
            return SPI2;

        case SPI_BANK_3:
            return SPI3;

        default:
            break;
    }

    return NULL;
}

/**
  * @brief  SPI interrupt callback. call the registered callback function according to inerrupt type
  * @param  priv: pointer to parameter of interrupt
  * @retval none
  */
static void spi_intr_cb(void *priv)
{
    tSPI_HANDLE* handle = (tSPI_HANDLE*)priv;
    eSPI_INTR_MODE mask;

    if ( handle->bank == SPI_BANK_3 )
        mask = SPI_EWBM_GetIntrMask((tSPI3_REG*)handle->reg);
    else
        mask = SPI_ARM_GetIntrMask((tSPI_REG*)handle->reg);

    if ( mask == SPI_INTR_MODE_TX )
    {
        gSpiIntrCb[handle->bank][SPI_INTR_MODE_TX].cb(gSpiIntrCb[handle->bank][SPI_INTR_MODE_TX].param);
    }
    else
    if ( mask == SPI_INTR_MODE_RX )
    {
        gSpiIntrCb[handle->bank][SPI_INTR_MODE_RX].cb(gSpiIntrCb[handle->bank][SPI_INTR_MODE_RX].param);
    }
    else
    if ( mask == SPI_MIS_RTMIS )
    {
        gSpiIntrCb[handle->bank][SPI_INTR_MODE_RT].cb(gSpiIntrCb[handle->bank][SPI_INTR_MODE_RT].param);
    }
    else
    if ( mask == SPI_MIS_RORMIS )
    {
        gSpiIntrCb[handle->bank][SPI_INTR_MODE_ROR].cb(gSpiIntrCb[handle->bank][SPI_INTR_MODE_ROR].param);
    }

    if ( handle->bank == SPI_BANK_3 )
        mask = SPI_EWBM_IntrClear((tSPI3_REG*)handle->reg, mask);
    else
        mask = SPI_ARM_IntrClear((tSPI_REG*)handle->reg, mask);
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup SPI_External_Func
  * @{
  */

/** @addtogroup SPI_External_Func_G1
  *  @brief    Initialization and Configuration functions
  * @{
  */

/**
  * @brief  Initializes the Master of SPI according to the specified parameters
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI mode
  * @retval  if success, return EWBMOK value
  */

int SPI_Init(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL )
        return EWBMINVARG;

    handle->reg = spi_get_baseaddr(handle->bank);
    if ( handle->reg == NULL )
        return EWBMINVARG;

    // Reset SPI clk
    CLKRST->peri_rst = SPI_CLK_RESET(handle->bank);
    while ( !(CLKRST->peri_rst & SPI_CLK_RESET(handle->bank)) );

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_Init((tSPI3_REG*)handle->reg);
    else
        ret = SPI_ARM_Init((tSPI_REG*)handle->reg);

    return ret;
}

/**
  * @brief  Initializes the Slave of SPI according to the specified parameters
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI mode
  * @retval  if success, return EWBMOK value
  */

int SPI_SlaveInit(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL )
        return EWBMINVARG;

    handle->reg = spi_get_baseaddr(handle->bank);
    if ( handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_SlaveInit((tSPI3_REG*)handle->reg);
    else
        ret = SPI_ARM_SlaveInit((tSPI_REG*)handle->reg);

    return ret;
}

/**
  * @brief  DeInitializes the SPI
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @retval  if success, return EWBMOK value
  */

int SPI_DeInit(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_DeInit((tSPI3_REG*)handle->reg);
    else
        ret = SPI_ARM_DeInit((tSPI_REG*)handle->reg);

    handle->reg = NULL;

    return ret;
}

/**
  * @}
  */


/** @addtogroup SPI_Func_G2
  *  @brief    Operation functions
  * @{
  */

/**
  * @brief  Set to Prescale configuration
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @retval  if success, return EWBMOK value
  */

int SPI_PreScale(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_PreScale((tSPI3_REG*)handle->reg, handle->prescale);
    else
        ret = SPI_ARM_PreScale((tSPI_REG*)handle->reg, handle->prescale);

    return ret;
}

/**
  * @brief  Set to Clock polarity configuration
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @retval  if success, return EWBMOK value
  */

int SPI_ClockPolarity(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_ClockPolarity((tSPI3_REG*)handle->reg, handle->clockPolarity);
    else
        ret = SPI_ARM_ClockPolarity((tSPI_REG*)handle->reg, handle->clockPolarity);

    return ret;
}

/**
  * @brief  Set to Clock Phase configuration
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @retval  if success, return EWBMOK value
  */

int SPI_ClockPhase(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_ClockPhase((tSPI3_REG*)handle->reg, handle->clockPhase);
    else
        ret = SPI_ARM_ClockPhase((tSPI_REG*)handle->reg, handle->clockPhase);

    return ret;
}

/**
  * @brief  Set to data size
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @retval  if success, return EWBMOK value
  */

int SPI_DataSize(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_DataSize((tSPI3_REG*)handle->reg, handle->dataFormat);
    else
        ret = SPI_ARM_DataSize((tSPI_REG*)handle->reg, handle->dataFormat);

    return ret;
}

/**
  * @brief  Set to Frame Format
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @retval  if success, return EWBMOK value
  */

int SPI_FrameFormat(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_FrameFormat((tSPI3_REG*)handle->reg, handle->frameFormat);
    else
        ret = SPI_ARM_FrameFormat((tSPI_REG*)handle->reg, handle->frameFormat);

    return ret;
}

/**
  * @brief  Set to mode select
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @retval  if success, return EWBMOK value
  */

int SPI_ModeSelect(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_ModeSelect((tSPI3_REG*)handle->reg, handle->mode);
    else
        ret = SPI_ARM_ModeSelect((tSPI_REG*)handle->reg, handle->mode);

    return ret;
}

/**
  * @brief  Set to enable of SPI
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @retval  if success, return EWBMOK value
  */

int SPI_Enable(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_Enable((tSPI3_REG*)handle->reg);
    else
        ret = SPI_ARM_Enable((tSPI_REG*)handle->reg);

    return ret;
}

/**
  * @brief  Set to disable of SPI
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @retval  if success, return EWBMOK value
  */

int SPI_Disable(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_Disable((tSPI3_REG*)handle->reg);
    else
        ret = SPI_ARM_Disable((tSPI_REG*)handle->reg);

    return ret;
}

/**
  * @brief  Set to interrupt enable
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @param  txCb : pointer to callback function when tx interrupt occurs.
  * @param  txIntCbParam : Tx callback parameter
  * @param  rxCb : pointer to callback function when rx interrupt occurs.
  * @param  rxIntCbParam : Rx callback parameter
  * @retval  if success, return EWBMOK value
  */

int SPI_EnableInterrupt(tSPI_HANDLE* handle, SPICB txCb, void* txIntCbParam, SPICB rxCb, void* rxIntCbParam)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        SPI_EWBM_EnableInterrupt((tSPI3_REG*)handle->reg, handle->intrMode);
    else
        SPI_ARM_EnableInterrupt((tSPI_REG*)handle->reg, handle->intrMode);

    switch ( handle->intrMode )
    {
        case SPI_INTR_MODE_TX:
            {
                if ( txCb )
                {
                    gSpiIntrCb[handle->bank][SPI_INTR_MODE_TX].cb = txCb;
                    gSpiIntrCb[handle->bank][SPI_INTR_MODE_TX].param = txIntCbParam;
                }
            }
            break;

        case SPI_INTR_MODE_RX:
            {
                if ( rxCb )
                {
                    gSpiIntrCb[handle->bank][SPI_INTR_MODE_RX].cb = rxCb;
                    gSpiIntrCb[handle->bank][SPI_INTR_MODE_RX].param = rxIntCbParam;
                }
            }
            break;

        case SPI_INTR_MODE_TXRX:
            {
                if ( txCb )
                {
                    gSpiIntrCb[handle->bank][SPI_INTR_MODE_TX].cb = txCb;
                    gSpiIntrCb[handle->bank][SPI_INTR_MODE_TX].param = txIntCbParam;
                }

                if ( rxCb )
                {
                    gSpiIntrCb[handle->bank][SPI_INTR_MODE_RX].cb = rxCb;
                    gSpiIntrCb[handle->bank][SPI_INTR_MODE_RX].param = rxIntCbParam;
                }
            }
            break;

		case SPI_INTR_MODE_RT:
			{
				if ( rxCb )
				{
                    gSpiIntrCb[handle->bank][SPI_INTR_MODE_RT].cb = rxCb;
                    gSpiIntrCb[handle->bank][SPI_INTR_MODE_RT].param = rxIntCbParam;
                }
			}
			break;
			
        default:
            return EWBMINVARG;
    }

    external_isr_set((IRQn_Type)(SPI1_IRQn+handle->bank), spi_intr_cb, (void*)handle);

    return EWBMOK;
}

/**
  * @brief  Set to interrupt disable
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @retval  if success, return EWBMOK value
  */

int SPI_DisableInterrupt(tSPI_HANDLE* handle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        SPI_EWBM_DisableInterrupt((tSPI3_REG*)handle->reg, handle->intrMode);
    else
        SPI_ARM_DisableInterrupt((tSPI_REG*)handle->reg, handle->intrMode);

    switch ( handle->intrMode )
    {
        case SPI_INTR_MODE_TX:
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_TX].cb = NULL;
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_TX].param = NULL;
            break;

        case SPI_INTR_MODE_RX:
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_RX].cb = NULL;
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_RX].param = NULL;
            break;

        case SPI_INTR_MODE_RT:
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_RT].cb = NULL;
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_RT].param = NULL;
            break;

        case SPI_INTR_MODE_ROR:
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_ROR].cb = NULL;
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_ROR].param = NULL;
            break;

        case SPI_INTR_MODE_TXRX:
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_TX].cb = NULL;
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_TX].param = NULL;
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_RX].cb = NULL;
            gSpiIntrCb[handle->bank][SPI_INTR_MODE_RX].param = NULL;
            break;

        default:
            return EWBMINVARG;
    }

    external_isr_clear((IRQn_Type)(SPI1_IRQn+handle->bank));

    return EWBMOK;
}

/**
  * @brief  Set to interrupt clear
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI
  * @param mode : interrupt type that want to clear
  * @retval  if success, return EWBMOK value
  */

int SPI_IntrClear(tSPI_HANDLE* handle, eSPI_INTR_MODE mode)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_IntrClear((tSPI3_REG*)handle->reg, mode);
    else
        ret = SPI_ARM_IntrClear((tSPI_REG*)handle->reg, mode);

    return ret;
}

#ifdef __ENABLE_DMA__
/**
  * @brief  Transmit data through DMA
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI module
  * @param  data: pointer to the transmission data
  * @param  len: transmission data length
  * @param  dmaHandle: pointer to a tDMA_HANDLE structure that contains the configuration information for SPI module
  * @retval  if success, return EWBMOK value
  */

int SPI_TransmitDma(tSPI_HANDLE* handle, uint16_t* data, uint32_t len, tDMA_HANDLE* dmaHandle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    dmaHandle->srcAddr = data;

    if ( handle->bank == SPI_BANK_3 )
		dmaHandle->dstAddr = (uint16_t *)&((tSPI3_REG *)handle->reg)->data;
    else
		dmaHandle->dstAddr = (uint16_t *)&((tSPI_REG *)handle->reg)->dr;

    dmaHandle->transLength = len;
    return DMA_Start(dmaHandle);
}

/**
  * @brief  Receive data through DMA
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI module
  * @param  data: pointer to the receive data
  * @param  len: receive data length
  * @param  dmaHandle: pointer to a tDMA_HANDLE structure that contains the configuration information for DMA module
  * @retval  if success, return EWBMOK value
  */
int SPI_ReceiveDma(tSPI_HANDLE* handle, uint16_t* data, uint32_t len, tDMA_HANDLE* dmaHandle)
{
    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
		dmaHandle->srcAddr = (uint16_t *)&((tSPI3_REG *)handle->reg)->data;
    else
        dmaHandle->srcAddr = (uint16_t *)&((tSPI_REG *)handle->reg)->dr;

    dmaHandle->dstAddr = data;
    dmaHandle->transLength = len;
    return DMA_Start(dmaHandle);
}

/**
  * @brief  set DMA enable for SPI
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI module
  * @retval  if success, return EWBMOK value
  */

int SPI_DmaEnable(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_DmaEnable((tSPI3_REG*)handle->reg, handle->dmaMode);
    else
        ret = SPI_ARM_DmaEnable((tSPI_REG*)handle->reg, handle->dmaMode);

    return ret;
}

/**
  * @brief  set DMA disable for SPI
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI module
  * @retval  if success, return EWBMOK value
  */

int SPI_DmaDisable(tSPI_HANDLE* handle)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_DmaDisable((tSPI3_REG*)handle->reg, handle->dmaMode);
    else
        ret = SPI_ARM_DmaDisable((tSPI_REG*)handle->reg, handle->dmaMode);

    return ret;
}
#endif
/**
  * @brief  Transmit data
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI module
  * @param  data: tx data
  * @retval  if success, return EWBMOK value
  */

int SPI_Write(tSPI_HANDLE* handle, uint16_t data)
{
    int ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_Write((tSPI3_REG*)handle->reg, data);
    else
        ret = SPI_ARM_Write((tSPI_REG*)handle->reg, data);

    return ret;
}

/**
  * @brief  Receive data
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI module
  * @retval  return received data
  */

uint16_t SPI_Read(tSPI_HANDLE* handle)
{
    uint16_t ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_Read((tSPI3_REG*)handle->reg);
    else
        ret = SPI_ARM_Read((tSPI_REG*)handle->reg);

    return ret;
}

/**
  * @brief  SPI Status
  * @param  handle: pointer to a tSPI_HANDLE structure that contains the configuration information for SPI module
  * @param  data: check SPI status
  * @retval return SET or RESET value
  */
uint8_t SPI_Status(tSPI_HANDLE* handle, uint8_t data)
{
    uint8_t ret;

    if ( handle == NULL || handle->reg == NULL )
        return EWBMINVARG;

    if ( handle->bank == SPI_BANK_3 )
        ret = SPI_EWBM_Status((tSPI3_REG*)handle->reg, data);
    else
        ret = SPI_ARM_Status((tSPI_REG*)handle->reg, data);

    return ret;
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
