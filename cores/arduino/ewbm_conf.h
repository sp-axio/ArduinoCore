/**
  ******************************************************************************
  * @file    ewbm_conf.h
  * @author  eWBM
  * @version v0.1
  * @date    06-06-2016
  * @brief   Header file of eWBM MS500 configure
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

#ifndef __EWBM_CONF_H__
#define __EWBM_CONF_H__

typedef enum 
{
    EWBMOK      = 0,
    EWBMERR     = -1,
    EWBMMEM     = -2,
    EWBMINVARG  = -4,
    EWBMTMOUT   = -5
} eEWBM_ERROR;

#ifndef NULL
    #define NULL         ((void*)0)
#endif

#ifndef bool
    #define bool unsigned char
#endif

#ifndef true
    #define true (1)
#endif

#ifndef false
    #define false (0)
#endif

#endif // __EWBM_CONF_H__

