/**
  ******************************************************************************
  * @file    clp300_modular.c
  * @author  eWBM
  * @version  1.0
  * @date    2016-06-10
  * @brief   Public Key Accelerator Firmware for  Modular
  *          arithmetic operation
  ******************************************************************************
  * @attention
  *
  * Copyright(c) 2015 ~ 2020 eWBM Korea , Ltd.
  * All rights reserved.
  * This software contains confidential information of eWBM Korea Co.,Ltd.
  * and unauthorized distribution of this software, or any portion of it, are
  * prohibited.
  *
  * Brief history
  * -------------
  *  2016-06-07 :  Draft by Leon Cho
  *  2016-06-08 :  Modular Addition and Subraction module
  *  implement with Test Module
  *  2016-06-10 : Modular Division,Reduction and so on
  *  imaplement
  *
  ******************************************************************************
  */

#include "clp300.h"

#define MODULAR_OP_BUF_SZ   512

static int pka_modular_precompute(uint8_t* m, uint32_t size)
{
    int ret = CRYPTO_OK;
#if !defined(USE_SMALL_STACK)
    uint8_t r_inv[MODULAR_OP_BUF_SZ] = {0};
#else
    uint8_t* r_inv = NULL;

    r_inv = (uint8_t*)calloc(sizeof(uint8_t), MODULAR_OP_BUF_SZ);
    if ( r_inv == NULL )
        return CRYPTO_MEMORY_ERROR;
#endif

    ret = ESAL_PKA_MOD_R_Inverse(m, (void*)r_inv, size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_CALC_MP, 0, size>>3);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"C0", (void*)r_inv, size>>3);
    if ( ret != CRYPTO_OK)
        return ret;

    ret = ESAL_PKA_FirmwareStart( PKA_ENTRY_CALC_R_SQR,0,size>>3);
    if ( ret != CRYPTO_OK )
        return ret;

#if defined(USE_SMALL_STACK)
    if ( r_inv != NULL )
    {
        free(r_inv);
        r_inv = NULL;
    }
#endif

    return CRYPTO_OK;
}

int ESAL_PKA_MOD_R_Inverse(uint8_t* m, uint8_t* r_inv, uint32_t size)
{
    uint32_t temp_sz = 0;
    int ret = 0;

    temp_sz = size>>3;

    ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"D0", m, temp_sz);
    if ( ret != CRYPTO_OK)
        return ret;

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_CALC_R_INV, 0, temp_sz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( r_inv )
    {
        ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"C0", (void*)r_inv, temp_sz);
        if ( ret != CRYPTO_OK)
            return ret;
    }

    return CRYPTO_OK;
}

int ESAL_PKA_MOD_Subraction(uint8_t* a, uint8_t* b, uint8_t* m,  uint8_t* c, uint32_t size)
{
    int ret  = CRYPTO_OK;
    uint32_t bytesz = size>>3;

    if ( !bytesz || (bytesz > 512) )
        return CRYPTO_INVALID_ARG;

    ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"A0", a, bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"B0", b, bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( m != NULL )
    {
        ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"D0", m, bytesz);
        if ( ret != CRYPTO_OK )
            return ret;
    }

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_MODSUB, 0, bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( c != NULL )
    {
        ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"A0", (void*)c, bytesz);
        if ( ret != CRYPTO_OK )
            return ret;
    }

    return CRYPTO_OK;
}

int ESAL_PKA_MOD_Addition(uint8_t* a, uint8_t* b, uint8_t* m, uint8_t* c, uint32_t size)
{
    int ret  = CRYPTO_OK;
    uint32_t bytesz = size>>3;

    if ( !bytesz || (bytesz > 512) )
        return CRYPTO_INVALID_ARG;

    ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"A0", a, bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"B0", b, bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( m != NULL )
    {
        ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"D0", m, bytesz);
        if ( ret != CRYPTO_OK )
            return ret;
    }

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_MODADD, 0, bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( c != NULL )
    {
        ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"A0", (void*)c, bytesz);
        if ( ret != CRYPTO_OK )
            return ret;
    }

    return CRYPTO_OK;
}

int ESAL_PKA_MOD_Multiplication(uint8_t* x, uint8_t* y, uint8_t* m, uint8_t* c, uint32_t size)
{
    int ret = CRYPTO_OK;
    uint32_t bytesz = 0;
    tPKA_CMD_LIST cmdlist[] = {
        {
            .reg_name = (uint8_t*)"A0",
            .reg_val = x
        },
        {
            .reg_name = (uint8_t*)"B0",
            .reg_val = y
        },
    };

    CRYPTO_ASSERT((size ==0)||(size>512));
    CRYPTO_ASSERT((x == (void*)0));
    CRYPTO_ASSERT((y == (void*)0));

    bytesz = size>>3;
    if ( m != NULL )
    {
        ret = pka_modular_precompute(m, size);
        if ( ret != CRYPTO_OK )
            return ret;
    }


    ret = ESAL_PKA_WriteOperandMultiple(cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]), bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_MODMULT,0,bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( c != NULL )
    {
        ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"A0", (void*)c, bytesz);
        if ( ret != CRYPTO_OK )
            return ret;
    }

    return CRYPTO_OK;
}

int ESAL_PKA_MOD_Inversion(uint8_t* x, uint8_t* m, uint8_t* c, uint32_t size)
{
    uint32_t bytesz = size>>3;
    int ret = CRYPTO_OK;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"A0",
            .reg_val = x
        },
        {
            .reg_name = (uint8_t*)"D0",
            .reg_val = m
        }
    };

    CRYPTO_ASSERT((bytesz == 0) || (bytesz > 512));
    CRYPTO_ASSERT(!x);
    CRYPTO_ASSERT(!m);

    ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd)/sizeof(cmd[0]), bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_MODINV,0,bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( c != NULL )
    {
        ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"C0", (void*)c, bytesz);
        if ( ret != CRYPTO_OK )
            return ret;
    }

    return CRYPTO_OK;
}

int ESAL_PKA_MOD_Reduction(uint8_t* x, uint8_t* m, uint8_t* c, uint32_t size)
{
    uint32_t bytesz = size>>3;
    int ret = CRYPTO_OK;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"C0",
            .reg_val = x
        },
        {
            .reg_name = (uint8_t*)"D0",
            .reg_val = m
        }
    };

    CRYPTO_ASSERT((bytesz == 0) || (bytesz > 512));
    CRYPTO_ASSERT(!x);
    CRYPTO_ASSERT(!m);

    ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd)/sizeof(cmd[0]), bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_REDUCE,0,bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( c != NULL )
    {
        ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"A0",(void*)c,bytesz);
        if ( ret != CRYPTO_OK )
            return ret;
    }

    return CRYPTO_OK;
}

int ESAL_PKA_MOD_Divisoin(uint8_t* y, uint8_t* x, uint8_t* m, uint8_t* c, uint32_t size)
{
    uint32_t bytesz = size>>3;
    int ret = CRYPTO_OK;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"C0",
            .reg_val = y
        },
        {
            .reg_name = (uint8_t*)"A0",
            .reg_val = x
        },
        {
            .reg_name = (uint8_t*)"D0",
            .reg_val = m
        }
    };

    CRYPTO_ASSERT((bytesz == 0) || (bytesz > 512));
    CRYPTO_ASSERT((!x)||(!y));
    CRYPTO_ASSERT(!m);

    ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd) / sizeof(cmd[0]), bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_MODDIV, 0, bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( c != NULL )
    {
        ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"C0", (void*)c, bytesz);
        if ( ret != CRYPTO_OK )
            return ret;
    }

    return CRYPTO_OK;
}

int ESAL_PKA_MOD_Exponentiation(uint8_t* x, uint8_t* y, uint8_t* m, uint8_t* c, uint8_t fullwidth, uint32_t size)
{
    uint32_t bytesz = size>>3;
    int ret = CRYPTO_OK;
    unsigned int check_exponent;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"A0",
            .reg_val = x
        },
        {
            .reg_name = (uint8_t*)"D2",
            .reg_val = y
        }
    };

    CRYPTO_ASSERT((size == 0) || (size > 512));
    CRYPTO_ASSERT(!x||!y);

    ret = pka_modular_precompute(m,size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd)/sizeof(cmd[0]), bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( fullwidth )
    {
        check_exponent = (0x1UL << PKA_FLAG_F0);
    }
    else
    {
        check_exponent = 0;
    }

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_MODEXP, check_exponent, bytesz);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( c != NULL )
    {
        ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"A0", (void*)c, bytesz);
        if ( ret != CRYPTO_OK )
            return ret;
    }

    return CRYPTO_OK;
}