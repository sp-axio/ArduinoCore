/**
  ******************************************************************************
  * @file    spacc.c
  * @author  eWBM
  * @version  0.1
  * @date    2016-07-01
  * @brief   SPAcc
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
  *  2016-07-01 :
  *
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>

#include "spacc.h"

#define SPACC_DEFAULT_TAG_SIZE  (16)
#define SPACC_AADCOPY_FLAG      (0x80000000)

#define IP_ICV_OFFSET           (0)
#define IP_ICV_APPEND           (1)
#define IP_ICV_IGNORE           (2)

#define ICV_HASH                (0)
#define ICV_HASH_ENCRYPT        (1)
#define ICV_ENCRYPT_HASH        (2)
#define ICV_IGNORE              (3)


#define SPACC_ASSERT(x) \
   ({ \
      if(x) {  \
         return SPACC_FAILED;  \
      }  \
   })

typedef struct _tSPACC_DDT
{
    void*    data;
    uint32_t dataSz;
    uint32_t mustBeNull[2];
} tSPACC_DDT;


static uint32_t gTagSz   = SPACC_DEFAULT_TAG_SIZE;
static uint32_t gKeyMode = 0;

static inline void* spacc_memcpy(void* dest, const void* src, uint32_t len)
{
    uint32_t* firsts = (uint32_t*)src;
    uint32_t* firstd = (uint32_t*)dest;

    while ( len != 0 )
    {
        *firstd++ = *firsts++;
        len = len - 4;
    }

    return firstd;
}

static void DdtInit(tSPACC_DDT* ddt, void* data, int dataSz)
{
    ddt->data          = data;
    ddt->dataSz        = dataSz;
    ddt->mustBeNull[0] = 0;
    ddt->mustBeNull[1] = 0;
}

static int SPAccWaitForComplete_Ex(void)
{
    tSPACC_REG* baseAddr = SPACC;
    uint32_t stat;

    while ( baseAddr->fifo_stat & 0x80000000 )
    {
        continue;
    }

    baseAddr->stat_pop = 0x00000001;   /* STAT_POP */
    stat = baseAddr->status;           /* STATUS */
    stat = (stat >> 24) & 0xff;
    // Result of operation:
    //   0=Ok,
    //   1=ICV Fail
    //   2=Memory Error
    //   3=Block Error
    //   4=Security Error

    return stat;
}

static int spacc_packet_enqueue_ddt_ex(tSPACC_CONFIG* config)
{
    tSPACC_REG* baseAddr = SPACC;
    tSPACC_DDT  srcphy __attribute__((aligned (8)));
    tSPACC_DDT  dstphy __attribute__((aligned (8)));
    tSPACC_DDT* pSrcDDT;
    tSPACC_DDT* pDstDDT;

    SPACC_ASSERT((config->encMode != SPACC_OP_DECRYPT) && (config->encMode != SPACC_OP_ENCRYPT));

    pSrcDDT = &srcphy;
    pDstDDT = &dstphy;

    if ( (config->aesMode == SPACC_AES_CCM) || (config->aesMode == SPACC_AES_GCM) )
    {
        DdtInit(pSrcDDT, (void*)config->in, config->inSz + SPACC_DUMMY_SIZE_FORCCMGCM);
        DdtInit(pDstDDT, (void*)config->out, config->inSz + SPACC_DUMMY_SIZE_FORCCMGCM);
    }
    else
    {
        DdtInit(pSrcDDT, (void*)config->in, config->inSz);
        DdtInit(pDstDDT, (void*)config->out, config->inSz);
    }

    config->ctrl.asFields.ENCRYPT = config->encMode;
    if ( config->encMode == SPACC_OP_DECRYPT )
        config->ctrl.asFields.KEY_EXP = 0x1;

    if ( config->preAddSz & SPACC_AADCOPY_FLAG )
    {
        config->ctrl.asFields.AAD_COPY = 0x1;
        config->preAddSz &= ~(SPACC_AADCOPY_FLAG);
        config->ivOffset = SPACC_AADCOPY_FLAG;
    }
    else
    {
        config->ctrl.asFields.AAD_COPY = 0x0;
        config->ivOffset = 0;
    }

    switch ( config->icvCmd )
    {
        case IP_ICV_OFFSET:
            config->icvOffset = config->inSz + config->aadSz;
            config->ctrl.asFields.ICV_APPEND = 0;
            break;

        case IP_ICV_APPEND:
            config->ctrl.asFields.ICV_APPEND = 1;
            break;
    }

    baseAddr->src_ptr = (uint32_t)pSrcDDT;
    baseAddr->dst_ptr = (uint32_t)pDstDDT;

    switch ( config->keyMode )
    {
        case SPACC_KEY_DUK:
        case SPACC_KEY_PKF:
        case SPACC_KEY_UKEY:
            config->ctrl.asFields.SEC_KEY = 0x1;
            break;

        default:
            config->ctrl.asFields.SEC_KEY = 0x0;
            break;
    }

    baseAddr->proc_len = config->inSz - config->postAddSz + config->aadSz;
    baseAddr->icv_len = config->icvSz;
    baseAddr->icv_offset = config->icvOffset;
    baseAddr->pre_add_len = config->preAddSz;
    baseAddr->post_add_len = config->postAddSz;
    baseAddr->offset = config->aadOffset + (config->aadOffset << 16);
    baseAddr->iv_offset = config->ivOffset;
    baseAddr->aux_info = 0;

    switch ( config->aesMode )
    {
        case SPACC_AES_CBC:
        case SPACC_AES_CTR:
        case SPACC_AES_CCM:
        case SPACC_AES_GCM:
            if ( config->iv != NULL )
            {
                // Initial Vector Copy
                spacc_memcpy((uint32_t*)&baseAddr->ctx_ciph_key[config->keyMode][0x20], config->iv, config->ivSz);
                if ( (config->ivSz == 12) && (config->aesMode == SPACC_AES_GCM) )
                {
                    uint8_t one[4] = {0, 0, 0, 1};
                    spacc_memcpy((uint32_t*)&baseAddr->ctx_ciph_key[config->keyMode][0x2c], one, 4);
                }
            }
            break;
    }

   baseAddr->ctrl = config->ctrl.as32Bits;
   SPAccWaitForComplete_Ex();

   return SPACC_OK;
}

static int spacc_cipher_set_operation(tSPACC_CONFIG* config)
{
    config->ctrl.as32Bits = 0;
    if ( (config->aesMode == SPACC_AES_GCM) || (config->aesMode == SPACC_AES_CCM) )
    {
        switch ( config->prot )
        {
            case ICV_HASH:
                config->ctrl.asFields.ICV_PT = 0x1;
                break;

            case ICV_HASH_ENCRYPT:
                config->ctrl.asFields.ICV_ENC    = 0x1;
                config->ctrl.asFields.ICV_PT     = 0x1;
                config->ctrl.asFields.ICV_APPEND = 0x1;
                break;

            case ICV_ENCRYPT_HASH:
                config->ctrl.asFields.ICV_ENC = 0x0;
                config->ctrl.asFields.ICV_PT  = 0x0;
                break;

            default:
                return SPACC_INVALID_ALG;
        }

        config->icvSz = gTagSz;
    }

    return SPACC_OK;
}

static int spacc_cipher_write_context(tSPACC_CONFIG* config)
{
    tSPACC_REG* baseAddr = SPACC;

    SPACC_ASSERT((config == NULL) || (config->iv == NULL));
    SPACC_ASSERT((config->keyMode < SPACC_KEY_NORMAL) || (config->keyMode > SPACC_KEY_DUK));

    config->ctrl.asFields.CIPH_ALG  = 0x2;
    config->ctrl.asFields.CTX_IDX   = config->keyMode;
    config->ctrl.asFields.CIPH_MODE = config->aesMode;
    config->ctrl.asFields.MSG_BEGIN = 0x1;
    config->ctrl.asFields.MSG_END   = 0x1;

    switch ( config->aesMode )
    {
        case SPACC_AES_ECB:
        case SPACC_AES_CBC:
        case SPACC_AES_CTR:
        case SPACC_AES_CCM:
        case SPACC_AES_GCM:
            if ( config->keyMode != SPACC_KEY_NORMAL )
            {
                spacc_memcpy((uint32_t*)&baseAddr->ctx_ciph_key[config->keyMode][0x20], config->iv, config->ivSz);
            }
            else
            {
                if ( config->key == NULL )
                    return SPACC_MEMORY_ERROR;

                spacc_memcpy((uint32_t*)&baseAddr->ctx_ciph_key[config->keyMode][0x00], config->key, config->keySz);
            }
            break;

        default:
            return SPACC_INVALID_ALG;
    }

    baseAddr->key_sz = (0x1<<31)|(config->keyMode<<8)|(config->keySz<<0);

    return SPACC_OK;
}


void ESAL_SPACC_SetContext(tSPACC_CONFIG* config, eSPACC_KEY_MODE keyMode, eSPACC_OPERATION op, eSPACC_AES_MODE aesMode)
{
    config->keyMode   = keyMode;
    config->operation = op;
    config->aesMode   = aesMode;
    gKeyMode          = config->keyMode;
    config->postAddSz = 0;
    config->aadOffset = 0;
}

int ESAL_SPACC_SetCryptoAlgoContext(tSPACC_CONFIG* config, uint8_t* key, uint32_t keySz, uint8_t* iv, uint32_t ivSz, uint32_t aadSz)
{
    int ret = SPACC_OK;
    SPACC_ASSERT(!config);

    switch ( config->operation )
    {
        case SPACC_OP_AES:
            {
                config->key = key;
                config->keySz = keySz;
                config->iv = iv;
                config->ivSz = ivSz;

                switch ( config->aesMode )
                {
                    case SPACC_AES_GCM:
                    case SPACC_AES_CCM:
                        config->prot = ICV_ENCRYPT_HASH;
                        config->icvCmd = IP_ICV_OFFSET;
                        config->aadSz = aadSz;
                        config->preAddSz = aadSz;
                        break;

                    default:
                        config->prot = 0;
                        config->icvCmd = 0;
                        config->aadSz = 0;
                        config->preAddSz = 0;
                        break;
                }

                ret = spacc_cipher_set_operation(config);
                if ( ret != SPACC_OK )
                    return ret;

                ret = spacc_cipher_write_context(config);
                if ( ret != SPACC_OK )
                    return ret;
            }
            break;

        default:
            return SPACC_INVALID_ALG;

    }

    return SPACC_OK;
}


int ESAL_SPACC_AESEnqueueEx(tSPACC_CONFIG* config, const uint8_t* in, uint8_t* out, uint32_t inSz, eSPACC_ENC_MODE encMode)
{
    switch ( config->operation )
    {
        case SPACC_OP_AES:
            {
                config->encMode = encMode;
                config->in  = in;
                config->out  = out;
                config->inSz  = inSz;
                spacc_packet_enqueue_ddt_ex(config);
            }
            break;

        default:
            return SPACC_INVALID_ARGUMENT;
    }

    return SPACC_OK;
}


int ESAL_SPACC_HashEx(void* in, void* out, uint32_t inSz, uint32_t outSz, int mode, const uint8_t* hmacKey, uint32_t hmacKeySz, uint32_t ctxId)
{
    tSPACC_REG* baseAddr = SPACC;

    int retCode = 0;
    tSPACC_CTRL_REG     spacc_ctrl_reg;
    tSPACC_KEY_SZ_REG   spacc_key_sz_reg;
    tSPACC_DDT* pSrcDDT;
    tSPACC_DDT* pDstDDT;

    // RLH_TODO: this may not work for gcc
    tSPACC_DDT srcDDT __attribute__((aligned(8)));
    tSPACC_DDT dstDDT __attribute__((aligned(8)));
    pSrcDDT =  &srcDDT;
    pDstDDT =  &dstDDT;

    // create DDT for message
    DdtInit(pSrcDDT, (void*)in, inSz);
    DdtInit(pDstDDT, (void*)out, outSz);

    // 1.  Allocate a handle
    // we set the cipher mode to CRYPTO_MODE_NULL since we are only hashing
    spacc_ctrl_reg.as32Bits = 0;


    switch ( mode )
    {
        case SPACC_HMODE_HMAC_SHA1:
            spacc_ctrl_reg.asFields.HASH_ALG  = 0x2;
            spacc_ctrl_reg.asFields.HASH_MODE = 0x2;
            break;

        case SPACC_HMODE_HMAC_SHA256:
            spacc_ctrl_reg.asFields.HASH_ALG  = 0x4;
            spacc_ctrl_reg.asFields.HASH_MODE = 0x2;
            break;

        case SPACC_HMODE_SSLMAC_SHA1:
            spacc_ctrl_reg.asFields.HASH_ALG  = 0x2;
            spacc_ctrl_reg.asFields.HASH_MODE = 0x1;
            break;

        case SPACC_HMODE_HASH_SHA1:
            spacc_ctrl_reg.asFields.HASH_ALG  = 0x2;
            spacc_ctrl_reg.asFields.HASH_MODE = 0x0;
            break;

        case SPACC_HMODE_HASH_SHA256:
            spacc_ctrl_reg.asFields.HASH_ALG  = 0x4;
            spacc_ctrl_reg.asFields.HASH_MODE = 0x0;
            break;

        default:
            return SPACC_INVALID_MODE;

    }

    spacc_ctrl_reg.asFields.MSG_BEGIN = 1;
    spacc_ctrl_reg.asFields.MSG_END   = 1;
    spacc_ctrl_reg.asFields.CTX_IDX   = ctxId;

    // 2.  Set context
    // here we write the HMAC key (if any) to the context page for this job
    // if no key provided, then use the Secure Context.

    // if hmac key is null, then set to use SKP CTX.
    // RLH_TODO: actually this not possible so remove.
    if ( hmacKey == NULL )
    {
        spacc_ctrl_reg.asFields.SEC_KEY = 1;
    }
    else
    {
        spacc_ctrl_reg.asFields.SEC_KEY = 0;
        if ( (mode == SPACC_HMODE_HMAC_SHA1) || (mode == SPACC_HMODE_HMAC_SHA256) )
        {
            // can only access spacc memory as 32 bit words so use special memcpy
            spacc_memcpy((uint32_t*)(baseAddr->ctx_hash_key[ctxId]), hmacKey, hmacKeySz);
        }
    }

    // 3.  Set operation
    // OP_ENCRYPT means to produce a hash/hmac output
    // we set all of the ICV options to zero since we want the ICV (hash output) at the start of the output buffer
    // setting icvcmd to IP_ICV_APPEND is handy if we want to cipher/hash in the same SPAcc command
    spacc_ctrl_reg.asFields.ENCRYPT    = 1;
    spacc_ctrl_reg.asFields.ICV_PT     = 1;
    spacc_ctrl_reg.asFields.ICV_APPEND = 0;

    // start the job (we pass the job size as pre_aad_sz since we only want the HMAC-MD5 tag as output not the entire message)
    // proc_len   == size of message
    // pre_aad_sz == size of message, since we are not setting AAD_COPY then the output from the SPAcc is simply the HASH tag
    //               if we set pre_aad_sz to less than proc_len then it'll still produce the valid hash but it will also output
    //               some of the plaintext to the output buffer which we do not want
    // post_aad_sz == 0 since we consume all of the message in the pre_aad_sz

    // setup ddt addresses
    baseAddr->src_ptr        = (uint32_t)pSrcDDT;
    baseAddr->dst_ptr        = (uint32_t)pDstDDT;

    baseAddr->proc_len       = inSz;
    baseAddr->icv_len        = 0;
    baseAddr->icv_offset     = 0;
    baseAddr->pre_add_len    = inSz;
    baseAddr->post_add_len   = 0;
    baseAddr->iv_offset      = 0;
    baseAddr->offset         = 0;
    baseAddr->aux_info       = 0;  // currently not used for the supported AES or HASH modes

    spacc_key_sz_reg.as32Bits = 0;
    spacc_key_sz_reg.asFields.SIZE    = hmacKeySz;
    spacc_key_sz_reg.asFields.CTX_TDX = ctxId;
    spacc_key_sz_reg.asFields.CIPHER  = 0;

    baseAddr->key_sz = spacc_key_sz_reg.as32Bits;

    // writing ctrl starts the process
    baseAddr->ctrl = spacc_ctrl_reg.as32Bits;


    // wait for the job to complete and allow the user to abort if need be
    retCode = SPAccWaitForComplete_Ex();

    return retCode;
}

void ESAL_SPACC_SetTagLen(uint32_t tagSz)
{
    gTagSz = tagSz;
}

void ESAL_SPACC_GetIV(uint8_t* iv, uint32_t ivSz)
{
    tSPACC_REG* baseAddr = SPACC;

    uint32_t* ciph_iv = (uint32_t*)&baseAddr->ctx_ciph_key[gKeyMode][0x20];

    spacc_memcpy((uint32_t*)iv, (void*)(ciph_iv), ivSz);
}