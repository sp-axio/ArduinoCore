#ifndef __SPACC_H__
#define __SPACC_H__

#include "ewbm_device.h"

#define SPACC_DUMMY_SIZE_FORCCMGCM     128

typedef enum
{
    SPACC_OK                      =   0,
    SPACC_FAILED                  =  -1,
    SPACC_INPROGRESS              =  -2,
    SPACC_INVALID_HANDLE          =  -3,
    SPACC_INVALID_CONTEXT         =  -4,
    SPACC_INVALID_SIZE            =  -5,
    SPACC_NOT_INITIALIZED         =  -6,
    SPACC_NO_MEM                  =  -7,
    SPACC_INVALID_ALG             =  -8,
    SPACC_INVALID_KEY_SIZE        =  -9,
    SPACC_INVALID_ARGUMENT        = -10,
    SPACC_MODULE_DISABLED         = -11,
    SPACC_NOT_IMPLEMENTED         = -12,
    SPACC_INVALID_BLOCK_ALIGNMENT = -13,
    SPACC_INVALID_MODE            = -14,
    SPACC_INVALID_KEY             = -15,
    SPACC_AUTHENTICATION_FAILED   = -16,
    SPACC_INVALID_IV_SIZE         = -17,
    SPACC_MEMORY_ERROR            = -18,
    SPACC_LAST_ERROR              = -19,
    SPACC_HALTED                  = -20,
    SPACC_TIMEOUT                 = -21,
    SPACC_SRM_FAILED              = -22,
    SPACC_COMMON_ERROR_MAX        =-100,
    SPACC_INVALID_ICV_KEY_SIZE    =-100,
    SPACC_INVALID_PARAMETER_SIZE  =-101,
    SPACC_SEQUENCE_OVERFLOW       =-102,
    SPACC_DISABLED                =-103,
    SPACC_INVALID_VERSION         =-104,
    SPACC_FATAL                   =-105,
    SPACC_INVALID_PAD             =-106,
    SPACC_FIFO_FULL               =-107,
    SPACC_INVALID_SEQUENCE        =-108,
    SPACC_INVALID_FIRMWARE        =-109,
    SPACC_NOT_FOUND               =-110,
    SPACC_CMD_FIFO_INACTIVE       =-111
} eSPACC_ERROR;

typedef enum
{
    SPACC_OP_AES     = 0,
    SPACC_OP_HASH
} eSPACC_OPERATION;

typedef enum
{
    SPACC_AES_ECB = 0,
    SPACC_AES_CBC = 1,
    SPACC_AES_CTR = 2,
    SPACC_AES_CCM = 3,
    SPACC_AES_GCM = 5
} eSPACC_AES_MODE;

typedef enum
{
    SPACC_OP_DECRYPT = 0x0,
    SPACC_OP_ENCRYPT = 0x1
} eSPACC_ENC_MODE;

typedef enum
{
    SPACC_KEY_NORMAL   = 0x0,
    SPACC_KEY_UKEY,
    SPACC_KEY_PKF,
    SPACC_KEY_DUK
} eSPACC_KEY_MODE;

typedef enum
{
    SPACC_HMODE_HMAC_SHA1     = 0,
    SPACC_HMODE_HMAC_SHA256,
    SPACC_HMODE_SSLMAC_SHA1,
    SPACC_HMODE_HASH_SHA1,
    SPACC_HMODE_HASH_SHA256
} eSPACC_HASH_MODE;

typedef union _tSPACC_CTRL_REG
{
   struct
   {
      uint32_t CIPH_ALG    : 3;
      uint32_t HASH_ALG    : 5;
      uint32_t CIPH_MODE   : 4;
      uint32_t HASH_MODE   : 2;
      uint32_t MSG_BEGIN   : 1;
      uint32_t MSG_END     : 1;
      uint32_t CTX_IDX     : 8;
      uint32_t ENCRYPT     : 1;
      uint32_t AAD_COPY    : 1;
      uint32_t ICV_PT      : 1;
      uint32_t ICV_ENC     : 1;
      uint32_t ICV_APPEND  : 1;
      uint32_t KEY_EXP     : 1;
      uint32_t reserved1   : 1;
      uint32_t SEC_KEY     : 1;
   } asFields;
   uint32_t as32Bits;
} tSPACC_CTRL_REG;

typedef union _tSPACC_KEY_SZ_REG
{
   struct {
      uint32_t SIZE       : 8;
      uint32_t CTX_TDX    : 8;
      uint32_t reserved1  : 15;
      uint32_t CIPHER     : 1;
   } asFields;
   uint32_t as32Bits;
} tSPACC_KEY_SZ_REG;

typedef struct _tSPACC_CONFIG
{
    eSPACC_KEY_MODE keyMode;
    uint8_t*        key;
    uint8_t*        iv;
    uint32_t        keySz;
    uint32_t        ivSz;
    const uint8_t*  in;
    uint8_t*        out;
    uint32_t        inSz;
    uint32_t        operation;
    uint32_t        aesMode;
    uint32_t        prot;
    uint32_t        icvCmd;
    uint32_t        encMode;
    uint32_t        aadSz;
    uint32_t        procSz;
    uint32_t        preAddSz;
    uint32_t        postAddSz;
    uint32_t        icvSz;
    uint32_t        icvOffset;
    uint32_t        ivOffset;
    uint32_t        aadOffset;
    tSPACC_CTRL_REG ctrl;
} tSPACC_CONFIG;


void ESAL_SPACC_SetContext(tSPACC_CONFIG* config, eSPACC_KEY_MODE keyMode, eSPACC_OPERATION op, eSPACC_AES_MODE aesMode);
int  ESAL_SPACC_SetCryptoAlgoContext(tSPACC_CONFIG* config, uint8_t* key, uint32_t keySz, uint8_t* iv, uint32_t ivSz, uint32_t aadSz);
int  ESAL_SPACC_AESEnqueueEx(tSPACC_CONFIG* config, const uint8_t* in, uint8_t* out, uint32_t inSz, eSPACC_ENC_MODE encMode);
int  ESAL_SPACC_HashEx(void* in, void* out, uint32_t inSz, uint32_t outSz, int mode, const uint8_t* hmacKey, uint32_t hmacKeySz, uint32_t ctxId);
void ESAL_SPACC_SetTagLen(uint32_t tagSz);
void ESAL_SPACC_GetIV(uint8_t* iv, uint32_t ivSz);
#endif

