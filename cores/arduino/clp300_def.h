#ifndef __CRYPTO_DEF_H__
#define __CRYPTO_DEF_H__

typedef enum _ePKA_ERROR_MODE
{
    CRYPTO_OK                           = 0,
    CRYPTO_FAIL                         = -1,
    CRYPTO_NO_MEM                       = -7,
    CRYPTO_INVALID_ARG                  = -8,
    CRYPTO_INVALID_SIZE                 = -9,
    CRYPTO_INVALID_MODE                 = -10,
    CRYPTO_INVALID_KEYSZ                = -11,
    CRYPTO_INVALID_KEY                  = -12,
    CRYPTO_AUTHENTICATION_FAILED        = -13,
    CRYPTO_TIMEOUT                      = -14,
    CRYPTO_MEMORY_ERROR                 = -15,
    CRYPTO_NOT_IMPLEMENT                = -16,
    CRYPTO_BUSY                         = -17
} ePKA_ERROR_MODE;

#define CRYPTO_ASSERT(x) \
    ({  \
        if(x) { \
            return CRYPTO_FAIL; \
        }   \
    })
#endif
