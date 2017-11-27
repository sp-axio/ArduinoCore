#ifndef __CRYPTO_DEF_H__
#define __CRYPTO_DEF_H__

enum _e_ErrorMode {
    CRYPTO_OK                           = 0,
    CRYPTO_FAIL                         = -1,
    CRYPTO_TRNG_FAIL                    = -2,   
    CRYTPO_AES_FAIL                     = -3,
    CRYPTO_HASH_FAIL                    = -4,
    CRYTPO_HMAC_FAIL                    = -5,
    CRYPTO_IV_FAIL                      = -6,
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
};

enum _e_CryptoMode {
    CRYPTO_MODE_NULL                    = 0,
    CRYPTO_MODE_AES_ECB,
    CRYPTO_MODE_AES_CBC,
    CRYPTO_MODE_AES_CTR,
    CRYPTO_MODE_AES_CCM,
    CRYPTO_MODE_AES_GCM,
    CRYPTO_MODE_HASH_SHA1,
    CRYPTO_MODE_HMAC_SHA1,
    CRYPTO_MODE_HASH_SHA256,
    CRYPTO_MODE_HMAC_SHA256,
    CRYPTO_MODE_LAST
};

#define CRYPTO_ASSERT(x) \
    ({  \
        if(x) { \
            return CRYPTO_FAIL; \
        }   \
    })
#endif
