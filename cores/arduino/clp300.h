#ifndef __CLP300_H__
#define __CLP300_H__

#include "ewbm_device.h"
#include "clp300_operand.h"
#include "clp300_hw.h"
#include "clp300_def.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#define CURVE_MAX_DATA_SIZE 256
#define CURVE_MAX_CURVES    15

#define PKA_READ_REG        0x00
#define PKA_WRITE_REG       0x01

typedef struct _tPKA_CURVE_DATA
{
   uint32_t size;
   uint8_t  rr[CURVE_MAX_DATA_SIZE];
   uint8_t  m[CURVE_MAX_DATA_SIZE];    //modulus m                                                   -- p
   uint8_t  mp[CURVE_MAX_DATA_SIZE];   // modular inverse of m (mod R)
   uint8_t  x[CURVE_MAX_DATA_SIZE];    // Q_x  the x coordinate of the generator                     -- G base point (compressed)
   uint8_t  y[CURVE_MAX_DATA_SIZE];    // Q_y  the y coordinate of the generator
   uint8_t  z[CURVE_MAX_DATA_SIZE];    // Q_z   the z coordinate of the generator
   uint8_t  r[CURVE_MAX_DATA_SIZE];    // R^2 mod m - used to convert an integer to an M-residue
   uint8_t  a[CURVE_MAX_DATA_SIZE];    // domain parameter 'a'                                       -- a
   uint8_t  b[CURVE_MAX_DATA_SIZE];    // domain parameter 'b'                                       -- b
   uint8_t  order[CURVE_MAX_DATA_SIZE + 1];   // incase 160+1                                        -- n
   char*    comment;                           // a uint32_t description of the curve
   uint8_t  nr[CURVE_MAX_DATA_SIZE];   // R^2 mod n
   uint8_t  np[CURVE_MAX_DATA_SIZE];   // inverse of n mod R
   uint8_t  n[CURVE_MAX_DATA_SIZE];    // order of the curve                                         -- n
} tPKA_CURVE_DATA;

typedef enum
{
    SECP_192R1  = 0,
    SECP_224R1,
    SECP_256R1,
    SECP_384R1,
    SECP_MAXPARAM
} ePKA_CURVE_PARAM;

typedef enum
{
    PKA_BYTELANE_LITTLE = 0,
    PKA_BYTELANE_BIG,
    PKA_BYTGELANE_MAX
} ePKA_BYTELANE;

// CLP300 Device control API
int ESAL_PKA_ReadOperandRegister(uint8_t* opname, void* outstream, uint32_t size);
int ESAL_PKA_WriteOperandRegister(uint8_t* opname, void* instream, uint32_t size);
int ESAL_PKA_WriteOperandMultiple(tPKA_CMD_LIST* pcmd, uint32_t len, uint32_t size);
int ESAL_PKA_FirmwareStart(uint32_t entry, uint32_t flags, uint32_t size);
int ESAL_PKA_FirmwareInit(ePKA_BYTELANE endian);
int ESAL_PKA_RegisterIoctl(int reg, uint32_t* val, int mode);
int ESAL_PKA_Flags(uint8_t* name, ePKA_OP op);

// CLP300 Modular Arithmetic operation API
int ESAL_PKA_MOD_R_Inverse(uint8_t* m, uint8_t* r_inv, uint32_t size);
int ESAL_PKA_MOD_Subraction(uint8_t* a, uint8_t* b, uint8_t* m, uint8_t* c, uint32_t size);
int ESAL_PKA_MOD_Multiplication(uint8_t* x, uint8_t* y, uint8_t* m, uint8_t* c, uint32_t size);
int ESAL_PKA_MOD_Inversion(uint8_t* x, uint8_t* m, uint8_t* c, uint32_t size);
int ESAL_PKA_MOD_Reduction(uint8_t* x, uint8_t* m, uint8_t* c, uint32_t size);
int ESAL_PKA_MOD_Divisoin(uint8_t* y, uint8_t* x, uint8_t* m, uint8_t* c, uint32_t size);
int ESAL_PKA_MOD_Addition(uint8_t* a, uint8_t* b, uint8_t* m, uint8_t* c, uint32_t size);

// CLP300 PKA operation API
int ESAL_PKA_ECC_Init(void);
tPKA_CURVE_DATA* ESAL_PKA_ECC_GetSECPCurveParam(void);
void ESAL_PKA_ECC_SetSECPCurveParam(ePKA_CURVE_PARAM curv);
int ESAL_PKA_ECC_PointMultiflication(tPKA_CURVE_DATA* pc, int keysz, int loadbase, uint8_t* key, uint8_t* blind, uint8_t* px, uint8_t* py);
int ESAL_PKA_ECC_PointAddition(tPKA_CURVE_DATA* pc, uint8_t* px, uint8_t* py, uint8_t* qx, uint8_t* qy);
int ESAL_PKA_ECC_PointVerification(tPKA_CURVE_DATA* pc, uint8_t* rx, uint8_t* ry);
int ESAL_PKA_ZeroWriteOperandRegister(uint8_t* opname, int sz);
int ESAL_PKA_ECC_Shamir(tPKA_CURVE_DATA* pc, uint8_t* qx, uint8_t* qy, uint8_t* k1, uint8_t* k2);
int ESAL_PKA_ECC_PointDouble(tPKA_CURVE_DATA* pc, uint8_t* px, uint8_t* py);


#ifdef __cplusplus
} // extern "C"
#endif
#endif
