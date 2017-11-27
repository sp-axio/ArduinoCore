#ifndef __PKA_OPERAND_REG_H__
#define __PKA_OPERAND_REG_H__

#define PKA_OP_256BIT_OFFSET        (5) // (1 << 5)  = 0x20
#define PKA_OP_512BIT_OFFSET        (6) // (1 << 6)  = 0x40
#define PKA_OP_1024BIT_OFFSET       (7) // (1 << 7)  = 0x80
#define PKA_OP_2048BIT_OFFSET       (8) // (1 << 8)  = 0x100
#define PKA_OP_4096BIT_OFFSET       (9) // (1 << 9)  = 0x200

#define PKA_ENTRY_PMULT             0x18
#define PKA_ENTRY_PDBL              0x19
#define PKA_ENTRY_PADD              0x1B
#define PKA_ENTRY_PVER              0x1D
#define PKA_ENTRY_MODEXP            0x13
#define PKA_ENTRY_MODMULT           0x0A
#define PKA_ENTRY_MODADD            0x0B
#define PKA_ENTRY_MODSUB            0x0C
#define PKA_ENTRY_MODDIV            0x0D
#define PKA_ENTRY_MODINV            0x0E
#define PKA_ENTRY_REDUCE            0x0F
#define PKA_ENTRY_CALC_MP           0x10
#define PKA_ENTRY_CALC_R_INV        0x11
#define PKA_ENTRY_CALC_R_SQR        0x12
#define PKA_ENTRY_SHAMIR            0x22

typedef enum
{
   PKA_OP_NOP,
   PKA_OP_CLR,
   PKA_OP_SET,
   PKA_OP_XOR,
   PKA_OP_MAX,
} ePKA_OP;

typedef struct _tPKA_CMD_LIST
{
    uint8_t* reg_name;
    uint8_t* reg_val;
} tPKA_CMD_LIST;

#endif

