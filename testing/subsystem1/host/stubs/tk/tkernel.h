/* Host-test stub: just enough micro T-Kernel types for subsystem1.h. */
#ifndef STUB_TKERNEL_H
#define STUB_TKERNEL_H
#include <stdint.h>
typedef int32_t  W;
typedef int32_t  INT;
typedef uint32_t UW;
typedef uint8_t  UB;
typedef int32_t  ID;
typedef int32_t  ER;
typedef int32_t  PRI;
typedef int32_t  TMO;
typedef void   (*FP)(void);
#define IMPORT   extern
#define EXPORT
#define E_OK     0
#endif
