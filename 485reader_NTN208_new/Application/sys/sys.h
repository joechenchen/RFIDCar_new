#ifndef __SYS_H
#define __SYS_H	
#include "nrf.h"

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef int8_t   int8;
typedef int16_t int16;
typedef int32_t int32; 
typedef float float32_t;

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

void copybuf(u8 *dest,const u8 *str,u16 size);
void my_memset(u8 *dest,u16 size);
uint32_t my_memcmp_const(int32 *src,int32 const_value,u32 size);

#endif
