#include "sys.h"

/*--------------拷贝-----------------*/
void copybuf(u8 *dest,const u8 *str,u16 size)
{
	u16 i;
	for(i=0;i<size;i++)
	{
		*(dest+i) = *(str+i);
	}
}

/*--------------清空BUFF-----------------*/
void my_memset(u8 *dest,u16 size)
{
	u16 i;
	for(i=0;i<size;i++)
	{
		*(dest+i) = 0;
	}
}

/*---------------比较函数---------------------*/
/*
if Return value < 0 then it indicates str1 is less than str2.
if Return value > 0 then it indicates str2 is less than str1.
if Return value = 0 then it indicates str1 is equal to str2.
*/
//int memcmp(const void *str1, const void *str2, size_t n)
//{
//		const char* pSrc1 = (char*)src1;
//		const char* pSrc2 = (char*)src2;
//		while (len-- > 0)
//		{
//			if (*pSrc1++ != *pSrc2++) 
//			{
//				return *pSrc1 < *pSrc2 ? -1 : 1;
//			}
//		}
//		return 0; 
//}
//not equal return 1
uint32_t my_memcmp_const(int32 *src,int32 const_value,u32 size)
{
	u32 i;
	for(i=0;i<size;i++)
	{
		if(const_value != *(src++))
		{
			return 1;
		}
	}
	return 0;
}
