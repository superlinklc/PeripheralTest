#ifndef __TYPE_H
#define __TYPE_H
#include "stm32f7xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif 

#define usart4MaxLen 100

typedef struct _UsartReData
{
    uint8_t usart4Flag;
    uint8_t usart4Len;
    uint8_t usart4Data[usart4MaxLen];
      

}data;




#endif

