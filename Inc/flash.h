


#ifndef _FLASH_H_
#define _FLASH_H_


#include "main.h"


HAL_StatusTypeDef Flash_clear();

uint8_t* Flash_load();

HAL_StatusTypeDef Flash_store();


#endif



