
#ifndef _DRV8323_H_
#define _DRV8323_H_

#include "spi.h"


typedef struct{

	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef* NSS_GPIOx;
	uint16_t NSS_GPIO_Pin;

	uint8_t txBuf[2];
	uint8_t rxBuf[2];

	uint8_t rxFlag;

}DRV_TypeDef;




extern DRV_TypeDef drv8323;



void DRV_Init();


void DRV_WriteData(DRV_TypeDef *hdrv, uint8_t addr, uint16_t data);



void DRV_ReadData(DRV_TypeDef *hdrv, uint8_t addr);

void DRV_ReadData_IT(DRV_TypeDef *hdrv, uint8_t addr);



#endif /* _DRV8323_H_ */


