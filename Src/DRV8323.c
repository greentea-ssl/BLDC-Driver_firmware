
#include "DRV8323.h"


DRV_TypeDef drv8323;



void DRV_Init()
{

	drv8323.NSS_GPIOx = SPI3_NSS_GPIO_Port;
	drv8323.NSS_GPIO_Pin = SPI3_NSS_Pin;
	drv8323.hspi = &hspi3;



}


void DRV_WriteData(DRV_TypeDef *hdrv, uint8_t addr, uint16_t data)
{

	// MSB
	hdrv->txBuf[0] = (addr << 3) | (data >> 8);
	// LSB
	hdrv->txBuf[1] = data & 0xff;

	HAL_GPIO_WritePin(hdrv->NSS_GPIOx, hdrv->NSS_GPIO_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hdrv->hspi, hdrv->txBuf, hdrv->rxBuf, 2, 1);

	HAL_GPIO_WritePin(hdrv->NSS_GPIOx, hdrv->NSS_GPIO_Pin, GPIO_PIN_SET);

}



void DRV_ReadData(DRV_TypeDef *hdrv, uint8_t addr)
{

	// MSB
	hdrv->txBuf[0] = 0x80 | (addr << 3);
	// LSB
	hdrv->txBuf[1] = 0x00;

	HAL_GPIO_WritePin(hdrv->NSS_GPIOx, hdrv->NSS_GPIO_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hdrv->hspi, hdrv->txBuf, hdrv->rxBuf, 2, 1);

	HAL_GPIO_WritePin(hdrv->NSS_GPIOx, hdrv->NSS_GPIO_Pin, GPIO_PIN_SET);

	HAL_Delay(1);

}

void DRV_ReadData_IT(DRV_TypeDef *hdrv, uint8_t addr)
{

	// MSB
	hdrv->txBuf[0] = 0x80 | (addr << 3);
	// LSB
	hdrv->txBuf[1] = 0x00;

	HAL_GPIO_WritePin(hdrv->NSS_GPIOx, hdrv->NSS_GPIO_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive_IT(hdrv->hspi, hdrv->txBuf, hdrv->rxBuf, 2);

	HAL_GPIO_WritePin(hdrv->NSS_GPIOx, hdrv->NSS_GPIO_Pin, GPIO_PIN_SET);

	hdrv->rxFlag = 0;

}



