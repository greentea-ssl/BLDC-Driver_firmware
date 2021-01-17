
#include "DRV8323.h"


extern SPI_HandleTypeDef hspi3;



DRV_TypeDef drv8323;



void DRV_Init()
{

	drv8323.NSS_GPIOx = SPI3_NSS_GPIO_Port;
	drv8323.NSS_GPIO_Pin = SPI3_NSS_Pin;
	drv8323.hspi = &hspi3;



}


void DRV_WriteData(DRV_TypeDef *hdrv, regAddr_t addr)
{

	// MSB
	hdrv->txBuf[0] = (addr << 3) | (hdrv->Reg.words[addr] >> 8);
	// LSB
	hdrv->txBuf[1] = hdrv->Reg.words[addr] & 0xff;

	HAL_GPIO_WritePin(hdrv->NSS_GPIOx, hdrv->NSS_GPIO_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hdrv->hspi, hdrv->txBuf, hdrv->rxBuf, 2, 1);

	HAL_GPIO_WritePin(hdrv->NSS_GPIOx, hdrv->NSS_GPIO_Pin, GPIO_PIN_SET);

}



void DRV_ReadData(DRV_TypeDef *hdrv, regAddr_t addr)
{

	// MSB
	hdrv->txBuf[0] = 0x80 | (addr << 3);
	// LSB
	hdrv->txBuf[1] = 0x00;

	HAL_GPIO_WritePin(hdrv->NSS_GPIOx, hdrv->NSS_GPIO_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hdrv->hspi, hdrv->txBuf, hdrv->rxBuf, 2, 1);

	HAL_GPIO_WritePin(hdrv->NSS_GPIOx, hdrv->NSS_GPIO_Pin, GPIO_PIN_SET);

	hdrv->Reg.words[addr] = (hdrv->rxBuf[0] << 8) | hdrv->rxBuf[1];

}

void DRV_ReadData_IT(DRV_TypeDef *hdrv, regAddr_t addr)
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



