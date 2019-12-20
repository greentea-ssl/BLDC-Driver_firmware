
#ifndef _DRV8323_H_
#define _DRV8323_H_

#include "spi.h"



typedef union{
	struct{
		unsigned VDS_LC			:1;
		unsigned VDS_HC			:1;
		unsigned VDS_LB			:1;
		unsigned VDS_HB			:1;
		unsigned VDS_LA			:1;
		unsigned VDS_HA			:1;
		unsigned OTSD			:1;
		unsigned UVLO			:1;
		unsigned GDF			:1;
		unsigned VDS_OCP		:1;
		unsigned FAULT			:1;
	};
	uint16_t word;
}FaultStatus1_t;

typedef union{
	struct{
		unsigned VGS_LC			:1;
		unsigned VGS_HC			:1;
		unsigned VGS_LB			:1;
		unsigned VGS_HB			:1;
		unsigned VGS_LA			:1;
		unsigned VGS_HA			:1;
		unsigned CPUV			:1;
		unsigned OTW			:1;
		unsigned SC_OC			:1;
		unsigned SB_OC			:1;
		unsigned SA_OC			:1;
	};
	uint16_t word;
}FaultStatus2_t;

typedef union{
	struct{
		unsigned CLR_FLT		:1;
		unsigned BRAKE			:1;
		unsigned COAST			:1;
		unsigned sglPWM_DIR		:1;
		unsigned sglPWM_COM		:1;
		unsigned PWM_MODE		:2;
		unsigned OTW_REP		:1;
		unsigned DIS_GDF		:1;
		unsigned DIS_CPUV		:1;
		unsigned Reserved		:1;
	};
	uint16_t word;
}DriverControl_t;

typedef union{
	struct{
		unsigned IDRIVEN_HS		:4;
		unsigned IDRIVEP_HS		:4;
		unsigned LOCK			:3;
	};
	uint16_t word;
}GateDrive_HS_t;

typedef union{
	struct{
		unsigned IDRIVEN_LS		:4;
		unsigned IDRIVEP_LS		:4;
		unsigned TDRIVE			:2;
		unsigned CBC			:1;
	};
	uint16_t word;
}GateDrive_LS_t;

typedef union{
	struct{
		unsigned VDS_LVL		:4;
		unsigned OCP_DEG		:2;
		unsigned OCP_MODE		:2;
		unsigned OCP_TIME		:2;
		unsigned TRETRY			:1;
	};
	uint16_t word;
}OCP_Control_t;

typedef union{
	struct{
		unsigned SEN_LVL		:2;
		unsigned CSA_CAL_C		:1;
		unsigned CSA_CAL_B		:1;
		unsigned CSA_CAL_A		:1;
		unsigned DIS_SEN		:1;
		unsigned CSA_GAIN		:2;
		unsigned LS_REF			:1;
		unsigned VREF_DIV		:1;
		unsigned CSA_FET		:1;
	};
	uint16_t word;
}CSA_Control_t;


typedef union{

	struct{
		FaultStatus1_t		FaultStatus1;
		FaultStatus2_t		FaultStatus2;
		DriverControl_t 	DriverControl;
		GateDrive_HS_t		GateDrive_HS;
		GateDrive_LS_t		GateDrive_LS;
		OCP_Control_t		OCP_Control;
		CSA_Control_t		CSA_Control;
	};

	uint32_t words[7];

}Register_t;


typedef struct{

	Register_t Reg;

	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef* NSS_GPIOx;
	uint16_t NSS_GPIO_Pin;

	uint8_t txBuf[2];
	uint8_t rxBuf[2];

	uint8_t rxFlag;

}DRV_TypeDef;


typedef enum{
	ADDR_FaultStatus1 = 0x00,
	ADDR_FaultStatus2,
	ADDR_DriverControl,
	ADDR_GateDrive_HS,
	ADDR_GateDrive_LS,
	ADDR_OCP_Control,
	ADDR_CSA_Control,
}regAddr_t;



extern DRV_TypeDef drv8323;



void DRV_Init();


void DRV_WriteData(DRV_TypeDef *hdrv, regAddr_t addr);


void DRV_ReadData(DRV_TypeDef *hdrv, regAddr_t addr);

void DRV_ReadData_IT(DRV_TypeDef *hdrv, regAddr_t addr);



#endif /* _DRV8323_H_ */


