
#ifndef _CANCOM__H
#define _CANCOM__H


#include "main.h"



extern uint8_t motorChannel;




void CAN_Init();


uint8_t getChannel();

void sendToMain();



#endif /* _CANCOM__H */
