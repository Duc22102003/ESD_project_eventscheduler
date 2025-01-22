/*
 * Task_ReadTemp.h
 *
 *  Created on: Jan 17, 2025
 *      Author: nguye
 */

#ifndef INC_TASK_READTEMP_H_
#define INC_TASK_READTEMP_H_

// *************** INTERFACE **************



void Task_ReadTemp (void);


// ************ INCLUDE ***********
#include <dht22.h>
#include <SON_CDC_Logger.h>
#include <math.h>
// ************ DEFINE *************



// *********** IMPLEMENTATION *************


inline void Task_ReadTemp (void) {



float temp = 0, humid = 0;


DHT22_GetTemp_Humidity(&temp, &humid);

int intPart = (int)floorf(temp);

printf("t=%d\n", intPart);

}

#endif /* INC_TASK_READTEMP_H_ */
