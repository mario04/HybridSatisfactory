/*
 * deca_main.h
 *
 *  Created on: 12 ott 2015
 *      Author: Emil Kallias
 */

#ifndef SRC_UWBCORE_APPLICATION_DECA_MAIN_H_
#define SRC_UWBCORE_APPLICATION_DECA_MAIN_H_


/*
 * @fn      deca_main()
 * @brief   main entry point
**/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdio.h>

#define TAG_DEVICE (0) // In order to programm the device.

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
osThreadId uwbInitTaskHandle;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void uwb_interrupt_cb(void);

void UwbInitTask(void const * argument);

#endif /* SRC_UWBCORE_APPLICATION_DECA_MAIN_H_ */
