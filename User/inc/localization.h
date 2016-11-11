// -------------------------------------------------------------------------------------------------------------------
//
//  File: localization.h - application localization function
//
//  Author: Orlando Tovar, May 2016
//	Company: ISMB (Istituto Superiore Mario Boella)
// -------------------------------------------------------------------------------------------------------------------

#ifndef __LOCALIZATION_H
#define __LOCALIZATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include "instance.h"
#include "arm_math.h"
#include "uart.h"


/* Defined constants --------------------------------------------------------*/

osThreadId ThreadLocid;	// ID for Localization thread
osThreadId localizationTaskHandle;
extern double inst_idist[MAX_ANCHOR_LIST_SIZE];

extern osMessageQId  MsgUwb;
extern osMessageQId  MsgLoc;

/* Defined macro ------------------------------------------------------------*/
#define COOPERATIVE (1) // Enable or disable the Cooperative approach

#define PV_MODEL (1) // Enable or disable the PV model. If 0 then P_MODEL

#if PV_MODEL
	#define STATEVEC	(2) // x, y, vx, vy
	#define QNUM		(0.1)
#else
	#define	STATEVEC	(1) // x, y
	#define QNUM 		(0.05)
#endif

// Constants
#define STD_DIST (0.2) // [m] - Standard deviation of the UWB ranging 
#define TIME_LOC (0.1) // time to take UWB measurements -> in seconds

#define GRAVITY (9.80665)  // m/s^2
#define ERROR_ACC (0.008*GRAVITY)
//#define ERROR_GYRO (0.1*DEG2RAD)
#define LIMIT_INIT_TRIES (10)
#define EKF_TRIES 		(1)

/* Defined types ------------------------------------------------------------*/
typedef struct {                                 // Message object structure
	double *acceleration;
//	double *gyroscope;
	double *compass;
}Ins_data;

typedef struct
{
	double *Range;
	float32_t *anch3_pos;
	uint8 anch3_posTrue;
}Ranging_data; // Structure for location data



typedef struct 
{
	float32_t *estPos;
	uint8 estPosTrue;	
}localization_data; // send the coordinates estimated to the uwb thread


typedef struct
{
    double x;
    double y;
    double z;
}Coordinates;  // Structure for coordinates

typedef struct
{
	Coordinates* Coordinates;
	uint8 *AnchorPos;
	uint8 Nummeasurements;
	double *Range;
}LocData; // Structure for location data

typedef struct
{
	double x; //Estimated X coordinate
	double y; //Estimated Y coordinate
	double z; //Estimated Z coordinate
	uint8 Connectivity;
}msg2gateway;   // In this structure are save the valures for the gateway msg


typedef struct
{
	arm_matrix_instance_f32 X;
	arm_matrix_instance_f32 P;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 I;
	arm_matrix_instance_f32 B;
	arm_matrix_instance_f32 F;
}PV_EKF;

typedef struct {
  float32_t *array;
  size_t used;
  size_t size;
} Array;

typedef struct{
	Array vecX;
	Array vecP;
	Array vecQ;
	Array vecI;
	Array vecB;
	Array vecF;
}vecPV_EKF;

void initArray(Array *a, size_t initialSize);
void insertArray(Array *a, float32_t element);
void freeArray(Array *a);
void zeros(uint32_t n, uint32_t m,arm_matrix_instance_f32* zero, Array * vec);
void eye(uint32_t number, float32_t  param, arm_matrix_instance_f32 * identity, Array * vec);
void diagMatrix(arm_matrix_instance_f32* m,float32_t vec[], uint8 number, Array * arr);
void CopyBinA(uint8 i, uint8 j, arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);
void printMatrix(arm_matrix_instance_f32 * m);
void EKF_PV(PV_EKF *PVSys,LocData* Loc);
void CalculateY(LocData* Loc,arm_matrix_instance_f32* Mat,arm_matrix_instance_f32* result, Array * vec);
//void EKF_P(PV_EKF *PVSys,LocData* Loc);
float32_t trace(arm_matrix_instance_f32 * P);
void GetDistance(LocData* Loc,arm_matrix_instance_f32* Xp, arm_matrix_instance_f32* result, Array * vec);
Coordinates LLS(LocData* Loc);
/* Defined functions ------------------------------------------------------- */
void Locthread (void const *argument);

#ifdef __cplusplus
}
#endif

#endif
