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
extern osMessageQId  MsgIns;
extern osMessageQId  MsgUwb;

/* Defined macro ------------------------------------------------------------*/
#define NUM_COORD (3) // define the number of coordinates system -> defines if the localization is 2D or 3D

#define PRINT_RANGE (0) // 1-Enable 0-Disable, Print the measurements from anchors
#define PRINT_INS (0) // 1-Enable 0-Disable, Print the Inertial measurements
#define PRINT_POS (0) // 1-Enable 0-Disable, to print the estimated position

#define COOPERATIVE (0) // Enable or disable the Cooperative approach
#define HYBRID (1) // Enable or disable the Hybrid Algorithm

#define PVA_MODEL (0)
#define PV_MODEL (0)
#define P_MODEL (0)

#define DEG2RAD (PI/180) // to convert from degrees to radians
#define RAD2DEG (180/PI) // to convert from radians to degrees

#if(HYBRID) // By default will use PVA model

#define ROTATION  (296*DEG2RAD) /*ISMB*/ // Rotational angle of the y axis (Local) with respect to the north

#else // Will use the regular EKF
#define P_MODEL (1) //Enable P model
#define PV_MODEL (0) //Enable PV Model

#if(P_MODEL)
#define QNUM 0.1
#endif
#if(PV_MODEL)
#define QNUM 0.05
#endif

#endif

#define NEW_DATA (1)
#define CLEAR_NEW_DATA (0)
#define PROD (0) //Product operation
#define SUM (1)  //Summation operation
#define SUB (2)  //Subtraction operation

// Constants
#define STD_DIST (0.2) // [m] - Standard deviation of the UWB ranging

#define TIME_INS (0.01) // time to take inertial measurements -> in seconds // Considering the magnetometer frequency
//#define TIME_UWB (1) // time to take UWB measurements -> in seconds
//#define TIME_LOC (TIME_INS + TIME_UWB) // Localization dt -> in seconds

#define EMD (1.85*DEG2RAD)      // Eath Magnetic declination
#define GRAVITY (9.80665)  // m/s^2
#define ERROR_ACC (0.008*GRAVITY)
//#define ERROR_GYRO (0.1*DEG2RAD)
#define LIMIT_INIT_TRIES (10)

//#define MEAN_NO_ACC (1.0094*GRAVITY) //m/s^2  -> This is the magnitud of the acceleration vector when acceleration is 0
//#define MEAN_NO_ACC (1.02*GRAVITY) //m/s^2  -> This is the magnitud of the acceleration vector when acceleration is 0
//#define MEAN_NO_GYRO (0.1076*DEG2RAD) // rad/s -> This is the magnitud of the angular speed vector when angular speed is 0
//#define PSI_ERROR (40*PI/180) // Yaw angle error estimation bymeasurements
//#define PSI_ERROR_YAW_PROCESS (0.01*PI/180) // Yaw angle error estimation by process
//#define PSI_ERROR_PROCESS (0.1*PI/180) // Yaw angle error estimation by process

#define UPDATE (1)
#define NOT_UPDATE (0)
/* Defined types ------------------------------------------------------------*/
typedef struct {                                 // Message object structure
	double *acceleration;
//	double *gyroscope;
	double *compass;
}Ins_data;

typedef struct
{
	double *Range;
	double *anch3_pos;
}Ranging_data; // Structure for location data

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
}PVA_EKF;

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
}vecPVA_EKF;

void initArray(Array *a, size_t initialSize);
void insertArray(Array *a, float32_t element);
void freeArray(Array *a);
void zeros(uint32_t n, uint32_t m,arm_matrix_instance_f32* zero, Array * vec);
void eye(uint32_t number, float32_t  param, arm_matrix_instance_f32 * identity, Array * vec);
void diagMatrix(arm_matrix_instance_f32* m,float32_t vec[], uint8 number, Array * arr);
void CopyBinA(uint8 i, uint8 j, arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);
void printMatrix(arm_matrix_instance_f32 * m);
void EKF_PVA(PVA_EKF *PVASys,LocData* Loc,arm_matrix_instance_f32 *ins_meas,arm_matrix_instance_f32 *DCMbn);
void CalculateY(LocData* Loc,arm_matrix_instance_f32* Mat,arm_matrix_instance_f32* result, Array * vec);
void EKF_PVA2(PVA_EKF *PVASys,LocData* Loc);

void GetDistance(LocData* Loc,arm_matrix_instance_f32* Xp, arm_matrix_instance_f32* result, Array * vec);
Coordinates LLS(LocData* Loc);
/* Defined functions ------------------------------------------------------- */
void convert_ins_data(long *input,double *output);
void Locthread (void const *argument);

#ifdef __cplusplus
}
#endif

#endif
