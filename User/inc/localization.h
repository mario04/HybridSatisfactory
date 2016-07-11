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

#define PI (3.14159265358979323846264338327)
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
}Ranging_data; // Structure for location data

typedef struct
{
    uint8 numColumns;
    uint8 numRows;
    double** matrix;
}Matrix; // Matrix structure used in EKF

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
	double roll;
	double pitch;
	double yaw;
}Euler;  // Structure for Euler angles in Radians

//typedef struct
//{
//	Matrix X;
//	Matrix P;
//	Matrix Q;
//	Matrix H;
//	Matrix DCMbn;
//	Matrix twoI; // This is a constant matrix 2*identity
//	Euler euler;
//}AHRS;  // Structure for AHRS

typedef struct
{
	Matrix X;
	Matrix P;
//	Matrix Xhat;
//	Matrix Phat;
	Matrix Q;
	Matrix I;
	Matrix B;
	Matrix F;
}PVA_EKF;  // Structure for AHRS


/* Defined functions ------------------------------------------------------- */
void convert_ins_data(long *input,double *output);
void send_ins_data(void);
void Locthread (void const *argument);
//---------------Matrix Functions -------------------------
void CreateNewMatrix(uint8 n,uint8 m, Matrix* matrix); // Create a matrix
void FreeMatrix(Matrix* m); // Free memory and matrix structure
double VecMagnitud(double *ins_meas, uint8 dimension);
void MathMatrix(Matrix* m1,Matrix* m2,Matrix* result,uint8 Operation); // Computes a matrix operation
void ConstantOp(Matrix* m1,double number,uint8 Operation);
void TransposeMatrix(Matrix* m, Matrix* transpose); // Transpose a matrix
void SkewMatrix (double *Vector , double param,Matrix* skewmatrix);
void zeros(uint8 n, uint8 m,Matrix* zero);
void eye(uint8 number, double param,Matrix* identity);
void CopyBinA(uint8 i, uint8 j,Matrix*A,Matrix*B);
void diagMatrix(Matrix* m,double vec[]);
void diagop(Matrix* m,double param,uint8 Operation);
void printMatrix(Matrix* m);
void InverseGauss(Matrix* m,Matrix* inverse);
Coordinates LLS(LocData* Loc);
void CreateR_EKF(Matrix* Rparam,uint8 NumMeas,double Std_dis);
void GetDistance(LocData* Loc,Matrix* Xp, Matrix* result);
void CalculateY(LocData* Loc,Matrix* Mat,Matrix* result);
#if(HYBRID==0)
void EKF(LocData* Loc,Matrix* Xp,Matrix* Pp,Matrix* Fp,Matrix* Qp,Matrix* Ip,Matrix* Rp);
#else // Functions for the hybrid algorithm
Euler Euler_Stim(Matrix *Iner_Meas);
void euler2dcm(Euler *angles,Matrix* dcm);
Euler dcm2euler(Matrix *dcm);
//Coordinates localxyz2enu(Coordinates* Coor,double Deg);
//void EKF_AHRS(AHRS *AttitudeSys,Matrix *ins_meas);
//void EKF_PVA(PVA_EKF *PVASys,AHRS *AttitudeSys,LocData* Loc,Matrix *ins_meas,uint8 update);
void EKF_PVA(PVA_EKF *PVASys,LocData* Loc,Matrix *ins_meas,Matrix *DCMbn);
//void EKF_PVA_PREDICT(PVA_EKF *PVASys,LocData* Loc,Matrix *ins_meas,Matrix *DCMbn);
//void EKF_PVA_UPDATE(PVA_EKF *PVASys,LocData* Loc);
//void CorrectAttitude(AHRS *AttitudeSys);
#endif

#ifdef __cplusplus
}
#endif

#endif
