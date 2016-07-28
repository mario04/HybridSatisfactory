// -------------------------------------------------------------------------------------------------------------------
//
//  File: localization.h - application localization function
//
//  Author: Orlando Tovar, Jun 2016
//	Company: ISMB (Istituto Superiore Mario Boella)
// -------------------------------------------------------------------------------------------------------------------

#include "compiler.h"
#include "cmsis_os.h"
#include "localization.h"
#include <math.h>
#include <float.h>
#include <uwbMain.h>
#include "../../User/inc/compiler.h"

double range_bias[MAX_ANCHOR_LIST_SIZE] = {0.2,0.27,0.4,0.6};

Coordinates coordinates[MAX_ANCHOR_LIST_SIZE]={

		// ISMB -> Local {X,Y,Z} coordinates
		{23.71,6.33,2.3},    //Anchor0
		{12.41,4.55,2.3},   //Anchor1
		{17.97,8.85,1.4},    //Anchor2
		{16.75,0,2.3}     //Anchor3

//		// ISMB -> NED coordinates
//		{-10.2168,22.3126,-2.3},    //Anchor0
//		{-4.3939,12.4661,-2.3},   //Anchor1
//		{-4.6187,19.4913,-1.4},    //Anchor2
//		{-10.6658,12.9153,-2.3}     //Anchor3
};

//void InitializeEKF(Matrix* Xparam,Matrix* Pparam,Matrix* Fparam,Matrix* Qparam,Matrix* Iparam)
//{
//		//Initialization and constant parameters for EKF
//		//This function provides Localization 2-D
//#if(PV_MODEL)
//		//-----EKF Parameters-----//
//		double deltaT=TIME_LOC;
//		double std_coord=2; //Standard Deviation Coordinate
//		double Qnum=QNUM; //If Qnum is small is better use in a smooth paths, if Qnum is Large is better use in irregular paths//variance
//		double std_vel=0.1;//Standard Deviation Velocity
//		double dt3=deltaT*deltaT*deltaT;
//		double dt4=deltaT*deltaT*deltaT*deltaT;
//		#if(NUM_COORD==2)
//			double I[NUM_COORD*2][NUM_COORD*2]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
//			double F[NUM_COORD*2][NUM_COORD*2]={{1,0,deltaT,0},{0,1,0,deltaT},{0,0,1,0},{0,0,0,1}}; //Const  matrix 4x4
//			double Q[NUM_COORD*2][NUM_COORD*2]={
//												{(dt4*Qnum*0.25),0,(dt3*Qnum*0.5),0},
//												{0,(dt4*Qnum*0.25),0,(dt3*Qnum*0.5)},
//												{(dt3*Qnum*0.5),0,(deltaT*deltaT*Qnum),0},
//												{0,(dt3*Qnum*0.5),0,(deltaT*deltaT*Qnum)}
//												};  //const matrix 4x4
//			double X[NUM_COORD*2][1]={ //Initial Position
//										{18}, // x Coord
//										{5}, // y Coord
//										{1}, // x Vel
//										{1} //  y vel
//									   }; //Matrix 4x1
//			double P[NUM_COORD*2][NUM_COORD*2]={//Initial Covariance Matrix
//												{std_coord*std_coord,0,0,0},{0,std_coord*std_coord,0,0},{0,0,std_vel*std_vel,0},{0,0,0,std_vel*std_vel}
//												}; // 4x4 matrix
//		#else
//			double I[NUM_COORD*2][NUM_COORD*2]={{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
//			double F[NUM_COORD*2][NUM_COORD*2]={{1,0,0,deltaT,0,0},{0,1,0,0,deltaT,0},{0,0,1,0,0,deltaT},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}}; //Const  matrix 4x4
//			double Q[NUM_COORD*2][NUM_COORD*2]={
//												{(dt4*Qnum*0.25),0,0,(dt3*Qnum*0.5),0,0},
//												{0,(dt4*Qnum*0.25),0,0,(dt3*Qnum*0.5),0},
//												{0,0,(dt4*Qnum*0.25),0,0,(dt3*Qnum*0.5)},
//												{(dt3*Qnum*0.5),0,0,(deltaT*deltaT*Qnum),0,0},
//												{0,(dt3*Qnum*0.5),0,0,(deltaT*deltaT*Qnum),0},
//												{0,0,(dt3*Qnum*0.5),0,0,(deltaT*deltaT*Qnum)}
//												};  //const matrix 4x4
//			double X[NUM_COORD*2][1]={ //Initial Position
//										{18}, // x Coord
//										{5}, // y Coord
//										{0.78}, // z Coord
//										{1}, // x Vel
//										{1}, //  y vel
//										{1}  // z vel
//									   }; //Matrix 4x1
//			double P[NUM_COORD*2][NUM_COORD*2]={//Initial Covariance Matrix
//												{std_coord*std_coord,0,0,0,0,0},
//												{0,std_coord*std_coord,0,0,0,0},
//												{0,0,std_vel*std_vel,0,0,0},
//												{0,0,0,std_vel*std_vel,0,0},
//												{0,0,0,0,std_vel*std_vel,0},
//												{0,0,0,0,0,std_vel*std_vel}
//												}; // 4x4 matrix
//		#endif
//
//	    //----------------------------------------------------------------------------------------------------------------------//
//		uint8 i,j;
//		for(i=0;i<NUM_COORD*2;i++)
//		{
//			for(j=0;j<NUM_COORD*2;j++)
//			{
//				Xparam->matrix[i][0]=X[i][0];
//				Pparam->matrix[i][j]=P[i][j];
//				Fparam->matrix[i][j]=F[i][j];
//				Qparam->matrix[i][j]=Q[i][j];
//				Iparam->matrix[i][j]=I[i][j];
//			}
//		}
//#endif
//
//#if(P_MODEL)
//		//-----EKF Parameters-----//
//		double deltaT=TIME_LOC;
//		double std_coord=2; //Standard Deviation Coordinate
//		double Qnum= 0.2;//QNUM; //If Qnum is small is better use in a smooth paths, if Qnum is Large is better use in irregular paths//standard Deviation
//		#if(NUM_COORD==2)
//			double I[NUM_COORD][NUM_COORD]={{1,0},{0,1}};
//			double Q[NUM_COORD][NUM_COORD]={
//											{Qnum*Qnum*deltaT*deltaT,0},{0,Qnum*Qnum*deltaT*deltaT}
//										   };  //const matrix 2x2
//			double X[NUM_COORD][1]={ //Initial Position
//									{18}, // x Coord
//									{5}, // y Coord
//									}; //Matrix 2x1
//			double P[NUM_COORD][NUM_COORD]={//Initial Covariance Matrix
//											{std_coord*std_coord,0},{0,std_coord*std_coord}
//											}; // 2x2 matrix
//		#else
//			double I[NUM_COORD][NUM_COORD]={{1,0,0},{0,1,0},{0,0,1}};
//						double Q[NUM_COORD][NUM_COORD]={
//														{Qnum*Qnum*deltaT*deltaT,0,0},{0,Qnum*Qnum*deltaT*deltaT,0},{0,0,Qnum*Qnum*deltaT*deltaT}
//													   };  //const matrix 3x3
//						double X[NUM_COORD][1]={ //Initial Position
//												{18}, 	// x Coord
//												{5}, 	// y Coord
//												{0.78}, // z Coord
//												}; //Matrix 3x1
//						double P[NUM_COORD][NUM_COORD]={//Initial Covariance Matrix
//														{std_coord*std_coord,0,0},{0,std_coord*std_coord,0},{0,0,std_coord*std_coord}
//														}; // 3x3 matrix
//		#endif
//
//		//----------------------------------------------------------------------------------------------------------------------//
//		uint8 i,j;
//		for(i=0;i<NUM_COORD;i++)
//		{
//			for(j=0;j<NUM_COORD;j++)
//			{
//				Xparam->matrix[i][0]=X[i][0];
//				Pparam->matrix[i][j]=P[i][j];
//				Fparam->matrix[i][j]=I[i][j];
//				Qparam->matrix[i][j]=Q[i][j];
//				Iparam->matrix[i][j]=I[i][j];
//			}
//		}
//#endif
//	}

double VecMagnitud(double *ins_meas, uint8 dimension)
{
	double magnitud;

	if(dimension==2)
	{
		magnitud = sqrt(pow(ins_meas[0],2) + pow(ins_meas[1],2));
	}
	else
	{
		magnitud = sqrt(pow(ins_meas[0],2) + pow(ins_meas[1],2) + pow(ins_meas[2],2));
	}

	return magnitud;
}

void convert_ins_data(long *input,double *output)
{
	output[0] = NEW_DATA; // to Indicate there is a new data
	output[1] = ((double) input[0]) / 65536;
	output[2] = ((double) input[1]) / 65536;
	output[3] = ((double) input[2]) / 65536;
}



//void initSystem(AHRS *AttitudeSys,PVA_EKF *PVASys,LocData *info)
//{
//	// TODO : Initialization of old EKF implementations
//
//	osEvent evt;
//
//	Ins_data *inertial;
//	Ranging_data *uwb;
//	Matrix ins_meas,auxB,auxF;
//
//	uint8 i, run=1,Ninit=1,Nacc=0,Ngyro=0,Ncmps=0,ins_data_ready = CLEAR_NEW_DATA, uwb_data_ready = CLEAR_NEW_DATA;
//
//	Euler EulerAngles;
//	Coordinates LLScoord; //For LLS output, Coordinates Estimation
//	double Qvector[2] = {PSI_ERROR_PROCESS*PSI_ERROR_PROCESS,PSI_ERROR_YAW_PROCESS*PSI_ERROR_YAW_PROCESS};
//	double PvectorPVA[3] = {0.3*0.3,0.1*0.1,ERROR_ACC*ERROR_ACC};
//	double QvectorPVA[3] = {0.25*pow(TIME_INS,4)*pow(ERROR_ACC,2),pow(TIME_INS,2)*pow(ERROR_ACC,2),pow(ERROR_ACC,2)};
//
//	// Initialization AHRS
//	zeros(NUM_COORD*2,1,&AttitudeSys->X);//Contains attitude error and bias gyro
//	eye(NUM_COORD*2, 0.001,&AttitudeSys->P);
//	CreateNewMatrix(NUM_COORD*2,NUM_COORD*2,&AttitudeSys->Q); //Const  Matrix 6x6
//	diagMatrix(&AttitudeSys->Q,Qvector);
//	CreateNewMatrix(NUM_COORD,NUM_COORD,&AttitudeSys->DCMbn); //Const  Matrix 3x3
//	zeros(3,3,&ins_meas);
//	eye(NUM_COORD, 2,&AttitudeSys->twoI);
//	zeros(NUM_COORD,NUM_COORD*2,&AttitudeSys->H);//Contains attitude error and bias gyro
//	AttitudeSys->H.matrix[0][1] = -GRAVITY;
//	AttitudeSys->H.matrix[1][0] = GRAVITY;
//	AttitudeSys->H.matrix[2][2] = -1;
//
//	//Initialization PVA
//	zeros(NUM_COORD*3,1,&PVASys->X);
//	eye(NUM_COORD*3,1,&PVASys->I);
//
//	CreateNewMatrix(NUM_COORD*3,NUM_COORD*3,&PVASys->P);
//	CreateNewMatrix(NUM_COORD*3,NUM_COORD*3,&PVASys->Q);
//	diagMatrix(&PVASys->P,PvectorPVA);
//	diagMatrix(&PVASys->Q,QvectorPVA);
//
//	zeros(NUM_COORD*3,NUM_COORD,&PVASys->B);
//	eye(NUM_COORD,0.5*TIME_INS*TIME_INS,&auxB);
//	CopyBinA(0,0,&PVASys->B,&auxB);
//	FreeMatrix(&auxB);
//	eye(NUM_COORD,TIME_INS,&auxB);
//	CopyBinA(NUM_COORD,0,&PVASys->B,&auxB);
//	FreeMatrix(&auxB);
//
//	eye(NUM_COORD*3,1,&PVASys->F);
//	eye(NUM_COORD,TIME_INS,&auxF);
//	CopyBinA(0,NUM_COORD,&PVASys->F,&auxF);
//	FreeMatrix(&auxF);
//
//	while(run)
//	{
//	  	osThreadYield();
//		// Receive Inertial Data
//		evt = osMessageGet(MsgIns, 1);
//		if(evt.status == osEventMessage)
//		{
//			inertial = evt.value.p;
//
//			if(inertial->acceleration[0]==NEW_DATA) // It is going to accumulate all data before all data (acc,gyro,cmpss) is ready
//			{
//				for(i=0;i<3;i++)
//					ins_meas.matrix[0][i] += inertial->acceleration[i+1] * GRAVITY;
//				Nacc++;
//			}
//			if(inertial->gyroscope[0]==NEW_DATA) // It is going to accumulate all data before all data (acc,gyro,cmpss) is ready
//			{
//				for(i=0;i<3;i++)
//					ins_meas.matrix[1][i] += inertial->gyroscope[i+1] * DEG2RAD;
//				Ngyro++;
//			}
//			if(inertial->compass[0]==NEW_DATA) // It is going to accumulate all data before all data (acc,gyro,cmpss) is ready
//			{
//				for(i=0;i<3;i++)
//					ins_meas.matrix[2][i] += inertial->compass[i+1];
//				Ncmps++;
//			}
//			if(inertial->acceleration[0]==NEW_DATA && inertial->gyroscope[0]==NEW_DATA && inertial->compass[0]==NEW_DATA) // When ready makes takes the mean, the mean value is also accumulated until there is ranging
//			{
//				inertial->acceleration[0]=CLEAR_NEW_DATA;
//				inertial->gyroscope[0]=CLEAR_NEW_DATA;
//				inertial->compass[0]=CLEAR_NEW_DATA;
//
//				ins_data_ready = NEW_DATA;
//			 }
//		 }
//		 // Receive UWB data
//		 evt = osMessageGet(MsgUwb,1);
//		 if(evt.status == osEventMessage)
//		 {
//			 uwb = evt.value.p;
//			 uwb_data_ready = NEW_DATA;
//		 }
//		 if(ins_data_ready==NEW_DATA && uwb_data_ready==NEW_DATA) // Take data and finish initialization
//		 {
//			 for(i=0;i<3;i++)
//			 {
//				 ins_meas.matrix[0][i] = ins_meas.matrix[0][i]/Nacc;
//				 ins_meas.matrix[1][i] = ins_meas.matrix[1][i]/Ngyro;
//				 ins_meas.matrix[2][i] = ins_meas.matrix[2][i]/Ncmps;
//			 }
//			 Nacc = 0;
//			 Ngyro = 0;
//			 Ncmps = 0;
//			 for(i=0;i<MAX_ANCHOR_LIST_SIZE;i++)
//			 {
//				 if (uwb->Range[i] != 0)
//				 {
//					 info->Range[info->Nummeasurements] = uwb->Range[i] - range_bias[info->Nummeasurements];
//					 info->AnchorPos[info->Nummeasurements] = i;
//					 info->Nummeasurements++; // Count the number of measurements
//				  }
//			  }
//			  if(info->Nummeasurements > NUM_COORD && info->Nummeasurements<=MAX_ANCHOR_LIST_SIZE) // Estimates Position
//			  {
//				  LLScoord= LLS(info); // Save the position estimated
//				  if(LLScoord.z > 0)
//					  LLScoord.z = -1.7; // Medium heigh of a person
//				  PVASys->X.matrix[0][0] = LLScoord.x;
//				  PVASys->X.matrix[1][0] = LLScoord.y;
//				  PVASys->X.matrix[2][0] = LLScoord.z;
//				  EulerAngles = Euler_Stim(&ins_meas); // Initial Euler angles Euler Angles
//				  euler2dcm(&EulerAngles,&AttitudeSys->DCMbn); // Initial DCMbn
//				  run = 0; // Initialization has been finished
//			   }
//			   else // Check Number of tries
//			   {
//				   Ninit++;
//				   if(Ninit==LIMIT_INIT_TRIES)
//				   {
//					   // Coordinates for center of the room
//					   PVASys->X.matrix[0][0] = -7.9545;
//					   PVASys->X.matrix[1][0] = 16.7495;
//					   PVASys->X.matrix[2][0] = -1.7;
//					   EulerAngles = Euler_Stim(&ins_meas); // Estimates Euler Angles
//					   euler2dcm(&EulerAngles,&AttitudeSys->DCMbn); // Initial DCMbn
//					   run = 0; // Initialization has been finished
//				   }
//			   }
//			   info->Nummeasurements=0;
//			   ins_data_ready = CLEAR_NEW_DATA;
//			   uwb_data_ready = CLEAR_NEW_DATA;
//			   for(i=0;i<3;i++) // Reset all values if have to make again the initialization
//			   {
//				   ins_meas.matrix[0][i] = 0;
//				   ins_meas.matrix[1][i] = 0;
//				   ins_meas.matrix[2][i] = 0;
//			   }
//		   }
//	  }
//	  FreeMatrix(&ins_meas);
//}

void initSystem(PVA_EKF *PVASys,LocData *info, vecPVA_EKF * vecPVASys)
{
	// TODO : Initialization of old EKF implementations
	osEvent evt;
	Ranging_data *uwb;
	arm_matrix_instance_f32 auxB,auxF;
	Array vecAuxB, vecAuxF;
	uint8 i, run=1,Ninit=1;
	Coordinates LLScoord; //For LLS output, Coordinates Estimation
	float32_t PvectorPVA[3] = {0.3*0.3,0.1*0.1,ERROR_ACC*ERROR_ACC};
	float32_t QvectorPVA[3] = {0.25*pow(TIME_INS,4)*pow(ERROR_ACC,2),pow(TIME_INS,2)*pow(ERROR_ACC,2),pow(ERROR_ACC,2)};


	//Initialization PVA
	zeros(NUM_COORD*3,1,&PVASys->X, &vecPVASys->vecX);
	eye(NUM_COORD*3,1,&PVASys->I, &vecPVASys->vecI);

	diagMatrix(&PVASys->P,PvectorPVA, NUM_COORD*3, &vecPVASys->vecP);
	diagMatrix(&PVASys->Q,QvectorPVA, NUM_COORD*3, &vecPVASys->vecQ);

	zeros(NUM_COORD*3,NUM_COORD,&PVASys->B, &vecPVASys->vecB);
	eye(NUM_COORD,0.5*TIME_INS*TIME_INS,&auxB, &vecAuxB);

	CopyBinA(0,0,&PVASys->B,&auxB);
	freeArray(&vecAuxB);
	eye(NUM_COORD,TIME_INS,&auxB, &vecAuxB);
	CopyBinA(NUM_COORD,0,&PVASys->B,&auxB);
	freeArray(&vecAuxB);
	eye(NUM_COORD*3,1,&PVASys->F, &vecPVASys->vecF);
	eye(NUM_COORD,TIME_INS,&auxF, &vecAuxF);
	CopyBinA(0,NUM_COORD,&PVASys->F,&auxF);
	freeArray(&vecAuxF);

	while(run)
	{
	  	osThreadYield();
	  	// Receive UWB data
	  	evt = osMessageGet(MsgUwb,1);
	  	if(evt.status == osEventMessage)
		{
			 uwb = evt.value.p;

			 for(i=0;i<MAX_ANCHOR_LIST_SIZE;i++)
			 {
				 if (uwb->Range[i] != 0)
				 {
					 info->Range[info->Nummeasurements] = uwb->Range[i] - range_bias[info->Nummeasurements];
					 info->AnchorPos[info->Nummeasurements] = i;
					 info->Nummeasurements++; // Count the number of measurements
				  }
			 }

			 if(info->Nummeasurements > NUM_COORD && info->Nummeasurements<=MAX_ANCHOR_LIST_SIZE) // Estimates Position
			 {
				  LLScoord= LLS(info); // Save the position estimated
				  if(LLScoord.z > 0)
					  LLScoord.z = -1.7; // Medium heigh of a person
				  PVASys->X.pData[0] = LLScoord.x;
				  PVASys->X.pData[1] = LLScoord.y;
				  PVASys->X.pData[2] = LLScoord.z;
				  run = 0; // Initialization has been finished
			 }
			 else // Check Number of tries
			 {
				 Ninit++;
				 if(Ninit==LIMIT_INIT_TRIES)
				 {
//					 // Coordinates for center of the room
					 PVASys->X.pData[0] = -7.9545;
					 PVASys->X.pData[1] = 16.7495;
					 PVASys->X.pData[2] = -1.7;
					 run = 0; // Initialization has been finished
				}
			}
			info->Nummeasurements=0;
		}
	}
}

void Locthread(void const *argument)
{
  osEvent  evt;
  Ins_data *inertial;
  Ranging_data *uwb;
  uint8 i, uwb_data_ready = CLEAR_NEW_DATA;
  Euler EulerAngles;
  LocData info;
//Coordinates Position;
  PVA_EKF PVASys;
  vecPVA_EKF vecPVASys;
  arm_matrix_instance_f32 ins_meas, DCMbn;
  Array vecIns_meas, vecDCMbn;

  info.Coordinates=coordinates;
  info.Range = (double*)pvPortMalloc(MAX_ANCHOR_LIST_SIZE*sizeof(double));
  info.AnchorPos = (uint8*)pvPortMalloc(MAX_ANCHOR_LIST_SIZE*sizeof(uint8));
  info.Nummeasurements = 0;

  zeros(2,3, &ins_meas, &vecIns_meas);
  initSystem(&PVASys,&info, &vecPVASys); // Initialize all Structures
  zeros(3,3,&DCMbn, &vecDCMbn);

  while(1)
  {
	  osThreadYield();
//	  // Receive Inertial Data
//	  evt = osMessageGet(MsgIns, 1);
//	  if(evt.status == osEventMessage)
//	  {
//		 inertial = evt.value.p;
//
//		 if(inertial->acceleration[0]==NEW_DATA) // It is going to accumulate all data before all data (acc,gyro,cmpss) is ready
//		 {
//			for(i=0;i<3;i++)
//				ins_meas.matrix[0][i] = inertial->acceleration[i+1] * GRAVITY - PVASys.X.matrix[i + NUM_COORD*2][0];
////				ins_meas.matrix[0][i] += inertial->acceleration[i+1] * GRAVITY;
////			Nacc++;
//		 }
//		 if(inertial->compass[0]==NEW_DATA) // It is going to accumulate all data before all data (acc,gyro,cmpss) is ready
//		 {
//			for(i=0;i<3;i++)
//				ins_meas.matrix[1][i] = inertial->compass[i+1];
////				ins_meas.matrix[1][i] += inertial->compass[i+1];
////			Ncmps++;
//		 }
//		 if(inertial->acceleration[0]==NEW_DATA && inertial->compass[0]==NEW_DATA) // When Magnetometer is ready takes the mean value of measurements
//		 {
//			inertial->acceleration[0]=CLEAR_NEW_DATA;
//			inertial->compass[0]=CLEAR_NEW_DATA;
////			for(i=0;i<3;i++) // Performs Mean values a correct bias
////			{
////				ins_meas.matrix[0][i] = ins_meas.matrix[0][i]/Nacc - PVASys.X.matrix[i + NUM_COORD*2][0]; // acc bias correction
////				ins_meas.matrix[1][i] = ins_meas.matrix[1][i]/Ncmps;
////			}
////			Nacc = 0;
////			Ncmps = 0;
////			ins_data_ready = NEW_DATA;
//		 }
//	  }
	  // Receive UWB data
	  evt = osMessageGet(MsgUwb,1);
	  if(evt.status == osEventMessage)
	  {
		  uwb = evt.value.p;

		  for(i=0;i<MAX_ANCHOR_LIST_SIZE;i++)
		  {
			  if (uwb->Range[i] != 0)
			  {
				  info.Range[info.Nummeasurements] = uwb->Range[i] - range_bias[info.Nummeasurements]; // Correct bias
				  info.AnchorPos[info.Nummeasurements] = i;
				  info.Nummeasurements++; // Count the number of measurements
			  }
		  }
//		  uwb_data_ready = NEW_DATA;

		  if(info.Nummeasurements > NUM_COORD && info.Nummeasurements<=MAX_ANCHOR_LIST_SIZE) // Estimates Position
		  {
//			  // TODO: Ergonomics application
//
//			  // Euler Angles Estimation
//			  EulerAngles = Euler_Stim(&ins_meas);
//			  // DCMbn Estimation
//			  euler2dcm(&EulerAngles,&DCMbn);
//			  //Position Estimation
			  //EKF_PVA(&PVASys,&info,&ins_meas,&DCMbn);
			  printMatrix(&PVASys.X);
		  }
		  else // Send the predicted data
		  {
			  // TODO: When it is not possible to Update
		  }
		  info.Nummeasurements=0;
		  uwb_data_ready = CLEAR_NEW_DATA;
	  }
////	  if(ins_data_ready) // Perform ergonomics application and Prediction of hybrid algorithm
////	  {
////		  // TODO: Ergonomics application
////		  // Makes update of the EKF PVA and generates a predicted position
////		  EulerAngles = Euler_Stim(&ins_meas);
////		  euler2dcm(&EulerAngles,&DCMbn);
////		  EKF_PVA_PREDICT(&PVASys,&info,&ins_meas,&DCMbn);
////		  ConstantOp(&ins_meas,0,PROD); // Reset ins_Meas
////		  ins_data_ready = CLEAR_NEW_DATA;
////	  }
////	  if(uwb_data_ready) // Perfom Localization
////	  {
////
////		  if(info.Nummeasurements > NUM_COORD && info.Nummeasurements<=MAX_ANCHOR_LIST_SIZE) // Estimates Position
////		  {
////			  // TODO: Ergonomics application
////			  // Makes update of the EKF PVA and generates a predicted position
////			  EKF_PVA_UPDATE(&PVASys,&info);
////			  ConstantOp(&PVASys.Xhat,0,PROD);
////			  ConstantOp(&PVASys.Phat,0,PROD);
////			  printMatrix(&PVASys.X);
////		  }
////		  else // Send the predicted data
////		  {
////			  // TODO: When it is not possible to Update
////		  }
////		  info.Nummeasurements=0;
////		  uwb_data_ready = CLEAR_NEW_DATA;
////	  }
////	  t =xPortGetFreeHeapSize();
////	  sprintf((char*)&d[0], "Available bytes: %d",(int)t);
////	  uartWriteLineNoOS((char *) d); //send some data
 }

  osThreadTerminate(NULL);
}


void CreateNewMatrix(uint8 n,uint8 m, Matrix* matrix)
{
    uint8 i;

    matrix->numRows = n;
    matrix->numColumns = m;

//    matrix->matrix= (double**)malloc(n*sizeof(double*));
    matrix->matrix = (double**)pvPortMalloc(n*sizeof(double*));

	for (i=0;i<n;i++)
	{
//		matrix->matrix[i] = (double*)malloc(m*sizeof(double));
		matrix->matrix[i] = (double*)pvPortMalloc(m*sizeof(double));
	}
}

void initArray(Array *a, size_t initialSize) {
  a->array = (float32_t *)pvPortMalloc(initialSize * sizeof(float32_t ));
  a->used = 0;
  a->size = initialSize;
}

void insertArray(Array *a, float32_t element) {
  if (a->used < a->size) {
	  a->array[a->used++] = element;
  }

}

void freeArray(Array *a) {
	vPortFree(a->array);
	a->array = NULL;
	a->used = a->size = 0;
}


void FreeMatrix(Matrix* m)
{
	uint8 i;
	uint8 f=m->numRows;

    for(i=0;i<f;i++)
    {
//    	free(m->matrix[i]);
    	vPortFree(m->matrix[i]);
    }
//    free(m->matrix);
    vPortFree(m->matrix);

    m->numColumns=0;
    m->numRows=0;
}

void TransposeMatrix(Matrix* m, Matrix* transpose)
{
	CreateNewMatrix(m->numColumns,m->numRows,transpose);

	uint8 i,j;

	for(i=0;i<m->numRows;i++)
	{
		for(j=0;j<m->numColumns;j++)
		{
			transpose->matrix[j][i]=m->matrix[i][j];
		}
	}
}

void MathMatrix(Matrix* m1,Matrix* m2,Matrix* result,uint8 Operation)
{
	CreateNewMatrix(m1->numRows,m2->numColumns,result);

	uint8 i,j,k;

	switch (Operation)
	{
		case PROD:
		{
			double accum;
			for(i=0;i<m1->numRows;i++)
			{
				for(j=0;j<m2->numColumns;j++)
				{
					accum=0;
					for(k=0;k<m1->numColumns;k++)
					{
						accum +=  (m1->matrix[i][k])*(m2->matrix[k][j]);
					}
					result->matrix[i][j] = accum;
				}
			}
			break;
		}
		case SUM:
		{
			for(i=0;i<m1->numRows;i++)
			{
				for(j=0;j<m2->numColumns;j++)
				{
					result->matrix[i][j] = m1->matrix[i][j] + m2->matrix[i][j];
				}
			}
			break;
		}
		case SUB:
		{
			for(i=0;i<m1->numRows;i++)
			{
				for(j=0;j<m2->numColumns;j++)
				{
					result->matrix[i][j] = m1->matrix[i][j] - m2->matrix[i][j];
				}
			}
			break;
		}
	}
}

void ConstantOp(Matrix* m1,double number,uint8 Operation)
{
	uint8 i,j;
	switch (Operation)
	{
		case PROD:
		{
			for(i=0;i<m1->numRows;i++)
			{
				for(j=0;j<m1->numColumns;j++)
				{
					m1->matrix[i][j] = m1->matrix[i][j] * number;
				}
			}
			break;
		}
		case SUM:
		{
			for(i=0;i<m1->numRows;i++)
			{
				for(j=0;j<m1->numColumns;j++)
				{
					m1->matrix[i][j] = m1->matrix[i][j] + number;
				}
			}
			break;
		}
		case SUB:
		{
			for(i=0;i<m1->numRows;i++)
			{
				for(j=0;j<m1->numColumns;j++)
				{
					m1->matrix[i][j] = m1->matrix[i][j] - number;
				}
			}
			break;
		}
	}
}
void diagMatrix(arm_matrix_instance_f32* m,float32_t vec[], uint8 number, Array *arr)
{
	uint8 i,row = 0, col, aux=0, aux2 = 0;

	initArray(arr, number*number);

	for(i=0;i<number*number;i++)
	{
		col = i%number;
		if(row == col){
			if(aux < NUM_COORD){
				aux++;
			}
			else{
				aux = 1;
				aux2++;
			}
			insertArray(arr,vec[aux2]);
		}
		else
			insertArray(arr,0);

		if((i+1)%number == 0)
			row++;
	}

	arm_mat_init_f32(m, number, number, arr->array);

}

void diagop(Matrix* m,double param,uint8 Operation)
{
	uint8 i,j;

	for(i=0;i<m->numRows;i++)
	{
		for(j=0;j<m->numColumns;j++)
		{
			if(i==j)
			{
				switch (Operation)
				{
					case PROD:
					{
						m->matrix[i][j]= m->matrix[i][j] * param;
						break;
					}
					case SUM:
					{
						m->matrix[i][j]= m->matrix[i][j] + param;
						break;
					}
					case SUB:
					{
						m->matrix[i][j]= m->matrix[i][j] - param;
						break;
					}
				}
			}
		}
	}
}

void printMatrix(arm_matrix_instance_f32 * m)
{
	uint8 i,j;
	char d[200];

	sprintf((char*)&d[0], "The matrix is the following\n");
	uartWriteNoOS(d, strlen(d));


		for(i=0;i<m->numCols*m->numRows;i++)
		{
			sprintf((char*)&d[0],"%f \n",m->pData[i]);
			uartWriteNoOS(d, strlen(d));
		}

}

void SkewMatrix (double *Vector , double param,Matrix* skewmatrix)
{
//    sigma_x = vect(1);
//    sigma_y = vect(2);
//    sigma_z = vect(3);
//
//    S = [  0      -sigma_z  sigma_y;
//    		sigma_z     0     -sigma_x;
//    	   -sigma_y  sigma_x    0     ];

	double S[3][3] = {{0,-Vector[2],Vector[1]},{Vector[2],0,-Vector[0]},{-Vector[1],Vector[0],0}};

	CreateNewMatrix(3,3,skewmatrix);

	uint8 i,j;

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			skewmatrix->matrix[i][j] = S[i][j] * param;
		}
	}
}

void eye(uint32_t number, float32_t  param, arm_matrix_instance_f32 * identity, Array * vec)
{

	uint8 i, row = 0, col;
	initArray(vec, number*number);
	for(i=0;i<number*number;i++)
	{
		col = i%number;
		if(row == col)
			insertArray(vec,param);
		else
			insertArray(vec,0);

		if((i+1)%number == 0)
			row++;
	}
	arm_mat_init_f32(identity, number, number, vec->array);

}

void InverseGauss(Matrix* m,Matrix* b) // Calculate invers matrix
{
	uint8 i=0,p=0,j=0,q=0;
	double tem=0,temp=0,temp1=0,temp2=0,temp4=0,temp5=0;

	CreateNewMatrix(m->numRows,m->numRows,b);

	for(i=0;i<m->numRows;i++)
	{
		for(j=0;j<m->numRows;j++)
		{
			if(i==j)
			{
				b->matrix[i][j]=1;
			}
			else
			{
				b->matrix[i][j]=0;
			}
		}
	}

	for(i=0;i<m->numRows;i++)
	{
			temp=m->matrix[i][i];
			if(temp<0)
			{
				temp=temp*(-1);
			}
			p=i;
			for(j=i+1;j<m->numRows;j++)
			{
				if(m->matrix[j][i]<0)
				{
					tem=m->matrix[j][i]*(-1);
				}
				else
				{
					tem=m->matrix[j][i];
				}
				if(temp<0)
				{
					temp=temp*(-1);
				}
				if(tem>temp)
				{
					p=j;
					temp=m->matrix[j][i];
				}
			}
			//row exchange in both the matrix
			for(j=0;j<m->numRows;j++)
			{
				temp1=m->matrix[i][j];
				m->matrix[i][j]=m->matrix[p][j];
				m->matrix[p][j]=temp1;
				temp2=b->matrix[i][j];
				b->matrix[i][j]=b->matrix[p][j];
				b->matrix[p][j]=temp2;
			}
			//dividing the row by a[i][i]
			temp4=m->matrix[i][i];
			for(j=0;j<m->numRows;j++)
			{
				m->matrix[i][j]=(double)m->matrix[i][j]/temp4;
				b->matrix[i][j]=(double)b->matrix[i][j]/temp4;
			}
			//making other elements 0 in order to make the matrix m[][] an indentity matrix and obtaining a inverse b[][] matrix
			for(q=0;q<m->numRows;q++)
			{
				if(q==i)
					continue;
				temp5=m->matrix[q][i];
				for(j=0;j<m->numRows;j++)
				{
					m->matrix[q][j]=m->matrix[q][j]-(temp5*m->matrix[i][j]);
					b->matrix[q][j]=b->matrix[q][j]-(temp5*b->matrix[i][j]);
				}
			}
		}
}

void CopyBinA(uint8 i, uint8 j, arm_matrix_instance_f32* A, arm_matrix_instance_f32* B)
{
	uint8 k,l = 0, aux = i, aux2 = 0;

	while(l < B->numCols*B->numRows){
		k = aux*A->numCols + j;
		while(aux2 < B->numCols){
			A->pData[k]= B->pData[l];
			l++;
			aux2++;
			k++;
		}
		aux++;
		aux2 = 0;
	}
}

void zeros(uint32_t n, uint32_t m, arm_matrix_instance_f32 * a, Array * vec)
{
	uint16_t i;

	initArray(vec, n*m);

	for(i=0;i<n*m;i++)
	{
		insertArray(vec,0);

	}
	  arm_mat_init_f32(a, n, m, vec->array);

}

void CreateR_EKF(Matrix* Rparam,uint8 NumMeas,double Std_dist)
{
	uint8 i,j;
	CreateNewMatrix(NumMeas,NumMeas,Rparam);
	for(i=0;i<NumMeas;i++)
	{
		for(j=0;j<NumMeas;j++)
		{
			if(i==j)
			{
				Rparam->matrix[i][j]=(Std_dist)*(Std_dist);
			}
			else
			{
				Rparam->matrix[i][j]=0;
			}
		}
	}
}


void GetDistance(LocData* Loc,arm_matrix_instance_f32* Xp, arm_matrix_instance_f32* result) // to get the Euclidean distances 2-D between two points
{
	//---------------Information about Loc---------------------------------------
   	// Loc.Cordinates Vector with whole Anchor Coordinates.
	// Loc.AnchorPos Vector with Position of Anchors that sent a reported distance.
	// Loc.Nummeasurements Number of total collected measurements.
	// Loc.Range Vector with collected measurements.
	//----------------------------------------------------------------------------
	//---------------Information about EKFData------------------------------------
	// EKFData->F Matrix 4x4 constant
	// EKFData->P Matrix 4x4 constant
	// EKFData->Q Matrix 4x4 constant
	// EKFData->R Matrix Loc.Nummeasurements x Loc.Nummeasurements change every time according to the connectivity
	// EKFData->X Matrix 4x1 Change with the EKF filter application
	// EKFData->Qnumber double Initial Parameter
	// EKFData->StdDis  double Initial Parameter
	// EKFData->StdVel  double Initial Parameter
	// EKFData->dt      double Initial Parameter
	//------------------------------------------------------------------------------

	double A,B,C;

	Array vec;
	initArray(&vec, Loc->Nummeasurements);
	arm_mat_init_f32(result, Loc->Nummeasurements, 1, vec.array);
	uint8 i;

	for(i=0;i<(Loc->Nummeasurements);i++)
	{
		A = (Xp->pData[0]) - (Loc->Coordinates[Loc->AnchorPos[i]].x); // X coordinate
		B = (Xp->pData[1]) - (Loc->Coordinates[Loc->AnchorPos[i]].y); // Y coordinate
		#if(NUM_COORD==2)
			//result->matrix[i][0] =raiz((A*A) + (B*B));
		#else
			C = (Xp->pData[3]) - (Loc->Coordinates[Loc->AnchorPos[i]].z); // z coordinate
//			result->matrix[i][0] =raiz((A*A) + (B*B) + (C*C));
			result->pData[i] =sqrt((A*A) + (B*B) + (C*C));
		#endif
	}


	freeArray(&vec);
}

void CalculateY(LocData* Loc,arm_matrix_instance_f32* Mat,arm_matrix_instance_f32* result) // Calculate Y matrix
{
	//---------------Information about Loc---------------------------------------
   	// Loc.Cordinates Vector with whole Anchor Coordinates.
	// Loc.AnchorPos Vector with Position of Anchors that sent a reported distance.
	// Loc.Nummeasurements Number of total collected measurements.
	// Loc.Range Vector with collected measurements.
	//----------------------------------------------------------------------------

	Array vec;
	initArray(&vec, Loc->Nummeasurements);
	arm_mat_init_f32(result, Loc->Nummeasurements, 1, vec.array);


	uint8 i;

	for(i=0;i<(Loc->Nummeasurements);i++)
	{
		result->pData[i] =(Loc->Range[i]) - Mat->pData[i];

	}
	freeArray(&vec);
}
Coordinates LLS(LocData* Loc)
{
   	// Log.Cordinates Vector with whole Anchor Coordinates.
	// Log.AnchorPos Vector with Position of Anchors that sent a reported distance.
	// Log.Nummeasurements Number of total collected measurements.
	// Log.Range Vector with collected measurements.


	//The reference Anchor Always will be the first one

	uint8 i=0;

	Coordinates Coord;

	arm_matrix_instance_f32 A, P, AT, product, inverse, result;
	Array vecA, vecP, vecAT, vecProduct, vecInverse, vecResult;

	initArray(&vecA, ((Loc->Nummeasurements)-1)*NUM_COORD);
	for(i = 0; i<(Loc->Nummeasurements-1); i++){
		insertArray(&vecA, (2*((Loc->Coordinates[Loc->AnchorPos[i+1]].x) - (Loc->Coordinates[Loc->AnchorPos[0]].x))));
		insertArray(&vecA, (2*((Loc->Coordinates[Loc->AnchorPos[i+1]].y) - (Loc->Coordinates[Loc->AnchorPos[0]].y))));
		insertArray(&vecA, (2*((Loc->Coordinates[Loc->AnchorPos[i+1]].z) - (Loc->Coordinates[Loc->AnchorPos[0]].z))));
	}
	arm_mat_init_f32(&A,((Loc->Nummeasurements)-1),NUM_COORD, vecA.array);

	initArray(&vecP, ((Loc->Nummeasurements)-1));
	for(i = 0; i<(Loc->Nummeasurements-1); i++){
			insertArray(&vecP, (
					(Loc->Range[0]*Loc->Range[0]) - (Loc->Range[i+1]*Loc->Range[i+1]) -
					(((Loc->Coordinates[Loc->AnchorPos[0]].x)*(Loc->Coordinates[Loc->AnchorPos[0]].x)) +
					((Loc->Coordinates[Loc->AnchorPos[0]].y)*(Loc->Coordinates[Loc->AnchorPos[0]].y)) +
					((Loc->Coordinates[Loc->AnchorPos[0]].z)*(Loc->Coordinates[Loc->AnchorPos[0]].z))) +
					(((Loc->Coordinates[Loc->AnchorPos[i+1]].x)*(Loc->Coordinates[Loc->AnchorPos[i+1]].x)) +
				    ((Loc->Coordinates[Loc->AnchorPos[i+1]].y)*(Loc->Coordinates[Loc->AnchorPos[i+1]].y)) +
				    ((Loc->Coordinates[Loc->AnchorPos[i+1]].z)*(Loc->Coordinates[Loc->AnchorPos[i+1]].z)))
				    ));
	}
	arm_mat_init_f32(&P,((Loc->Nummeasurements)-1),1, vecP.array);

	initArray(&vecAT, ((Loc->Nummeasurements)-1)*NUM_COORD);
	arm_mat_init_f32(&AT,((Loc->Nummeasurements)-1),NUM_COORD, vecAT.array);
	arm_mat_trans_f32(&A,&AT);

	initArray(&vecProduct, ((Loc->Nummeasurements)-1)*NUM_COORD);
	arm_mat_init_f32(&product,((Loc->Nummeasurements)-1),NUM_COORD, vecProduct.array);
	arm_mat_mult_f32(&AT, &A, &product);
	freeArray(&vecA);

	initArray(&vecInverse, ((Loc->Nummeasurements)-1)*NUM_COORD);
	arm_mat_init_f32(&inverse,((Loc->Nummeasurements)-1),NUM_COORD, vecInverse.array);
	arm_mat_inverse_f32(&product, &inverse);

	arm_mat_mult_f32(&AT, &inverse, &product); // free vector memory before?
	freeArray(&vecAT);
	freeArray(&vecInverse);

	initArray(&vecResult, ((Loc->Nummeasurements)-1)*NUM_COORD);
	arm_mat_init_f32(&result,((Loc->Nummeasurements)-1),1, vecResult.array);
	arm_mat_cmplx_mult_f32(&product, &P, &result);
	freeArray(&vecProduct);
	freeArray(&vecP);

	Coord.x = result.pData[0];
	Coord.y = result.pData[1];
	Coord.z = result.pData[2];

	freeArray(&vecResult);
	return Coord;
}
#if(HYBRID==0)
void EKF(LocData* Loc,Matrix* Xp,Matrix* Pp,Matrix* Fp,Matrix* Qp,Matrix* Ip,Matrix* Rp) // EKF algorithm
{

	//---------------Information about Loc---------------------------------------
   	// Loc.Cordinates Vector with whole Anchor Coordinates.
	// Loc.AnchorPos Vector with Position of Anchors that sent a reported distance.
	// Loc.Nummeasurements Number of total collected measurements.
	// Loc.Range Vector with collected measurements.
	//----------------------------------------------------------------------------
	Matrix Xhat,P,Result,Transpose,Product,S,Htrans,Sinv,K,Xnew,Pnew,h,Y,H;
	//--------------Predicted Phase-------------------------------------------------
	//---------Xhat Computation-------------

	MathMatrix(Fp,Xp,&Xhat,PROD); //Matrix 4x1
	//---------------------------------------
	//---------P Computation-------------
	TransposeMatrix(Fp,&Transpose);
	MathMatrix(Fp,Pp,&Product,PROD);
	MathMatrix(&Product,&Transpose,&Result,PROD);
	FreeMatrix(&Transpose);
	FreeMatrix(&Product);

	MathMatrix(&Result,Qp,&P,SUM); //Matrix 4x4

	FreeMatrix(&Result);

	//----------------------------------------------------------------------------------
	//-------------------------- Updated Phase ------------------------------------------
	// h=Distance(Xhatposition,AnchorCoord) //Matrix Loc.Nummeasurementsx1
	// Y=Loc.Range - h  //Matrix Loc.Nummeasurementsx1
	//---------h Computation-------------
	GetDistance(Loc,&Xhat,&h); //Matrix Loc.Nummeasurementsx1
	//---------Y Computation-------------
	CalculateY(Loc,&h,&Y);  //Matrix Loc.Nummeasurementsx1
	//---------H Computation------------------------------------
    #if(PV_MODEL==1)
	CreateNewMatrix(Loc->Nummeasurements,NUM_COORD*2,&H);
	uint8 i,j;
	// TODO: Enable PV model for 3D Localization
	for(i=0;i<(Loc->Nummeasurements);i++)
	{
		H.matrix[i][0]=((Xhat.matrix[0][0]) - (Loc->Coordinates[Loc->AnchorPos[i]].x)) / h.matrix[i][0]; //X Coordinates
		H.matrix[i][1]=((Xhat.matrix[1][0]) - (Loc->Coordinates[Loc->AnchorPos[i]].y)) / h.matrix[i][0]; //Y Coordinates
		H.matrix[i][2]=0;
		H.matrix[i][3]=0;
	}

   #else
	CreateNewMatrix(Loc->Nummeasurements,NUM_COORD,&H);
	uint8 i,j;
	for(i=0;i<(Loc->Nummeasurements);i++)
	{
		H.matrix[i][0]=((Xhat.matrix[0][0]) - (Loc->Coordinates[Loc->AnchorPos[i]].x)) / h.matrix[i][0]; //X Coordinates
		H.matrix[i][1]=((Xhat.matrix[1][0]) - (Loc->Coordinates[Loc->AnchorPos[i]].y)) / h.matrix[i][0]; //Y Coordinates
		#if(NUM_COORD==3)
		H.matrix[i][2]=((Xhat.matrix[2][0]) - (Loc->Coordinates[Loc->AnchorPos[i]].z)) / h.matrix[i][0]; //Z Coordinates
		#endif
	}
   #endif
	FreeMatrix(&h);
	//---------------------------------------------------------------------------------------------------------------
	//---------S Computation-------------
	TransposeMatrix(&H,&Htrans); //Matrix 4XLoc->Nummeasurements
	MathMatrix(&H,&P,&Product,PROD); //Matrix  Loc->NummeasurementsX4
	MathMatrix(&Product,&Htrans,&Result,PROD);  //Matrix Loc->Nummeasurements x Loc->Nummeasurements
	MathMatrix(&Result,Rp,&S,SUM);  //Matrix Loc->Nummeasurements x Loc->Nummeasurements
	FreeMatrix(&Product);
	FreeMatrix(&Result);
	//----------------------------------
	//---------K Computation-------------
	MathMatrix(&P,&Htrans,&Product,PROD); //Product P*H'
	FreeMatrix(&Htrans);
	InverseGauss(&S,&Sinv);
	FreeMatrix(&S);
	MathMatrix(&Product,&Sinv,&K,PROD);// K=P*H'*(S^-1);  //Matrix 4XLoc->Nummeasurements
	FreeMatrix(&Product);
	FreeMatrix(&Sinv);
	//-------------------------------------
	//---------X Update Computation-------------
	MathMatrix(&K,&Y,&Result,PROD);//(K*Y)
	FreeMatrix(&Y);
	MathMatrix(&Xhat,&Result,&Xnew,SUM); // Update State Vector
	FreeMatrix(&Result);
	FreeMatrix(&Xhat);
	//----------------------------------------
	//---------P Update Computation-------------
	MathMatrix(&K,&H,&Product,PROD);
	FreeMatrix(&H);
	FreeMatrix(&K);
	MathMatrix(Ip,&Product,&Result,SUB);
	FreeMatrix(&Product);
	MathMatrix(&Result,&P,&Pnew,PROD); // Update Coovariance Matrix
	FreeMatrix(&Result);
	FreeMatrix(&P);

	for(i=0;i<Pnew.numRows;i++) // Copy the new matrixes in the old ones
	{
		Xp->matrix[i][0] = Xnew.matrix[i][0];

		for(j=0;j<Pnew.numColumns;j++)
		{
			Pp->matrix[i][j] = Pnew.matrix[i][j];
		}
	}
	FreeMatrix(&Xnew);
	FreeMatrix(&Pnew);
}
#else
Euler Euler_Stim(Matrix *Iner_Meas)
{
	Euler Euler_angles; // {-pi,pi}
	Matrix ACC;
	double X,Y;

	CreateNewMatrix(1,3,&ACC);

	ACC.matrix[0][0] = -Iner_Meas->matrix[0][0];
	ACC.matrix[0][1] = -Iner_Meas->matrix[0][1];
	ACC.matrix[0][2] = -Iner_Meas->matrix[0][2];

	// Roll Estimation
	if(ACC.matrix[0][0]==0 && ACC.matrix[0][2]==0)
	{
		if(ACC.matrix[0][1] > 0)
		{
			Euler_angles.roll = PI/2;
		}
		else
		{
			Euler_angles.roll = -PI/2;
		}
	}
	else
	{

		Y =  ACC.matrix[0][1];
		X = (ACC.matrix[0][2]/abs(ACC.matrix[0][2])) * sqrt(pow(ACC.matrix[0][2],2) +  0.000001*pow(ACC.matrix[0][0],2));

		Euler_angles.roll =  atan2(Y,X);

		if(Euler_angles.roll == 0 && Y <0)
		{
			Euler_angles.roll = -PI/2;
		}
		else if (Euler_angles.roll == 0 && Y >0)
		{
			Euler_angles.roll = PI/2;
		}
		else if (X==-INFINITY)
		{
			if(Y > 0 )
			{
				Euler_angles.roll = PI/2;
			}
			else if (Y < 0 )
			{
				Euler_angles.roll = -PI/2;
			}
		}

	}
	// Pitch Estimation
	if(ACC.matrix[0][1]==0 && ACC.matrix[0][2]==0)
	{
		if(ACC.matrix[0][0] < 0)
		{
			Euler_angles.pitch = PI/2;
		}
		else
		{
			Euler_angles.pitch = PI/2;
		}
	}
	else
	{
		Y = -ACC.matrix[0][0];
		X = sqrt(pow(ACC.matrix[0][1],2) +  pow(ACC.matrix[0][2],2));

		Euler_angles.pitch = atan(Y/X);

		if(Euler_angles.pitch > 0)
		{
			if(ACC.matrix[0][2] < 0) // Segundo Cuadrante
			{
				Euler_angles.pitch = PI - Euler_angles.pitch;
			}
		}
		else if (Euler_angles.pitch < 0)
		{
			if(ACC.matrix[0][2] < 0) // Tercer cuadrante
			{
				Euler_angles.pitch = -PI - Euler_angles.pitch;
			}
		}
	}

	// Yaw Estimation

//	X = Iner_Meas->matrix[2][0]*cos(Euler_angles.pitch) +
//		 Iner_Meas->matrix[2][1]*sin(Euler_angles.roll)*sin(Euler_angles.pitch) -
//		 Iner_Meas->matrix[2][2]*cos(Euler_angles.roll)*sin(Euler_angles.pitch);
//
//	Y = Iner_Meas->matrix[2][1]*cos(Euler_angles.roll) +
//		 Iner_Meas->matrix[2][2]*sin(Euler_angles.roll);
//
//	Euler_angles.yaw = -atan2(Y,X);
//
//	if(Euler_angles.yaw > 0)
//	{
//		Euler_angles.yaw -= EMD;
//	}
//	else if(Euler_angles.yaw < 0)
//	{
//		Euler_angles.yaw += EMD;
//	}

	X = Iner_Meas->matrix[1][0]*cos(Euler_angles.pitch) +
			 Iner_Meas->matrix[1][1]*sin(Euler_angles.roll)*sin(Euler_angles.pitch) -
			 Iner_Meas->matrix[1][2]*cos(Euler_angles.roll)*sin(Euler_angles.pitch);

	Y = Iner_Meas->matrix[1][1]*cos(Euler_angles.roll) +
		 Iner_Meas->matrix[1][2]*sin(Euler_angles.roll);

	Euler_angles.yaw = -atan2(Y,X);

	if(Euler_angles.yaw > 0)
	{
		Euler_angles.yaw -= EMD;
	}
	else if(Euler_angles.yaw < 0)
	{
		Euler_angles.yaw += EMD;
	}

	// Old implementations angles suppoerted from 0 to 360
//	// Roll Estimation
//	if(ACC.matrix[0][0]==0 && ACC.matrix[0][2]==0)
//	{
//		if(ACC.matrix[0][1] > 0)
//		{
//			Euler_angles.roll = PI/2;
//		}
//		else
//		{
//			Euler_angles.roll = 3*PI/2;
//		}
//	}
//	else
//	{
//
//		Y =  ACC.matrix[0][1];
//		X = (ACC.matrix[0][2]/abs(ACC.matrix[0][2])) * sqrt(pow(ACC.matrix[0][2],2) +  0.000001*pow(ACC.matrix[0][0],2));
//
//		Euler_angles.roll =  atan(Y/X); // Primer cuadrante si no entra ninguna condicion
//
//		if (Euler_angles.roll > 0)
//		{
//			if(ACC.matrix[0][1] < 0 && ACC.matrix[0][2] < 0) // Tercer cuadrante
//			{
//				Euler_angles.roll = PI + Euler_angles.roll;
//			}
//		}
//		else if (Euler_angles.roll < 0) // cuadrantes 2 y 4
//		{
//			if(ACC.matrix[0][1] > 0 && ACC.matrix[0][2] < 0) // Segundo cuadrante
//			{
//				Euler_angles.roll = PI + Euler_angles.roll;
//			}
//			else if(ACC.matrix[0][1] < 0 && ACC.matrix[0][2] > 0) // Cuarto cuadrante
//			{
//				Euler_angles.roll = 2*PI + Euler_angles.roll;
//			}
//		}
//
//		if(Euler_angles.roll == 0 && Y <0)
//		{
//			Euler_angles.roll = 3*PI/2;
//		}
//		else if (Euler_angles.roll == 0 && Y >0)
//		{
//			Euler_angles.roll = PI/2;
//		}
//		else if (Euler_angles.roll == 0 && X!=INFINITY && X!=-INFINITY)
//		{
//			Euler_angles.roll = 0;
//		}
//
//	}
//	// Pitch Estimation
//	if(ACC.matrix[0][1]==0 && ACC.matrix[0][2]==0)
//	{
//		if(ACC.matrix[0][0] < 0)
//		{
//			Euler_angles.pitch = PI/2;
//		}
//		else
//		{
//			Euler_angles.pitch = 3*PI/2;
//		}
//	}
//	else
//	{
//		Y = -ACC.matrix[0][0];
//		X = sqrt(pow(ACC.matrix[0][1],2) +  pow(ACC.matrix[0][2],2));
//
//		Euler_angles.pitch = atan(Y/X); // Primer cuadrante, si no entra a ningun if
//
//		if(Euler_angles.pitch > 0)
//		{
//			if(ACC.matrix[0][2] < 0) // Segundo Cuadrante
//			{
//				Euler_angles.pitch = PI - Euler_angles.pitch;
//			}
//		}
//		else if (Euler_angles.pitch < 0)
//		{
//			if(ACC.matrix[0][2] > 0) // Cuarto Cuadrante
//			{
//				Euler_angles.pitch = 2*PI + Euler_angles.pitch;
//			}
//			else if(ACC.matrix[0][2] < 0) // Tercer cuadrante
//			{
//				Euler_angles.pitch = PI - Euler_angles.pitch;
//			}
//		}
//	}
//
//	// Yaw Estimation
//
//	X = Iner_Meas->matrix[2][0]*cos(Euler_angles.pitch) +
//		 Iner_Meas->matrix[2][1]*sin(Euler_angles.roll)*sin(Euler_angles.pitch) -
//		 Iner_Meas->matrix[2][2]*cos(Euler_angles.roll)*sin(Euler_angles.pitch);
//
//	Y = Iner_Meas->matrix[2][1]*cos(Euler_angles.roll) +
//		 Iner_Meas->matrix[2][2]*sin(Euler_angles.roll);
//
//	if(X==0 && Y>0)
//	{
//		Euler_angles.yaw = 3*PI/2 - EMD;
//	}
//	else if(X==0 && Y<0)
//	{
//		Euler_angles.yaw = PI/2 - EMD;
//	}
//	else if (Y==0 && X!=0)
//	{
//		Euler_angles.yaw = 2*PI - EMD;
//	}
//	else if(X !=0 && Y!=0)
//	{
//		if(X>0 && Y>0)
//		{
//			Euler_angles.yaw = 2*PI - atan(Y/X) - EMD;
//		}
//		else if (X>0 && Y<0) // Cuadrante 4
//		{
//			Euler_angles.yaw = -atan(Y/X) - EMD; // Yaw angle is negative
//		}
//		else if (X<0) // Cuadrante 2 y 3
//		{
//			Euler_angles.yaw = PI - atan(Y/X) - EMD; // Yaw angle is negative
//		}
//	}
	FreeMatrix(&ACC);

	return Euler_angles;
}

void euler2dcm(Euler *angles,Matrix* dcm)
{
//	EULR2DCM       Euler angle vector to direction cosine
//	               matrix conversion.
//
//		DCMbn = eulr2dcm(eul_vect)
//
//	   INPUTS
//	       angles(1) = roll angle in radians
//
//	       angles(2) = pitch angle in radians
//
//	       angles(3) = yaw angle in radians
//
//	   OUTPUTS
//	       DCMbn = 3x3 direction cosine matrix providing the
//	             transformation from the body frame
//	             to the nVIGATION frame

	  uint8 i,j;
	  double roll = angles->roll, pitch = angles->pitch, yaw = angles->yaw;
	  double croll,sroll,cpitch,spitch,cyaw,syaw;

	  croll = cos(roll);
	  sroll = sin(roll);
	  cpitch = cos(pitch);
	  spitch = sin(pitch);
	  cyaw = cos(yaw);
	  syaw = sin(yaw);

	  double R[3][3] = {{cpitch*cyaw,sroll*spitch*cyaw-croll*syaw,croll*spitch*cyaw+sroll*syaw},
					    {cpitch*syaw,sroll*spitch*syaw+croll*cyaw,croll*spitch*syaw-sroll*cyaw},
					    {-spitch,sroll*cpitch,croll*cpitch}};

	  for(i=0;i<3;i++)
	  {
		  for(j=0;j<3;j++)
		  {
			  dcm->matrix[i][j] = R[i][j];
		  }
	  }
}

Euler dcm2euler(Matrix *dcm)
{
//	DCM2EULR       Direction cosine matrix to Euler angle
//	               vector conversion.
//
//		eul_vect = dcm2eulr(DCMbn)
//
//	   INPUTS
//	       DCMbn = 3x3 direction cosine matrix providing the
//	             transformation from the body frame
//	             to the navigation frame
//
//	   OUTPUTS
//	       eul_vect(1) = roll angle in radians
//
//	       eul_vect(2) = pitch angle in radians
//
//	       eul_vect(3) = yaw angle in radians

	double phi,theta,psi;

	// Roll Estimation
	phi = atan2(dcm->matrix[2][1],dcm->matrix[2][2]);
	// Pitch Estimation
	theta = asin(-dcm->matrix[2][0]);
	// Yaw Estimation
	psi = atan2(dcm->matrix[1][0],dcm->matrix[0][0]);

	Euler eul_vect = {phi,theta,psi};

	return eul_vect;
}

void EKF_PVA(PVA_EKF *PVASys,LocData* Loc,arm_matrix_instance_f32 *ins_meas,arm_matrix_instance_f32 *DCMbn)
{

	Array vecAccb, vecAccn, vecProduct, vecResult, vecXhat, vecTranspose, vecPhat, vecS, vecInverse, vecK, vecXnew, vecPnew;
	arm_matrix_instance_f32 auxF, product, result, transpose, inverse;
	arm_matrix_instance_f32 Xhat,Phat,accb,accn,Xnew,Pnew,h,Y,H,S,R,K;
	uint8 i,j,k=0;

	// Predict phase
	//accn estimation

	initArray(&vecAccb, NUM_COORD);
	arm_mat_init_f32(&accb, NUM_COORD, 1, vecAccb.array);
	accb.pData[0] = ins_meas->pData[0];
	accb.pData[1] = ins_meas->pData[1];
	accb.pData[2] = ins_meas->pData[2];
	arm_mat_init_f32(&accn, NUM_COORD, 1, vecAccn.array);
	arm_mat_cmplx_mult_f32(DCMbn, &accb, &accn);
	freeArray(&vecAccb);

	// F estimation
//	eye(NUM_COORD,-0.5*TIME_INS*TIME_INS,&auxF);
	initArray(&vecProduct, NUM_COORD*NUM_COORD);
	arm_mat_init_f32(&product, NUM_COORD, NUM_COORD, vecProduct.array);
	arm_mat_mult_f32(&auxF, DCMbn, &product);
	CopyBinA(0,2*NUM_COORD,&PVASys->F,&product);

	//eye(NUM_COORD,-TIME_INS,&auxF);
	arm_mat_mult_f32(&auxF, DCMbn, &product);
	CopyBinA(NUM_COORD,2*NUM_COORD,&PVASys->F,&product);  // Updated F
	freeArray(&vecProduct);

	// Xhat estimation
	initArray(&vecProduct, NUM_COORD*3);
	arm_mat_init_f32(&product, NUM_COORD*3, 1, vecProduct.array);
	arm_mat_cmplx_mult_f32(&PVASys->F,&PVASys->X,&product);

	initArray(&vecResult, NUM_COORD*3);
	arm_mat_init_f32(&result, NUM_COORD*3, 1, vecResult.array);
	arm_mat_cmplx_mult_f32(&PVASys->B,&accn,&result);
	freeArray(&vecAccn);

	initArray(&vecXhat, NUM_COORD*3);
	arm_mat_init_f32(&Xhat, NUM_COORD*3, 1, vecXhat.array);
	arm_mat_add_f32(&product, &result, &Xhat);
	freeArray(&vecProduct);
	freeArray(&vecResult);

	// Phat estimation
	initArray(&vecTranspose, NUM_COORD*9*NUM_COORD);
	arm_mat_init_f32(&transpose, NUM_COORD*3, NUM_COORD*3, vecTranspose.array);
	arm_mat_trans_f32(&PVASys->F,&transpose);

	initArray(&vecProduct, NUM_COORD*9*NUM_COORD);
	arm_mat_init_f32(&product, NUM_COORD*3, NUM_COORD*3, vecProduct.array);
	arm_mat_mult_f32(&PVASys->F,&PVASys->P,&product);

	initArray(&vecResult, NUM_COORD*9*NUM_COORD);
	arm_mat_init_f32(&result, NUM_COORD*3, NUM_COORD*3, vecResult.array);
	arm_mat_mult_f32(&product,&transpose,&result);
	freeArray(&vecProduct);
	freeArray(&vecTranspose);
	initArray(&vecPhat, NUM_COORD*9*NUM_COORD);
	arm_mat_init_f32(&Phat, NUM_COORD*3, NUM_COORD*3, vecPhat.array);
	arm_mat_add_f32(&result,&PVASys->Q,&Phat);
	freeArray(&vecResult);

	// Update phase
	// h=Distance(Xhatposition,AnchorCoord) //Matrix Loc.Nummeasurementsx1
	// Y=Loc.Range - h  //Matrix Loc.Nummeasurementsx1

	// R computation
//	eye(Loc->Nummeasurements, STD_DIST*STD_DIST, &R);

	// h Computation
	GetDistance(Loc,&Xhat,&h); //Matrix Loc.Nummeasurementsx1
	// Y Computation
	CalculateY(Loc,&h,&Y);  //Matrix Loc.Nummeasurementsx1
	// H Computation
//	zeros(Loc->Nummeasurements,NUM_COORD*3,&H);
	for(i=0;i<(Loc->Nummeasurements);i++)
	{
		k = i*H.numCols;
		H.pData[k]=((Xhat.pData[0]) - (Loc->Coordinates[Loc->AnchorPos[i]].x)) / h.pData[i]; //X Coordinates
		H.pData[k+1]=((Xhat.pData[1]) - (Loc->Coordinates[Loc->AnchorPos[i]].y)) / h.pData[i]; //Y Coordinates
		#if(NUM_COORD==3)
		H.pData[k+2]=((Xhat.pData[2]) - (Loc->Coordinates[Loc->AnchorPos[i]].z)) / h.pData[i]; //Z coordinates
		#endif
	}
	// S Computation

	initArray(&vecTranspose, Loc->Nummeasurements*NUM_COORD*3);
	arm_mat_init_f32(&transpose, NUM_COORD*3, Loc->Nummeasurements, vecTranspose.array);
	arm_mat_trans_f32(&H,&transpose);

	initArray(&vecProduct, NUM_COORD*3*Loc->Nummeasurements);
	arm_mat_init_f32(&product, Loc->Nummeasurements, NUM_COORD*3, vecProduct.array);
	arm_mat_cmplx_mult_f32(&H,&Phat,&product);

	initArray(&vecResult, Loc->Nummeasurements*Loc->Nummeasurements);
	arm_mat_init_f32(&result, Loc->Nummeasurements, Loc->Nummeasurements, vecResult.array);
	arm_mat_cmplx_mult_f32(&product,&transpose,&result);
	freeArray(&vecProduct);

	initArray(&vecS, Loc->Nummeasurements*Loc->Nummeasurements);
	arm_mat_init_f32(&S, Loc->Nummeasurements, Loc->Nummeasurements, vecS.array);
	arm_mat_add_f32(&result,&R,&S);
	freeArray(&vecResult);

	// K Computation
	initArray(&vecProduct, NUM_COORD*NUM_COORD*3);
	arm_mat_init_f32(&product, NUM_COORD*3, NUM_COORD, vecProduct.array);
	arm_mat_cmplx_mult_f32(&Phat,&transpose,&product);
	freeArray(&vecTranspose);

	initArray(&vecInverse, Loc->Nummeasurements*Loc->Nummeasurements);
	arm_mat_init_f32(&inverse, Loc->Nummeasurements, Loc->Nummeasurements, vecInverse.array);
	arm_mat_inverse_f32(&S,&inverse);
	freeArray(&vecS);

	initArray(&vecK, NUM_COORD*NUM_COORD*3);
	arm_mat_init_f32(&K, NUM_COORD*3, NUM_COORD, vecK.array);
	arm_mat_cmplx_mult_f32(&product,&inverse,&K);
	freeArray(&vecProduct);
	freeArray(&vecInverse);

	// X Update Computation
	initArray(&vecResult, NUM_COORD*3);
	arm_mat_init_f32(&result, NUM_COORD*3, 1, vecResult.array);
	arm_mat_cmplx_mult_f32(&K,&Y,&result);

	initArray(&vecXnew, NUM_COORD*3);
	arm_mat_init_f32(&Xnew, NUM_COORD*3, 1, vecXnew.array);
	arm_mat_add_f32(&Xhat,&result,&Xnew);

	freeArray(&vecResult);

	// P Update Computation
	initArray(&vecProduct, NUM_COORD*3*NUM_COORD*3);
	arm_mat_init_f32(&product, NUM_COORD*3, NUM_COORD*3, vecProduct.array);
	arm_mat_cmplx_mult_f32(&K,&H,&product);
	freeArray(&vecK);

	initArray(&vecResult, NUM_COORD*NUM_COORD*9);
	arm_mat_init_f32(&result, NUM_COORD*3, NUM_COORD*3, vecResult.array);
	arm_mat_sub_f32(&PVASys->I,&product,&result);

	freeArray(&vecProduct);

	initArray(&vecPnew, NUM_COORD*NUM_COORD*9);
	arm_mat_init_f32(&Pnew, NUM_COORD*3, NUM_COORD*3, vecPnew.array);
	arm_mat_mult_f32(&result,&Phat,&Pnew);

	freeArray(&vecResult);
	freeArray(&vecPhat);

	k = 0;
	for(i=0;i<Pnew.numRows;i++) // Copy the new matrixes in the old ones
	{
		PVASys->X.pData[i] = Xnew.pData[i];
		for(j=0;j<Pnew.numCols;j++)
		{
			PVASys->P.pData[k] = Pnew.pData[k];
			k++;
		}
	}
	freeArray(&vecXnew);
	freeArray(&vecPnew);
}

//void EKF_PVA_PREDICT(PVA_EKF *PVASys,LocData* Loc,Matrix *ins_meas,Matrix *DCMbn)
//{
//	Matrix auxF,product,result,transpose;
//	Matrix Xhat,Phat,accb,accn;
//	uint8 i,j;
//
//	// Predict phase
//	//accn estimation
//	CreateNewMatrix(NUM_COORD,1,&accb);
//	accb.matrix[0][0] = ins_meas->matrix[0][0];
//	accb.matrix[1][0] = ins_meas->matrix[0][1];
//	accb.matrix[2][0] = ins_meas->matrix[0][2];
//	MathMatrix(DCMbn,&accb,&accn,PROD);
//	FreeMatrix(&accb);
//
//	// F estimation
//	eye(NUM_COORD,-0.5*TIME_INS*TIME_INS,&auxF);
//	MathMatrix(&auxF,DCMbn,&product,PROD);
//	FreeMatrix(&auxF);
//	CopyBinA(0,2*NUM_COORD,&PVASys->F,&product);
//	FreeMatrix(&product);
//	eye(NUM_COORD,-TIME_INS,&auxF);
//	MathMatrix(&auxF,DCMbn,&product,PROD);
//	FreeMatrix(&auxF);
//	CopyBinA(NUM_COORD,2*NUM_COORD,&PVASys->F,&product);  // Updated F
//	FreeMatrix(&product);
//
//	// Xhat estimation
//	MathMatrix(&PVASys->F,&PVASys->X,&product,PROD);
//	MathMatrix(&PVASys->B,&accn,&result,PROD);
//	FreeMatrix(&accn);
//	MathMatrix(&product,&result,&Xhat,SUM); // Xhat estimation
//	FreeMatrix(&product);
//	FreeMatrix(&result);
//
//	// Phat estimation
//	TransposeMatrix(&PVASys->F,&transpose);
//	MathMatrix(&PVASys->F,&PVASys->P,&product,PROD);
//
//	MathMatrix(&product,&transpose,&result,PROD);
//	FreeMatrix(&transpose);
//	FreeMatrix(&product);
//
//	MathMatrix(&result,&PVASys->Q,&Phat,SUM); //Phat
//	FreeMatrix(&result);
//
//	for(i=0;i<Phat.numRows;i++)
//	{
//		PVASys->Xhat.matrix[i][0] = Xhat.matrix[i][0];
//		for(j=0;j<Phat.numColumns;j++)
//		{
//			PVASys->Phat.matrix[i][j]=Phat.matrix[i][j];
//		}
//	}
//
//	FreeMatrix(&Xhat);
//	FreeMatrix(&Phat);
//}
//void EKF_PVA_UPDATE(PVA_EKF *PVASys,LocData* Loc)
//{
//	Matrix product,result,transpose,inverse;
//	Matrix Xnew,Pnew,h,Y,H,S,R,K;
//	uint8 i,j;
//
//	// Update phase
//	// h=Distance(Xhatposition,AnchorCoord) //Matrix Loc.Nummeasurementsx1
//	// Y=Loc.Range - h  //Matrix Loc.Nummeasurementsx1
//
//	// R computation
//	CreateR_EKF(&R,Loc->Nummeasurements,STD_DIST);
//	// h Computation
//	GetDistance(Loc,&PVASys->Xhat,&h); //Matrix Loc.Nummeasurementsx1
//	// Y Computation
//	CalculateY(Loc,&h,&Y);  //Matrix Loc.Nummeasurementsx1
//	// H Computation
//	zeros(Loc->Nummeasurements,NUM_COORD*3,&H);
//	for(i=0;i<(Loc->Nummeasurements);i++)
//	{
//		H.matrix[i][0]=((PVASys->Xhat.matrix[0][0]) - (Loc->Coordinates[Loc->AnchorPos[i]].x)) / h.matrix[i][0]; //X Coordinates
//		H.matrix[i][1]=((PVASys->Xhat.matrix[1][0]) - (Loc->Coordinates[Loc->AnchorPos[i]].y)) / h.matrix[i][0]; //Y Coordinates
//		#if(NUM_COORD==3)
//		H.matrix[i][2]=((PVASys->Xhat.matrix[2][0]) - (Loc->Coordinates[Loc->AnchorPos[i]].z)) / h.matrix[i][0]; //Z coordinates
//		#endif
//	}
//	FreeMatrix(&h);
//
//	// S Computation
//	TransposeMatrix(&H,&transpose); //Matrix 4XLoc->Nummeasurements
//	MathMatrix(&H,&PVASys->P,&product,PROD); //Matrix  Loc->NummeasurementsX4
//	MathMatrix(&product,&transpose,&result,PROD);  //Matrix Loc->Nummeasurements x Loc->Nummeasurements
//	MathMatrix(&result,&R,&S,SUM);  //Matrix Loc->Nummeasurements x Loc->Nummeasurements
//	FreeMatrix(&product);
//	FreeMatrix(&result);
//	FreeMatrix(&R);
//
//	// K Computation
//	MathMatrix(&PVASys->P,&transpose,&product,PROD); //Product P*H'
//	FreeMatrix(&transpose);
//	InverseGauss(&S,&inverse);
//	FreeMatrix(&S);
//	MathMatrix(&product,&inverse,&K,PROD);// K=P*H'*(S^-1);  //Matrix 4XLoc->Nummeasurements
//	FreeMatrix(&product);
//	FreeMatrix(&inverse);
//
//	// X Update Computation
//	MathMatrix(&K,&Y,&result,PROD);//(K*Y)
//	FreeMatrix(&Y);
//	MathMatrix(&PVASys->Xhat,&result,&Xnew,SUM); // Update State Vector X
//	FreeMatrix(&result);
//
//	// P Update Computation
//	MathMatrix(&K,&H,&product,PROD);
//	FreeMatrix(&H);
//	FreeMatrix(&K);
//	MathMatrix(&PVASys->I,&product,&result,SUB);
//	FreeMatrix(&product);
//	MathMatrix(&result,&PVASys->Phat,&Pnew,PROD); // Update Coovariance Matrix
//	FreeMatrix(&result);
//
//	for(i=0;i<Pnew.numRows;i++) // Copy the new matrixes in the old ones
//	{
//		PVASys->X.matrix[i][0] = Xnew.matrix[i][0];
//
//		for(j=0;j<Pnew.numColumns;j++)
//		{
//			PVASys->P.matrix[i][j] = Pnew.matrix[i][j];
//		}
//	}
//	FreeMatrix(&Xnew);
//	FreeMatrix(&Pnew);
//}

////----------Coordinates Conversion-----------------
//Coordinates localxyz2enu(Coordinates* Coor,double Deg)
//{
//	//Convert Easting-Northing-Up to local coordinates
//
//	//transform matrix coordinate rotation counterclockwise
//
//	//Deg angle in Degree
//
//	//Coord Matrix 2 x 1, x and y coordinates For The localization 2-D
//
//	Matrix MatrixTran,Coord,Result;
//
//	CreateNewMatrix(2,2,&MatrixTran);
//
//	MatrixTran.matrix[0][0]=cos(Deg);
//	MatrixTran.matrix[0][1]=sin(Deg);
//	MatrixTran.matrix[1][0]=-sin(Deg);
//	MatrixTran.matrix[1][1]=cos(Deg);
//
//	CreateNewMatrix(2,1,&Coord);
//	Coord.matrix[0][0]=Coor->x;
//	Coord.matrix[1][0]=Coor->y;
//
//	MathMatrix(&MatrixTran,&Coord,&Result,PROD);
//	FreeMatrix(&MatrixTran);
//	FreeMatrix(&Coord);
//	Coordinates res={Result.matrix[0][0],Result.matrix[1][0],Coor->z};
//	FreeMatrix(&Result);
//	return res;
//}
#endif


