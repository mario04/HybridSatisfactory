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
};



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





void initSystem(PVA_EKF *PVASys,LocData *info, vecPVA_EKF * vecPVASys)
{
	// TODO : Initialization of old EKF implementations
	osEvent evt;
	Ranging_data *uwb;
	localization_data dataqueue;
	arm_matrix_instance_f32 auxF;
	//arm_matrix_instance_f32 auxB;
	Array vecAuxF;
	//Array vecAuxB;
	uint8 i, run=1,Ninit=1;
	Coordinates LLScoord; //For LLS output, Coordinates Estimation
	float32_t PvectorPVA[3] = {0.3*0.3,0.1*0.1,ERROR_ACC*ERROR_ACC};
	float32_t QvectorPVA[3] = {0.25*pow(TIME_INS,4)*pow(ERROR_ACC,2),pow(TIME_INS,2)*pow(ERROR_ACC,2),pow(ERROR_ACC,2)};


	//Initialization PV
	zeros(NUM_COORD*3,1,&PVASys->X, &vecPVASys->vecX);
	eye(NUM_COORD*3,1,&PVASys->I, &vecPVASys->vecI);

	diagMatrix(&PVASys->P,PvectorPVA, NUM_COORD*3, &vecPVASys->vecP);
	diagMatrix(&PVASys->Q,QvectorPVA, NUM_COORD*3, &vecPVASys->vecQ);

	//zeros(NUM_COORD*3,NUM_COORD,&PVASys->B, &vecPVASys->vecB);
	//eye(NUM_COORD,0.5*TIME_INS*TIME_INS,&auxB, &vecAuxB);

//	CopyBinA(0,0,&PVASys->B,&auxB);
//	freeArray(&vecAuxB);
//	eye(NUM_COORD,TIME_INS,&auxB, &vecAuxB);
//	CopyBinA(NUM_COORD,0,&PVASys->B,&auxB);
//	freeArray(&vecAuxB);
	eye(NUM_COORD*3,1,&PVASys->F, &vecPVASys->vecF);
	eye(NUM_COORD,TIME_INS,&auxF, &vecAuxF);
	CopyBinA(0,NUM_COORD,&PVASys->F,&auxF);
	freeArray(&vecAuxF);

	eye(NUM_COORD,-0.5*TIME_INS*TIME_INS,&auxF, &vecAuxF);
	auxF.pData[0] = 0;
	auxF.pData[4] = 0;
	CopyBinA(0,2*NUM_COORD,&PVASys->F,&auxF);
	freeArray(&vecAuxF);

	eye(NUM_COORD,-TIME_INS,&auxF, &vecAuxF);
	auxF.pData[0] = 0;
	auxF.pData[4] = 0;
	CopyBinA(NUM_COORD,2*NUM_COORD,&PVASys->F,&auxF);
	freeArray(&vecAuxF);

	while(run)
	{
	  	osThreadYield();
	  	// Receive UWB data
	  	evt = osMessageGet(MsgUwb,1); // 1 ms?? 
	  	/*evt = osMessageGet(MsgUwb, osWaitForever);*/
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
					  LLScoord.z = 1.7; // Medium heigh of a person
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
					 PVASys->X.pData[0] = 7.9545;
					 PVASys->X.pData[1] = 16.7495;
					 PVASys->X.pData[2] = 1.7;
					 run = 0; // Initialization has been finished
				}
			}
			 dataqueue.estPos = &PVASys->X; // In order to send the estimation to the uwb thread.
			 osMessagePut(MsgLoc, &dataqueue, osWaitForever);
			info->Nummeasurements=0;
		}
	}
}

void Locthread(void const *argument)
{
  osEvent  evt;
  Ranging_data *uwb;
  localization_data dataqueue;
  uint8 i, uwb_data_ready = CLEAR_NEW_DATA;
  LocData info;
//Coordinates Position;
  PVA_EKF PVASys;
  vecPVA_EKF vecPVASys;
  arm_matrix_instance_f32 ins_meas, DCMbn;
  Array vecIns_meas, vecDCMbn;

  info.Coordinates=coordinates;
  info.Coordinates[MAX_ANCHOR_LIST_SIZE-1].x = uwb->anch3_pos[0];
  info.Coordinates[MAX_ANCHOR_LIST_SIZE-1].y = uwb->anch3_pos[1];
  info.Coordinates[MAX_ANCHOR_LIST_SIZE-1].z = uwb->anch3_pos[2];
  info.Range = (double*)pvPortMalloc(MAX_ANCHOR_LIST_SIZE*sizeof(double));
  info.AnchorPos = (uint8*)pvPortMalloc(MAX_ANCHOR_LIST_SIZE*sizeof(uint8));
  info.Nummeasurements = 0;

  //zeros(2,3, &ins_meas, &vecIns_meas);
  initSystem(&PVASys,&info, &vecPVASys); // Initialize all Structures
  //zeros(3,3,&DCMbn, &vecDCMbn);

  while(1)
  {
	  osThreadYield();
	  evt = osMessageGet(MsgUwb,1);
	  // evt = osMessageGet(MsgUwb, osWaitForever);
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
		 

		  if(info.Nummeasurements > NUM_COORD && info.Nummeasurements<=MAX_ANCHOR_LIST_SIZE) // Estimates Position
		  {
			  //EKF_PVA(&PVASys,&info,&ins_meas,&DCMbn);
			  EKF_PVA2(&PVASys,&info);
			  printMatrix(&PVASys.X);
			  // Send the data to the uwb thread and send a localization message.
		  }
		  else // Send the predicted data
		  {
			  // TODO: When it is not possible to Update
		  }
		  dataqueue.estPos = &PVASys.X;
		  osMessagePut(MsgLoc, &dataqueue, osWaitForever);
		  info.Nummeasurements=0;
		  uwb_data_ready = CLEAR_NEW_DATA;
	  }
 }
  freeArray(&vecIns_meas);
  freeArray(&vecDCMbn);
  freeArray(&vecPVASys.vecB);
  freeArray(&vecPVASys.vecF);
  freeArray(&vecPVASys.vecI);
  freeArray(&vecPVASys.vecP);
  freeArray(&vecPVASys.vecQ);
  freeArray(&vecPVASys.vecX);


  osThreadTerminate(NULL);
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



void printMatrix(arm_matrix_instance_f32 * m)
{
	uint8 i;
	char d[200];

	sprintf((char*)&d[0], "The matrix is the following\n");
	uartWriteNoOS(d, strlen(d));


		for(i=0;i<m->numCols*m->numRows;i++)
		{
			sprintf((char*)&d[0],"%f \n",m->pData[i]);
			uartWriteNoOS(d, strlen(d));
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

void GetDistance(LocData* Loc,arm_matrix_instance_f32* Xp, arm_matrix_instance_f32* result, Array * vec) // to get the Euclidean distances 2-D between two points
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


	initArray(vec, Loc->Nummeasurements);
	arm_mat_init_f32(result, Loc->Nummeasurements, 1, vec->array);
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

}

void CalculateY(LocData* Loc,arm_matrix_instance_f32* Mat,arm_matrix_instance_f32* result, Array * vec) // Calculate Y matrix
{
	//---------------Information about Loc---------------------------------------
   	// Loc.Cordinates Vector with whole Anchor Coordinates.
	// Loc.AnchorPos Vector with Position of Anchors that sent a reported distance.
	// Loc.Nummeasurements Number of total collected measurements.
	// Loc.Range Vector with collected measurements.
	//----------------------------------------------------------------------------


	initArray(vec, Loc->Nummeasurements);
	arm_mat_init_f32(result, Loc->Nummeasurements, 1, vec->array);


	uint8 i;

	for(i=0;i<(Loc->Nummeasurements);i++)
	{
		result->pData[i] =(Loc->Range[i]) - Mat->pData[i];

	}

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
	freeArray(&vecProduct);

	initArray(&vecProduct, ((Loc->Nummeasurements)-1)*NUM_COORD);
	arm_mat_init_f32(&product,((Loc->Nummeasurements)-1),NUM_COORD, vecProduct.array);
	arm_mat_mult_f32(&inverse, &AT, &product);
	freeArray(&vecAT);
	freeArray(&vecInverse);

	initArray(&vecResult, ((Loc->Nummeasurements)-1));
	arm_mat_init_f32(&result,((Loc->Nummeasurements)-1),1, vecResult.array);
	arm_mat_mult_f32(&product, &P, &result);

	freeArray(&vecProduct);
	freeArray(&vecP);

	Coord.x = result.pData[0];
	Coord.y = result.pData[1];
	Coord.z = result.pData[2];

	freeArray(&vecResult);
	return Coord;
}

void EKF_PVA(PVA_EKF *PVASys,LocData* Loc,arm_matrix_instance_f32 *ins_meas,arm_matrix_instance_f32 *DCMbn)
{

	Array vecAccb, vecAccn, vecProduct, vecResult, vecXhat, vecTranspose, vecPhat, vecS, vecInverse, vecK, vecXnew, vecPnew;
	Array vecR, vecH, vecY, vech, vecAuxF;
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

	initArray(&vecAccn, NUM_COORD);
	arm_mat_init_f32(&accn, NUM_COORD, 1, vecAccn.array);
	arm_mat_mult_f32(DCMbn, &accb, &accn);
	freeArray(&vecAccb);
	// F estimation
	eye(NUM_COORD,-0.5*TIME_INS*TIME_INS,&auxF, &vecAuxF);
	initArray(&vecProduct, NUM_COORD*NUM_COORD);
	arm_mat_init_f32(&product, NUM_COORD, NUM_COORD, vecProduct.array);
	arm_mat_mult_f32(&auxF, DCMbn, &product);
	freeArray(&vecAuxF);
	CopyBinA(0,2*NUM_COORD,&PVASys->F,&product);
	freeArray(&vecProduct);

	eye(NUM_COORD,-TIME_INS,&auxF, &vecAuxF);
	arm_mat_mult_f32(&auxF, DCMbn, &product);
	freeArray(&vecAuxF);
	CopyBinA(NUM_COORD,2*NUM_COORD,&PVASys->F,&product);  // Updated F
	freeArray(&vecProduct);

	// Xhat estimation
	initArray(&vecProduct, NUM_COORD*3);
	arm_mat_init_f32(&product, NUM_COORD*3, 1, vecProduct.array);
	arm_mat_mult_f32(&PVASys->F,&PVASys->X,&product);

	initArray(&vecResult, NUM_COORD*3);
	arm_mat_init_f32(&result, NUM_COORD*3, 1, vecResult.array);
	arm_mat_mult_f32(&PVASys->B,&accn,&result);
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
	eye(Loc->Nummeasurements, STD_DIST*STD_DIST, &R, &vecR);

	// h Computation
	GetDistance(Loc,&Xhat,&h, &vech); //Matrix Loc.Nummeasurementsx1
	// Y Computation
	CalculateY(Loc,&h,&Y, &vecY);  //Matrix Loc.Nummeasurementsx1
	// H Computation
	zeros(Loc->Nummeasurements,NUM_COORD*3,&H, &vecH);
	for(i=0;i<(Loc->Nummeasurements);i++)
	{
		k = i*H.numCols;
		H.pData[k]=((Xhat.pData[0]) - (Loc->Coordinates[Loc->AnchorPos[i]].x)) / h.pData[i]; //X Coordinates
		H.pData[k+1]=((Xhat.pData[1]) - (Loc->Coordinates[Loc->AnchorPos[i]].y)) / h.pData[i]; //Y Coordinates
		#if(NUM_COORD==3)
		H.pData[k+2]=((Xhat.pData[2]) - (Loc->Coordinates[Loc->AnchorPos[i]].z)) / h.pData[i]; //Z coordinates
		#endif
	}
	freeArray(&vech);
	// S Computation

	initArray(&vecTranspose, Loc->Nummeasurements*NUM_COORD*3);
	arm_mat_init_f32(&transpose, NUM_COORD*3, Loc->Nummeasurements, vecTranspose.array);
	arm_mat_trans_f32(&H,&transpose);

	initArray(&vecProduct, NUM_COORD*3*Loc->Nummeasurements);
	arm_mat_init_f32(&product, Loc->Nummeasurements, NUM_COORD*3, vecProduct.array);
	arm_mat_mult_f32(&H,&Phat,&product);

	initArray(&vecResult, Loc->Nummeasurements*Loc->Nummeasurements);
	arm_mat_init_f32(&result, Loc->Nummeasurements, Loc->Nummeasurements, vecResult.array);
	arm_mat_mult_f32(&product,&transpose,&result);
	freeArray(&vecProduct);

	initArray(&vecS, Loc->Nummeasurements*Loc->Nummeasurements);
	arm_mat_init_f32(&S, Loc->Nummeasurements, Loc->Nummeasurements, vecS.array);
	arm_mat_add_f32(&result,&R,&S);
	freeArray(&vecResult);
	freeArray(&vecR);

	// K Computation
	initArray(&vecProduct, Loc->Nummeasurements*NUM_COORD*3);
	arm_mat_init_f32(&product, NUM_COORD*3, Loc->Nummeasurements, vecProduct.array);
	arm_mat_mult_f32(&Phat,&transpose,&product);
	freeArray(&vecTranspose);

	initArray(&vecInverse, Loc->Nummeasurements*Loc->Nummeasurements);
	arm_mat_init_f32(&inverse, Loc->Nummeasurements, Loc->Nummeasurements, vecInverse.array);
	arm_mat_inverse_f32(&S,&inverse);
	freeArray(&vecS);

	initArray(&vecK, Loc->Nummeasurements*NUM_COORD*3);
	arm_mat_init_f32(&K, NUM_COORD*3, Loc->Nummeasurements, vecK.array);
	arm_mat_mult_f32(&product,&inverse,&K);
	freeArray(&vecProduct);
	freeArray(&vecInverse);

	// X Update Computation
	initArray(&vecResult, NUM_COORD*3);
	arm_mat_init_f32(&result, NUM_COORD*3, 1, vecResult.array);
	arm_mat_mult_f32(&K,&Y,&result);
	freeArray(&vecY);

	initArray(&vecXnew, NUM_COORD*3);
	arm_mat_init_f32(&Xnew, NUM_COORD*3, 1, vecXnew.array);
	arm_mat_add_f32(&Xhat,&result,&Xnew);
	freeArray(&vecXhat);
	freeArray(&vecResult);

	// P Update Computation
	initArray(&vecProduct, NUM_COORD*3*NUM_COORD*3);
	arm_mat_init_f32(&product, NUM_COORD*3, NUM_COORD*3, vecProduct.array);
	arm_mat_mult_f32(&K,&H,&product);
	freeArray(&vecK);
	freeArray(&vecH);

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

void EKF_PVA2(PVA_EKF *PVASys,LocData* Loc)
{

	//Array vecAccb,vecAccn, vecAuxF;
	Array vecProduct, vecResult, vecXhat, vecTranspose, vecPhat, vecS, vecInverse, vecK, vecXnew, vecPnew;
	Array vecR, vecH, vecY, vech;
	//arm_matrix_instance_f32 auxF,accb,accn;
	arm_matrix_instance_f32 Xhat,Phat, Xnew,Pnew,h,Y,H,S,R,K,product, result, transpose, inverse;
	uint8 i,j,k=0;


	// Predict phase
	//accn estimation

//	initArray(&vecAccb, NUM_COORD);
//	arm_mat_init_f32(&accb, NUM_COORD, 1, vecAccb.array);
//	accb.pData[0] = ins_meas->pData[0];
//	accb.pData[1] = ins_meas->pData[1];
//	accb.pData[2] = ins_meas->pData[2];
//
//	initArray(&vecAccn, NUM_COORD);
//	arm_mat_init_f32(&accn, NUM_COORD, 1, vecAccn.array);
//	arm_mat_mult_f32(DCMbn, &accb, &accn);
//	freeArray(&vecAccb);
	// F estimation
//	eye(NUM_COORD,-0.5*TIME_INS*TIME_INS,&auxF, &vecAuxF);
//	initArray(&vecProduct, NUM_COORD*NUM_COORD);
//	arm_mat_init_f32(&product, NUM_COORD, NUM_COORD, vecProduct.array);
//	arm_mat_mult_f32(&auxF, DCMbn, &product);
//	freeArray(&vecAuxF);
//	CopyBinA(0,2*NUM_COORD,&PVASys->F,&product);
//	freeArray(&vecProduct);

//	eye(NUM_COORD,-TIME_INS,&auxF, &vecAuxF);
//	arm_mat_mult_f32(&auxF, DCMbn, &product);
//	freeArray(&vecAuxF);
//	CopyBinA(NUM_COORD,2*NUM_COORD,&PVASys->F,&product);  // Updated F
//	freeArray(&vecProduct);



	// Xhat estimation
	initArray(&vecXhat, NUM_COORD*3);
	arm_mat_init_f32(&Xhat, NUM_COORD*3, 1, vecXhat.array);
	arm_mat_mult_f32(&PVASys->F,&PVASys->X,&Xhat);
//
//	initArray(&vecResult, NUM_COORD*3);
//	arm_mat_init_f32(&result, NUM_COORD*3, 1, vecResult.array);
//	arm_mat_mult_f32(&PVASys->B,&accn,&result);
//	freeArray(&vecAccn);
//
//	initArray(&vecXhat, NUM_COORD*3);
//	arm_mat_init_f32(&Xhat, NUM_COORD*3, 1, vecXhat.array);
//	arm_mat_add_f32(&product, &result, &Xhat);
//	freeArray(&vecProduct);
//	freeArray(&vecResult);
//
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
	eye(Loc->Nummeasurements, STD_DIST*STD_DIST, &R, &vecR);

	// h Computation
	GetDistance(Loc,&Xhat,&h, &vech); //Matrix Loc.Nummeasurementsx1
	// Y Computation
	CalculateY(Loc,&h,&Y, &vecY);  //Matrix Loc.Nummeasurementsx1
	// H Computation
	zeros(Loc->Nummeasurements,NUM_COORD*3,&H, &vecH);
	for(i=0;i<(Loc->Nummeasurements);i++)
	{
		k = i*H.numCols;
		H.pData[k]=((Xhat.pData[0]) - (Loc->Coordinates[Loc->AnchorPos[i]].x)) / h.pData[i]; //X Coordinates
		H.pData[k+1]=((Xhat.pData[1]) - (Loc->Coordinates[Loc->AnchorPos[i]].y)) / h.pData[i]; //Y Coordinates
		#if(NUM_COORD==3)
		H.pData[k+2]=((Xhat.pData[2]) - (Loc->Coordinates[Loc->AnchorPos[i]].z)) / h.pData[i]; //Z coordinates
		#endif
	}
	freeArray(&vech);
	// S Computation

	initArray(&vecTranspose, Loc->Nummeasurements*NUM_COORD*3);
	arm_mat_init_f32(&transpose, NUM_COORD*3, Loc->Nummeasurements, vecTranspose.array);
	arm_mat_trans_f32(&H,&transpose);

	initArray(&vecProduct, NUM_COORD*3*Loc->Nummeasurements);
	arm_mat_init_f32(&product, Loc->Nummeasurements, NUM_COORD*3, vecProduct.array);
	arm_mat_mult_f32(&H,&Phat,&product);

	initArray(&vecResult, Loc->Nummeasurements*Loc->Nummeasurements);
	arm_mat_init_f32(&result, Loc->Nummeasurements, Loc->Nummeasurements, vecResult.array);
	arm_mat_mult_f32(&product,&transpose,&result);
	freeArray(&vecProduct);

	initArray(&vecS, Loc->Nummeasurements*Loc->Nummeasurements);
	arm_mat_init_f32(&S, Loc->Nummeasurements, Loc->Nummeasurements, vecS.array);
	arm_mat_add_f32(&result,&R,&S);
	freeArray(&vecResult);
	freeArray(&vecR);

	// K Computation
	initArray(&vecProduct, Loc->Nummeasurements*NUM_COORD*3);
	arm_mat_init_f32(&product, NUM_COORD*3, Loc->Nummeasurements, vecProduct.array);
	arm_mat_mult_f32(&Phat,&transpose,&product);
	freeArray(&vecTranspose);

	initArray(&vecInverse, Loc->Nummeasurements*Loc->Nummeasurements);
	arm_mat_init_f32(&inverse, Loc->Nummeasurements, Loc->Nummeasurements, vecInverse.array);
	arm_mat_inverse_f32(&S,&inverse);
	freeArray(&vecS);

	initArray(&vecK, Loc->Nummeasurements*NUM_COORD*3);
	arm_mat_init_f32(&K, NUM_COORD*3, Loc->Nummeasurements, vecK.array);
	arm_mat_mult_f32(&product,&inverse,&K);
	freeArray(&vecProduct);
	freeArray(&vecInverse);

	// X Update Computation
	initArray(&vecResult, NUM_COORD*3);
	arm_mat_init_f32(&result, NUM_COORD*3, 1, vecResult.array);
	arm_mat_mult_f32(&K,&Y,&result);
	freeArray(&vecY);

	initArray(&vecXnew, NUM_COORD*3);
	arm_mat_init_f32(&Xnew, NUM_COORD*3, 1, vecXnew.array);
	arm_mat_add_f32(&Xhat,&result,&Xnew);
	freeArray(&vecXhat);
	freeArray(&vecResult);

	// P Update Computation
	initArray(&vecProduct, NUM_COORD*3*NUM_COORD*3);
	arm_mat_init_f32(&product, NUM_COORD*3, NUM_COORD*3, vecProduct.array);
	arm_mat_mult_f32(&K,&H,&product);
	freeArray(&vecK);
	freeArray(&vecH);

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

