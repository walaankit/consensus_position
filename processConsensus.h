#ifndef _CONSENSUS_PROCESSOR_H
#define _CONSENSUS_PROCESSOR_H



double latData[TOTAL_NODES],longData[TOTAL_NODES];		// GPS Navigation data for each node
double pitchVelConsensus[TOTAL_NODES],rollVelConsensus[TOTAL_NODES];		//Pitch and roll velocities required in each node for Consensus
double previousLatValue[TOTAL_NODES],previousLatValue[TOTAL_NODES];  //used to calculate lat long velocities in calculateVelocityGPS
char AdjMatrix[TOTAL_NODES][TOTAL_NODES];		//User defined Adjacency Matrix
int nodeNo;
/*********************************************************************************************************************
Initialize Adjacency Matrix as per User definition of the Network
********************************************************************************************************************/
void initializeNetworkTopology()  
	{
		
		for (int i=0;i<TOTAL_NODES;i++)
		{
			for (int j=0;j<TOTAL_NODES;j++)
			{
				AdjMatrix[i][j]=0;	//Initialise all elements to 0
			}
		}
		//This will change according to the network defined by user
		AdjMatrix[0][1]=AdjMatrix[1][0]=1;
		AdjMatrix[0][2]=AdjMatrix[2][0]=1;
		AdjMatrix[1][2]=AdjMatrix[2][1]=1;
		//AdjMatrix[2][4]=AdjMatrix[4][2]=1;
	}



/*********************************************************************************************************************
Geting GPS data from Message of fix
********************************************************************************************************************/
void getGPSData(nodeNo)
{
	latData[nodeNo - 1]=
	longData[nodeNo - 1]=
}

/*********************************************************************************************************************
Calculation of velocity using GPS data
********************************************************************************************************************/
void calculateVelocityGPS(nodeNo)
{
	
	LatVelocityGPS[nodeNo - 1] = latData[nodeNo - 1] - previousLatValue[nodeNo-1];
	LonVelocityGPS[nodeNo - 1] = longData[nodeNo - 1] - previousLonValue[nodeNo-1];
	
	previousLatValue[nodeNo-1]=latData[nodeNo - 1];
	previousLonValue[nodeNo-1]=longData[nodeNo - 1];
}	
	
/*********************************************************************************************************************
Call this function to calculate the velocities required by each node for Consensus
********************************************************************************************************************/
void computeConsensusVel(nodeNo)
{
        //Computing angles from lat/long data
        latConsensusError = 0;
        lonConsensusError = 0;
        for (int j=0; j<TOTAL_NODES; j++)
	{
		latConsensusError += ALPHA_PITCH*AdjMatrix[nodeNo-1][j]*(latData[j] - latData[nodeNo - 1]) ;
		lonConsensusError += ALPHA_ROLL*AdjMatrix[nodeNo-1][j]*(longData[j] - longData[nodeNo - 1]) ;
	}
	pitchVelConsensus[nodeNo-1] = latConsensusError - BETA_PITCH*LatVelocityGPS[nodeNo - 1];
	rollVelConsensus[nodeNo-1] = lonConsensusError - BETA_ROLL*LonVelocityGPS[nodeNo - 1];

	//Keeping the roll/pitch angles between +-20 deg
	if(pitchVelConsensus[nodeNo-1]>MAX_PITCH_VEL) pitchVelConsensus[nodeNo-1]=MAX_PITCH_VEL;
	
	else if(pitchVelConsensus[nodeNo-1]< -MAX_PITCH_VEL) pitchVelConsensus[nodeNo-1]=-MAX_PITCH_VEL;
	
	if(rollVelConsensus[nodeNo-1]>MAX_ROLL_VEL) rollVelConsensus[nodeNo-1]=MAX_ROLL_VEL;
	
	else if(rollVelConsensus[nodeNo-1]< -MAX_ROLL_VEL) rollVelConsensus[nodeNo-1]=-MAX_ROLL_VEL;
	
	}
	
	
/*********************************************************************************************************************
Drive quadrotors according to consensus velocities computed
********************************************************************************************************************/
void driveAllQuad()
{
	
}

#endif

