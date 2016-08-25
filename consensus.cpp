


#include <ros/ros.h>
//#include <processConsensus.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
//#include <sensor_msgs/NavSatFix.h>
/* run this program using the console pauser or add your own getch, system("pause") or input loop */
using namespace std;



#define TOTAL_NODES 5  //Total number of Nodes in the system

#define ALPHA_PITCH 0.2//0.1//0.5// 50000   //Scaling factor for positional average for pitch direction
#define ALPHA_ROLL 0.2////0.1//0.5 //50000    //Scaling factor for roll direction

#define BETA_PITCH 0.5//1//0.5//1	//scaling factors pitch and roll direction for velocities
#define BETA_ROLL 0.5//1//0.5//1

#define MAX_PITCH_VEL 5		//Maximum pitch and roll velocities allowed in meters/sec
#define MAX_ROLL_VEL 5






double latData[TOTAL_NODES],longData[TOTAL_NODES];		// GPS Navigation data for each node
double pitchVelConsensus[TOTAL_NODES],rollVelConsensus[TOTAL_NODES];		//Pitch and roll velocities required in each node for Consensus
double previousLatValue[TOTAL_NODES],previousLonValue[TOTAL_NODES];  //used to calculate lat long velocities in calculateVelocityGPS
int AdjMatrix[TOTAL_NODES][TOTAL_NODES];		//User defined Adjacency Matrix
double LatVelocityGPS[TOTAL_NODES],LonVelocityGPS[TOTAL_NODES]; 
int temp = 0;

double quat_x[TOTAL_NODES],quat_y[TOTAL_NODES],quat_z[TOTAL_NODES],quat_w[TOTAL_NODES];

double yaw[TOTAL_NODES], yaw_setpt;

double err[TOTAL_NODES], prev_err[TOTAL_NODES], integral[TOTAL_NODES], control[TOTAL_NODES];


void initializeNetworkTopology(void) ;
void calculateVelocityGPS(int);
void computeConsensusVel(int);


						//**Callback functions for Quadrotors**//

	//Quad1 callback function	
	void quad1msgrcv(const geometry_msgs::PoseStamped &quad1rcv)
	{
		
	latData[0] = quad1rcv.pose.position.x;
	longData[0] = quad1rcv.pose.position.y;	
	quat_x[0] = quad1rcv.pose.orientation.x;
	quat_y[0] = quad1rcv.pose.orientation.y;	
        quat_z[0] = quad1rcv.pose.orientation.z;
	quat_w[0] = quad1rcv.pose.orientation.w;	
	}
	 
	//Quad2 callback function
	void quad2msgrcv(const geometry_msgs::PoseStamped &quad2rcv)
	{
		
	latData[1] = quad2rcv.pose.position.x;
	longData[1] = quad2rcv.pose.position.y;	
	quat_x[1] = quad2rcv.pose.orientation.x;
	quat_y[1] = quad2rcv.pose.orientation.y;	
        quat_z[1] = quad2rcv.pose.orientation.z;
	quat_w[1] = quad2rcv.pose.orientation.w;	
	}

	//Quad3 callback function
	void quad3msgrcv(const geometry_msgs::PoseStamped &quad3rcv)
	{
		
	latData[2] = quad3rcv.pose.position.x;
	longData[2] = quad3rcv.pose.position.y;
	quat_x[2] = quad3rcv.pose.orientation.x;
	quat_y[2] = quad3rcv.pose.orientation.y;	
        quat_z[2] = quad3rcv.pose.orientation.z;
	quat_w[2] = quad3rcv.pose.orientation.w;		
	}	



//Quad4 callback function
	void quad4msgrcv(const geometry_msgs::PoseStamped &quad4rcv)
	{
		
	latData[3] = quad4rcv.pose.position.x;
	longData[3] = quad4rcv.pose.position.y;	
	quat_x[3] = quad4rcv.pose.orientation.x;
	quat_y[3] = quad4rcv.pose.orientation.y;	
        quat_z[3] = quad4rcv.pose.orientation.z;
	quat_w[3] = quad4rcv.pose.orientation.w;	
	}	



//Quad5 callback function
	void quad5msgrcv(const geometry_msgs::PoseStamped &quad5rcv)
	{
		
	latData[4] = quad5rcv.pose.position.x;
	longData[4] = quad5rcv.pose.position.y;		
	quat_x[4] = quad5rcv.pose.orientation.x;
	quat_y[4] = quad5rcv.pose.orientation.y;	
        quat_z[4] = quad5rcv.pose.orientation.z;
	quat_w[4] = quad5rcv.pose.orientation.w;
	}	





#include "heading_hold.cpp"


int main(int argc, char **argv) {
	
	// Quad Connectivity
	initializeNetworkTopology();

	//Initializing Ros client library
	
	ros::init(argc,argv,"Consensus");
	
	//Creating Node handle object
	
	ros::NodeHandle nh;

						//**Creating Publisher objects for Quadrotors**//

	

	//Publisher object for Quadrotor 1

	ros::Publisher quad1pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
        
        //Publisher object for Quadrotor 2
        ros::Publisher quad2pub = nh.advertise<geometry_msgs::Twist>("uav1/cmd_vel",1000);
        
        //Publisher object for Quadrotor 3
        ros::Publisher quad3pub = nh.advertise<geometry_msgs::Twist>("uav2/cmd_vel",1000);

	//Publisher object for Quadrotor 4
        ros::Publisher quad4pub = nh.advertise<geometry_msgs::Twist>("uav3/cmd_vel",1000);
        
        //Publisher object for Quadrotor 5
        ros::Publisher quad5pub = nh.advertise<geometry_msgs::Twist>("uav4/cmd_vel",1000);





						//**Creating Message Objects for Quadrotors**//
	geometry_msgs::Twist quad1snd;
	geometry_msgs::Twist quad2snd;
	geometry_msgs::Twist quad3snd;
	geometry_msgs::Twist quad4snd;
	geometry_msgs::Twist quad5snd;





					//** Subscriber objects**//

// Create a subscriber object.
   ros::Subscriber quad1sub = nh.subscribe("/ground_truth_to_tf/pose",1000, &quad1msgrcv);
   ros::Subscriber quad2sub = nh.subscribe("/uav1/ground_truth_to_tf/pose",1000, &quad2msgrcv);
   ros::Subscriber quad3sub = nh.subscribe("/uav2/ground_truth_to_tf/pose",1000, &quad3msgrcv);
   ros::Subscriber quad4sub = nh.subscribe("/uav3/ground_truth_to_tf/pose",1000, &quad4msgrcv);
   ros::Subscriber quad5sub = nh.subscribe("/uav4/ground_truth_to_tf/pose",1000, &quad5msgrcv);




// Loop Start

ros::Rate rate(100); //was 10 in position consensus without heading hold, check
while(ros::ok())

{
	


// Code to receive Quadcopters data from seperate topics
	
    ros::spinOnce();

    
   /*for(int i =1; i <= TOTAL_NODES ; ++i)
    { 
	previousLatValue[i-1]=latData[i - 1];
	previousLonValue[i-1]=longData[i - 1];
     }*/	
	
	
	
//*********************************************Consensus Algorithm implementation***********************************************************




/*********************************************************************************************************************
Calculation of velocity using GPS data // in this program using pose data
Heading hold implementation on all the quadrotors
********************************************************************************************************************/
for(int i = 1; i <=TOTAL_NODES; i++ )

{ 
calculateVelocityGPS(i);
head_hold(i);
}

/*********************************************************************************************************************
Call this function to calculate the velocities required by each node for Consensus
********************************************************************************************************************/
for(int i = 1; i <=TOTAL_NODES; ++i )

{ 
computeConsensusVel(i);
}
	
/*************************************************************************************************************************	
	                                       // Message Publish to Quadrotors
*************************************************************************************************************************/
//ros::Duration(3).sleep();

if(temp<200)
{
quad1snd.linear.z = 0.5;
quad2snd.linear.z = 1.5;         // To give some initial height to Quadrotors
quad3snd.linear.z =2.5;
quad4snd.linear.z =3.5;
quad5snd.linear.z =4.5;
temp++;
}


else if(temp>=200)
{
quad1snd.linear.z = 0;
quad2snd.linear.z = 0;
quad3snd.linear.z =0;
quad4snd.linear.z =0;
quad5snd.linear.z =0;




quad1snd.linear.x = pitchVelConsensus[0];
quad2snd.linear.x = pitchVelConsensus[1];
quad3snd.linear.x = pitchVelConsensus[2];
quad4snd.linear.x = pitchVelConsensus[3];
quad5snd.linear.x = pitchVelConsensus[4];




quad1snd.linear.y = rollVelConsensus[0];
quad2snd.linear.y = rollVelConsensus[1];
quad3snd.linear.y = rollVelConsensus[2];
quad4snd.linear.y = rollVelConsensus[3];
quad5snd.linear.y = rollVelConsensus[4];







}
quad1snd.angular.z =  -control[0];
quad2snd.angular.z =  -control[1];
quad3snd.angular.z =  -control[2];
quad4snd.angular.z =  -control[3];
quad5snd.angular.z =  -control[4];


quad1pub.publish(quad1snd);
quad2pub.publish(quad2snd);
quad3pub.publish(quad3snd);
quad4pub.publish(quad4snd);
quad5pub.publish(quad5snd);
rate.sleep();
}


	
	return 0;
}







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
		AdjMatrix[1][2]=AdjMatrix[2][1]=1;
		
                AdjMatrix[2][3]=AdjMatrix[3][2]=1;
		AdjMatrix[3][4]=AdjMatrix[4][3]=1;
                AdjMatrix[4][0]=AdjMatrix[4][0]=1;;
		
		for (int i=0;i<TOTAL_NODES;i++)
		{
			for (int j=0;j<TOTAL_NODES;j++)
			{
				if(i!=j) AdjMatrix[i][j]=1;	//Initialise all elements to 0
			}
		}
                
		
	}


void calculateVelocityGPS(int nodeNo)
{
	
	LatVelocityGPS[nodeNo - 1] = latData[nodeNo - 1] - previousLatValue[nodeNo-1];
	LonVelocityGPS[nodeNo - 1] = longData[nodeNo - 1] - previousLonValue[nodeNo-1];
	
	previousLatValue[nodeNo-1]=latData[nodeNo - 1];
	previousLonValue[nodeNo-1]=longData[nodeNo - 1];
}	



void computeConsensusVel(int nodeNo)
{
        //Computing angles from lat/long data
        double latConsensusError = 0;
        double lonConsensusError = 0;
	double distLat=0;
	double distActual,distLon=0;
        for (int j=0; j<TOTAL_NODES; j++)
	{
		distLat= latData[j] - latData[nodeNo - 1]; //calculate distance along Lat/x direction
		distLon= longData[j] - longData[nodeNo - 1]; // Calculate distance along Lon/y direction
		distActual= sqrt(distLat*distLat + distLon*distLon);	//Calculate actual distance between 2 quadcoptors

		/* Distance based Adjecency Matrix*/
		if (distActual>75) AdjMatrix[nodeNo-1][j]=AdjMatrix[j][nodeNo-1]=0;
		else AdjMatrix[nodeNo-1][j]=AdjMatrix[j][nodeNo-1]=1;


		latConsensusError += ALPHA_PITCH * AdjMatrix[nodeNo-1][j] * (distLat) ;
		lonConsensusError += ALPHA_ROLL * AdjMatrix[nodeNo-1][j] * (distLon) ;
	}
	pitchVelConsensus[nodeNo-1] = latConsensusError - BETA_PITCH * LatVelocityGPS[nodeNo - 1];
	rollVelConsensus[nodeNo-1] = lonConsensusError - BETA_ROLL * LonVelocityGPS[nodeNo - 1];

	//Keeping the roll/pitch angles between +-20 deg
	if(pitchVelConsensus[nodeNo-1]>MAX_PITCH_VEL) pitchVelConsensus[nodeNo-1]=MAX_PITCH_VEL;
	
	else if(pitchVelConsensus[nodeNo-1]< -MAX_PITCH_VEL) pitchVelConsensus[nodeNo-1]=-MAX_PITCH_VEL;
	
	if(rollVelConsensus[nodeNo-1]>MAX_ROLL_VEL) rollVelConsensus[nodeNo-1]=MAX_ROLL_VEL;
	
	else if(rollVelConsensus[nodeNo-1]< -MAX_ROLL_VEL) rollVelConsensus[nodeNo-1]=-MAX_ROLL_VEL;
	
	}
	
