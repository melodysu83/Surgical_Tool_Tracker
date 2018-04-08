#include "myproject/MyTracker_MotionPlanner.h"

MyTracker_MotionPlanner::MyTracker_MotionPlanner()
{
	
}


MyTracker_MotionPlanner::~MyTracker_MotionPlanner()
{

}


tf::Transform MyTracker_MotionPlanner::ComputeNullMotion()
{
	tf::Transform TF_INCR;

	tfScalar X = 0;
	tfScalar Y = 0;
	tfScalar Z = 0;
	Delta_Pos.setValue(X,Y,Z); 

	tfScalar  W = 1;
	tfScalar QX = 0;
	tfScalar QY = 0;
	tfScalar QZ = 0;
	
	tf::Quaternion q_temp(QX,QY,QZ,W);		
	Delta_Ori = q_temp;

	TF_INCR.setOrigin(Delta_Pos);      //add position increment
	TF_INCR.setRotation(Delta_Ori);    //add rotation increment
	return TF_INCR;
}



tf::Transform MyTracker_MotionPlanner::ComputeIncrMotion()
{
	tf::Transform TF_INCR;

	tf::Vector3 pos_inc = Desired_Pos-Current_Pos;
	double distance = pos_inc.length();

	if(distance>CLOSE_ENOUGH)
	{
		tf::Vector3 inc = distance < maxin? pos_inc:pos_inc*maxin/distance;
		Delta_Pos.setValue(inc.getX(),inc.getY(),inc.getZ());
	}
	else
		Delta_Pos.setValue(0,0,0);
	
	tfScalar  W = 1;
	tfScalar QX = 0;
	tfScalar QY = 0;
	tfScalar QZ = 0;
	
	tf::Quaternion q_temp(QX,QY,QZ,W);	
	
	Delta_Ori = q_temp;
	TF_INCR.setOrigin(Delta_Pos);      //add position increment
	TF_INCR.setRotation(Delta_Ori);    //add rotation increment
	
	return TF_INCR;
}



bool MyTracker_MotionPlanner::set_ArmType(int armtype)
{
	bool armOK = armtype == LEFT_ARM || armtype == RIGHT_ARM;
	ArmType = armtype;

	return armOK;
}


bool MyTracker_MotionPlanner::set_Current_Pos(boost::array<int, 6> currpos)
{
	tfScalar X,Y,Z;

	// (1) store current postion value
	if(ArmType == LEFT_ARM)
	{
		X = currpos[0];
		Y = currpos[1];
		Z = currpos[2];
	}
	else if(ArmType == RIGHT_ARM)
	{
		X = currpos[3];
		Y = currpos[4];
		Z = currpos[5];
	}
	else //weird case
	{
		return false;
	}

	Current_Pos.setValue(X,Y,Z);

	return true;
}


bool MyTracker_MotionPlanner::set_Current_Ori(boost::array<float, 18> rot_matix)
{
	tfScalar m00, m01, m02, m10, m11, m12, m20, m21, m22;
	tfScalar qw,qx,qy,qz,tr;

	if(ArmType == LEFT_ARM)
	{
		m00 = rot_matix[0];
		m01 = rot_matix[1];
		m02 = rot_matix[2];
		m10 = rot_matix[3];
		m11 = rot_matix[4];
		m12 = rot_matix[5];
		m20 = rot_matix[6];
		m21 = rot_matix[7];
		m22 = rot_matix[8];
	}
	else if(ArmType == RIGHT_ARM)
	{
		m00 = rot_matix[9];
		m01 = rot_matix[10];
		m02 = rot_matix[11];
		m10 = rot_matix[12];
		m11 = rot_matix[13];
		m12 = rot_matix[14];
		m20 = rot_matix[15];
		m21 = rot_matix[16];
		m22 = rot_matix[17];
	}	
	else // weird case
	{
		return false;
	}

	tr = m00 + m11 + m22;

	if (tr > 0) 
	{ 
		  tfScalar S = sqrt(tr+1.0) * 2; // S=4*qw 
		  qw = 0.25 * S;
		  qx = (m21 - m12) / S;
		  qy = (m02 - m20) / S; 
		  qz = (m10 - m01) / S; 
	} 
	else if ((m00 > m11)&(m00 > m22)) 
	{ 
		  tfScalar S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
		  qw = (m21 - m12) / S;
		  qx = 0.25 * S;
		  qy = (m01 + m10) / S; 
		  qz = (m02 + m20) / S; 
	} 
	else if (m11 > m22) 
	{ 
		  tfScalar S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		  qw = (m02 - m20) / S;
		  qx = (m01 + m10) / S; 
		  qy = 0.25 * S;
		  qz = (m12 + m21) / S; 
	} 
	else 
	{ 
		  tfScalar S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		  qw = (m10 - m01) / S;
		  qx = (m02 + m20) / S;
		  qy = (m12 + m21) / S;
		  qz = 0.25 * S;
	}

		
	tf::Quaternion q_temp(qx,qy,qz,qw);
	Current_Ori = q_temp;

	return true;
}



bool MyTracker_MotionPlanner::set_Desired_Pos( RAVEN_MOTION_STATUS_LIST* RAVEN_MOTION)
{
	// default "desired pos" as current pos" (RAVEN_MOTION == NO_MOTION)
	tfScalar X,Y,Z;
	X = Current_Pos.getX(); 
	Y = Current_Pos.getY();
	Z = Current_Pos.getZ();
	
	
	// push the desired pos a fixed distance away (RAVEN_MOTION == START_MOTION)
	if(RAVEN_MOTION[0] == START_MOTION)
	{
		X += DEL_POS_ENTIRE;
	}
	else if(RAVEN_MOTION[1] == START_MOTION)
	{
		X -= DEL_POS_ENTIRE;
	}
	else if(RAVEN_MOTION[0] == IN_MOTION || RAVEN_MOTION[1] == IN_MOTION)
	{
		X = Desired_Pos.getX();  // keep the existing "desired x pos" (RAVEN_MOTION == IN_MOTION)
	}

	// push the desired pos a fixed distance away (RAVEN_MOTION == START_MOTION)
	if(RAVEN_MOTION[2] == START_MOTION)
	{
		Y += DEL_POS_ENTIRE;
	}
	else if(RAVEN_MOTION[3] == START_MOTION)
	{
		Y -= DEL_POS_ENTIRE;
	}
	else if(RAVEN_MOTION[2] == IN_MOTION || RAVEN_MOTION[3] == IN_MOTION)
	{
		Y = Desired_Pos.getY();  // keep the existing "desired y pos" (RAVEN_MOTION == IN_MOTION)
	}

	// push the desired pos a fixed distance away (RAVEN_MOTION == START_MOTION)
	if(RAVEN_MOTION[4] == START_MOTION)
	{
		Z += DEL_POS_ENTIRE;
	}
	else if(RAVEN_MOTION[5] == START_MOTION)
	{
		Z -= DEL_POS_ENTIRE;
	}
	else if(RAVEN_MOTION[4] == IN_MOTION || RAVEN_MOTION[5] == IN_MOTION)
	{
		Z = Desired_Pos.getZ();  // keep the existing "desired z pos" (RAVEN_MOTION == IN_MOTION)
	}

	Desired_Pos.setValue(X,Y,Z);	
}



bool MyTracker_MotionPlanner::set_Desired_Ori()
{
	Desired_Ori = Current_Ori;
}


bool* MyTracker_MotionPlanner::check_goal_reached()
{
	bool* result;
	result = new bool[3];

	int differencex = Desired_Pos.getX()-Current_Pos.getX();
	int differencey = Desired_Pos.getY()-Current_Pos.getY();
	int differencez = Desired_Pos.getZ()-Current_Pos.getZ();
	
	result[0] = (differencex<CLOSE_ENOUGH || differencex>-CLOSE_ENOUGH);
	result[1] = (differencey<CLOSE_ENOUGH || differencey>-CLOSE_ENOUGH);
	result[2] = (differencez<CLOSE_ENOUGH || differencez>-CLOSE_ENOUGH);

	return result;
}
