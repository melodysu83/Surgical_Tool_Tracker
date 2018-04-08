#include "myproject/MyTracker_FwdKinematics.h"

MyTracker_FwdKinematics::MyTracker_FwdKinematics()
{
	lw =  14;  // in mm
	lg =  20;  // in mm
	d4 = 470;  // in mm (d4=470mm for Raven Tool)
	a3 =   0;  // in mm (a3=0 for Raven Tool)

	double dx = 300.74;
	double dy = 61;
	double dz = -7;
	this->RAVEN_BASE2ZERO[0] = dx;
	this->RAVEN_BASE2ZERO[1] = dy;
	this->RAVEN_BASE2ZERO[2] = dz;
	this->RAVEN_BASE2ZERO[3] = -dx;
	this->RAVEN_BASE2ZERO[4] = dy;
	this->RAVEN_BASE2ZERO[5] = dz;

	this->OFFSET_XB = 0;
	this->OFFSET_YB = 0;
	this->OFFSET_ZB = 0;
	this->OFFSET_AA = 0;
}



MyTracker_FwdKinematics::~MyTracker_FwdKinematics()
{

}


void MyTracker_FwdKinematics::load_camera_pose(Mat rVec, Mat tVec)
{
	cameraRotationVector = rVec.clone();
	cameraTranslationVector = tVec.clone();
}


void MyTracker_FwdKinematics::load_raven_frame_setting(double x_left,double y_left,double z_left,double x_right,double y_right,double z_right)
{
	RAVEN_BASE2ZERO[0] = x_left;
	RAVEN_BASE2ZERO[1] = y_left;
	RAVEN_BASE2ZERO[2] = z_left;
	RAVEN_BASE2ZERO[3] = x_right;
	RAVEN_BASE2ZERO[4] = y_right;
	RAVEN_BASE2ZERO[5] = z_right;	
}


void MyTracker_FwdKinematics::load_xyz_offset(vector<int> offset)
{
	OFFSET_XB = offset[0];
	OFFSET_YB = offset[1];
	OFFSET_ZB = offset[2];
	OFFSET_AA = offset[3];
}



bool MyTracker_FwdKinematics::set_ArmType(int armtype)
{
	bool armOK = armtype == LEFT_ARM || armtype == RIGHT_ARM;
	ArmType = armtype;

	return armOK;
}



bool MyTracker_FwdKinematics::set_current_ravenstate(raven_2::raven_state curr_rav_stat)
{
	CURRENT_RAVEN_ORI.clear();
	CURRENT_RAVEN_POS.clear();
	CURRENT_RAVEN_JPOS.clear();

	if(ArmType == LEFT_ARM)
	{
		for(int i=0; i<3; i++)
			CURRENT_RAVEN_POS.push_back(curr_rav_stat.pos[i]/1000); // micro meter to mm

		for(int i=0; i<9; i++)
			CURRENT_RAVEN_ORI.push_back(curr_rav_stat.ori[i]);

		for(int i=0; i<8; i++)
			CURRENT_RAVEN_JPOS.push_back(curr_rav_stat.jpos[i]);
	}
	else if(ArmType == RIGHT_ARM)
	{
		for(int i=3; i<6; i++)
			CURRENT_RAVEN_POS.push_back(curr_rav_stat.pos[i]/1000); // micro meter to mm

		for(int i=9; i<18; i++)
			CURRENT_RAVEN_ORI.push_back(curr_rav_stat.ori[i]);

		for(int i=8; i<16; i++)
			CURRENT_RAVEN_JPOS.push_back(curr_rav_stat.jpos[i]);
	}
}


vector<double> MyTracker_FwdKinematics::compute_distance_to_cam()
{
	double dx,dy,dz, distance; // in mm
	RAVEN_MODEL_DISTANCE.clear();

	for(int i=0;i<RAVEN_MODEL_POINTS.size();i++)
	{
		dx = RAVEN_MODEL_POINTS[i].x - cameraTranslationVector.at<double>(0);
		dy = RAVEN_MODEL_POINTS[i].y - cameraTranslationVector.at<double>(1);
		dz = RAVEN_MODEL_POINTS[i].z - cameraTranslationVector.at<double>(2);

		distance = sqrt(dx*dx+dy*dy+dz*dz);
		RAVEN_MODEL_DISTANCE.push_back(distance);
	}
	return RAVEN_MODEL_DISTANCE;
}



vector<Point3d>  MyTracker_FwdKinematics::compute_model_points()
{
	Point3d tmp1,A,B,C,D,E;

	// tool section length
	vector<double> gripper(3),wrist(3),toolstick(3);
	gripper[0] = lg;	wrist[0] = -lw;		toolstick[0] = -d4;
	gripper[1] = 0;		wrist[1] = 0;		toolstick[1] = 0;
	gripper[2] = 0;		wrist[2] = 0;		toolstick[2] = 0;

	double angle7 = CURRENT_RAVEN_JPOS[7]*CV_PI/180;
	double angle6 = CURRENT_RAVEN_JPOS[6]*CV_PI/180;
	double angle5 = CURRENT_RAVEN_JPOS[5]*CV_PI/180 + OFFSET_AA*CV_PI/180; // tool shaft angle offset
	double angleg = (angle7-angle6)/2;

	// rotation matrix around axes
	vector<double> rotateZ_angleg(9);
	vector<double> rotateZ_angle7(9);
	vector<double> rotateZ_angle6(9);
	vector<double> rotateY_angle5(9);	
	
	rotateZ_angleg[0] = cos(angleg);	rotateZ_angleg[1] = sin(angleg);	rotateZ_angleg[2] = 0;
	rotateZ_angleg[3] = -rotateZ_angleg[1];	rotateZ_angleg[4] = rotateZ_angleg[0];	rotateZ_angleg[5] = 0;
	rotateZ_angleg[6] = 0;			rotateZ_angleg[7] = 0;			rotateZ_angleg[8] = 1;

	rotateZ_angle7[0] = cos(-angle7);	rotateZ_angle7[1] = sin(-angle7);	rotateZ_angle7[2] = 0;
	rotateZ_angle7[3] = -rotateZ_angle7[1];	rotateZ_angle7[4] = rotateZ_angle7[0];	rotateZ_angle7[5] = 0;
	rotateZ_angle7[6] = 0;			rotateZ_angle7[7] = 0;			rotateZ_angle7[8] = 1;

	rotateZ_angle6[0] = cos(angle6);	rotateZ_angle6[1] = sin(angle6);	rotateZ_angle6[2] = 0;
	rotateZ_angle6[3] = -rotateZ_angle6[1];	rotateZ_angle6[4] = rotateZ_angle6[0];	rotateZ_angle6[5] = 0;
	rotateZ_angle6[6] = 0;			rotateZ_angle6[7] = 0;			rotateZ_angle6[8] = 1;

	rotateY_angle5[0] = cos(angle5);	rotateY_angle5[1] = 0;			rotateY_angle5[2] = -sin(angle5);
	rotateY_angle5[3] = 0;			rotateY_angle5[4] = 1;			rotateY_angle5[5] = 0;
	rotateY_angle5[6] = -rotateY_angle5[2];	rotateY_angle5[7] = 0;			rotateY_angle5[8] = rotateY_angle5[0];


	// compute offset
	vector<double> tmp      = matmat_mul3x3(CURRENT_RAVEN_ORI,rotateZ_angleg);
	vector<double> offset_D = matvec_mul3x1(matmat_mul3x3(CURRENT_RAVEN_ORI,rotateZ_angle7),gripper);
	vector<double> offset_E = matvec_mul3x1(matmat_mul3x3(CURRENT_RAVEN_ORI,rotateZ_angle6),gripper);
	vector<double> offset_B = matvec_mul3x1(tmp,wrist);
	vector<double> offset_A = matvec_mul3x1(matmat_mul3x3(tmp,rotateY_angle5),toolstick);

	// compute result
	C = Point3d(CURRENT_RAVEN_POS[0],CURRENT_RAVEN_POS[1],CURRENT_RAVEN_POS[2]);
	D = Point3d(C.x+offset_D[0], C.y+offset_D[1], C.z+offset_D[2]);
	E = Point3d(C.x+offset_E[0], C.y+offset_E[1], C.z+offset_E[2]);
	B = Point3d(C.x+offset_B[0], C.y+offset_B[1], C.z+offset_B[2]);
	A = Point3d(C.x+offset_A[0], C.y+offset_A[1], C.z+offset_A[2]);

	// prepare output - in the raven base frame
	RAVEN_MODEL_POINTS.clear();
	tmp1.x =  A.z + RAVEN_BASE2ZERO[0] + OFFSET_XB;
	tmp1.y = -A.y + RAVEN_BASE2ZERO[1] + OFFSET_YB;
	tmp1.z =  A.x + RAVEN_BASE2ZERO[2] + OFFSET_ZB;
	RAVEN_MODEL_POINTS.push_back(tmp1);

	tmp1.x =  B.z + RAVEN_BASE2ZERO[0] + OFFSET_XB;
	tmp1.y = -B.y + RAVEN_BASE2ZERO[1] + OFFSET_YB;
	tmp1.z =  B.x + RAVEN_BASE2ZERO[2] + OFFSET_ZB;
	RAVEN_MODEL_POINTS.push_back(tmp1);
	
	tmp1.x =  C.z + RAVEN_BASE2ZERO[0] + OFFSET_XB;
	tmp1.y = -C.y + RAVEN_BASE2ZERO[1] + OFFSET_YB;
	tmp1.z =  C.x + RAVEN_BASE2ZERO[2] + OFFSET_ZB;
	RAVEN_MODEL_POINTS.push_back(tmp1);

	tmp1.x =  D.z + RAVEN_BASE2ZERO[0] + OFFSET_XB;
	tmp1.y = -D.y + RAVEN_BASE2ZERO[1] + OFFSET_YB;
	tmp1.z =  D.x + RAVEN_BASE2ZERO[2] + OFFSET_ZB;
	RAVEN_MODEL_POINTS.push_back(tmp1);

	tmp1.x =  E.z + RAVEN_BASE2ZERO[0] + OFFSET_XB;
	tmp1.y = -E.y + RAVEN_BASE2ZERO[1] + OFFSET_YB;
	tmp1.z =  E.x + RAVEN_BASE2ZERO[2] + OFFSET_ZB;
	RAVEN_MODEL_POINTS.push_back(tmp1);

	return RAVEN_MODEL_POINTS;

	/*
	// Alternative method of getting A and B: not reliable

	double theta6 = (-angle7+angle6)/2;
	double theta5 = CURRENT_RAVEN_JPOS[5]*CV_PI/180;
	double theta4 = (CURRENT_RAVEN_JPOS[4]*CV_PI-90)/180;
	vector<double> Tc(16), T46(16), T34(16), inv_T46(16), inv_T34(16);

	// transdormation matrix of point c
	Tc[0]  = CURRENT_RAVEN_ORI[0];	Tc[1]  = CURRENT_RAVEN_ORI[1];	Tc[2]  = CURRENT_RAVEN_ORI[2];	Tc[3]  = CURRENT_RAVEN_POS[0];
	Tc[4]  = CURRENT_RAVEN_ORI[3];	Tc[5]  = CURRENT_RAVEN_ORI[4];	Tc[6]  = CURRENT_RAVEN_ORI[5];	Tc[7]  = CURRENT_RAVEN_POS[1];
	Tc[8]  = CURRENT_RAVEN_ORI[6];	Tc[9]  = CURRENT_RAVEN_ORI[7];	Tc[10] = CURRENT_RAVEN_ORI[8];	Tc[11] = CURRENT_RAVEN_POS[2];
	Tc[12] = 0;			Tc[13] = 0;			Tc[14] = 0;			Tc[15] = 1;
	
	// transformation matrix (frame4 to frame6)
	T46[0]  = cos(theta5)*cos(theta6);	T46[1]  = -cos(theta5)*sin(theta6);	T46[2]  = sin(theta5);	T46[3]  = cos(theta5)*lw;
	T46[4]  = -sin(theta6);			T46[5]  = -cos(theta6);			T46[6]  = 0;		T46[7]  = 0;
	T46[8]  = sin(theta5)*cos(theta6);	T46[9]  = -sin(theta5)*sin(theta6);	T46[10] = -cos(theta5);	T46[11] = sin(theta5)*lw;
	T46[12] = 0;				T46[13] = 0;				T46[14] = 0;		T46[15] = 1;
	
	// transformation matrix (frame3 to frame4)
	T34[0]  = cos(theta4);	T34[1]  = -sin(theta4);	T34[2]  = 0;	T34[3]  = a3;
	T34[4]  = sin(theta4);	T34[5]  = cos(theta4);	T34[6]  = 0;	T34[7]  = 0;
	T34[8]  = 0;		T34[9]  = 0;		T34[10] = 1;	T34[11] = d4;
	T34[12] = 0;		T34[13] = 0;		T34[14] = 0;	T34[15] = 1;

	// the corrsponding inverse matices
	inv_T46 = inverse4x4(T46);
	inv_T34 = inverse4x4(T34);
	vector<double> offset_B = matmat_mul4x4(Tc,inv_T46);
	vector<double> offset_A = matmat_mul4x4(offset_B,inv_T34);

	//compute result
	B.x = offset_B[3];
	B.y = offset_B[7];
	B.z = offset_B[11];

	A.x = offset_A[3];
	A.y = offset_A[7];
	A.z = offset_A[11];

	*/
}



vector<Point3d>  MyTracker_FwdKinematics::compute_axes_points()
{


	// change position from zero frame to base frame
	double xO = CURRENT_RAVEN_POS[0];
	double yO = CURRENT_RAVEN_POS[1];
	double zO = CURRENT_RAVEN_POS[2];

	double xB =  zO + RAVEN_BASE2ZERO[0] + OFFSET_XB;
	double yB = -yO + RAVEN_BASE2ZERO[1] + OFFSET_YB;
	double zB =  xO + RAVEN_BASE2ZERO[2] + OFFSET_ZB;

	// include orientation info
	double xO_x_axis = xO+CURRENT_RAVEN_ORI[0]*AXIS_LENGTH;
	double yO_x_axis = yO+CURRENT_RAVEN_ORI[3]*AXIS_LENGTH;
	double zO_x_axis = zO+CURRENT_RAVEN_ORI[6]*AXIS_LENGTH;

	double xO_y_axis = xO+CURRENT_RAVEN_ORI[1]*AXIS_LENGTH;
	double yO_y_axis = yO+CURRENT_RAVEN_ORI[4]*AXIS_LENGTH;
	double zO_y_axis = zO+CURRENT_RAVEN_ORI[7]*AXIS_LENGTH;

	double xO_z_axis = xO+CURRENT_RAVEN_ORI[2]*AXIS_LENGTH;
	double yO_z_axis = yO+CURRENT_RAVEN_ORI[5]*AXIS_LENGTH;
	double zO_z_axis = zO+CURRENT_RAVEN_ORI[8]*AXIS_LENGTH;

	double xB_x_axis =  zO_x_axis + RAVEN_BASE2ZERO[0] + OFFSET_XB;
	double yB_x_axis = -yO_x_axis + RAVEN_BASE2ZERO[1] + OFFSET_YB;
	double zB_x_axis =  xO_x_axis + RAVEN_BASE2ZERO[2] + OFFSET_ZB;

	double xB_y_axis =  zO_y_axis + RAVEN_BASE2ZERO[0] + OFFSET_XB;
	double yB_y_axis = -yO_y_axis + RAVEN_BASE2ZERO[1] + OFFSET_YB;
	double zB_y_axis =  xO_y_axis + RAVEN_BASE2ZERO[2] + OFFSET_ZB;

	double xB_z_axis =  zO_z_axis + RAVEN_BASE2ZERO[0] + OFFSET_XB;
	double yB_z_axis = -yO_z_axis + RAVEN_BASE2ZERO[1] + OFFSET_YB;
	double zB_z_axis =  xO_z_axis + RAVEN_BASE2ZERO[2] + OFFSET_ZB;

	// store the points in a vector
	RAVEN_AXES_POINTS.clear();
	RAVEN_AXES_POINTS.push_back(Point3d (xB,yB,zB));
	RAVEN_AXES_POINTS.push_back(Point3d (xB_x_axis,yB_x_axis,zB_x_axis));
	RAVEN_AXES_POINTS.push_back(Point3d (xB_y_axis,yB_y_axis,zB_y_axis));
	RAVEN_AXES_POINTS.push_back(Point3d (xB_z_axis,yB_z_axis,zB_z_axis));

	return RAVEN_AXES_POINTS;
}



vector<double> MyTracker_FwdKinematics::matmat_mul3x3(vector<double>M1, vector<double>M2)
{
	//result(3x3) = M1(3x3)*M2(3x3)

	vector<double> result(9);

	result[0] = M1[0]*M2[0]+M1[1]*M2[3]+M1[2]*M2[6];
	result[1] = M1[0]*M2[1]+M1[1]*M2[4]+M1[2]*M2[7];
	result[2] = M1[0]*M2[2]+M1[1]*M2[5]+M1[2]*M2[8];
	result[3] = M1[3]*M2[0]+M1[4]*M2[3]+M1[5]*M2[6];
	result[4] = M1[3]*M2[1]+M1[4]*M2[4]+M1[5]*M2[7];
	result[5] = M1[3]*M2[2]+M1[4]*M2[5]+M1[5]*M2[8];
	result[6] = M1[6]*M2[0]+M1[7]*M2[3]+M1[8]*M2[6];
	result[7] = M1[6]*M2[1]+M1[7]*M2[4]+M1[8]*M2[7];
	result[8] = M1[6]*M2[2]+M1[7]*M2[5]+M1[8]*M2[8];
	
	return result;
}


vector<double> MyTracker_FwdKinematics::matmat_mul4x4(vector<double>M1, vector<double>M2)
{
	//result(4x4) = M1(4x4)*M2(4x4)

	vector<double> result(16);

	result[0]  = M1[0] *M2[0] +M1[1]*M2[4] +M1[2] *M2[8] +M1[3]*M2[12];
	result[1]  = M1[0] *M2[1] +M1[1]*M2[5] +M1[2] *M2[9] +M1[3]*M2[13];
	result[2]  = M1[0] *M2[2] +M1[1]*M2[6] +M1[2]*M2[10] +M1[3]*M2[14];
	result[3]  = M1[0] *M2[3] +M1[1]*M2[7] +M1[2]*M2[11] +M1[3]*M2[15];
	result[4]  = M1[4] *M2[0] +M1[5]*M2[4] +M1[6] *M2[8] +M1[7]*M2[12];
	result[5]  = M1[4] *M2[1] +M1[5]*M2[5] +M1[6] *M2[9] +M1[7]*M2[13];
	result[6]  = M1[4] *M2[2] +M1[5]*M2[6] +M1[6]*M2[10] +M1[7]*M2[14];
	result[7]  = M1[4] *M2[3] +M1[5]*M2[7] +M1[6]*M2[11] +M1[7]*M2[15];
	result[8]  = M1[8] *M2[0] +M1[9]*M2[4]+M1[10] *M2[8]+M1[11]*M2[12];
	result[9]  = M1[8] *M2[1] +M1[9]*M2[5]+M1[10] *M2[9]+M1[11]*M2[13];
	result[10] = M1[8] *M2[2] +M1[9]*M2[6]+M1[10]*M2[10]+M1[11]*M2[14];
	result[11] = M1[8] *M2[3] +M1[9]*M2[7]+M1[10]*M2[11]+M1[11]*M2[15];
	result[12] = M1[12]*M2[0]+M1[13]*M2[4]+M1[14] *M2[8]+M1[15]*M2[12];
	result[13] = M1[12]*M2[1]+M1[13]*M2[5]+M1[14] *M2[9]+M1[15]*M2[13];
	result[14] = M1[12]*M2[2]+M1[13]*M2[6]+M1[14]*M2[10]+M1[15]*M2[14];
	result[15] = M1[12]*M2[3]+M1[13]*M2[7]+M1[14]*M2[11]+M1[15]*M2[15];
	
	return result;
}


vector<double> MyTracker_FwdKinematics::matvec_mul3x1(vector<double>M, vector<double>V)
{
	//result(3x1) = M(3x3)*V(3x1)

	vector<double> result(3);

	result[0] = M[0]*V[0]+M[1]*V[1]+M[2]*V[2];
	result[1] = M[3]*V[0]+M[4]*V[1]+M[5]*V[2];
	result[2] = M[6]*V[0]+M[7]*V[1]+M[8]*V[2];
	
	return result;
}

vector<double> MyTracker_FwdKinematics::inverse4x4(vector<double> m)
{
	//result(4x4) = inverse(m(4x4))

	vector<double> result(16);
	double inv[16], det;

	inv[0] = m[5]  * m[10] * m[15] - 
	     m[5]  * m[11] * m[14] - 
	     m[9]  * m[6]  * m[15] + 
	     m[9]  * m[7]  * m[14] +
	     m[13] * m[6]  * m[11] - 
	     m[13] * m[7]  * m[10];

	inv[4] = -m[4]  * m[10] * m[15] + 
	      m[4]  * m[11] * m[14] + 
	      m[8]  * m[6]  * m[15] - 
	      m[8]  * m[7]  * m[14] - 
	      m[12] * m[6]  * m[11] + 
	      m[12] * m[7]  * m[10];

	inv[8] = m[4]  * m[9] * m[15] - 
	     m[4]  * m[11] * m[13] - 
	     m[8]  * m[5] * m[15] + 
	     m[8]  * m[7] * m[13] + 
	     m[12] * m[5] * m[11] - 
	     m[12] * m[7] * m[9];

	inv[12] = -m[4]  * m[9] * m[14] + 
	       m[4]  * m[10] * m[13] +
	       m[8]  * m[5] * m[14] - 
	       m[8]  * m[6] * m[13] - 
	       m[12] * m[5] * m[10] + 
	       m[12] * m[6] * m[9];

	inv[1] = -m[1]  * m[10] * m[15] + 
	      m[1]  * m[11] * m[14] + 
	      m[9]  * m[2] * m[15] - 
	      m[9]  * m[3] * m[14] - 
	      m[13] * m[2] * m[11] + 
	      m[13] * m[3] * m[10];

	inv[5] = m[0]  * m[10] * m[15] - 
	     m[0]  * m[11] * m[14] - 
	     m[8]  * m[2] * m[15] + 
	     m[8]  * m[3] * m[14] + 
	     m[12] * m[2] * m[11] - 
	     m[12] * m[3] * m[10];

	inv[9] = -m[0]  * m[9] * m[15] + 
	      m[0]  * m[11] * m[13] + 
	      m[8]  * m[1] * m[15] - 
	      m[8]  * m[3] * m[13] - 
	      m[12] * m[1] * m[11] + 
	      m[12] * m[3] * m[9];

	inv[13] = m[0]  * m[9] * m[14] - 
	      m[0]  * m[10] * m[13] - 
	      m[8]  * m[1] * m[14] + 
	      m[8]  * m[2] * m[13] + 
	      m[12] * m[1] * m[10] - 
	      m[12] * m[2] * m[9];

	inv[2] = m[1]  * m[6] * m[15] - 
	     m[1]  * m[7] * m[14] - 
	     m[5]  * m[2] * m[15] + 
	     m[5]  * m[3] * m[14] + 
	     m[13] * m[2] * m[7] - 
	     m[13] * m[3] * m[6];

	inv[6] = -m[0]  * m[6] * m[15] + 
	      m[0]  * m[7] * m[14] + 
	      m[4]  * m[2] * m[15] - 
	      m[4]  * m[3] * m[14] - 
	      m[12] * m[2] * m[7] + 
	      m[12] * m[3] * m[6];

	inv[10] = m[0]  * m[5] * m[15] - 
	      m[0]  * m[7] * m[13] - 
	      m[4]  * m[1] * m[15] + 
	      m[4]  * m[3] * m[13] + 
	      m[12] * m[1] * m[7] - 
	      m[12] * m[3] * m[5];

	inv[14] = -m[0]  * m[5] * m[14] + 
	       m[0]  * m[6] * m[13] + 
	       m[4]  * m[1] * m[14] - 
	       m[4]  * m[2] * m[13] - 
	       m[12] * m[1] * m[6] + 
	       m[12] * m[2] * m[5];

	inv[3] = -m[1] * m[6] * m[11] + 
	      m[1] * m[7] * m[10] + 
	      m[5] * m[2] * m[11] - 
	      m[5] * m[3] * m[10] - 
	      m[9] * m[2] * m[7] + 
	      m[9] * m[3] * m[6];

	inv[7] = m[0] * m[6] * m[11] - 
	     m[0] * m[7] * m[10] - 
	     m[4] * m[2] * m[11] + 
	     m[4] * m[3] * m[10] + 
	     m[8] * m[2] * m[7] - 
	     m[8] * m[3] * m[6];

	inv[11] = -m[0] * m[5] * m[11] + 
	       m[0] * m[7] * m[9] + 
	       m[4] * m[1] * m[11] - 
	       m[4] * m[3] * m[9] - 
	       m[8] * m[1] * m[7] + 
	       m[8] * m[3] * m[5];

	inv[15] = m[0] * m[5] * m[10] - 
	      m[0] * m[6] * m[9] - 
	      m[4] * m[1] * m[10] + 
	      m[4] * m[2] * m[9] + 
	      m[8] * m[1] * m[6] - 
	      m[8] * m[2] * m[5];

	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	if (det == 0)
	{
		for (int i = 0; i < 16; i++)
			result[i] = 0;
	}
	else
	{
		det = 1.0 / det;
		for (int i = 0; i < 16; i++)
			result[i] = inv[i] * det;
	}
	return result;
}
