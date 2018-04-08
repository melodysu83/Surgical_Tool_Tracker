#ifndef MYTRACKER_FWDKINEMATICS_H
#define MYTRACKER_FWDKINEMATICS_H

#include "MyTracker_MotionPlanner.h"

class MyTracker_FwdKinematics
{
	private:	
		int ArmType;
		int  OFFSET_XB;
		int  OFFSET_YB;
		int  OFFSET_ZB;
		int  OFFSET_AA;

		double lw;
		double lg;
		double d4;
		double a3;
		cv::Mat CAMERA_TRANSLATION;
		vector<double> CURRENT_RAVEN_ORI;
		vector<double> CURRENT_RAVEN_POS;
		vector<double> CURRENT_RAVEN_JPOS;
		vector<Point3d> RAVEN_MODEL_POINTS;    // in mm (in raven base frame)
		vector<Point3d> RAVEN_AXES_POINTS;    // in mm (in raven base frame)
		vector<double> RAVEN_MODEL_DISTANCE;  // in mm

		cv::Mat cameraRotationVector;	// Camera frame origin in respect to Raven base frame
		cv::Mat cameraTranslationVector;

		double RAVEN_BASE2ZERO[6];

	public:
		MyTracker_FwdKinematics();
		~MyTracker_FwdKinematics();

		void load_camera_pose(Mat,Mat);
		void load_raven_frame_setting(double,double,double,double,double,double);
		void load_xyz_offset(vector<int>);

		bool set_ArmType(int);
		bool set_current_ravenstate(raven_2::raven_state);

		vector<Point3d> compute_model_points();
		vector<Point3d> compute_axes_points();
		vector<double> compute_distance_to_cam();

		vector<double> matmat_mul3x3(vector<double>, vector<double>);
		vector<double> matmat_mul4x4(vector<double>, vector<double>);
		vector<double> matvec_mul3x1(vector<double>, vector<double>);
		vector<double> inverse4x4(vector<double>);
		
		
};


#endif
