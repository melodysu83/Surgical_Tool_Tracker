#ifndef MYTRACKER_IMAGECALIBRATOR_H
#define MYTRACKER_IMAGECALIBRATOR_H

#include "MyTracker_Display.h"

class MyTracker_ImageCalibrator
{
	private:
		cv::Mat distortion;
		cv::Mat intrinsicMat;
		cv::Matx33f intrinsic;

		cv::Mat cameraRotationVector;	// Camera frame origin in respect to Raven base frame
		cv::Mat cameraTranslationVector;

		cv::Mat rotation_vector;  // Raven base frame origin in respect to Camera frame
		cv::Mat translation_vector;
		cv::Mat inliers;

	    	IplImage *color;
		IplImage *color_chess;
		IplImage *color_proj_pnp;
		IplImage *color_proj_guess;
		IplImage *gray;
		CvPoint2D32f* corners;
		double** corners_3d;
		vector<cv::Point2d> corners_2d_cam;
		vector<cv::Point2d> corners_2d_guess;
		int corner_count;
		int wasChessboardFound;
		int CHESSBOARD_WIDTH;
		int CHESSBOARD_HEIGHT;
		int CHESSBOARD_INTERSECTION_COUNT;
		char* file_name1;
		char* file_name2;
		char* file_name3;
		char* file_name4;
		char* file_name5;
		char* window_name_cali;

		int which_camera;
		MyTracker_Display CONSOLE;

	public:
	    	MyTracker_ImageCalibrator();
		~MyTracker_ImageCalibrator(void);
		void init(int, int,char*,int);
		void load_image(cv_bridge::CvImagePtr);
	    	bool find_chessboard_corners();
		CvPoint2D32f* get_corners();
		int get_corner_count();
		void draw_corners();
		void list_corners();
		void show_two_images();
		void show_four_images();
		void solve_pnp();
		void list_pnp_result();
		bool load_3D_points();
		bool load_cam_pose();
		bool load_cam_pose_offline();
		bool load_cam_instrinsics();
		Mat get_camera_rotat();
		Mat get_camera_trans();
		void set_camera_pose();
		void getEulerAngles(Mat &,Vec3d &);
		vector<cv::Point2d> project(vector<Point3d>);
};

#endif
