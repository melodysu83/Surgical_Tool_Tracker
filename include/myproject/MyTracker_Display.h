#ifndef MYTRACKER_DISPLAY_H
#define MYTRACKER_DISPLAY_H

#include "MyTracker_FwdKinematics.h"

class MyTracker_Display
{
	private:		
		double TIME_INTERVAL;
		int DISPARITY_ALGORITHM;

		int RAVEN_SUB_COUNT;
		int RAVEN_PUB_COUNT;
		int IMAGE_SUB_COUNT;
		int IMAGE_PUB_COUNT;

		int RAVEN_SUB_COUNT_LAST;
		int RAVEN_PUB_COUNT_LAST;
		int IMAGE_SUB_COUNT_LAST;
		int IMAGE_PUB_COUNT_LAST;

		double RAVEN_SUB_RATE;
		double RAVEN_PUB_RATE;
		double IMAGE_SUB_RATE;
		double IMAGE_PUB_RATE;

		int  OFFSET_XB;
		int  OFFSET_YB;
		int  OFFSET_ZB;
		int  OFFSET_AA;

		int SEG_BLURY_SCALE;
		int SEG_BLURY_INTENSITY;
		int SEG_THRES_LOWBOUND;
		int SEG_FILL_BLACKSPOT;

		int DEFAULT_SEG_BLURY_SCALE;
		int DEFAULT_SEG_BLURY_INTENSITY;
		int DEFAULT_SEG_THRES_LOWBOUND;
		int DEFAULT_SEG_FILL_BLACKSPOT;

		int DISP_MINDISP;	// minDisparity
		int DISP_NUMDISP;	// numDisparities
		int DISP_WINSIZE;	// blockSize, window size
		int DISP_P1;		// p1
		int DISP_P2;		// p2
		int DISP_MAXDIFF;	// disp12MaxDiff
		int DISP_FLTRCAP;	// preFilterCap
		int DISP_UNQRATE;	// uniquenessRatio
		int DISP_SPKSIZE;	// speckleWindowSize
		int DISP_SPKRNGE;	// speckleRange

		bool SHOW_COLORWHEEL;
		bool SHOW_ROS_TOPIC;
		bool SHOW_TOOL_TRACKING;
		bool INCLUDE_TOOL_TRACKING_OFFSET;
		bool INCLUDE_MANUAL_SEGMENTATION_PARAMETERS;
		bool DEPTHMAP_FILE_SAVE_PENDING;

		char* OFFSET_FILE_NAME;
		char* SEGMENT_FILE_NAME;
		char* DISPARITY_FILE_NAME;

		RAVEN_MOTION_STATUS_LIST RAVEN_MOTION[6]; 
		// x+,   x-,    y+,   y-,    z+,   z-  ........zero frame (not raven base frame)
 	        // up, down, front, back, right, left
	public:
		
		MyTracker_Display();		
		int get_key();

		void init();
		void init(int);
		bool check_show_ros();
		void toggle_show_ros();
		bool check_tool_tracking();
		bool check_show_colorwheel();
		void reset_show_colorwheel();
		void toggle_tool_tracking(bool);
		void toggle_tool_tracking_offset();
		void toggle_manual_segmentation_parameter();
		void toggle_manual_disparity_parameter();
		void set_tool_tracking_offset();
		void reset_tool_tracking_offset();
		void set_manual_segmentation_parameter();
		void reset_manual_segmentation_parameter();
		void set_manual_disparity_parameter();
		void save_depthmap_to_file(cv::Mat, cv::Mat, cv::Mat);

		vector<int> load_tool_tracking_offset();
		vector<int> load_manual_segmentation_parameter();
		vector<int> load_manual_disparity_parameter();
		
		RAVEN_MOTION_STATUS_LIST load_raven_motion_flag(int);
		void modify_raven_motion_flag(int,RAVEN_MOTION_STATUS_LIST);
	
		void display_menu(bool);
		void display_ros_topics(int,int,int,int,double);
		void display_start_word();
		void display_ending_word();
		void display_calibration_instruction();
		void display_3D_points(double**,vector<cv::Point2d>,int,int,bool);
		void display_2D_points(CvPoint2D32f*,int,int,bool);
		void display_cam_pose(cv::Mat, cv::Mat, cv::Mat, cv::Mat,  vector<cv::Point2d>, int,int, bool);
		void display_cam_pose_offline(cv::Mat, cv::Mat, cv::Mat, cv::Mat);
		void display_projection_comparison(CvPoint2D32f*,vector<cv::Point2d>,vector<cv::Point2d>,int);
		void display_tool_prediction(double,double,double,double,double,double,int,int);
		void display_system_message(int);

		void no_chessboard_found();
		void no_running_raven();
		void not_calibrated_warning();
		void no_valid_operation_mode();
		void no_valid_camera_info_file(int,int,int);
		void no_correct_camera_set(int);

		SYSTEM_STATUS_LIST check_user_selection_c(int,SYSTEM_STATUS_LIST);
		SYSTEM_STATUS_LIST check_user_selection_p(int,bool);
};

#endif
