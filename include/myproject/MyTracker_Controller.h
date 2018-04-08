#ifndef MYTRACKER_CONTROLLER_H
#define MYTRACKER_CONTROLLER_H

#include "MyTracker_DoSegmentation.h"
class MyTracker_Controller
{
	private:
		int RAVEN_SUB_COUNT;
		int RAVEN_PUB_COUNT;
		int IMAGE_SUB_COUNT;
		int IMAGE_PUB_COUNT;

		int operating_mode;
		int which_camera;
		int segmentation_version;
		int disparity_algorithm;

		IMAGE_STATUS_LIST  IMAGE_STATUS;
		SYSTEM_STATUS_LIST SYSTEM_STATUS;
		RAVEN_STATUS_LIST  RAVENINFO_STATUS;
		RAVEN_MOTION_STATUS_LIST RAVEN_MOTION[6];
		RAVEN_MOTION_STATUS_LIST ZOMBIE_RAVEN_MOTION[6];

		pthread_t console_thread;
		pthread_t ros_image_thread;
		pthread_t ros_raven_thread;
	
		ros::NodeHandle nh_; 
		ros::Publisher   raven_pub_;
		ros::Subscriber  raven_sub_;
		
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Subscriber imageN_sub_;
		image_transport::Publisher image_pub_;

		int RAVEN_RECORD_SIZE;
		vector<Point3d> RAVEN_POS;
		raven_2::raven_state CURR_RAVEN_STATE;
		tf::Transform TF_INCR[2];
		
		cv_bridge::CvImagePtr CVPTR_RAW;
		cv_bridge::CvImagePtr CVPTR_NXT;
		cv_bridge::CvImagePtr CVPTR_SEG;

		MyTracker_Display CONSOLE;
		MyTracker_MotionPlanner LEFT_MOTION;
		MyTracker_MotionPlanner RIGHT_MOTION;
		MyTracker_FwdKinematics RAVEN_KINEMATICS;
		
		bool CAMERA_CALIBRATED;
		bool NEW_SEGMENTED_IMAGE;
		bool GOODBYE;
		bool online_processing;
		bool record_dice_coefficient;

		cv::Mat CAMERA_ROTATION;
		cv::Mat CAMERA_TRANSLATION;
		char* window_name_cali;
		char* window_name_segm;
		MyTracker_ImageCalibrator My_CALIBRATOR;
		MyTracker_DoSegmentation  My_SEGMENTATION;

	public:
		MyTracker_Controller(int argc, char** argv);
		~MyTracker_Controller();

		void init_sys();
		void load_ros_param();
		bool init_ros(int, char**);
		bool check_if_online();

		void start_thread();
		void join_thread();
		static void sigint_handler(int);
		void *console_process(void);
		void *ros_image_process(void);
		void *ros_raven_process(void);
		static void *static_console_process(void*);
		static void *static_ros_image_process(void*);
		static void *static_ros_raven_process(void*);

		void ravenPb();
		void ravenCb(const raven_2::raven_state);
		void imagePb();
		void imageCb(const sensor_msgs::ImageConstPtr&);
		void imageNCb(const sensor_msgs::ImageConstPtr&);

		vector<Point2d> process_raven_state(bool);
		void process_image_segment(bool);
		void process_image_calibrate();
		void process_image_nothing();
		void process_raven_command();
		

};

#endif
