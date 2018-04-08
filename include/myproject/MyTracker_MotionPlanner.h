#ifndef MYTRACKER_MOTIONPLANNER_H
#define MYTRACKER_MOTIONPLANNER_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <fstream>
#include <iostream>
#include <time.h> 
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <iomanip>
#include <ros/transport_hints.h>
#include <pthread.h>
#include <termios.h>
#include <signal.h>
#include <queue>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/cv.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

#include "raven_2/raven_automove.h"
#include "raven_2/raven_state.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#define maxin 5                 // maximum raven incremental command magnitude
#define LEFT_ARM 0
#define RIGHT_ARM 1
#define LEFT_CAMERA 0
#define RIGHT_CAMERA 1
#define RAVEN_LOOP_RATE 1000     // in Hz
#define IMAGE_LOOP_RATE 30       // in Hz

#define DEL_POS_THRESHOLD 180	// in micro meter (= 0.18mm = 0.018cm)
#define DEL_POS_STEP      30    // in mocro meter (= 0.03mm = 0.003cm)  per raven_automove publish at 1000Hz
#define DEL_POS_ENTIRE    15000 // in micro meter (= 15mm = 1.5cm)        per keyboard command
#define CLOSE_ENOUGH      2000  // in mocro meter (= 2mm = 0.2cm)
#define AXIS_LENGTH       20    // in mm

#define TOOL_WIDTH        5       // in mm (half of the width)
#define DEPTBALL_WIDTH    50      // in mm 
#define GRASPER_WIDTH     4       // in mm
#define GRASPER_TIP_WIDTH 2       // in mm

#define OBJECT_SIZE_SCALE 30000 // for mode 1: computing the probability map from ravenstate prior
#define UNCERTAINTY_SCALE 50000
#define OBJECT_SIZE_FACTOR 350  // for mode 2: compute the raven configuration model (thickness)

enum IMAGE_STATUS_LIST{
	EMPTY_IMAGE,       // 0
	START_LOADING,     // 1
	DONE_LOADING,      // 2
	START_PROCESSING,  // 3
	DONE_PROCESSING,   // 4
	START_PUBLISHING,  // 5
	DONE_PUBLISHING    // 6
};

enum SYSTEM_STATUS_LIST{
	JUST_STARTING,                   // 0
	SHOW_MENU,                       // 1
	PENDING_USER_SELECTION,          // 2
	CAMERA_CALIBRATE_MODE,           // 3
	CAMERA_CALIBRATE_PNP_READY_MODE, // 4
	EXIT_ALL,                        // 5
};

enum RAVEN_STATUS_LIST{
	IN_USE,		// 0
	NOT_IN_USE,     // 1
};

enum RAVEN_MOTION_STATUS_LIST{
	NO_MOTION,     // 0
	START_MOTION,  // 1
	IN_MOTION,     // 2
};

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

class MyTracker_MotionPlanner
{
	private:	
		int ArmType;
		tf::Vector3 Delta_Pos;
		tf::Quaternion Delta_Ori;

		tf::Vector3 Current_Pos;    // current raven position
		tf::Vector3 Desired_Pos;    // desired raven position
		tf::Quaternion Current_Ori; // current raven rotation
		tf::Quaternion Desired_Ori; // desired raven rotation

	public:
		MyTracker_MotionPlanner();
		~MyTracker_MotionPlanner();
		tf::Transform ComputeNullMotion();		
		tf::Transform ComputeIncrMotion();

		bool set_ArmType(int);
		bool set_Current_Pos(boost::array<int, 6>);
		bool set_Current_Ori(boost::array<float, 18>);

		bool set_Desired_Pos( RAVEN_MOTION_STATUS_LIST*);
		bool set_Desired_Ori();

		bool* check_goal_reached();
};


#endif
