#include "myproject/MyTracker_Controller.h"


MyTracker_Controller::MyTracker_Controller(int argc, char** argv):it_(nh_)
{
	// load ROS parameters from launch file
	load_ros_param();


	string s1 = "Chess Board Corners";
	string s2 = "Surgical Tool Segmentation";
	
	window_name_cali = new char[s1.length() + 1];
	strcpy(window_name_cali, s1.c_str());
	
	window_name_segm = new char[s2.length() + 1];
	strcpy(window_name_segm, s2.c_str());

	CAMERA_CALIBRATED = false;

	//init the console object
	CONSOLE.init(disparity_algorithm);

	//init the path planner object
	LEFT_MOTION.set_ArmType(LEFT_ARM);
	RIGHT_MOTION.set_ArmType(RIGHT_ARM);

	//init foward Kinematics object
	RAVEN_KINEMATICS.set_ArmType(LEFT_ARM);

	//init the camera calibrator object
	which_camera = LEFT_CAMERA; //1: LEFT_CAMERA, 0:RIGHT_CAMERA
	My_CALIBRATOR.init(8,6,window_name_cali,which_camera);
	
	//init the image segmentation object
	record_dice_coefficient = false;
	My_SEGMENTATION.init(window_name_segm,segmentation_version,record_dice_coefficient,which_camera,disparity_algorithm);
	
	//init the controller object
	init_sys();
	if(!init_ros(argc,argv))
	{	
		CONSOLE.display_system_message(4);
		exit(1);
	}
}


MyTracker_Controller::~MyTracker_Controller()
{
}

void MyTracker_Controller::init_sys()
{
	this->RAVEN_SUB_COUNT = 0;
	this->RAVEN_PUB_COUNT = 0;
	this->IMAGE_SUB_COUNT = 0;
	this->IMAGE_PUB_COUNT = 0;

	this->IMAGE_STATUS = EMPTY_IMAGE;
	this->SYSTEM_STATUS = JUST_STARTING;
	this->RAVENINFO_STATUS = NOT_IN_USE;
	this->RAVEN_RECORD_SIZE = 100;

	this->GOODBYE = false;
	this->NEW_SEGMENTED_IMAGE = false;

	for(int i=0;i<6;i++)
		this->ZOMBIE_RAVEN_MOTION[i] = NO_MOTION;
}



void MyTracker_Controller::load_ros_param()
{
	/* segmentation_version list:
	   0: only shows the ravenstate tip pose & axes
	   1: the six grid plot (with motion,ravenstate,combined prob)
	   2: shows the ravenstate tip pose & estimated projection of pose
	   3: add model width estimate
	   4: demo mode (without showing intermediate result)
	*/                                                                
	nh_.param("/Surgical_Tool_Tracker/segmentation_version", segmentation_version, 4);
	if(segmentation_version<0 || segmentation_version>4)
	{
		//invalid version input
		segmentation_version = 4;
	}


	/* disparity algorithm options:
	   0: Call the constructor for StereoBM
           1: Call the constructor for StereoSGBM (first attempt parameters)
           2: Call the constructor for StereoSGBM (magic parameters)
	   3: Call the constructor for StereoSGBM (modified parameters)
	   4: Call the constructor for StereoSGBM (Manually adjust the parameters online)
	*/
	nh_.param("/Surgical_Tool_Tracker/disparity_algorithm", disparity_algorithm, 3); // feed into My_SEGMENTATION object
	if(disparity_algorithm<0 || disparity_algorithm>4)
	{
		disparity_algorithm = 3; 

	}

	online_processing = check_if_online(); // online or offline processing
}



bool MyTracker_Controller::init_ros(int argc, char** argv)
{
	while(!ros::ok())
		CONSOLE.display_system_message(0);

	// Setup Raven publish/subscribe relation
	raven_pub_ = nh_.advertise<raven_2::raven_automove>("raven_automove",1);	
	raven_sub_ = nh_.subscribe("ravenstate",1,&MyTracker_Controller::ravenCb,this);

	// Setup Image publish/subscribe relation
	//
	
	if(which_camera == LEFT_CAMERA)
	{
		image_sub_ = it_.subscribe("/stereo/left/image_raw", 1, &MyTracker_Controller::imageCb, this);	  // left camera Todo: use image_rect
		imageN_sub_ = it_.subscribe("/stereo/right/image_raw", 1, &MyTracker_Controller::imageNCb, this); // right camera
	}
	else if(which_camera == RIGHT_CAMERA)
	{
		image_sub_ = it_.subscribe("/stereo/right/image_raw", 1, &MyTracker_Controller::imageCb, this); // right camera
		imageN_sub_ = it_.subscribe("/stereo/left/image_raw", 1, &MyTracker_Controller::imageNCb, this);// left camera	
	}
	else
		CONSOLE.no_correct_camera_set(which_camera);

	if(record_dice_coefficient)
		image_pub_ = it_.advertise("/surgical_tool_tracker/image_segmented_new", 1); 
	else
		image_pub_ = it_.advertise("/surgical_tool_tracker/image_segmented", 1); 

	return true;
}


bool MyTracker_Controller::check_if_online()
{
	nh_.getParam("/Surgical_Tool_Tracker/operating_mode", operating_mode);
	if(operating_mode == 1)  //online
		return true;

	else if(operating_mode == 0)  //offline
		return false;

	else
	{
		CONSOLE.no_valid_operation_mode();
		exit(1);
	}
}

void MyTracker_Controller::start_thread()
{
	pthread_create(&console_thread,NULL,MyTracker_Controller::static_console_process,this);
	pthread_create(&ros_image_thread,NULL,MyTracker_Controller::static_ros_image_process,this);
	pthread_create(&ros_raven_thread,NULL,MyTracker_Controller::static_ros_raven_process,this);

}


void MyTracker_Controller::join_thread()
{
	pthread_join(console_thread,NULL);
	pthread_join(ros_image_thread,NULL);
	pthread_join(ros_raven_thread,NULL);

	signal(SIGINT,sigint_handler);
	CONSOLE.display_system_message(1);
}

void MyTracker_Controller::sigint_handler(int param)
{
  	exit(1);
}


void *MyTracker_Controller::console_process()
{	
	double timeInterval;
	int theKey;
	ros::Time time;
	time = time.now();

	while ((time.now()-time).toSec()<2){}
	time = time.now();

	while (ros::ok() && !GOODBYE)
  	{
		switch(SYSTEM_STATUS)
		{
			case JUST_STARTING:
				CONSOLE.display_start_word();
				SYSTEM_STATUS = SHOW_MENU;
				break;

			case SHOW_MENU:
				CONSOLE.display_menu(online_processing);
				RAVEN_KINEMATICS.load_xyz_offset(CONSOLE.load_tool_tracking_offset());
				My_SEGMENTATION.load_segmentation_param(CONSOLE.load_manual_segmentation_parameter());
				SYSTEM_STATUS = PENDING_USER_SELECTION;
				break;
			
			case PENDING_USER_SELECTION:
				theKey = CONSOLE.get_key();
				SYSTEM_STATUS = CONSOLE.check_user_selection_p(theKey,online_processing);
				break;

			case CAMERA_CALIBRATE_MODE:
				//Todo: stop something (ex: publishing, but start cali)
				theKey = CONSOLE.get_key();
				SYSTEM_STATUS = CONSOLE.check_user_selection_c(theKey,SYSTEM_STATUS);
				break;	

			case CAMERA_CALIBRATE_PNP_READY_MODE:	
				theKey = CONSOLE.get_key();
				SYSTEM_STATUS = CONSOLE.check_user_selection_c(theKey,SYSTEM_STATUS);
				break;	
			case EXIT_ALL:	
				process_image_nothing();
				GOODBYE = true;
				CONSOLE.display_system_message(online_processing?2:3);
				break;	
		}

		if (CONSOLE.check_show_ros())
		{
			timeInterval = (time.now()-time).toSec();
			if(timeInterval > 1)
			{
				//Todo: show more IMAGE_SUB_COUNT1
				CONSOLE.display_ros_topics(RAVEN_SUB_COUNT,RAVEN_PUB_COUNT,IMAGE_SUB_COUNT,IMAGE_PUB_COUNT,timeInterval);
				time = time.now();
			}
		}
		
	}
	
}


void *MyTracker_Controller::ros_image_process()
{
	static ros::Rate loop_rate(IMAGE_LOOP_RATE);
	while (ros::ok() && !GOODBYE)
  	{
		if(CVPTR_SEG)
		{
			//(0) calibrate camera: for extrinsic parameters
			if(SYSTEM_STATUS == CAMERA_CALIBRATE_MODE || SYSTEM_STATUS == CAMERA_CALIBRATE_PNP_READY_MODE)
				process_image_calibrate();

			//(1) find surgical tool
			else if(CONSOLE.check_tool_tracking())
				if(online_processing) // if online processing, need to calibrate first
				{
					if(CAMERA_CALIBRATED)
						if(RAVEN_PUB_COUNT>0)
						{
							bool console_show = IMAGE_PUB_COUNT % IMAGE_LOOP_RATE == 0;
							process_image_segment(console_show);
						}
						else
						{
							CONSOLE.no_running_raven();
							SYSTEM_STATUS = SHOW_MENU;
						}
					else
					{
						CONSOLE.not_calibrated_warning();
						SYSTEM_STATUS = SHOW_MENU;
					}
				}
				else  // if offline processing
				{
					if(CAMERA_CALIBRATED)
					{
						bool console_show = IMAGE_PUB_COUNT % IMAGE_LOOP_RATE == 0;
						process_image_segment(console_show);
					}
					else
					{
						// don't need to calibrate
						// but we want calibration data when rosbag was created.
						process_image_calibrate(); // load previously calibrated result.
					}
				}

			else
				process_image_nothing();
			
			//(2) publish segmented image (which is included in the segmentation function)
			imagePb();

			//(3) release cvbridge object
			CVPTR_RAW.reset();
			CVPTR_NXT.reset();
			CVPTR_SEG.reset();
		}
		
		//(5) wait for next publish
		ros::spinOnce();
		loop_rate.sleep();
		
	}
}



void *MyTracker_Controller::ros_raven_process()
{
	static ros::Rate loop_rate(RAVEN_LOOP_RATE);
	while (ros::ok() && !GOODBYE)
  	{	
		//(1) update raven state (for future computation uses)
		LEFT_MOTION.set_Current_Pos(CURR_RAVEN_STATE.pos);
		LEFT_MOTION.set_Current_Ori(CURR_RAVEN_STATE.ori);

		RIGHT_MOTION.set_Current_Pos(CURR_RAVEN_STATE.pos);
		RIGHT_MOTION.set_Current_Ori(CURR_RAVEN_STATE.ori);			
		
		//(3) prepare user command
		process_raven_command();

		//(4) publish raven command
		ravenPb();

		//(5) wait for next publish
		ros::spinOnce();
		loop_rate.sleep();
	}
}


void * MyTracker_Controller::static_console_process(void* classRef)
{
	return ((MyTracker_Controller *)classRef)->console_process();
}


void * MyTracker_Controller::static_ros_image_process(void* classRef)
{
	return ((MyTracker_Controller *)classRef)->ros_image_process();
}


void * MyTracker_Controller::static_ros_raven_process(void* classRef)
{
	return ((MyTracker_Controller *)classRef)->ros_raven_process();
}


void MyTracker_Controller::ravenCb(const raven_2::raven_state msg)
{
	if(RAVENINFO_STATUS == NOT_IN_USE)
	{
		// (1) save the updated raven_state 
		CURR_RAVEN_STATE.runlevel = msg.runlevel;
		CURR_RAVEN_STATE.sublevel = msg.sublevel;
		CURR_RAVEN_STATE.last_seq = msg.last_seq;
		CURR_RAVEN_STATE.dt = msg.dt;

		for(int i=0; i<2; i++)
		{
			CURR_RAVEN_STATE.type[i] = msg.type[i];
			CURR_RAVEN_STATE.grasp_d[i] = msg.grasp_d[i];
		}

		for(int i=0; i<6; i++)
		{
			CURR_RAVEN_STATE.pos[i] = msg.pos[i];
			CURR_RAVEN_STATE.pos_d[i] = msg.pos_d[i];
		}

		for(int i=0; i<18; i++)
		{
			CURR_RAVEN_STATE.ori[i] = msg.ori[i];
			CURR_RAVEN_STATE.ori_d[i] = msg.ori_d[i];
		}

		for(int i=0; i<16; i++)
		{
			CURR_RAVEN_STATE.jpos[i] = msg.jpos[i];
			CURR_RAVEN_STATE.jpos_d[i] = msg.jpos_d[i];
		}

		// (2) update recieved data count
		RAVEN_SUB_COUNT ++;
	}
}


void MyTracker_Controller::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	if(!CVPTR_SEG && (IMAGE_STATUS == EMPTY_IMAGE || IMAGE_STATUS == DONE_PUBLISHING))
	{
		IMAGE_STATUS = START_LOADING;
		try
		{
			CVPTR_RAW = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			CVPTR_SEG = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			CONSOLE.display_system_message(5);
			ROS_ERROR("%s",e.what());
			return;
		}
		IMAGE_STATUS = DONE_LOADING;
	}
	IMAGE_SUB_COUNT ++;
}


void MyTracker_Controller::imageNCb(const sensor_msgs::ImageConstPtr& msg) // reads image frames from the other camera
{
	if(!CVPTR_NXT && (IMAGE_STATUS == EMPTY_IMAGE || IMAGE_STATUS == DONE_PUBLISHING))
	{
		try
		{
			CVPTR_NXT = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	

		}
		catch (cv_bridge::Exception& e)
		{
			CONSOLE.display_system_message(5);
			ROS_ERROR("%s",e.what());
			return;
		}
	}
}


void MyTracker_Controller::ravenPb()
{
	static raven_2::raven_automove msg_raven_automove;	

	// (1) wrap up the new command	
	msg_raven_automove.hdr.stamp = msg_raven_automove.hdr.stamp.now(); //hdr

	tf::transformTFToMsg(TF_INCR[LEFT_ARM], msg_raven_automove.tf_incr[LEFT_ARM]);   //tf_incr
	tf::transformTFToMsg(TF_INCR[RIGHT_ARM], msg_raven_automove.tf_incr[RIGHT_ARM]);

	// (2) send new command
	raven_pub_.publish(msg_raven_automove);

	// (3) update published data count
	RAVEN_PUB_COUNT ++;
}


void MyTracker_Controller::imagePb() 
{
	// (1) check if ready to publish	
	if(IMAGE_STATUS == DONE_PROCESSING)
	{
		IMAGE_STATUS = START_PUBLISHING;

		// (2) send new command
		if(NEW_SEGMENTED_IMAGE)
		{
			try
			{	
				image_pub_.publish(CVPTR_SEG->toImageMsg());
				NEW_SEGMENTED_IMAGE = false;
			}
			catch (cv_bridge::Exception& e)
			{
				CONSOLE.display_system_message(5);
				ROS_ERROR("%s",e.what());
				return;
			}
		}
		IMAGE_STATUS = DONE_PUBLISHING;
	}

	// (3) update published data count
	IMAGE_PUB_COUNT ++;
}


void MyTracker_Controller::process_image_calibrate()
{
	//load new image at every time instance
	My_CALIBRATOR.load_image(CVPTR_SEG);

	if(online_processing) //online process
	{
		if(IMAGE_STATUS == DONE_LOADING)
		{
			IMAGE_STATUS = START_PROCESSING;
			bool wasChessboardFound;
		
			// find the corners
			wasChessboardFound = My_CALIBRATOR.find_chessboard_corners();

			//show found corner on image
			My_CALIBRATOR.draw_corners();

			//prepare for pnp		
			if(SYSTEM_STATUS == CAMERA_CALIBRATE_PNP_READY_MODE)
			{
				if (wasChessboardFound) // true only when the entire chessboard is found
				{
					//solve pnp
					My_CALIBRATOR.list_corners();
					My_CALIBRATOR.solve_pnp();
					My_CALIBRATOR.list_pnp_result();

					//save resulting camera pose
					CAMERA_CALIBRATED = true;
					CAMERA_ROTATION = My_CALIBRATOR.get_camera_rotat();
					CAMERA_TRANSLATION = My_CALIBRATOR.get_camera_trans();
				}
				else
				{
					CONSOLE.no_chessboard_found();
				}

				//update system state
				SYSTEM_STATUS = CAMERA_CALIBRATE_MODE;
			}

			My_SEGMENTATION.load_camera_pose(CAMERA_ROTATION,CAMERA_TRANSLATION);
			RAVEN_KINEMATICS.load_camera_pose(CAMERA_ROTATION,CAMERA_TRANSLATION);
			IMAGE_STATUS = DONE_PROCESSING;
		}
	}
	else //offline process
	{
		My_CALIBRATOR.set_camera_pose();

		CAMERA_CALIBRATED = true;
		CAMERA_ROTATION = My_CALIBRATOR.get_camera_rotat();
		CAMERA_TRANSLATION = My_CALIBRATOR.get_camera_trans();

		My_SEGMENTATION.load_camera_pose(CAMERA_ROTATION,CAMERA_TRANSLATION);
		RAVEN_KINEMATICS.load_camera_pose(CAMERA_ROTATION,CAMERA_TRANSLATION);
		IMAGE_STATUS = DONE_PROCESSING;
	}
	
}



void MyTracker_Controller::process_image_segment(bool console_show)
{
	vector<Point2d> PIXEL_POS;
	vector<double>  RAVEN_DIS;

	if(CVPTR_SEG && IMAGE_STATUS == DONE_LOADING)
	{
		My_SEGMENTATION.load_image(CVPTR_SEG);
		if(CVPTR_NXT)
		{
			if(disparity_algorithm == 4)
				My_SEGMENTATION.load_disparity_param(CONSOLE.load_manual_disparity_parameter());

			My_SEGMENTATION.load_imageN(CVPTR_NXT); // This is the image frame from neighboring camera
			My_SEGMENTATION.prep_disparity_map();

			Mat disparity = My_SEGMENTATION.get_disparity();			
			Mat reprojection = My_SEGMENTATION.get_reprojection();
			Mat disp2depth = My_SEGMENTATION.get_disp2depth();
			CONSOLE.save_depthmap_to_file(disparity,reprojection,disp2depth);
		}
		IMAGE_STATUS = START_PROCESSING;
		
		//(1) project raven state 3D to 2D
		PIXEL_POS = process_raven_state(console_show);

		switch(segmentation_version)
		{
			case 0: 
			case 1: 
				RAVEN_DIS.clear();
				break;
			case 2:
			case 3: 
			case 4: 
				RAVEN_DIS = RAVEN_KINEMATICS.compute_distance_to_cam();
				break;
		}
		
		My_SEGMENTATION.load_kinematics(RAVEN_POS,PIXEL_POS,RAVEN_DIS);

		//(2) segment image
		My_SEGMENTATION.prep_probability(); 
		if(segmentation_version >=3)
			My_SEGMENTATION.start_segmentation();

		//(3) draw
		My_SEGMENTATION.show_image();
		My_SEGMENTATION.draw_segmentation_colorwheel(CONSOLE.check_show_colorwheel());
		CONSOLE.reset_show_colorwheel();

		//(4) return the image back for publishing
		CVPTR_SEG->image = My_SEGMENTATION.publish_image();
		NEW_SEGMENTED_IMAGE = true;

		IMAGE_STATUS = DONE_PROCESSING;
	}
}


void MyTracker_Controller::process_image_nothing()
{
	if(cvGetWindowHandle(window_name_segm)!=0)
		cvDestroyWindow(window_name_segm);

	if(cvGetWindowHandle(window_name_cali)!=0)
		cvDestroyWindow(window_name_cali);

	IMAGE_STATUS = DONE_PROCESSING;
}

void MyTracker_Controller::process_raven_command()
{
	int count = 0;

	//(1) load which direction to move (and if to move at all)
	for(int i=0;i<6;i++)	
		RAVEN_MOTION[i] = CONSOLE.load_raven_motion_flag(i);
	
	//(2) decide desired location
	LEFT_MOTION.set_Desired_Pos(RAVEN_MOTION);
	//RIGHT_MOTION.set_Desired_Pos(RAVEN_MOTION);
	RIGHT_MOTION.set_Desired_Pos(ZOMBIE_RAVEN_MOTION); // this is to make sure we do not try to send motion to this arm

	LEFT_MOTION.set_Desired_Ori();
	RIGHT_MOTION.set_Desired_Ori();

	for(int i=0;i<6;i++)
	{
		if(RAVEN_MOTION[i] == START_MOTION)
		{
			CONSOLE.modify_raven_motion_flag(i,IN_MOTION);
		}
	}

	//(3) check if goal is reached (or at least close enough)
	bool* check1 = LEFT_MOTION.check_goal_reached();
	bool* check2 = RIGHT_MOTION.check_goal_reached();

	//(4) constrct incremental command
	if(SYSTEM_STATUS == CAMERA_CALIBRATE_MODE || SYSTEM_STATUS == CAMERA_CALIBRATE_PNP_READY_MODE  || SYSTEM_STATUS == EXIT_ALL) // all without motion command
	{
		TF_INCR[LEFT_ARM] = LEFT_MOTION.ComputeNullMotion(); 
		TF_INCR[RIGHT_ARM] = RIGHT_MOTION.ComputeNullMotion(); 
	}
	else
	{	//move a little
		// TF_INCR[LEFT_ARM] and TF_INCR[RIGHT_ARM] will be published
		// based on keyboard and LEFT_MOTION, RIGHT_MOTION, RAVEN_CURRENT_STATE
		TF_INCR[LEFT_ARM] = LEFT_MOTION.ComputeIncrMotion(); 
		TF_INCR[RIGHT_ARM] = RIGHT_MOTION.ComputeNullMotion(); 
	}
	
}


vector<Point2d> MyTracker_Controller::process_raven_state(bool console_show)
{
	//(0) load raven state
	RAVENINFO_STATUS = IN_USE;
	RAVEN_KINEMATICS.set_current_ravenstate(CURR_RAVEN_STATE);

	RAVENINFO_STATUS = NOT_IN_USE; // unlock CURR_RAVEN_STATE for new input

	//(1) compute raven model and axes points
	vector<Point3d> model_points = RAVEN_KINEMATICS.compute_model_points(); // base frame
	vector<Point3d> axes_points = RAVEN_KINEMATICS.compute_axes_points();

	//(2) save end effector tip point
	RAVEN_POS.push_back(axes_points[0]);  // prepare for 3D to 2D projection
	if(RAVEN_POS.size()-RAVEN_RECORD_SIZE > 0)  // keep RAVEN_POS vector at a bounded size
	{
		vector<Point3d> tmp1(RAVEN_POS.end()-RAVEN_RECORD_SIZE, RAVEN_POS.end());
		RAVEN_POS.clear();
		RAVEN_POS = tmp1;
	}

	//(3) project raven state 3D to 2D
	vector<Point2d> pixel_coords;
	vector<Point3d> world_coords = axes_points;
	world_coords.insert( world_coords.end(), model_points.begin(), model_points.end() );
	pixel_coords = My_CALIBRATOR.project(world_coords);

	model_points.clear();
	axes_points.clear();
	
	//(4) console display
	if(console_show && CONSOLE.check_show_ros())
	{
		int xP = pixel_coords[0].x;
		int yP = pixel_coords[0].y;
		int xO = CURR_RAVEN_STATE.pos[0]/1000;
		int yO = CURR_RAVEN_STATE.pos[1]/1000;
		int zO = CURR_RAVEN_STATE.pos[2]/1000;
		int xB = axes_points[0].x;
		int yB = axes_points[0].y;
		int zB = axes_points[0].z;

		CONSOLE.display_tool_prediction(xO,yO,zO,xB,yB,zB,xP,yP);
	}

	return pixel_coords;
}

