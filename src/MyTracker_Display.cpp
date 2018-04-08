#include "myproject/MyTracker_Display.h"

MyTracker_Display::MyTracker_Display()
{
	init();
}


int MyTracker_Display::get_key() 
{
    	int character;
    	struct termios orig_term_attr;
    	struct termios new_term_attr;

    	// set the terminal to raw mode 
    	tcgetattr(fileno(stdin), &orig_term_attr);
    	memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    	new_term_attr.c_lflag &= ~(ECHO|ICANON);
    	new_term_attr.c_cc[VTIME] = 0;
    	new_term_attr.c_cc[VMIN] = 0;
    	tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    	// read a character from the stdin stream without blocking 
    	//   returns EOF (-1) if no character is available 
    	character = fgetc(stdin);

   	// restore the original terminal attributes 
    	tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    	return character;
}


void MyTracker_Display::init()
{
	this->TIME_INTERVAL = 1;
	
	this->RAVEN_SUB_COUNT_LAST = 0;
	this->RAVEN_PUB_COUNT_LAST = 0;
	this->IMAGE_SUB_COUNT_LAST = 0;
	this->IMAGE_PUB_COUNT_LAST = 0;

	this->RAVEN_SUB_COUNT = 0;
	this->RAVEN_PUB_COUNT = 0;
	this->IMAGE_SUB_COUNT = 0;
	this->IMAGE_PUB_COUNT = 0;

	this->RAVEN_SUB_RATE = 0;
	this->RAVEN_PUB_RATE = 0;
	this->IMAGE_SUB_RATE = 0;
	this->IMAGE_PUB_RATE = 0;
	
	this->OFFSET_XB = 0;
	this->OFFSET_YB = 0;
	this->OFFSET_ZB = 0;
	this->OFFSET_AA = 0;

	this->DEFAULT_SEG_BLURY_SCALE = 25;
	this->DEFAULT_SEG_BLURY_INTENSITY = 15;
	this->DEFAULT_SEG_THRES_LOWBOUND = 100;
	this->DEFAULT_SEG_FILL_BLACKSPOT = 5;

	this->DISP_MINDISP = 0;		// minDisparity
	this->DISP_NUMDISP = 64;	// numDisparities
	this->DISP_WINSIZE = 5;		// blockSize, window size
	this->DISP_P1 = 600;		// p1
	this->DISP_P2 = 2400;		// p2
	this->DISP_MAXDIFF = 10;	// disp12MaxDiff
	this->DISP_FLTRCAP = 4;		// preFilterCap
	this->DISP_UNQRATE = 1;		// uniquenessRatio
	this->DISP_SPKSIZE = 900;	// speckleWindowSize
	this->DISP_SPKRNGE = 2;		// speckleRange

	this->SEG_BLURY_SCALE = DEFAULT_SEG_BLURY_SCALE;
	this->SEG_BLURY_INTENSITY = DEFAULT_SEG_BLURY_INTENSITY;
	this->SEG_THRES_LOWBOUND = DEFAULT_SEG_THRES_LOWBOUND;
	this->SEG_FILL_BLACKSPOT = DEFAULT_SEG_FILL_BLACKSPOT;

	this->SHOW_COLORWHEEL = false;
	this->SHOW_ROS_TOPIC = false;
	this->SHOW_TOOL_TRACKING = false;
	this->INCLUDE_TOOL_TRACKING_OFFSET = false;
	this->INCLUDE_MANUAL_SEGMENTATION_PARAMETERS = false;
	this->DEPTHMAP_FILE_SAVE_PENDING = false;

	this->DISPARITY_ALGORITHM = 3;

	for(int i=0;i<6;i++)
		RAVEN_MOTION[i] = NO_MOTION;

	string str = "/home/biorobotics/catkin_ws/src/myproject/camera_info/camera_extrinsic/XYZ_offset.txt";
	OFFSET_FILE_NAME = new char[str.length() + 1];
	strcpy(OFFSET_FILE_NAME, str.c_str());

	string str1 = "/home/biorobotics/catkin_ws/src/myproject/camera_info/camera_extrinsic/segm_param.txt";
	SEGMENT_FILE_NAME = new char[str1.length() + 1];
	strcpy(SEGMENT_FILE_NAME, str1.c_str());

	string str2 = "/home/biorobotics/catkin_ws/src/myproject/camera_info/camera_extrinsic/disp_param.txt";
	DISPARITY_FILE_NAME = new char[str2.length() + 1];
	strcpy(DISPARITY_FILE_NAME, str2.c_str());
}


void MyTracker_Display::init(int disparity_algorithm)
{
	init();
	this->DISPARITY_ALGORITHM = disparity_algorithm;

}


bool MyTracker_Display::check_show_ros()
{
	return this->SHOW_ROS_TOPIC;
}

bool MyTracker_Display::check_tool_tracking()
{
	return this->SHOW_TOOL_TRACKING;
}


bool MyTracker_Display::check_show_colorwheel()
{
	return this->SHOW_COLORWHEEL;
}



void MyTracker_Display::reset_show_colorwheel()
{
	this->SHOW_COLORWHEEL = false;
}


void MyTracker_Display::toggle_show_ros()
{
	SHOW_ROS_TOPIC = !SHOW_ROS_TOPIC;

	if(SHOW_ROS_TOPIC)
		cout<<endl<<"'r' Key Pressed: Enable Ros topics display."<<endl;		
		
	else
	{
		cout<<endl<<"'r' Key Pressed: Disable Ros topics display."<<endl;	
	}
}


void MyTracker_Display::toggle_tool_tracking(bool forcequit)
{
	SHOW_TOOL_TRACKING = !SHOW_TOOL_TRACKING;

	if(SHOW_TOOL_TRACKING && forcequit)
	{
		SHOW_TOOL_TRACKING = false;
		reset_tool_tracking_offset();
	}
	else if(SHOW_TOOL_TRACKING && !forcequit)
	{
		cout<<endl<<"'t' Key Pressed: Enable surgical tool tracking."<<endl;
		
	}		
	else
	{
		cout<<endl<<"'t' Key Pressed: Disable surgical tool tracking."<<endl;	
		reset_tool_tracking_offset();
	}
}




void MyTracker_Display::toggle_tool_tracking_offset()
{
	
	if(SHOW_TOOL_TRACKING)
	{
		INCLUDE_TOOL_TRACKING_OFFSET = !INCLUDE_TOOL_TRACKING_OFFSET;
		if(INCLUDE_TOOL_TRACKING_OFFSET)
		{
			cout<<endl<<"'o' Key Pressed: Enable surgical tool tracking xyz offset."<<endl;	
			set_tool_tracking_offset();
		}
		else
		{
			cout<<endl<<"'o' Key Pressed: Disable surgical tool tracking xyz offset."<<endl;
			reset_tool_tracking_offset();
			
		}
		cout<<"\t Set offset value to: (offset_XB,offset_YB,offset_ZB,offset_AA) = ("<<OFFSET_XB<<","<<OFFSET_YB<<","<<OFFSET_ZB<<","<<OFFSET_AA<<")"<<endl;	
	}
}



void MyTracker_Display::toggle_manual_segmentation_parameter()
{	
	if(SHOW_TOOL_TRACKING)
	{
		INCLUDE_MANUAL_SEGMENTATION_PARAMETERS = !INCLUDE_MANUAL_SEGMENTATION_PARAMETERS;
		if(INCLUDE_MANUAL_SEGMENTATION_PARAMETERS)
		{
			cout<<endl<<"'m' Key Pressed: Enable manual segmentation parameter settings."<<endl;	
			set_manual_segmentation_parameter();
		}
		else
		{
			cout<<endl<<"'m' Key Pressed: Disable manual segmentation parameter settings."<<endl;
			reset_manual_segmentation_parameter();
			
		}
		cout<<"\t Set segmentation parameters to: \n(BLURY_SCALE,BLURY_INTENSITY,THRES_LOWBOUND,FILL_BLACKSPOT) =(";
		cout<< SEG_BLURY_SCALE<<","<<SEG_BLURY_INTENSITY<<","<<SEG_THRES_LOWBOUND<<","<<SEG_FILL_BLACKSPOT<<")"<<endl;	
		
		SHOW_COLORWHEEL = true;
	}
}



void MyTracker_Display::toggle_manual_disparity_parameter()
{	
	if(SHOW_TOOL_TRACKING)
	{
		cout<<endl<<"'n' Key Pressed: Enable manual disparity parameter settings."<<endl;	
		set_manual_disparity_parameter();

		cout<<"\t Set disparity parameters to: \n(MINDISP,NUMDISP,WINSIZE,P1,P2,MAXDIFF,FLTRCAP,UNQRATE,SPKSIZE,SPKRNGE) =(";
		cout<< DISP_MINDISP<<","<<DISP_NUMDISP<<","<<DISP_WINSIZE<<","<<DISP_P1<<","<<DISP_P2<<","<<DISP_MAXDIFF<<","<<DISP_FLTRCAP;
		cout<<","<<DISP_UNQRATE<<","<<DISP_SPKSIZE<<","<<DISP_SPKRNGE<<")"<<endl;	
	}
}


void MyTracker_Display::save_depthmap_to_file(cv::Mat disparity, cv::Mat reprojection, cv::Mat disp2depth)
{
	
	const static time_t timer_start = time(NULL);
	time_t timer_curr = time(NULL);

	int seconds = (int)difftime(timer_curr, timer_start);
	stringstream ss;
	ss << seconds;
	string str = ss.str();

	if(DEPTHMAP_FILE_SAVE_PENDING)
	{
		cout<<"Writing depth and 3D reprojection result to file"<<endl<<"..."<<endl; 

		cv::FileStorage file("/home/biorobotics/catkin_ws/src/myproject/saved_depth_file/"+str+".yml", cv::FileStorage::WRITE);

		file << "disp2depth" << disp2depth;
		file << "reprojection" << reprojection;
		file << "disparity" << disparity;

		cout<<"Finished writing file."<<endl;
	}
	DEPTHMAP_FILE_SAVE_PENDING = false;
}



vector<int> MyTracker_Display::load_tool_tracking_offset()
{
	vector<int> offset(4);
	offset[0] = OFFSET_XB;
	offset[1] = OFFSET_YB;
	offset[2] = OFFSET_ZB;
	offset[3] = OFFSET_AA; 

	return offset;
}


vector<int> MyTracker_Display::load_manual_segmentation_parameter()
{
	vector<int> segm_param(4);
	segm_param[0] = SEG_BLURY_SCALE;
	segm_param[1] = SEG_BLURY_INTENSITY;
	segm_param[2] = SEG_THRES_LOWBOUND;
	segm_param[3] = SEG_FILL_BLACKSPOT;

	return segm_param;
}


vector<int> MyTracker_Display::load_manual_disparity_parameter()
{
	vector<int> disp_param(10);
	disp_param[0] = DISP_MINDISP;
	disp_param[1] = DISP_NUMDISP;
	disp_param[2] = DISP_WINSIZE;
	disp_param[3] = DISP_P1;
	disp_param[4] = DISP_P2;
	disp_param[5] = DISP_MAXDIFF;
	disp_param[6] = DISP_FLTRCAP;
	disp_param[7] = DISP_UNQRATE;
	disp_param[8] = DISP_SPKSIZE;
	disp_param[9] = DISP_SPKRNGE;

	return disp_param;
}


void MyTracker_Display::set_tool_tracking_offset()
{
	string buf,data;
	stringstream buffer;
	vector<string> tokens;

	//load the text file and put it into a single string:
	ifstream file(OFFSET_FILE_NAME);	
	buffer << file.rdbuf();	
	data = buffer.str();
	stringstream ss(data);

	if(file.is_open())
	{	
		while (ss >> buf)
			tokens.push_back(buf);
	}
	else 
		cout<<"File not found !"<<endl;

	OFFSET_XB = atof(tokens[0].c_str()); // in mm (in base frame)
	OFFSET_YB = atof(tokens[1].c_str()); 
	OFFSET_ZB = atof(tokens[2].c_str()); 
	OFFSET_AA = atof(tokens[3].c_str()); // the angle adjustment for model point A
}



void MyTracker_Display::reset_tool_tracking_offset()
{
	OFFSET_XB = 0;
	OFFSET_YB = 0;
	OFFSET_ZB = 0;
	OFFSET_AA = 0;
}


void MyTracker_Display::set_manual_segmentation_parameter()
{
	string buf,data;
	stringstream buffer;
	vector<string> tokens;

	//load the text file and put it into a single string:
	ifstream file(SEGMENT_FILE_NAME);	
	buffer << file.rdbuf();	
	data = buffer.str();
	stringstream ss(data);

	if(file.is_open())
	{	
		while (ss >> buf)
			tokens.push_back(buf);
	}
	else 
		cout<<"File not found !"<<endl;

	SEG_BLURY_SCALE = atof(tokens[0].c_str()); 
	SEG_BLURY_INTENSITY = atof(tokens[1].c_str()); 
	SEG_THRES_LOWBOUND = atof(tokens[2].c_str()); 
	SEG_FILL_BLACKSPOT = atof(tokens[3].c_str()); 
}


void MyTracker_Display::set_manual_disparity_parameter()
{
	string buf,data;
	stringstream buffer;
	vector<string> tokens;

	//load the text file and put it into a single string:
	ifstream file(DISPARITY_FILE_NAME);	
	buffer << file.rdbuf();	
	data = buffer.str();
	stringstream ss(data);

	if(file.is_open())
	{	
		while (ss >> buf)
			tokens.push_back(buf);
	}
	else 
		cout<<"File not found !"<<endl;

	DISP_MINDISP = atof(tokens[0].c_str()); 
	DISP_NUMDISP = atof(tokens[1].c_str()); 
	DISP_WINSIZE = atof(tokens[2].c_str()); 
	DISP_P1 = atof(tokens[3].c_str()); 
	DISP_P2 = atof(tokens[4].c_str()); 
	DISP_MAXDIFF = atof(tokens[5].c_str()); 
	DISP_FLTRCAP = atof(tokens[6].c_str()); 
	DISP_UNQRATE = atof(tokens[7].c_str()); 	
	DISP_SPKSIZE = atof(tokens[8].c_str()); 
	DISP_SPKRNGE = atof(tokens[9].c_str()); 
}



void MyTracker_Display::reset_manual_segmentation_parameter()
{
	this->SEG_BLURY_SCALE = DEFAULT_SEG_BLURY_SCALE;
	this->SEG_BLURY_INTENSITY = DEFAULT_SEG_BLURY_INTENSITY;
	this->SEG_THRES_LOWBOUND = DEFAULT_SEG_THRES_LOWBOUND;
	this->SEG_FILL_BLACKSPOT = DEFAULT_SEG_FILL_BLACKSPOT;
}



RAVEN_MOTION_STATUS_LIST MyTracker_Display::load_raven_motion_flag(int index)
{
	return RAVEN_MOTION[index];
}



void MyTracker_Display::modify_raven_motion_flag(int index,RAVEN_MOTION_STATUS_LIST action)
{
	if(index >= 6 || index < 0)
		cout<<endl<<"[WARNING]: cannot recognize Raven motion flag index "<<index<<"."<<endl;
	else
		RAVEN_MOTION[index] = action;
}



void MyTracker_Display::display_menu(bool online)
{
	string s1 = "Enabled";
	string s2 = "Disabled";

	string sr = SHOW_ROS_TOPIC ? s1 : s2;
	string st = SHOW_TOOL_TRACKING ? s1 : s2;
	string so = INCLUDE_TOOL_TRACKING_OFFSET ? s1 : s2;
	string sm = INCLUDE_MANUAL_SEGMENTATION_PARAMETERS ? s1 : s2;

	if(online)   // online processing
	{
		cout<<endl<<endl;
		cout<<"-------------------Main Menu------------------"<<endl;
		cout<<"(C): Camera calibration."<<endl;	
		cout<<"(T): Toggle tool tracking. ("<<st<<")"<<endl;
		if(SHOW_TOOL_TRACKING)
		{
			cout<<"(O): Toggle tool tracking xyz offset. ("<<so<<")"<<endl;
			cout<<"(M): Manual parameters for segmentation. ("<<sm<<")"<<endl;
			if(DISPARITY_ALGORITHM == 4)
				cout<<"(N): Manual parameters for disparity map."<<endl;
			cout<<"(I): Save depth result to file."<<endl;
		}
		cout<<"(R): Toggle ROS topic and console display. ("<<sr<<")"<<endl;
		cout<<"(W): Raven motion control - move up"<<endl;
		cout<<"(S): Raven motion control - move down"<<endl;
		cout<<"(A): Raven motion control - move left"<<endl;
		cout<<"(D): Raven motion control - move right."<<endl;
		cout<<"(F): Raven motion control - move forward"<<endl;
		cout<<"(B): Raven motion control - move backward."<<endl;
		cout<<"(Q): System exit."<<endl<<endl<<endl;
	}
	else   // offline processing
	{
		cout<<endl<<endl;
		cout<<"-------------------Main Menu------------------"<<endl;
		cout<<"(T): Toggle tool tracking. ("<<st<<")"<<endl;
		if(SHOW_TOOL_TRACKING)
		{
			cout<<"(O): Toggle tool tracking xyz offset. ("<<so<<")"<<endl;
			cout<<"(M): Manual parameters for segmentation. ("<<sm<<")"<<endl;
			if(DISPARITY_ALGORITHM == 4)
				cout<<"(N): Manual parameters for disparity map."<<endl;
			cout<<"(I): Save depth result to file."<<endl;
		}
		cout<<"(R): Toggle ROS topic and console display. ("<<sr<<")"<<endl;
		cout<<"(Q): System exit."<<endl<<endl<<endl;
	}
		

}

void MyTracker_Display::display_ros_topics(int rs,int rp,int is,int ip,double ti)
{
	TIME_INTERVAL = ti;
	
	RAVEN_SUB_COUNT_LAST = RAVEN_SUB_COUNT;
	RAVEN_PUB_COUNT_LAST = RAVEN_PUB_COUNT;
	IMAGE_SUB_COUNT_LAST = IMAGE_SUB_COUNT;
	IMAGE_PUB_COUNT_LAST = IMAGE_PUB_COUNT;

	RAVEN_SUB_COUNT = rs;
	RAVEN_PUB_COUNT = rp;
	IMAGE_SUB_COUNT = is;
	IMAGE_PUB_COUNT = ip;

	RAVEN_SUB_RATE = (RAVEN_SUB_COUNT-RAVEN_SUB_COUNT_LAST)/TIME_INTERVAL;
	RAVEN_PUB_RATE = (RAVEN_PUB_COUNT-RAVEN_PUB_COUNT_LAST)/TIME_INTERVAL;
	IMAGE_SUB_RATE = (IMAGE_SUB_COUNT-IMAGE_SUB_COUNT_LAST)/TIME_INTERVAL;
	IMAGE_PUB_RATE = (IMAGE_PUB_COUNT-IMAGE_PUB_COUNT_LAST)/TIME_INTERVAL;

	cout<<"raven subscribe number: "<<RAVEN_SUB_COUNT<<" ("<<RAVEN_SUB_RATE<<"Hz)"<<endl;
	cout<<"raven publish number:   "<<RAVEN_PUB_COUNT<<" ("<<RAVEN_PUB_RATE<<"Hz)"<<endl;
	cout<<"image subscribe number: "<<IMAGE_SUB_COUNT<<" ("<<IMAGE_SUB_RATE<<"Hz)"<<endl;
	cout<<"image publish number:   "<<IMAGE_PUB_COUNT<<" ("<<IMAGE_PUB_RATE<<"Hz)"<<endl;
	cout<<endl;
}

void MyTracker_Display::display_start_word()
{
	cout<<"\n\n\nstart welcome!!"<<endl;
}


void MyTracker_Display::display_ending_word()
{
	cout<<"end goodbye."<<endl;
}

void MyTracker_Display::display_calibration_instruction()
{
	cout<<endl<<endl;
	cout<<"-------------------Camera Calibration------------------"<<endl<<endl;
	cout<<"-------instructions-------"<<endl;	
	cout<<"This is a subroutine to help locate the camera pose with respect to "<<endl;
	cout<<"the global coordinate frame. "<<endl;
	cout<<"Step (1): Put the chess board in view, and you will see the detected"<<endl;
	cout<<"          chessboard corners in red except for the 4 outmost corners"<<endl;
	cout<<"          in green.                                                 "<<endl;
	cout<<"Step (2): Put the 3d location of the 4 outmost corners into the file"<<endl;
	cout<<"          \"\\camera_extrinsics\\3D_points.txt\" with the order of: "<<endl;
	cout<<"          point1------------point2                                  "<<endl;
	cout<<"            |                 |                                     "<<endl;
 	cout<<"          point3------------point4                                  "<<endl;
	cout<<"Step (3): Put an initial guess of camera position wrt the Raven base"<<endl;
	cout<<"          frame into file\"\\camera_extrinsics\\cam_pose_guess.txt\"."<<endl; 
	cout<<"          Note that the rotation is in degrees and translations are "<<endl;
	cout<<"          in mm, and with the format: (theta_x,theta_y,theta_z)     "<<endl;
	cout<<"                                      (trans_x,trans_y,trans_z)     "<<endl;
	cout<<"Step (4): Press \"p\" key when you are done with the files, and then"<<endl;
	cout<<"          Pnp algorithm would run. The camera rotation & translation"<<endl;
	cout<<"          matrix will be shown on console.                          "<<endl<<endl;
	cout<<"-----------menu-----------"<<endl;
	cout<<"(P): Solve Pnp for Calibration."<<endl;
	cout<<"(Q): Exit Camera calibration."<<endl<<endl;
}


void MyTracker_Display::display_3D_points(double** corners_3d, vector<cv::Point2d> corners_2d_cam, int CHESSBOARD_WIDTH,int CHESSBOARD_HEIGHT, bool post_pnp)
{
	if(post_pnp)
	{
		cout<<endl<<endl<<"(4) solve pnp"<<endl;
		cout<<"\tThese are 2D projection of the 3D corners based on pnp result."<<endl;
		for(int i=0;i<CHESSBOARD_HEIGHT; i++)
		{
			cout<<"\t";
			for(int j=0; j<CHESSBOARD_WIDTH; j++)
			{
				cout<<"("<<(int)corners_2d_cam[i*CHESSBOARD_WIDTH+j].x<<","<<(int)corners_2d_cam[i*CHESSBOARD_WIDTH+j].y<<")";
			}
			cout<<endl;
		}
		cout<<endl;
	}
	else
	{
		cout<<endl<<endl<<"(2) load 3D points from file"<<endl;
		cout<<"\tThese are the same set of corners in Raven base frame coordinates."<<endl;
		for(int i=0;i<CHESSBOARD_HEIGHT; i++)
		{
			cout<<"\t";
			for(int j=0; j<CHESSBOARD_WIDTH; j++)
			{
				cout<<"("<<(int)corners_3d[i*CHESSBOARD_WIDTH+j][0]<<",";
				cout<<(int)corners_3d[i*CHESSBOARD_WIDTH+j][1]<<",";
				cout<<(int)corners_3d[i*CHESSBOARD_WIDTH+j][2]<<")";
			}
			cout<<endl;
		}	
	}
}



void MyTracker_Display::display_2D_points(CvPoint2D32f* corners, int CHESSBOARD_WIDTH,int CHESSBOARD_HEIGHT, bool wasChessboardFound)
{	
	cout<<endl<<endl<<"(1) list of corners"<<endl;
	cout<<"\tThese corners are directly detected from 2D image frames."<<endl;
	if(wasChessboardFound)
	{
		for(int i=0;i<CHESSBOARD_HEIGHT; i++)
		{
			cout<<"\t";
			for(int j=0; j<CHESSBOARD_WIDTH; j++)
			{
				cout<<"("<<(int)corners[i*CHESSBOARD_WIDTH+j].x<<","<<(int)corners[i*CHESSBOARD_WIDTH+j].y<<")";
			}
			cout<<endl;
		}
	}
	else
		cout<<"loading error."<<endl;
}


void MyTracker_Display::display_cam_pose(cv::Mat rotation_vector, cv::Mat translation_vector, cv::Mat cameraRotationVector, cv::Mat cameraTranslationVector,  vector<cv::Point2d> corners_2d_guess, int CHESSBOARD_WIDTH,int CHESSBOARD_HEIGHT, bool post_pnp)
{
	if(post_pnp)
	{
		cout<<"\tRaven base frame origin in respect to Camera frame. (solvepnp Output)"<<endl;
		cout<<"\tRotation Vector: ("<<rotation_vector.at<double>(0)*180/CV_PI<<", ";
		cout<<rotation_vector.at<double>(1)*180/CV_PI<<", "<<rotation_vector.at<double>(2)*180/CV_PI<<")"<<endl;
		cout<<"\tTranslation Vector: ("<<translation_vector.at<double>(0)<<", ";
		cout<<translation_vector.at<double>(1)<<", "<<translation_vector.at<double>(2)<<")"<<endl<<endl;

		cout<<endl<<endl<<"(5) show pnp result"<<endl;
		cv::Mat rvec(3,1,cv::DataType<double>::type);
		cv::Rodrigues(cameraRotationVector, rvec);
		cout<<"\tCamera frame origin in respect to Raven base frame. (pnp result inversed!)"<<endl;
		cout<<"\tRotation Vector: ("<<rvec.at<double>(0)*180/CV_PI<<", ";
		cout<<rvec.at<double>(1)*180/CV_PI<<", "<<rvec.at<double>(2)*180/CV_PI<<")"<<endl;
		cout<<"\tTranslation Vector: ("<<cameraTranslationVector.at<double>(0)<<", ";
		cout<<cameraTranslationVector.at<double>(1)<<", "<<cameraTranslationVector.at<double>(2)<<")"<<endl<<endl;
		cout<<"\tThis is the camera pose matrix wrt Raven base frame."<<endl;
	}
	else
	{
		cout<<endl<<endl<<"(3) load initial guess of camera pose"<<endl;
		cout<<"\tThese are 2D projection of the 3D corners based on initial guess."<<endl;
		for(int i=0;i<CHESSBOARD_HEIGHT; i++)
		{
			cout<<"\t";
			for(int j=0; j<CHESSBOARD_WIDTH; j++)
			{
				cout<<"("<<(int)corners_2d_guess[i*CHESSBOARD_WIDTH+j].x<<","<<(int)corners_2d_guess[i*CHESSBOARD_WIDTH+j].y<<")";
			}
			cout<<endl;
		}
		cout<<endl;

		cv::Mat R;
		cv::Mat tvec(3,1,cv::DataType<double>::type);
		cv::Rodrigues(rotation_vector, R);
		tvec = -R.t()*translation_vector;
		cout<<"\tCamera frame origin in respect to Raven base frame. (Our guess)"<<endl;
		cout<<"\tRotation Vector: ("<<-rotation_vector.at<double>(0)*180/CV_PI<<", ";
		cout<<-rotation_vector.at<double>(1)*180/CV_PI<<", "<<-rotation_vector.at<double>(2)*180/CV_PI<<")"<<endl;
		cout<<"\tTranslation Vector: ("<<tvec.at<double>(0)<<", ";
		cout<<tvec.at<double>(1)<<", "<<tvec.at<double>(2)<<")"<<endl<<endl;

		cout<<"\tRaven base frame origin in respect to Camera frame. (solvepnp Input)"<<endl;
		cout<<"\tRotation Vector: ("<<rotation_vector.at<double>(0)*180/CV_PI<<", ";
		cout<<rotation_vector.at<double>(1)*180/CV_PI<<", "<<rotation_vector.at<double>(2)*180/CV_PI<<")"<<endl;
		cout<<"\tTranslation Vector: ("<<translation_vector.at<double>(0)<<", ";
		cout<<translation_vector.at<double>(1)<<", "<<translation_vector.at<double>(2)<<")"<<endl<<endl;
	}
	
	if(post_pnp)
	{
		
		cout<<"\t"<<cameraRotationVector.at<double>(0,0)<<"\t"<<cameraRotationVector.at<double>(0,1)<<"\t"<<cameraRotationVector.at<double>(0,2)<<"\t";
		cout<<cameraTranslationVector.at<double>(0)<<endl;
		cout<<"\t"<<cameraRotationVector.at<double>(1,0)<<"\t"<<cameraRotationVector.at<double>(1,1)<<"\t"<<cameraRotationVector.at<double>(1,2)<<"\t";
		cout<<cameraTranslationVector.at<double>(1)<<endl;
		cout<<"\t"<<cameraRotationVector.at<double>(2,0)<<"\t"<<cameraRotationVector.at<double>(2,1)<<"\t"<<cameraRotationVector.at<double>(2,2)<<"\t";
		cout<<cameraTranslationVector.at<double>(2)<<endl;
		cout<<"\t"<<0<<"\t"<<0<<"\t"<<0<<"\t"<<1<<endl<<endl;
	}
}


void MyTracker_Display::display_cam_pose_offline(cv::Mat rotation_vector, cv::Mat translation_vector, cv::Mat cameraRotationVector, cv::Mat cameraTranslationVector)
{
	cv::Mat R;
	cv::Mat tvec(3,1,cv::DataType<double>::type);
	cv::Rodrigues(rotation_vector, R);
	tvec = -R.t()*translation_vector;
	cout<<"-------------------Offline Processing------------------"<<endl;
	cout<<"(from previously calibrated result in cam_pose_offline.txt)"<<endl<<endl;
	cout<<"\tCamera frame origin in respect to Raven base frame. (Our guess)"<<endl;
	cout<<"\tRotation Vector: ("<<-rotation_vector.at<double>(0)*180/CV_PI<<", ";
	cout<<-rotation_vector.at<double>(1)*180/CV_PI<<", "<<-rotation_vector.at<double>(2)*180/CV_PI<<")"<<endl;
	cout<<"\tTranslation Vector: ("<<tvec.at<double>(0)<<", ";
	cout<<tvec.at<double>(1)<<", "<<tvec.at<double>(2)<<")"<<endl<<endl;

	cout<<"\tRaven base frame origin in respect to Camera frame. (solvepnp Input)"<<endl;
	cout<<"\tRotation Vector: ("<<rotation_vector.at<double>(0)*180/CV_PI<<", ";
	cout<<rotation_vector.at<double>(1)*180/CV_PI<<", "<<rotation_vector.at<double>(2)*180/CV_PI<<")"<<endl;
	cout<<"\tTranslation Vector: ("<<translation_vector.at<double>(0)<<", ";
	cout<<translation_vector.at<double>(1)<<", "<<translation_vector.at<double>(2)<<")"<<endl<<endl;
}


void MyTracker_Display::display_projection_comparison(CvPoint2D32f* corners,vector<cv::Point2d> corners_2d_guess,vector<cv::Point2d> corners_2d_cam,int CHESSBOARD_INTERSECTION_COUNT)
{
	double deltax, deltay, tmp_error;
	double average_error_guess = 0;
	double average_error_cam = 0;
	double max_error_guess = 0;
	double max_error_cam = 0;

	int index_guess = -1;
	int index_cam = -1;

	for(int i=0; i<CHESSBOARD_INTERSECTION_COUNT; i++)
	{
		deltax = corners[i].x-corners_2d_guess[i].x;
		deltay = corners[i].y-corners_2d_guess[i].y;
		tmp_error = deltax*deltax+deltay*deltay;
		average_error_guess = average_error_guess + tmp_error;
		if (tmp_error > max_error_guess)
		{
			index_guess = i;
			max_error_guess = tmp_error;
		}

		deltax = corners[i].x-corners_2d_cam[i].x;
		deltay = corners[i].y-corners_2d_cam[i].y;
		tmp_error = deltax*deltax+deltay*deltay;
		average_error_cam = average_error_cam + tmp_error;
		if (tmp_error > max_error_cam)
		{
			index_cam = i;
			max_error_cam = tmp_error;
		}
	}

	average_error_guess = average_error_guess/CHESSBOARD_INTERSECTION_COUNT;
	average_error_cam = average_error_cam/CHESSBOARD_INTERSECTION_COUNT;
	
	cout<<endl<<endl<<"(6) projection precision analysis:"<<endl;
	cout<<"\tWe want to analyze the projection error for the chess board corners between our"<<endl;
	cout<<"\tinitial guess versus the ground truth and solvepnp projection result versus the"<<endl;
	cout<<"\tground truth. Below are the average and maximum RMS projection error."<<endl;
	cout<<endl<<"\t\tIn terms of \"our guess\":"<<endl;
	cout<<"\t\tAverage error:\t"<<sqrt(average_error_guess)<<" pixel units"<<endl;

	if(index_guess == -1)
		cout<<"\t\tMax error:\t0  pixel units (cannot find a corner with particularly large error)"<<endl;
	else
		cout<<"\t\tMax error:\t"<<sqrt(max_error_guess)<<" pixel units (at the "<<index_guess<<"th corner.)"<<endl;

	cout<<endl<<"\t\tIn terms of \"solvepnp result\":"<<endl;
	cout<<"\t\tAverage error:\t"<<sqrt(average_error_cam)<<" pixel units"<<endl;

	if(index_cam == -1)
		cout<<"\t\tMax error:\t0  pixel units (cannot find a corner with particularly large error)"<<endl;
	else
		cout<<"\t\tMax error:\t"<<sqrt(max_error_cam)<<" pixel units (at the "<<index_cam<<"th corner.)"<<endl;

	cout<<endl<<"\tUnderstanding the Plot:"<<endl;
	cout<<"\t\tRed circles:           the 2d chess corners detection from image frame;"<<endl;
	cout<<"\t\tDark blue circles:     the reprojection resulting from solvepnp function;"<<endl;
	cout<<"\t\tGreenish grey circles: the reprojection resulting from our initial guess."<<endl;
}


void MyTracker_Display::display_tool_prediction(double xO,double yO,double zO,double xB,double yB,double zB,int xP,int yP)
{
	cout<<"Tool tip pos:  (xO,yO,zO) = ("<<(int)xO<<","<<(int)yO<<","<<(int)zO<<")  in mm"<<endl;
	cout<<"               (xB,yB,zB) = ("<<(int)xB<<","<<(int)yB<<","<<(int)zB<<")  in mm"<<endl;
	cout<<"Project to image: (xP,yP) = ("<<xP<<","<<yP<<")  in pixel unit"<<endl;
	cout<<endl;
}


void MyTracker_Display::display_system_message(int index)
{
	switch(index)
	{
		case 0:
			cout<<"Wait for ROS to initialize"<<endl;
			break;
		case 1:
			cout<<"Exit System..."<<endl;
			break;
		case 2:
			system("rosnode kill left"); 
			system("rosnode kill right"); 
			system("rosnode kill stereo_image_proc"); 
			system("rosnode kill Surgical_Tool_Tracker"); 
			break;
		case 3:
			system("rosnode kill player");
			system("rosnode kill Surgical_Tool_Tracker"); 
			break;
		case 4:
			ROS_ERROR("Fail to initialize ROS. Exiting!");
			break;
		case 5:
			cout<<"cv_bridge exception: ";
			break;
	}
}



void MyTracker_Display::no_chessboard_found()
{
	cout<<endl<<"[WARNING]: No Chessboard found."<<endl;
	cout<<"Chessboard may be out of view or partly occluded."<<endl;
	cout<<"Try adjusting the chessboard and repress \"p\" key."<<endl;
}


void MyTracker_Display::no_running_raven()
{
	cout<<endl<<"[WARNING]: No running Raven."<<endl;
	cout<<"The Raven software may be closed or freezing."<<endl;
	cout<<"Try again when Raven software is running."<<endl;
	cout<<"Press \"r\" key to examine the raven publish rate."<<endl;
}

void MyTracker_Display::not_calibrated_warning()
{
	cout<<endl<<"[WARNING]: Camera Not yet calibrated."<<endl;
	cout<<"Press \"c\" key to enter the calibration mode first."<<endl;
}

void MyTracker_Display::no_valid_operation_mode()
{
	cout<<endl<<"[WARNING]: The operating mode is invalid."<<endl;
	cout<<"Incorrect ROS parameter input, should be 1(online) or 0(offline)."<<endl;
	cout<<"Please recheck your roslaunch file."<<endl;
}

void MyTracker_Display::no_valid_camera_info_file(int topic,int start,int end)
{
	if(topic == 1) // instrinsic
	{
		cout<<endl<<"[WARNING]: The intrinsic matrix cannot be extracted in camera_info files."<<endl;
		cout<<"It was found in starting from string index: "<<start<<" to "<<end<<"."<<endl;
		cout<<"Please recheck if the yaml files are in the correct format."<<endl;
	}
	else if(topic == 2) // distortion
	{
		cout<<endl<<"[WARNING]: The distortion matrix cannot be extracted in camera_info files."<<endl;
		cout<<"It was found in starting from string index: "<<start<<" to "<<end<<"."<<endl;
		cout<<"Please recheck if the yaml files are in the correct format."<<endl;
	}
	else
	{
		cout<<endl<<"[WARNING]: Some information cannot be extracted in camera_info files."<<endl;
		cout<<"Please recheck if the yaml files are in the correct format."<<endl;
	}
}


void MyTracker_Display::no_correct_camera_set(int which_camera)
{
	cout<<endl<<"[WARNING]: The camera index is incorrect."<<endl;
	cout<<"The current camera index is "<<which_camera<<", but it should be "<<LEFT_CAMERA<<" or "<<RIGHT_CAMERA<<"."<<endl;
	cout<<"Please recheck and fix the camera index."<<endl;
}

SYSTEM_STATUS_LIST MyTracker_Display::check_user_selection_c(int theKey, SYSTEM_STATUS_LIST status)
{	
	SYSTEM_STATUS_LIST sys_status = status;
	
	if(theKey!=-1)
	{
		cout<<theKey<<endl;
		switch(theKey)
		{
			case 113: // 'q' or 'Q' : go back to main menu
			case 81:
				cout<<endl<<"'q' Key Pressed: Exit Camera Calibration Mode."<<endl;
				sys_status = SHOW_MENU;
				break;

			case 112: // 'p' or 'P' : solve pnp for calibration
			case 80:
				cout<<endl<<"'p' Key Pressed: Solve Pnp for Calibration"<<endl;
				sys_status = CAMERA_CALIBRATE_PNP_READY_MODE;
				break;

			default:
				cout<<"'other' Key Pressed: Unrecognized option."<<endl;
				break;
		}
	}
	return sys_status;
}

SYSTEM_STATUS_LIST MyTracker_Display::check_user_selection_p(int theKey, bool online)
{	
	SYSTEM_STATUS_LIST sys_status = PENDING_USER_SELECTION;
	if(online)  // online processing
	{
		if(theKey!=-1)
		{
			switch(theKey)
			{
				case 114: // 'r' or 'R' : toggle ros topic display
				case 82:
					toggle_show_ros();
					sys_status = SHOW_MENU;
					break;

				case 116: // 't' or 'T' : toggle tool tracking start/end
				case 84:
					toggle_tool_tracking(false); // forcequit = false
					sys_status = SHOW_MENU;
					break;

				case 111: // 'o' or 'O' : toggle tool-tracking xyz offset
				case 79:
					toggle_tool_tracking_offset();
					sys_status = SHOW_MENU;
					break;

				case 109: // 'm' or 'M' : toggle manual segmentation parameter
				case 77:
					toggle_manual_segmentation_parameter();
					sys_status = SHOW_MENU;
					break;

				case 110: // 'n' or 'N' : toggle manual disparity parameter
				case 78:
					toggle_manual_disparity_parameter();
					sys_status = SHOW_MENU;
					break;

				case 99: // 'c' or 'C' : go to calibration mode
				case 67:
					cout<<endl<<"'c' Key Pressed: Enter camera calibration mode."<<endl;

					//(1) enter camera calibration mode
					display_calibration_instruction();
					sys_status = CAMERA_CALIBRATE_MODE;

					//(2) disable tool tracking and rostopic show
					SHOW_ROS_TOPIC = false;
					toggle_tool_tracking(true); // forcequit = true

					//(3) quit all robot motion
					for(int i=0;i<6;i++)
						RAVEN_MOTION[i] = NO_MOTION;
					break;

				case 119: // 'w' or 'W' : raven move up (x+)
				case 87:
					RAVEN_MOTION[0] = START_MOTION;
					cout<<endl<<"'w' Key Pressed: Raven move up."<<endl;
					break;

				case 115: // 's' or 'S' : raven move down (x-)
				case 83:
					RAVEN_MOTION[1] = START_MOTION;
					cout<<endl<<"'s' Key Pressed: Raven move down."<<endl;
					break;

				case 97: // 'a' or 'A' : raven move left (z-)
				case 65:
					RAVEN_MOTION[5] = START_MOTION;
					cout<<endl<<"'a' Key Pressed: Raven move left."<<endl;
					break;

				case 98: // 'b' or 'B' : raven move backward (y-)
				case 66:
					RAVEN_MOTION[3] = START_MOTION;
					cout<<endl<<"'b' Key Pressed: Raven move backward."<<endl;
					break;

				case 100: // 'd' or 'D' : raven move right (z+)
				case 68:
					RAVEN_MOTION[4] = START_MOTION;
					cout<<endl<<"'d' Key Pressed: Raven move right."<<endl;
					break;

				case 102: // 'f' or 'F' : raven move forward (y+)
				case 70:
					RAVEN_MOTION[2] = START_MOTION;
					cout<<endl<<"'f' Key Pressed: Raven move forward."<<endl;
					break;

				case 105: // 'i' or 'I' : Save depth result to file
				case 73:
					RAVEN_MOTION[2] = START_MOTION;
					cout<<endl<<"'i' Key Pressed: Save depth result to file request received."<<endl;
					DEPTHMAP_FILE_SAVE_PENDING = true;
					break;

				case 113: // 'q' or 'Q' : system exit
				case 81:
					cout<<endl<<"'q' Key Pressed: Exit."<<endl;
					sys_status = EXIT_ALL;
					break;

				default:
					cout<<"'other' Key Pressed: Unrecognized option."<<endl;
					break;
			}
		}
	}
	else   // offline processing
	{
		if(theKey!=-1)
		{
			switch(theKey)
			{
				case 114: // 'r' or 'R' : toggle ros topic display
				case 82:
					toggle_show_ros();
					sys_status = SHOW_MENU;
					break;

				case 116: // 't' or 'T' : toggle tool tracking start/end
				case 84:
					toggle_tool_tracking(false); // forcequit = false
					sys_status = SHOW_MENU;
					break;

				case 111: // 'o' or 'O' : toggle tool-tracking xyz offset
				case 79:
					toggle_tool_tracking_offset();
					sys_status = SHOW_MENU;
					break;

				case 109: // 'm' or 'M' : toggle manual segmentation parameter
				case 77:
					toggle_manual_segmentation_parameter();
					sys_status = SHOW_MENU;
					break;

				case 110: // 'n' or 'N' : toggle manual disparity parameter
				case 78:
					toggle_manual_disparity_parameter();
					sys_status = SHOW_MENU;
					break;

				case 105: // 'i' or 'I' : Save depth result to file
				case 73:
					RAVEN_MOTION[2] = START_MOTION;
					cout<<endl<<"'i' Key Pressed: Save depth result to file request received."<<endl;
					DEPTHMAP_FILE_SAVE_PENDING = true;
					break;

				case 113: // 'q' or 'Q' : system exit
				case 81:
					cout<<endl<<"'q' Key Pressed: Exit."<<endl;
					sys_status = EXIT_ALL;
					break;

				default:
					cout<<"'other' Key Pressed: Unrecognized option."<<endl;
					break;
			}
		}
	}
	
	return sys_status;
}
