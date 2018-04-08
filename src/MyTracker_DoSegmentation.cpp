#include "myproject/MyTracker_DoSegmentation.h"


MyTracker_DoSegmentation::MyTracker_DoSegmentation()
{
	// User options for disparity computation:
	// -----------------------------------------------------------------------------------------------------------------------------------------
	this->image_of_use = 0;        		// 0: actual-image,  1: test-image-tower, 2: test-image-sculpture, 3: test-image-aloe, 4: test-image-myimage
	this->compute_roi_mode = 2;    		// 0: whole-image,   1: roi-square,       2: roi-sphere
	this->depth_map_mode = 2;      		// 0: norm(1-depth), 1: 1/depth,          2: log(1/depth),         3: true depth
	this->reproj_cali_mode = 1;    		// 0: no adjustment, 1: static offset,    2: dynamic offset
	this->reproj_to_3D_method = 1;          // 0: opencv func,   1: my own implementation
	this->disparity_post_filtering = 0;	// 0: use left dsp,  1: use right dsp,    2: pick max value,       3: use weighted least square (non are good, improve later)
	this->rectify_image = true;
	this->normalize_image = false;
	this->show_rectified_image = false;
	this->show_disparity = false;
	this->show_depthmap = false;                    // Todo: turn it off for now, better visualization later...
	this->show_depth_and_reprojection = false; 	// show depth values and 3D reprojection result on console for each image frame
	this->show_toolpos_wrt_cam = false; 		// show TOOLPOS_WRT_CAM on console to check if reasonable!
	this->show_lattice_points_only = false;        

}


MyTracker_DoSegmentation::~MyTracker_DoSegmentation()
{
	
}


void MyTracker_DoSegmentation::init(char* Wname, int ver, bool dice, int camera, int disp_algo)
{
	this->window_name_segm = Wname;
	this->version = ver;
	this->tissue_mean_B = 152;
	this->tissue_mean_G = 141;
	this->tissue_mean_R = 162;
	this->first_access = true;
	this->fourier_resize = 0.5;
	this->static_background = true;
	this->record_dice_coefficient = dice;
	this->depth_roi_width = 5;
	this->which_camera = camera;
	this->disparity_algorithm = disp_algo;
	this->rectification_offset = -14.0; // Todo: why is this happening?
	this->weirdstring = "Start disparity computaion!\n";
	this->depthstring = show_lattice_points_only ? "depth lattice":"depth map";

	string str4 = "/home/biorobotics/catkin_ws/src/myproject/camera_info/camera_intrinsics/camera_left.yaml";
	file_name4 = new char[str4.length() + 1];
	strcpy(file_name4, str4.c_str());

	string str5 = "/home/biorobotics/catkin_ws/src/myproject/camera_info/camera_intrinsics/camera_right.yaml";
	file_name5 = new char[str5.length() + 1];
	strcpy(file_name5, str5.c_str());

	init_process_colorwheel();
	init_disparity_param();
	load_cam_intrinsics();

	
}



void MyTracker_DoSegmentation::init_process_colorwheel()
{
	//colorwheel = imread("/home/biorobotics/catkin_ws/src/myproject/camera_info/pictures/colormap.png", CV_LOAD_IMAGE_COLOR);
	colorwheel = imread("/home/biorobotics/catkin_ws/src/myproject/camera_info/pictures/colorwheel.jpg", CV_LOAD_IMAGE_COLOR);
	resize(colorwheel,colorwheel,cv::Size(colorwheel.rows*0.25,colorwheel.cols*0.25));
	Mat BGR[3], HSV[3]; 
	Mat Oppo1, Oppo2, Oppo2_Oppo1, Color_diff, hsv_color, Y;

	split(colorwheel,BGR); // split source into independent color channels
	normalize(255+BGR[1]-BGR[2], Oppo1, 0, 255, NORM_MINMAX, CV_8UC1); 
	normalize(    BGR[1]+BGR[2],     Y, 0, 255, NORM_MINMAX, CV_8UC1); 
	normalize(255+BGR[0]-     Y, Oppo2, 0, 255, NORM_MINMAX, CV_8UC1); 
	normalize(255+BGR[0]-     Y, Oppo2, 0, 255, NORM_MINMAX, CV_8UC1); 

	normalize(255+Oppo2-Oppo1, colorwheel_oppo, 0, 255, NORM_MINMAX, CV_8UC1); 
}

void MyTracker_DoSegmentation::init_disparity_param()
{
	switch(disparity_algorithm)
	{
		case 0:
			DISP_NUMDISP = 16*4;	// numDisparities
			DISP_WINSIZE = 15;	// blockSize, window size
			break;
		case 1:
			DISP_MINDISP = 0;	// minDisparity
			DISP_NUMDISP = 16*2;	// numDisparities
			DISP_WINSIZE = 3;	// blockSize, window size
			DISP_P1 = 0;		// p1
			DISP_P2 = 0;		// p2
			DISP_MAXDIFF = 0;	// disp12MaxDiff
			DISP_FLTRCAP = 0;	// preFilterCap
			DISP_UNQRATE = 0;	// uniquenessRatio
			DISP_SPKSIZE = 0;	// speckleWindowSize
			DISP_SPKRNGE = 0;	// speckleRange
			DISP_MODE = StereoSGBM::MODE_HH; // mode
			break;
		case 2:
			DISP_MINDISP = -64;	// minDisparity
			DISP_NUMDISP = 192;	// numDisparities
			DISP_WINSIZE = 5;	// blockSize, window size
			DISP_P1 = 600;		// p1
			DISP_P2 = 2400;		// p2
			DISP_MAXDIFF = 10;	// disp12MaxDiff
			DISP_FLTRCAP = 4;	// preFilterCap
			DISP_UNQRATE = 1;	// uniquenessRatio
			DISP_SPKSIZE = 150;	// speckleWindowSize
			DISP_SPKRNGE = 2;	// speckleRange
			DISP_MODE = StereoSGBM::MODE_SGBM; // mode
			break;
		case 3:
			DISP_MINDISP = 0;	// minDisparity
			DISP_NUMDISP = 64;	// numDisparities
			DISP_WINSIZE = 5;	// blockSize, window size
			DISP_P1 = 600;		// p1
			DISP_P2 = 2400;		// p2
			DISP_MAXDIFF = 10;	// disp12MaxDiff
			DISP_FLTRCAP = 4;	// preFilterCap
			DISP_UNQRATE = 1;	// uniquenessRatio
			DISP_SPKSIZE = 900;	// speckleWindowSize
			DISP_SPKRNGE = 2;	// speckleRange
			DISP_MODE = StereoSGBM::MODE_HH; // mode
			break;
	}

	DISP_LAMBDA = 8000;   // WLS filter of the disparity
	DISP_SIGMA = 1;

	DISP_SHIFT = 46;
	DISP_SCALE = 0.0625;  // 1/16
	// http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereobm-stereobm
	// Output disparity map. It has the same size as the input images. When disptype==CV_16S, the map is a 16-bit signed single-channel image, containing disparity values scaled by 16. To get the true disparity values from such fixed-point representation, you will need to divide each disp element by 16.
}


void MyTracker_DoSegmentation::load_camera_pose(Mat rVec, Mat tVec)
{
	cameraRotationVector = rVec.clone();
	cameraTranslationVector = tVec.clone();
}


void MyTracker_DoSegmentation::load_image(cv_bridge::CvImagePtr Img)
{
	color_ipl = cvCreateImage(cvSize(Img->image.cols,Img->image.rows), IPL_DEPTH_8U, 3);
	gray_ipl = cvCreateImage(cvSize(Img->image.cols,Img->image.rows), IPL_DEPTH_8U, 1);
	
	// cv_ptr to iplimage*
	color_ipl->imageData = (char *) Img->image.data;
	cvCvtColor(color_ipl, gray_ipl, CV_RGB2GRAY); //source: color image, destination: gray image
	// iplimage* to Mat
	color =  cv::cvarrToMat(color_ipl);	
	gray =  cv::cvarrToMat(gray_ipl);
	
	if(first_access)
	{
		dftSize.width = getOptimalDFTSize(gray.cols*2*fourier_resize);  
		dftSize.height = getOptimalDFTSize(gray.rows*2*fourier_resize);

		// create panelty image
		PANELTY = cv::Mat(dftSize,CV_32FC1,cv::Scalar(0));
		float max_panelty_value = sqrt(gray.rows*fourier_resize/4 * gray.rows*fourier_resize/4 + gray.cols*fourier_resize/4 * gray.cols*fourier_resize/4);

		for(int r=0;r<dftSize.height;r++)
		for(int c=0;c<dftSize.width;c++)
		{
			int rr = abs(dftSize.height/2-r);
			int cc = abs(dftSize.width/2-c);
			float value = sqrt(rr * rr + cc * cc);

			PANELTY.at<float>(r,c) = value > max_panelty_value ? max_panelty_value : value;				
		}
		normalize(PANELTY, PANELTY, 0.4, 1, CV_MINMAX);	
		first_access = false;
	}
}



void MyTracker_DoSegmentation::load_imageN(cv_bridge::CvImagePtr Img)
{
	colorN_ipl = cvCreateImage(cvSize(Img->image.cols,Img->image.rows), IPL_DEPTH_8U, 3);
	grayN_ipl = cvCreateImage(cvSize(Img->image.cols,Img->image.rows), IPL_DEPTH_8U, 1);
	
	// cv_ptr to iplimage*
	colorN_ipl->imageData = (char *) Img->image.data;
	cvCvtColor(colorN_ipl, grayN_ipl, CV_RGB2GRAY); //source: color image, destination: gray image
	// iplimage* to Mat
	colorN =  cv::cvarrToMat(colorN_ipl);	
	grayN =  cv::cvarrToMat(grayN_ipl);
	//imshow("the other image",colorN);
	//cvWaitKey(30);
}



bool MyTracker_DoSegmentation::load_cam_intrinsics()
{
	int main = 1;
	int other = 2;
	
	if(which_camera == LEFT_CAMERA)
	{
		load_cam_intrinsic(file_name4,main);  // left camera info
		load_cam_intrinsic(file_name5,other); // right camera info
	}
	else if(which_camera == RIGHT_CAMERA)
	{
		load_cam_intrinsic(file_name5,main);  // left camera info
		load_cam_intrinsic(file_name4,other); // right camera info
	}
	else
		return false;

	return true;
}



void MyTracker_DoSegmentation::load_cam_intrinsic(char* file_name, int load_to)
{
	int main = 1;
	int other = 2;

	size_t start,end;
	string buf,data,target1,target2,data1,data2,data3;
	stringstream buffer;
	vector<string> tokens1;
	vector<string> tokens2;
	vector<string> tokens3;

	target1 = "data: [";
	target2 = "]\n";

	ifstream file(file_name);

	//load the text file and put it into a single string:
	buffer << file.rdbuf();	
	data = buffer.str();

	start = data.find(target1); // this is where intrinsic is
	if (start!=string::npos)
	{
		end = data.find(target2.c_str(),start+1);
		data1 = data.substr(start+7, end-start-7); // string.substr(starting_index, length_of_sub_string)
	}

	start = data.find(target1.c_str(),end+1); // this is where distortion is
	if (start!=string::npos)
	{
		end = data.find(target2.c_str(),start+1);
		data2 = data.substr(start+7, end-start-7); // string.substr(starting_index, length_of_sub_string)
	}

	start = data.find(target1.c_str(),end+1); // this is where rectification is
	if (start!=string::npos)
	{
		end = data.find(target2.c_str(),start+1);
		data3 = data.substr(start+7, end-start-7); // string.substr(starting_index, length_of_sub_string)
	}

	stringstream ss1(data1);
	if(file.is_open())
	{	
		while (ss1 >> buf)
			tokens1.push_back(buf.substr(0,buf.length()-1));
	}

	stringstream ss2(data2);
	if(file.is_open())
	{	
		while (ss2 >> buf)
			tokens2.push_back(buf.substr(0,buf.length()-1));
	}
		
	stringstream ss3(data3);
	if(file.is_open())
	{	
		while (ss3 >> buf)
			tokens3.push_back(buf.substr(0,buf.length()-1));
	}
	

	if(load_to == main)
	{
		// (0) prepare to load the values
		distortion = cv::Mat::zeros(1,5,CV_32F);
		intrinsicMat = cv::Mat::zeros(3,3,CV_32F);
		intrinsic = intrinsicMat.clone();
		rectification = intrinsicMat.clone();

		// (1) load intrinsic matrix
		intrinsic(0,0) = (double)atof(tokens1[0].c_str()); // 492.072810; .....these are values from right camera
		intrinsic(1,1) = (double)atof(tokens1[4].c_str()); // 494.288119; 
		intrinsic(0,2) = (double)atof(tokens1[2].c_str()); // 366.475055; 
		intrinsic(1,2) = (double)atof(tokens1[5].c_str()); // 217.969378; 
		intrinsic(2,2) = (double)1.0;
		/*
		// (2) load distortion matrix
		//turn off distortion!
		//performance improves so much after this is turned off!

		distortion.at<double>(0,0) = atof(tokens1[0].c_str()); // -0.339369;  
		distortion.at<double>(0,1) = atof(tokens1[1].c_str()); // 0.092632;
		distortion.at<double>(0,2) = atof(tokens1[2].c_str()); // 0.000787;
		distortion.at<double>(0,3) = atof(tokens1[3].c_str()); // -0.001135;
		distortion.at<double>(0,4) = atof(tokens1[4].c_str()); // 0.000000;
		*/

		// (3) load rectification matrix
		rectification(0,0) = (double)atof(tokens3[0].c_str()); 
		rectification(0,1) = (double)atof(tokens3[1].c_str()); 
		rectification(0,2) = (double)atof(tokens3[2].c_str());
		rectification(1,0) = (double)atof(tokens3[3].c_str());  
		rectification(1,1) = (double)atof(tokens3[4].c_str()); 
		rectification(1,2) = (double)atof(tokens3[5].c_str());
		rectification(2,0) = (double)atof(tokens3[6].c_str());
		rectification(2,1) = (double)atof(tokens3[7].c_str());
		rectification(2,2) = (double)atof(tokens3[8].c_str());
	}
	else if(load_to == other)
	{
		// (0) prepare to load the values
		distortionN = cv::Mat::zeros(1,5,CV_32F);
		intrinsicMatN = cv::Mat::zeros(3,3,CV_32F);
		intrinsicN = intrinsicMatN.clone();
		rectificationN = intrinsicMatN.clone();

		// (1) load intrinsic matrix
		intrinsicN(0,0) = (double)atof(tokens1[0].c_str()); // 492.072810; .....these are values from right camera
		intrinsicN(1,1) = (double)atof(tokens1[4].c_str()); // 494.288119; 
		intrinsicN(0,2) = (double)atof(tokens1[2].c_str()); // 366.475055; 
		intrinsicN(1,2) = (double)atof(tokens1[5].c_str()); // 217.969378; 
		intrinsicN(2,2) = (double)1.0;
		/*
		// (2) load distortion matrix
		//turn off distortion!
		//performance improves so much after this is turned off!

		distortionN.at<double>(0,0) = atof(tokens1[0].c_str()); // -0.339369;  
		distortionN.at<double>(0,1) = atof(tokens1[1].c_str()); // 0.092632;
		distortionN.at<double>(0,2) = atof(tokens1[2].c_str()); // 0.000787;
		distortionN.at<double>(0,3) = atof(tokens1[3].c_str()); // -0.001135;
		distortionN.at<double>(0,4) = atof(tokens1[4].c_str()); // 0.000000;
		*/

		// (3) load rectification matrix
		rectificationN(0,0) = (double)atof(tokens3[0].c_str()); 
		rectificationN(0,1) = (double)atof(tokens3[1].c_str()); 
		rectificationN(0,2) = (double)atof(tokens3[2].c_str());
		rectificationN(1,0) = (double)atof(tokens3[3].c_str());  
		rectificationN(1,1) = (double)atof(tokens3[4].c_str()); 
		rectificationN(1,2) = (double)atof(tokens3[5].c_str());
		rectificationN(2,0) = (double)atof(tokens3[6].c_str());
		rectificationN(2,1) = (double)atof(tokens3[7].c_str());
		rectificationN(2,2) = (double)atof(tokens3[8].c_str());
	}

	// (4) manually compute the rectification matrix
	Mat R = cv::Mat::zeros(3,3,6);
	R.at<double>(0,0) = 0.9999758789664913;		R.at<double>(0,1) = 0.003288680669011141; 	R.at<double>(0,2) = 0.006117684582443647; 
	R.at<double>(1,0) = -0.0033106878143531098;	R.at<double>(1,1) = 0.999988073190411; 		R.at<double>(1,2) = 0.003590657756601621; 
	R.at<double>(2,0) = -0.006105803091231333;	R.at<double>(2,1) = -0.0036108248900247144; 	R.at<double>(2,2) = 0.9999748402396056; 

	Mat T = cv::Mat::zeros(3,1,6);
	T.at<double>(0,0) = -39.0081196771388;  // in mm
	T.at<double>(1,0) = 0.1232890060403908;
	T.at<double>(2,0) = -0.6287479164457284;

	Mat P1, P2, RR, RRN;
	Rect roi1, roi2;
	

	stereoRectify( intrinsic, distortion, intrinsicN, distortionN, cv::Size(640,480), R, T, RR, RRN, P1, P2, disp2depth);
	disp2depth.at<double>(0,3) = -intrinsic(0,2);
	//we thought the disp2depth mapping matrix was wrong... but maybe not:))
	/*
	cout<<"raw disp2depth:\n"<<disp2depth<<endl;
	disp2depth.at<double>(0,3) = -intrinsic(0,2);
	disp2depth.at<double>(1,3) = -intrinsic(1,2);
	disp2depth.at<double>(3,2) = -disp2depth.at<double>(3,2); // this is a bug in openCV
	disp2depth.at<double>(3,3) = (intrinsic(0,2)-intrinsicN(0,2))/T.at<double>(0,0); 
	cout<<"new disp2depth:\n"<<disp2depth<<endl;*/

	rectification = RR.clone();
	rectificationN = RRN.clone();
}



Mat MyTracker_DoSegmentation::publish_image()
{
	Mat result;
	
	if(version>=3)
	{
		if(record_dice_coefficient)
		{
			int dstWidth = color.cols*2;
			int dstHeight = color.rows*2;
			cv::Mat dstMat = cv::Mat(dstHeight, dstWidth, CV_8UC3, cv::Scalar(0,0,0));

			cv::Rect roi11(cv::Rect(           0,           0, color.cols, color.rows));
			cv::Rect roi12(cv::Rect(  color.cols,           0, color.cols, color.rows));
			cv::Rect roi21(cv::Rect(           0,  color.rows, color.cols, color.rows));
			cv::Rect roi22(cv::Rect(  color.cols,  color.rows, color.cols, color.rows));

			cv::Mat targetROI11 = dstMat(roi11);
			cv::Mat targetROI12 = dstMat(roi12);
			cv::Mat targetROI21 = dstMat(roi21);
			cv::Mat targetROI22 = dstMat(roi22);

			cv::Mat segment_mask_new = segment_mask.clone();
			segment_mask_new = 255*segment_mask_new;
			segment_mask_new.convertTo(segment_mask_new,segment_fore.type());
		  	color.copyTo(targetROI11); 
			segment_fore.copyTo(targetROI22); 
			segment_back.copyTo(targetROI21); 
			segment_mask_new.copyTo(targetROI12); 
			result = dstMat.clone();
		}
		else
			result = segment_fore.clone();
	}
	else
		result = color.clone();	
	
	return result;
	
}


void MyTracker_DoSegmentation::load_kinematics(vector<cv::Point3d> world_coords, vector<cv::Point2d> pixel_coords, vector<double> distance)
{
	static cv::Point model_ptA_bk(320,-100000);

	PIXEL_POS.clear();
	RAVEN_POS.clear();
	RAVEN_DIS.clear();

	PIXEL_POS = pixel_coords;
	RAVEN_POS = world_coords;  // accumulate over time (base frame)
	RAVEN_DIS = distance;      // the distance to camera for each raven model point

	switch(version)
	{
		case 0:
		case 1:
			ptO = PIXEL_POS[0];
			ptX = PIXEL_POS[1];
			ptY = PIXEL_POS[2];
			ptZ = PIXEL_POS[3];
			break;
		case 2:
		case 3:
		case 4:
			ptO = PIXEL_POS[0];
			ptX = PIXEL_POS[1];
			ptY = PIXEL_POS[2];
			ptZ = PIXEL_POS[3];
			model_ptA = PIXEL_POS[4];
			model_ptB = PIXEL_POS[5];
			model_ptC = PIXEL_POS[6];
			model_ptD = PIXEL_POS[7];	
			model_ptE = PIXEL_POS[8];

			// sanity check (a strange bug during projection: sometimes 3D points are normal, but 2D result is insane)
			if(model_ptA.y>0)
				model_ptA = model_ptA_bk;
			else
				model_ptA_bk = model_ptA;

			break;
	}

	Point3d curr_tool_pos = RAVEN_POS.back();
	Point3d tmp;

	tmp.x = curr_tool_pos.x-cameraTranslationVector.at<double>(0);
	tmp.y = curr_tool_pos.y-cameraTranslationVector.at<double>(1);
	tmp.z = curr_tool_pos.z-cameraTranslationVector.at<double>(2);
	
	// Here we use the transpose of cameraRotationVector!
	TOOLPOS_WRT_CAM.x = cameraRotationVector.at<double>(0,0)*tmp.x+cameraRotationVector.at<double>(1,0)*tmp.y+cameraRotationVector.at<double>(2,0)*tmp.z;
	TOOLPOS_WRT_CAM.y = cameraRotationVector.at<double>(0,1)*tmp.x+cameraRotationVector.at<double>(1,1)*tmp.y+cameraRotationVector.at<double>(2,1)*tmp.z;
	TOOLPOS_WRT_CAM.z = cameraRotationVector.at<double>(0,2)*tmp.x+cameraRotationVector.at<double>(1,2)*tmp.y+cameraRotationVector.at<double>(2,2)*tmp.z;
	
	if(show_toolpos_wrt_cam)
	{
		static int showit=0;
		if(showit%30 == 0)
		{	
			cout<<"TOOLPOS_WRT_CAM (before rotation) = "<<tmp<<endl;
			cout<<"TOOLPOS_WRT_CAM (after rotation) =  "<<TOOLPOS_WRT_CAM<<endl; 
		}
		showit+=1;
	}
}


void MyTracker_DoSegmentation::load_segmentation_param(vector<int> segm_param)
{
	SEG_BLURY_SCALE = segm_param[0];
	SEG_BLURY_INTENSITY = segm_param[1];
	SEG_THRES_LOWBOUND = segm_param[2];
	SEG_FILL_BLACKSPOT = segm_param[3];
}


void MyTracker_DoSegmentation::load_disparity_param(vector<int> disp_param)
{
	//for SGBM:
	// (minDisparity,numDisparities,blockSize,P1,P2,,preFilterCap,uniquenessRatio,speckleWindowSize,speckleRange,StereoSGBM::MODE_SGBM)
	DISP_MINDISP = disp_param[0];	// minDisparity
	DISP_NUMDISP = disp_param[1];	// numDisparities
	DISP_WINSIZE = disp_param[2];	// blockSize, window size
	DISP_P1 = disp_param[3];	// p1
	DISP_P2 = disp_param[4];	// p2
	DISP_MAXDIFF = disp_param[5];	// disp12MaxDiff
	DISP_FLTRCAP = disp_param[6];	// preFilterCap
	DISP_UNQRATE = disp_param[7];	// uniquenessRatio
	DISP_SPKSIZE = disp_param[8];	// speckleWindowSize
	DISP_SPKRNGE = disp_param[9];	// speckleRange
	DISP_MODE = StereoSGBM::MODE_HH;// mode
}


void MyTracker_DoSegmentation::prep_probability()
{
	// initialize the intermediate images
	reset_intermediate_images();

	// start processing prior probability
	switch(version)
	{
		case 0:
			prep_kinematics_simple(); // load kinematics (whole raven tool tip pose)
			break;
		case 1:
			// (1) analyze kinematics
			prep_kinematics_simple();
			prep_kinematics_probability();

			// (2) analyze motion
			prep_motion_probability();

			// (3) combine together and create probability map
			combined_prob = feature_prob.mul(ravenstate_prob);  // maybe not the best.
			break;
		case 2:
			prep_kinematics_with_model(); // load kinematics (whole raven model)
			break;
		case 3:
		case 4:
			prep_kinematics_with_model_and_width(); // load kinematics (whole raven model with width estimation from distance)
			prep_log_likelihood_Q();
			prep_reference_shape_U();			
			break;

	}
}


void MyTracker_DoSegmentation::prep_kinematics_simple()
{	
	cv::line(ravenstate_mark, ptO, ptX, CV_RGB(255,0,0),3);   // x axis: (Red,   thickness 3)
	cv::line(ravenstate_mark, ptO, ptY, CV_RGB(0,255,0),3);   // y axis: (Green, thickness 3)
	cv::line(ravenstate_mark, ptO, ptZ, CV_RGB(0,0,255),3);   // z axis: (Blue,  thickness 3)
	cv::circle(ravenstate_mark, ptO, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
}


void MyTracker_DoSegmentation::prep_kinematics_with_model()
{
	cv::line(ravenstate_mark, model_ptA, model_ptB, CV_RGB(150,150,150),3); // model link: (Gray, thickness 3)
	cv::line(ravenstate_mark, model_ptB, model_ptC, CV_RGB(150,150,150),3);
	cv::line(ravenstate_mark, model_ptC, model_ptD, CV_RGB(100,100,100),3);
	cv::line(ravenstate_mark, model_ptC, model_ptE, CV_RGB(150,150,150),3);

	cv::circle(ravenstate_mark, model_ptA, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
	cv::circle(ravenstate_mark, model_ptB, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
	cv::circle(ravenstate_mark, model_ptC, 3, CV_RGB(255,255,0),3); // the point: (Yellow, thickness 0, radius 3)
	cv::circle(ravenstate_mark, model_ptD, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
	cv::circle(ravenstate_mark, model_ptE, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)

	cv::line(ravenstate_mark, ptO, ptX, CV_RGB(255,0,0),2);   // x axis: (Red,   thickness 2)
	cv::line(ravenstate_mark, ptO, ptY, CV_RGB(0,255,0),2);   // y axis: (Green, thickness 2)
	cv::line(ravenstate_mark, ptO, ptZ, CV_RGB(0,0,255),2);   // z axis: (Blue,  thickness 2)
}


void MyTracker_DoSegmentation::prep_kinematics_with_model_and_width()
{
	// include the width based on distance
	double sign_AB = (model_ptA.y-model_ptB.y)>0?1:-1;
	double sign_BC = (model_ptB.y-model_ptC.y)>0?1:-1;
	double sign_CD = (model_ptC.x-model_ptD.x)>0?1:-1;
	double sign_CE = (model_ptC.x-model_ptE.x)>0?1:-1;

	double norm_AB = cv::norm(model_ptA-model_ptB);
	double norm_BC = cv::norm(model_ptB-model_ptC);
	double norm_CD = cv::norm(model_ptC-model_ptD);
	double norm_CE = cv::norm(model_ptC-model_ptE);

	double factor_AB = TOOL_WIDTH*OBJECT_SIZE_FACTOR*sign_AB/norm_AB;
	double factor_BC = TOOL_WIDTH*OBJECT_SIZE_FACTOR*sign_BC/norm_BC;
	double factor_CD = GRASPER_TIP_WIDTH*OBJECT_SIZE_FACTOR*sign_CD/norm_CD;
	double factor_CE = GRASPER_TIP_WIDTH*OBJECT_SIZE_FACTOR*sign_CE/norm_CE;
	double factor_CDE = GRASPER_WIDTH*OBJECT_SIZE_FACTOR/norm_BC;
	
	cv::Point normal_to_AB( factor_AB*(model_ptA.y-model_ptB.y), factor_AB*(model_ptB.x-model_ptA.x));
	cv::Point normal_to_BC( factor_BC*(model_ptB.y-model_ptC.y), factor_BC*(model_ptC.x-model_ptB.x));
	cv::Point normal_to_CD( factor_CD*(model_ptC.y-model_ptD.y), factor_CD*(model_ptD.x-model_ptC.x));
	cv::Point normal_to_CE( factor_CE*(model_ptC.y-model_ptE.y), factor_CE*(model_ptE.x-model_ptC.x));
	cv::Point tangent_to_BC = factor_CDE*(model_ptB-model_ptC);

                                                                //         A1______A2
	model_ptA1 = model_ptA - normal_to_AB/(RAVEN_DIS[0]+1); //          /     /
	model_ptA2 = model_ptA + normal_to_AB/(RAVEN_DIS[0]+1); //         /     /
	model_ptB1 = model_ptB - normal_to_AB/(RAVEN_DIS[1]+1);	//      B1/_____/B2
	model_ptB2 = model_ptB + normal_to_AB/(RAVEN_DIS[1]+1); //      B3------B4
	model_ptB3 = model_ptB - normal_to_BC/(RAVEN_DIS[1]+1);	//       / C3  /
	model_ptB4 = model_ptB + normal_to_BC/(RAVEN_DIS[1]+1); //    C1/_/_|_/C2
	model_ptC1 = model_ptC - normal_to_BC/(RAVEN_DIS[2]+1);	//     / /  | |
	model_ptC2 = model_ptC + normal_to_BC/(RAVEN_DIS[2]+1); //     |/   |/
								//   D1 D  E E1        these are what the points represent.

	model_ptC3 = model_ptC + tangent_to_BC/(RAVEN_DIS[2]+1);
	model_ptD1 = model_ptD + normal_to_CD/(RAVEN_DIS[3]+1);
	model_ptE1 = model_ptE + normal_to_CE/(RAVEN_DIS[4]+1);
	
	if(version == 3)
	{
		cv::line(ravenstate_mark, model_ptA1, model_ptB1, CV_RGB(250,250,250),1); // model link: (Gray, thickness 1)
		cv::line(ravenstate_mark, model_ptA2, model_ptB2, CV_RGB(250,250,250),1); // model link: (Gray, thickness 1)
		cv::line(ravenstate_mark, model_ptA1, model_ptA2, CV_RGB(250,250,250),1); // model link: (Gray, thickness 1)
		cv::line(ravenstate_mark, model_ptB1, model_ptB2, CV_RGB(250,250,250),1); // model link: (Gray, thickness 1)

		cv::line(ravenstate_mark, model_ptB3, model_ptC1, CV_RGB(250,250,250),1);
		cv::line(ravenstate_mark, model_ptB4, model_ptC2, CV_RGB(250,250,250),1);
		cv::line(ravenstate_mark, model_ptB3, model_ptB4, CV_RGB(250,250,250),1);
		cv::line(ravenstate_mark, model_ptC1, model_ptC2, CV_RGB(250,250,250),1);

		cv::line(ravenstate_mark, model_ptC, model_ptC3, CV_RGB(250,250,250),1);
		cv::line(ravenstate_mark, model_ptC, model_ptD, CV_RGB(250,250,250),1);
		cv::line(ravenstate_mark, model_ptC, model_ptE, CV_RGB(250,250,250),1);
		cv::line(ravenstate_mark, model_ptC3, model_ptD1, CV_RGB(250,250,250),1);
		cv::line(ravenstate_mark, model_ptC3, model_ptE1, CV_RGB(250,250,250),1);
		cv::line(ravenstate_mark, model_ptD, model_ptD1, CV_RGB(250,250,250),1);
		cv::line(ravenstate_mark, model_ptE, model_ptE1, CV_RGB(250,250,250),1);

		cv::circle(ravenstate_mark, model_ptA, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
		cv::circle(ravenstate_mark, model_ptB, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
		cv::circle(ravenstate_mark, model_ptC, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
		cv::circle(ravenstate_mark, model_ptD, 3, CV_RGB(  0,255,0),0); // the point: ( Green, thickness 0, radius 3)
		cv::circle(ravenstate_mark, model_ptE, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
	}

	// the following are for depth map generation:   
	
	/*
	model_dep1 = model_ptA - depth_roi_width*normal_to_AB/(RAVEN_DIS[0]+1);
	model_dep3 = model_ptA + depth_roi_width*normal_to_AB/(RAVEN_DIS[0]+1); 
	model_dep2 = model_ptB - depth_roi_width*normal_to_AB/(RAVEN_DIS[1]+1);
	model_dep4 = model_ptB + depth_roi_width*normal_to_AB/(RAVEN_DIS[1]+1);
	model_dep5 = model_ptB - depth_roi_width*normal_to_BC/(RAVEN_DIS[1]+1);
	model_dep9 = model_ptB + depth_roi_width*normal_to_BC/(RAVEN_DIS[1]+1);
	model_dep7 = model_ptC - depth_roi_width*tangent_to_BC/(RAVEN_DIS[2]+1);
	model_dep6 = model_dep7 - depth_roi_width*normal_to_BC/(RAVEN_DIS[2]+1);
	model_dep8 = model_dep7 + depth_roi_width*normal_to_BC/(RAVEN_DIS[2]+1);
                           
	vector<Point> contour0,contour1,contour2;
	contour0.push_back(model_dep1); contour0.push_back(model_dep2); contour0.push_back(model_dep4); contour0.push_back(model_dep3);
	contour1.push_back(model_dep5); contour1.push_back(model_dep6); contour1.push_back(model_dep8); contour1.push_back(model_dep9);

	if(model_dep5.y<model_dep2.y)
	{
		contour2.push_back(model_dep5); 
		contour2.push_back(model_dep2); 
	}
	else
	{
		contour2.push_back(model_dep2); 
		contour2.push_back(model_dep5); 
	}

	if(model_dep4.y<model_dep9.y)
	{
		contour2.push_back(model_dep4); 
		contour2.push_back(model_dep9); 
	}
	else
	{
		contour2.push_back(model_dep9); 
		contour2.push_back(model_dep4); 
	}

	const cv::Point *pts0 = (const cv::Point*) Mat(contour0).data;
	const cv::Point *pts1 = (const cv::Point*) Mat(contour1).data;
	const cv::Point *pts2 = (const cv::Point*) Mat(contour2).data;
	int ncontours = 1;
	int* npts = new int[ncontours];
	npts[0] = 4;
	depth_roi_mask = cv::Mat(color.size(),CV_32FC1,cv::Scalar(0));
	cv::circle(depth_roi_mask, model_ptC, depth_roi_width/(RAVEN_DIS[2]+1), cv::Scalar(1),0);
	fillPoly(depth_roi_mask,&pts0,npts,ncontours,cv::Scalar(1),8,0); 
	fillPoly(depth_roi_mask,&pts1,npts,ncontours,cv::Scalar(1),8,0); 
	fillPoly(depth_roi_mask,&pts2,npts,ncontours,cv::Scalar(1),8,0); 
	depth_roi_mask.convertTo(depth_roi_mask, CV_8UC3);
	*/

	depth_roi_mask = cv::Mat(color.size(),CV_8UC1,cv::Scalar(255));
	depth_roi_width = DEPTBALL_WIDTH*OBJECT_SIZE_FACTOR/(RAVEN_DIS[2]+1);
	cv::circle(depth_roi_mask, model_ptC, depth_roi_width, cv::Scalar(0),CV_FILLED,0);
}



void MyTracker_DoSegmentation::prep_kinematics_probability()
{	
	double angle,minVal,maxVal;
	int index = RAVEN_POS.size()-1;
	Point minLoc,maxLoc; 

	double dx1 = RAVEN_POS[index].x-cameraTranslationVector.at<double>(0);
	double dy1 = RAVEN_POS[index].y-cameraTranslationVector.at<double>(1);
	double dz1 = RAVEN_POS[index].z-cameraTranslationVector.at<double>(2);

	double dx2 = color.cols/2-fabs(ptO.x-color.cols/2);
	double dy2 = color.rows/2-fabs(ptO.y-color.rows/2);

	distance_to_cam = sqrt(dx1*dx1+dy1*dy1+dz1*dz1);
	distance_to_edge = sqrt(dx2*dx2+dy2*dy2);
	
	// object size in image = Object size * focal length / object distance from camera
	object_size = OBJECT_SIZE_SCALE/(distance_to_cam+1);

	// the closer it is to the edge of image, we have less confidence (greater uncertainty)
	uncertainty_level = UNCERTAINTY_SCALE/(distance_to_edge+1);
	uncertainty_level = (uncertainty_level%2==0) ? uncertainty_level+1 : uncertainty_level;

	angle = angle_between(cv::Point(0,10),ptX-ptO);

	cv::RotatedRect rRect1 = RotatedRect(ptO, Size2f(object_size*2/2,object_size*2), angle);
	cv::RotatedRect rRect2 = RotatedRect(ptO, Size2f(object_size/2,object_size), angle);	
	cv::ellipse(ravenstate_prob, rRect1,CV_RGB(100,100,100),CV_FILLED);
	cv::ellipse(ravenstate_prob, rRect2,CV_RGB(255,255,255),CV_FILLED);

	//cv::blur(ravenstate_prob,ravenstate_prob,Size(uncertainty_level,uncertainty_level));
	cv::GaussianBlur(ravenstate_prob,ravenstate_prob, Size(uncertainty_level,uncertainty_level), 0, 0);
	
	// adjust the brightness
	minMaxLoc(ravenstate_prob, &minVal, &maxVal, &minLoc, &maxLoc);
	ravenstate_prob = ravenstate_prob*255/maxVal; 
}



void MyTracker_DoSegmentation::prep_motion_probability()
{
	Mat motion_roi;
	vector<vector<cv::Point> > contours;

	if(!gray_last.empty())
	{
		// (1) region of interest besed on kinematics
		Rect region_of_interest = Rect(std::max(0.0,PIXEL_POS[0].x-object_size),std::max(0.0, PIXEL_POS[0].y-object_size), 2*object_size, 2*object_size);
		Mat gray_roi = gray(region_of_interest);
		Mat gray_roi_last = gray_last(region_of_interest);
		
		// (2) compute absolute difference between subsequent images
		absdiff(gray_roi, gray_roi_last, motion_roi);

		// (3) draw raw motion on feature_thres
		motion_roi.copyTo(feature_thres(region_of_interest));
		cv::cvtColor(feature_thres,feature_thres,CV_GRAY2BGR);
		feature_thres = feature_thres + color*0.5;
		/*
		// not as stable...
		vector<cv::Mat> color_channels(3);
		vector<cv::Mat> motion_channels(3);

		split(color, color_channels);
		motion_channels.at(0) = color_channels[0];  //for blue channel
		motion_channels.at(1) = color_channels[1];  //for green channel
		motion_channels.at(2) = feature_prob;       //for red channel
		cv::merge(motion_channels, feature_thres);
		*/

		// (4) process the motion probability
		threshold(motion_roi, motion_roi, 80, 255, cv::THRESH_TOZERO);
 
		// (5) dilate to fill in holes, then find contours
		//dilate(motion_roi,motion_roi, Mat(),Point(-1,-1), 2); //anchor point(element center), iterations = 2 
		findContours(motion_roi, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		// (6) draw the contours
		for (size_t idx = 0; idx < contours.size(); idx++) 
		{
			if (contourArea(contours[idx])>100 && contourArea(contours[idx])<50000 )
				drawContours(motion_roi, contours, idx, cv::Scalar(255, 255, 255),CV_FILLED);
		}
		
		motion_roi.copyTo(feature_prob(region_of_interest));
	}
	gray_last = gray.clone();
}



void  MyTracker_DoSegmentation::draw_segmentation_result_ver3() 
{
	// (1) for the fourier transform part
	feature_thres = 0.5*color.clone();
	cv::line(feature_thres, model_ptA1+minLoc, model_ptB1+minLoc, CV_RGB(250,250,250),1); // model link: (Gray, thickness 1)
	cv::line(feature_thres, model_ptA2+minLoc, model_ptB2+minLoc, CV_RGB(250,250,250),1); // model link: (Gray, thickness 1)
	cv::line(feature_thres, model_ptA1+minLoc, model_ptA2+minLoc, CV_RGB(250,250,250),1); // model link: (Gray, thickness 1)
	cv::line(feature_thres, model_ptB1+minLoc, model_ptB2+minLoc, CV_RGB(250,250,250),1); // model link: (Gray, thickness 1)

	cv::line(feature_thres, model_ptB3+minLoc, model_ptC1+minLoc, CV_RGB(250,250,250),1);
	cv::line(feature_thres, model_ptB4+minLoc, model_ptC2+minLoc, CV_RGB(250,250,250),1);
	cv::line(feature_thres, model_ptB3+minLoc, model_ptB4+minLoc, CV_RGB(250,250,250),1);
	cv::line(feature_thres, model_ptC1+minLoc, model_ptC2+minLoc, CV_RGB(250,250,250),1);

	cv::line(feature_thres, model_ptC+minLoc, model_ptC3+minLoc, CV_RGB(250,250,250),1);
	cv::line(feature_thres, model_ptC+minLoc, model_ptD+minLoc, CV_RGB(250,250,250),1);
	cv::line(feature_thres, model_ptC+minLoc, model_ptE+minLoc, CV_RGB(250,250,250),1);
	cv::line(feature_thres, model_ptC3+minLoc, model_ptD1+minLoc, CV_RGB(250,250,250),1);
	cv::line(feature_thres, model_ptC3+minLoc, model_ptE1+minLoc, CV_RGB(250,250,250),1);
	cv::line(feature_thres, model_ptD+minLoc, model_ptD1+minLoc, CV_RGB(250,250,250),1);
	cv::line(feature_thres, model_ptE+minLoc, model_ptE1+minLoc, CV_RGB(250,250,250),1);

	cv::circle(feature_thres, model_ptA+minLoc, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
	cv::circle(feature_thres, model_ptB+minLoc, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
	cv::circle(feature_thres, model_ptC+minLoc, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)
	cv::circle(feature_thres, model_ptD+minLoc, 3, CV_RGB(  0,255,0),0); // the point: ( Green, thickness 0, radius 3)
	cv::circle(feature_thres, model_ptE+minLoc, 3, CV_RGB(255,255,0),0); // the point: (Yellow, thickness 0, radius 3)

	// (2) for final segmentation result    
	color.convertTo(segment_fore, CV_32FC3);
	segment_fore = segment_fore.mul(segment_mask); 
	segment_fore.convertTo(segment_fore, CV_8UC3);

	color.convertTo(segment_back, CV_32FC3);
	segment_back = segment_back.mul(segment_mask_inv); 
	segment_back.convertTo(segment_back, CV_8UC3);
}



void  MyTracker_DoSegmentation::draw_segmentation_result_ver4() 
{
	// for final segmentation result    
	color.convertTo(segment_fore, CV_32FC3);
	segment_fore = segment_fore.mul(segment_mask); 
	segment_fore.convertTo(segment_fore, CV_8UC3);

	color.convertTo(segment_back, CV_32FC3);
	segment_back = segment_back.mul(segment_mask_inv); 
	segment_back.convertTo(segment_back, CV_8UC3);

	// for final depth lattice result
	// (1) make sure there is disparity result for the current frame
	if(disparity.size()!=gray.size())
			disparity = cv::Mat(color.size(),CV_32FC1,cv::Scalar(0)); 
	else
		disparity.convertTo(disparity, CV_32FC1);

	// (2) make sure disparity is converted to color
	if(disparity.channels() == 1)
		cvtColor(disparity, disparity, CV_GRAY2BGR); 


	disparity.convertTo(depth_lattice, CV_32FC3);
	if(show_lattice_points_only)
	{
		// (3) prepare a depth lattice mask
		Mat segment_mask_tmp;
		cvtColor(segment_mask, segment_mask_tmp, CV_BGR2GRAY); 
	
		for(int i=2; i<segment_mask_tmp.rows-2; i+=5)
		for(int j=2; j<segment_mask_tmp.cols-2; j+=5)
		{
			segment_mask_tmp.at<float>(i+1,   j) = 1.0; 
			segment_mask_tmp.at<float>(  i,   j) = 1.0; 
			segment_mask_tmp.at<float>(i-1,   j) = 1.0;

			segment_mask_tmp.at<float>(i+1, j+1) = 1.0; 
			segment_mask_tmp.at<float>(  i, j+1) = 1.0; 
			segment_mask_tmp.at<float>(i-1, j+1) = 1.0;

			segment_mask_tmp.at<float>(i+1, j-1) = 1.0; 
			segment_mask_tmp.at<float>(  i, j-1) = 1.0; 
			segment_mask_tmp.at<float>(i-1, j-1) = 1.0; 
		}

		if(segment_mask_tmp.channels() == 1)
			cvtColor(segment_mask_tmp, segment_mask_tmp, CV_GRAY2BGR); 

		// (4) compute result
		depth_lattice = depth_lattice.mul(segment_mask_tmp);
		
	}
	else
		depth_lattice = disparity.clone();

	// (5) convert to the compatible type
	depth_lattice.convertTo(depth_lattice, CV_8UC3);	
}



void MyTracker_DoSegmentation::draw_dft_result_ver3(Mat E_t)
{
	// only overlay U and E_t

	// (1) for the time domain E_t result ovelay
	combined_prob = 0.5*ravenstate_prob.clone();
	cvtColor(combined_prob,combined_prob,CV_GRAY2BGR);


	int ncontours = 1;
	int* npts = new int[ncontours];
	npts[0] = 4;
		
	vector<Point> contour0,contour1,contour2,contour3;
	contour0.push_back(model_ptA1+minLoc); contour0.push_back(model_ptA2+minLoc); contour0.push_back(model_ptB2+minLoc); contour0.push_back(model_ptB1+minLoc);
	contour1.push_back(model_ptB3+minLoc); contour1.push_back(model_ptB4+minLoc); contour1.push_back(model_ptC2+minLoc); contour1.push_back(model_ptC1+minLoc);
	contour2.push_back(model_ptC3+minLoc); contour2.push_back(model_ptD1+minLoc); contour2.push_back(model_ptD+minLoc);  contour2.push_back(model_ptC+minLoc);
	contour3.push_back(model_ptC3+minLoc); contour3.push_back(model_ptE1+minLoc); contour3.push_back(model_ptE+minLoc);  contour3.push_back(model_ptC+minLoc);

	// create a pointer to the data as an array of points (via a conversion to a Mat object)
	const cv::Point *pts0 = (const cv::Point*) Mat(contour0).data;
	const cv::Point *pts1 = (const cv::Point*) Mat(contour1).data;
	const cv::Point *pts2 = (const cv::Point*) Mat(contour2).data;
	const cv::Point *pts3 = (const cv::Point*) Mat(contour3).data;

	fillPoly(combined_prob,&pts0,npts,ncontours,cv::Scalar(255,0,0),8,0); // linetype=8, shift=0
	fillPoly(combined_prob,&pts1,npts,ncontours,cv::Scalar(255,0,0),8,0); 
	fillPoly(combined_prob,&pts2,npts,ncontours,cv::Scalar(255,0,0),8,0); 
	fillPoly(combined_prob,&pts3,npts,ncontours,cv::Scalar(255,0,0),8,0); 

	// (2) for the segmentation mask
	segment_mask = cv::Mat(color.size(),CV_32FC1,cv::Scalar(0));
	fillPoly(segment_mask,&pts0,npts,ncontours,cv::Scalar(1),8,0); // 1. consider the ravenstate information
	fillPoly(segment_mask,&pts1,npts,ncontours,cv::Scalar(1),8,0); 
	fillPoly(segment_mask,&pts2,npts,ncontours,cv::Scalar(1),8,0); 
	fillPoly(segment_mask,&pts3,npts,ncontours,cv::Scalar(1),8,0); 

	Mat segment_mask_raw = segment_mask.clone();
	dilate(segment_mask,segment_mask, Mat(),Point(-1,-1), 1);  // 2. make it fatter (faded outward)
	blur(segment_mask,segment_mask,Size(SEG_BLURY_SCALE,SEG_BLURY_SCALE));

	segment_mask = SEG_BLURY_INTENSITY/10.0*segment_mask + segment_mask_raw;
	threshold(segment_mask, segment_mask, 1, 1+SEG_BLURY_INTENSITY/10.0, cv::THRESH_TRUNC);
	
	cvtColor(255*segment_mask, segment_mask1, CV_GRAY2BGR); 
	segment_mask1.convertTo(segment_mask1, CV_8UC3); 

	Mat inv_feature_prob = 260-feature_prob;
	inv_feature_prob.convertTo(inv_feature_prob, CV_32FC1);  // 3. consider the opponent color filtering

	segment_mask = segment_mask.mul(inv_feature_prob); // combine the results together

	cvtColor(segment_mask, segment_mask2, CV_GRAY2BGR); 
	segment_mask2.convertTo(segment_mask2, CV_8UC3); 

	threshold(segment_mask, segment_mask, SEG_THRES_LOWBOUND, 260, cv::THRESH_BINARY);  // 4. binary thresholding
	//threshold(segment_mask, segment_mask, 220, 260, cv::THRESH_TRUNC); // alternatively, apply gradient effect

	normalize(segment_mask, segment_mask, 0, 1, CV_MINMAX); 

	segment_mask.convertTo(segment_mask, CV_8UC1);
	vector<vector<cv::Point> > contours;
	findContours(segment_mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// 5. find and draw the contours


	for (size_t idx = 0; idx < contours.size(); idx++) 
		drawContours(segment_mask, contours, idx, cv::Scalar(1),CV_FILLED);

	cvtColor(255*segment_mask, segment_mask3, CV_GRAY2BGR); 
	segment_mask3.convertTo(segment_mask3, CV_8UC3); 

	Mat element = getStructuringElement(MORPH_RECT, Size(SEG_FILL_BLACKSPOT, SEG_FILL_BLACKSPOT), Point(-1,-1) ); // 6. fill in the black spots
	morphologyEx(segment_mask, segment_mask, CV_MOP_CLOSE, element);

	segment_mask.convertTo(segment_mask, CV_32FC1);
	segment_mask_inv = 1-segment_mask;
	cvtColor(segment_mask, segment_mask, CV_GRAY2BGR); 
	cvtColor(segment_mask_inv, segment_mask_inv, CV_GRAY2BGR); 

	// (3) for the freq domain E_t
	E_t.convertTo(energyfunc_freq, CV_8UC1, 255.0); 
	cvtColor(energyfunc_freq, energyfunc_freq, CV_GRAY2BGR);     
	energyfunc_freq.convertTo(energyfunc_freq, CV_8UC3); 
	
	cv::line(energyfunc_freq, Point(0,E_t.rows/2), Point(E_t.cols,E_t.rows/2), CV_RGB(250,0,0),1);
	cv::line(energyfunc_freq, Point(E_t.cols/2,0), Point(E_t.cols/2,E_t.rows), CV_RGB(250,0,0),1);
	resize(energyfunc_freq,energyfunc_freq,gray.size());

}



void MyTracker_DoSegmentation::draw_dft_result_ver4()
{
	int ncontours = 1;
	int* npts = new int[ncontours];
	npts[0] = 4;
		
	vector<Point> contour0,contour1,contour2,contour3;
	contour0.push_back(model_ptA1+minLoc); contour0.push_back(model_ptA2+minLoc); contour0.push_back(model_ptB2+minLoc); contour0.push_back(model_ptB1+minLoc);
	contour1.push_back(model_ptB3+minLoc); contour1.push_back(model_ptB4+minLoc); contour1.push_back(model_ptC2+minLoc); contour1.push_back(model_ptC1+minLoc);
	contour2.push_back(model_ptC3+minLoc); contour2.push_back(model_ptD1+minLoc); contour2.push_back(model_ptD+minLoc);  contour2.push_back(model_ptC+minLoc);
	contour3.push_back(model_ptC3+minLoc); contour3.push_back(model_ptE1+minLoc); contour3.push_back(model_ptE+minLoc);  contour3.push_back(model_ptC+minLoc);

	// create a pointer to the data as an array of points (via a conversion to a Mat object)
	const cv::Point *pts0 = (const cv::Point*) Mat(contour0).data;
	const cv::Point *pts1 = (const cv::Point*) Mat(contour1).data;
	const cv::Point *pts2 = (const cv::Point*) Mat(contour2).data;
	const cv::Point *pts3 = (const cv::Point*) Mat(contour3).data;

	// for the segmentation mask
	segment_mask = cv::Mat(color.size(),CV_32FC1,cv::Scalar(0));
	fillPoly(segment_mask,&pts0,npts,ncontours,cv::Scalar(1),8,0); // 1. consider the ravenstate information
	fillPoly(segment_mask,&pts1,npts,ncontours,cv::Scalar(1),8,0); 
	fillPoly(segment_mask,&pts2,npts,ncontours,cv::Scalar(1),8,0); 
	fillPoly(segment_mask,&pts3,npts,ncontours,cv::Scalar(1),8,0); 

	Mat segment_mask_raw = segment_mask.clone();
	dilate(segment_mask,segment_mask, Mat(),Point(-1,-1), 1);  // 2. make it fatter (faded outward)
	blur(segment_mask,segment_mask,Size(SEG_BLURY_SCALE,SEG_BLURY_SCALE));

	segment_mask = SEG_BLURY_INTENSITY/10.0*segment_mask + segment_mask_raw;
	threshold(segment_mask, segment_mask, 1, 1+SEG_BLURY_INTENSITY/10.0, cv::THRESH_TRUNC);

	Mat inv_feature_prob = 260-feature_prob;
	inv_feature_prob.convertTo(inv_feature_prob, CV_32FC1);  // 3. consider the opponent color filtering

	segment_mask = segment_mask.mul(inv_feature_prob); // combine the results together

	threshold(segment_mask, segment_mask, SEG_THRES_LOWBOUND, 260, cv::THRESH_BINARY);  // 4. binary thresholding
	//threshold(segment_mask, segment_mask, 220, 260, cv::THRESH_TRUNC); // alternatively, apply gradient effect

	normalize(segment_mask, segment_mask, 0, 1, CV_MINMAX); 

	segment_mask.convertTo(segment_mask, CV_8UC1);
	vector<vector<cv::Point> > contours;
	findContours(segment_mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// 5. find and draw the contours


	for (size_t idx = 0; idx < contours.size(); idx++) 
		drawContours(segment_mask, contours, idx, cv::Scalar(1),CV_FILLED); 

	Mat element = getStructuringElement(MORPH_RECT, Size(SEG_FILL_BLACKSPOT, SEG_FILL_BLACKSPOT), Point(-1,-1) ); // 6. fill in the black spots
	morphologyEx(segment_mask, segment_mask, CV_MOP_CLOSE, element);

	segment_mask.convertTo(segment_mask, CV_32FC1);
	segment_mask_inv = 1-segment_mask;
	cvtColor(segment_mask, segment_mask, CV_GRAY2BGR); 
	cvtColor(segment_mask_inv, segment_mask_inv, CV_GRAY2BGR); 
}


void MyTracker_DoSegmentation::draw_segmentation_colorwheel(bool do_it)
{
	if(do_it)
	{
		Mat colorwheel_mask, inv_colorwheel_mask;
		Mat inv_feature_prob = 260-colorwheel_oppo;
		inv_feature_prob.convertTo(inv_feature_prob, CV_32FC1);  // consider the opponent color filtering

		threshold(inv_feature_prob, inv_feature_prob, SEG_THRES_LOWBOUND, 260, cv::THRESH_BINARY);  // binary thresholding

		normalize(inv_feature_prob, inv_feature_prob, 0, 1, CV_MINMAX); 

		inv_feature_prob.convertTo(inv_feature_prob, CV_32FC1);
		colorwheel_mask = inv_feature_prob;
		inv_colorwheel_mask = 1-colorwheel_mask;
	
		cvtColor(colorwheel_mask, colorwheel_mask, CV_GRAY2BGR); 
		cvtColor(inv_colorwheel_mask, inv_colorwheel_mask, CV_GRAY2BGR); 

		Mat tmp;
		colorwheel.convertTo(tmp, CV_32FC3);
		colorwheel_mask = tmp.mul(colorwheel_mask); 
		colorwheel_mask.convertTo(colorwheel_mask, CV_8UC3);
		inv_colorwheel_mask = tmp.mul(inv_colorwheel_mask); 
		inv_colorwheel_mask.convertTo(inv_colorwheel_mask, CV_8UC3);

		int dstWidth = colorwheel.cols*3;
		int dstHeight =colorwheel.rows;
		cv::Mat dstMat = cv::Mat(dstHeight, dstWidth, CV_8UC3, cv::Scalar(0,0,0));

		cv::Rect roi11(cv::Rect(           0,       0, colorwheel.cols, colorwheel.rows));
		cv::Rect roi12(cv::Rect(  colorwheel.cols,  0, colorwheel.cols, colorwheel.rows));
		cv::Rect roi13(cv::Rect(2*colorwheel.cols,  0, colorwheel.cols, colorwheel.rows));

		cv::Mat targetROI11 = dstMat(roi11);
		cv::Mat targetROI12 = dstMat(roi12);
		cv::Mat targetROI13 = dstMat(roi13);

			 colorwheel.copyTo(targetROI11); 
		    colorwheel_mask.copyTo(targetROI12); 
		inv_colorwheel_mask.copyTo(targetROI13); 
	
		// add text to image
		putText(dstMat, "colorwheel", Point(5+0*colorwheel.cols, 25+0*colorwheel.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
		putText(dstMat, "tool colors",Point(5+1*colorwheel.cols, 25+0*colorwheel.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
		putText(dstMat, "bg colors",  Point(5+2*colorwheel.cols, 25+0*colorwheel.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	
		cv::imshow("segmented colorwheel",dstMat);
		cvWaitKey(30);	
	}
	
}

void MyTracker_DoSegmentation::draw_dft_freqplots(Mat complex_u,Mat complex_Q)
{
	// (1) compute the magnitude
	Mat planes_u[2] = {Mat::zeros(dftSize, CV_32F), Mat::zeros(dftSize, CV_32F)};     // log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
	Mat planes_Q[2] = {Mat::zeros(dftSize, CV_32F), Mat::zeros(dftSize, CV_32F)};

	split(complex_u, planes_u);  // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
	split(complex_Q, planes_Q);  

	magnitude(planes_u[0], planes_u[1], planes_u[0]);// planes[0] = magnitude
	magnitude(planes_Q[0], planes_Q[1], planes_Q[0]);// planes[0] = magnitude
	Mat mag_u = planes_u[0];
	Mat mag_Q = planes_Q[0];

	// (2) switch to logarithmic scale
	mag_u += Scalar::all(1);
	mag_Q += Scalar::all(1);
	log(mag_u, mag_u);
	log(mag_Q, mag_Q);

	// (3) crop the spectrum, if it has an odd number of rows or columns
	mag_u = mag_u(Rect(0, 0, mag_u.cols & -2, mag_u.rows & -2));
	mag_Q = mag_Q(Rect(0, 0, mag_Q.cols & -2, mag_Q.rows & -2));

	// (4) move origin to the image center
	int cx = mag_u.cols/2;
	int cy = mag_u.rows/2;

	Mat q0_u(mag_u, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
	Mat q1_u(mag_u, Rect(cx, 0, cx, cy));  // Top-Right
	Mat q2_u(mag_u, Rect(0, cy, cx, cy));  // Bottom-Left
	Mat q3_u(mag_u, Rect(cx, cy, cx, cy)); // Bottom-Right
	Mat q0_Q(mag_Q, Rect(0, 0, cx, cy));   
	Mat q1_Q(mag_Q, Rect(cx, 0, cx, cy));  
	Mat q2_Q(mag_Q, Rect(0, cy, cx, cy));  
	Mat q3_Q(mag_Q, Rect(cx, cy, cx, cy)); 

	Mat tmp;                           
	q0_u.copyTo(tmp); q3_u.copyTo(q0_u); tmp.copyTo(q3_u); // swap quadrants (Top-Left with Bottom-Right)
	q1_u.copyTo(tmp); q2_u.copyTo(q1_u); tmp.copyTo(q2_u); // swap quadrants (Top-Right with Bottom-Left)
	q0_Q.copyTo(tmp); q3_Q.copyTo(q0_Q); tmp.copyTo(q3_Q); 
	q1_Q.copyTo(tmp); q2_Q.copyTo(q1_Q); tmp.copyTo(q2_Q); 

	// (5) normalize to 0~1 grayscale image
	normalize(mag_u, mag_u, 0, 1, CV_MINMAX); 
	normalize(mag_Q, mag_Q, 0, 1, CV_MINMAX); 

	// (6) resize for display
	resize(mag_u,mag_u,gray.size());
	resize(mag_Q,mag_Q,gray.size());
	cvtColor(mag_u, mag_u, CV_GRAY2BGR);
	cvtColor(mag_Q, mag_Q, CV_GRAY2BGR);     
	mag_u.convertTo(ravenstate_freq, CV_8UC3, 255.0); 
	mag_Q.convertTo(feature_freq, CV_8UC3, 255.0);

	minLoc.x = (minLoc.x)/fourier_resize;
	minLoc.y = (minLoc.y)/fourier_resize;
}



void MyTracker_DoSegmentation::start_segmentation()
{
	// image segmentation :P
	// think what segmentation algorithm to use?
	// how to use "probability" map to help prioritize our search?

	// 1. prepare Q (and zero padding)
	// 2. prepare u (and zero padding)
	//    (1) calculate the size of DFT transform (should be already done)
	//    (2) normalize to 0~1 grayscale image
	Mat tmp_u, tmp_Q;
	resize(ravenstate_prob,tmp_u,Size(color.cols*fourier_resize,color.rows*fourier_resize));
	resize(feature_prob,tmp_Q,Size(color.cols*fourier_resize,color.rows*fourier_resize));
	normalize(tmp_u, tmp_u, 0, 1, NORM_MINMAX);
	normalize(tmp_Q, tmp_Q, 0, 1, NORM_MINMAX);

	//    (3) create target image and initialize them with 0's
	Mat u,Q;
	cv::copyMakeBorder(tmp_u, u,0,dftSize.height-tmp_u.rows,0,dftSize.width-tmp_u.cols,BORDER_CONSTANT,Scalar::all(0));
	cv::copyMakeBorder(tmp_Q, Q,0,dftSize.height-tmp_Q.rows,0,dftSize.width-tmp_Q.cols,BORDER_CONSTANT,Scalar::all(1));

	u.convertTo(u, CV_32F); 
	Q.convertTo(Q, CV_32F); 

	//    (4) prepare for fourier transform output
	Mat complex_u, complex_Q;
	Mat planes_u[] = {Mat_<float>(u), Mat::zeros(dftSize, CV_32F)};
	Mat planes_Q[] = {Mat_<float>(Q), Mat::zeros(dftSize, CV_32F)};
	merge(planes_u, 2, complex_u);
	merge(planes_Q, 2, complex_Q);

	// 3. fourier transform
 	dft(complex_u, complex_u,0,tmp_u.rows); // use "nonzeroRows" hint for faster processing
	dft(complex_Q, complex_Q,0,tmp_Q.rows);

	// 4. shape fitting loop 
	//    (1) translation/shifting
	cv::Mat E_t;
	cv::idft(complex_u.mul(complex_Q),E_t, DFT_SCALE | DFT_REAL_OUTPUT ); // compute the energy function
	//    (2) rotation and scale
	//    (3) deformation (optional: probably not for now)

	//    (4) compute final energy function
	resize(E_t ,E_t ,gray.size());
	E_t = compute_shifting_result(E_t);  // the result if saved to "minLoc"

	// 5. deal with duality/symmetry issue
	int min_value = compute_which_direction(tmp_Q);

	// 6. ensure smooth trajectory Walalaa
	//compute_kalman_filter();  // method 1
	//compute_smooth_traj(tmp_Q, min_value); // method 2

	// 6. final touch: color filter...?
	
	// 7. prepare display (the drawing part)

	switch(version)
	{
		case 3:
			draw_dft_freqplots(complex_u,complex_Q); 
			draw_dft_result_ver3(E_t);
			draw_segmentation_result_ver3();
			break;
		case 4:
			draw_dft_freqplots(complex_u,complex_Q); 
			draw_dft_result_ver4();
			draw_segmentation_result_ver4();
			break;
	}

}



void MyTracker_DoSegmentation::reset_intermediate_images()
{
	ravenstate_mark = color.clone()*0.5;
	
	feature_thres = cv::Mat(gray.rows, gray.cols, gray.type(), cv::Scalar(0,0,0));
	ravenstate_prob = cv::Mat(gray.rows, gray.cols, gray.type(), cv::Scalar(0,0,0)); 
	feature_prob = cv::Mat(gray.rows, gray.cols, gray.type(), cv::Scalar(0,0,0));
	combined_prob = cv::Mat(gray.rows, gray.cols, gray.type(), cv::Scalar(0,0,0));
}



void MyTracker_DoSegmentation::prep_log_likelihood_Q()
{
	static cv::Mat gray_last = gray.clone();
	static Point2d LAST_PIXEL_POS = PIXEL_POS[0];

	// Method One: use color filtering
	// (1) the Opponent color space
	Mat BGR[3], HSV[3]; 
	Mat Oppo1, Oppo2, Oppo2_Oppo1, Color_diff, hsv_color, Y;

	split(color,BGR); // split source into independent color channels
	normalize(255+BGR[1] - BGR[2], Oppo1, 0, 255, NORM_MINMAX, CV_8UC1); 
	normalize(BGR[1] + BGR[2],     Y, 0, 255, NORM_MINMAX, CV_8UC1); 
	normalize(255+BGR[0] -      Y, Oppo2, 0, 255, NORM_MINMAX, CV_8UC1); 
	normalize(255+Oppo2 - Oppo1, Oppo2_Oppo1, 0, 255, NORM_MINMAX, CV_8UC1); 	
	/*
	Point2d POS_DIFFERENCE = LAST_PIXEL_POS-PIXEL_POS[0];
	if(static_background && abs(POS_DIFFERENCE.x)+abs(POS_DIFFERENCE.y)>10)
	{
		normalize(gray - gray_last, feature_prob, 0, 255, NORM_MINMAX, CV_8UC1);
		feature_prob = cv::Scalar::all(255) - feature_prob;
		feature_prob = feature_prob.mul(Oppo2_Oppo1);
		normalize(feature_prob, feature_prob, 0, 255, NORM_MINMAX, CV_8UC1);
	}
	else
		feature_prob = Oppo2_Oppo1;*/ 

	feature_prob = Oppo2_Oppo1; //Walalaa

	// (2) the difference to tissue color
	//Mat diff_B = (BGR[0] - cv::Scalar::all(tissue_mean_B));
	//Mat diff_G = (BGR[1] - cv::Scalar::all(tissue_mean_G));
	//Mat diff_R = (BGR[2] - cv::Scalar::all(tissue_mean_R));
	//normalize(diff_B.mul(diff_B)+diff_G.mul(diff_G)+diff_R.mul(diff_R), Color_diff, 0, 255, NORM_MINMAX);

	// (3) hue in hsv color space
	//cvtColor(color, hsv_color, CV_BGR2HSV);
	//split(hsv_color,HSV); // split source into independent color channels

	// (4) combine the result
	//feature_prob = (HSV[0] + Color_diff + Oppo2_Oppo1)/2;
	//normalize(feature_prob, feature_prob, 0, 255, NORM_MINMAX);

	// Method Two: use edges detection (not performing as well, so we gave this up)
	/*
	//(1) change color map from RGB to medical imaging friendly one (Todo: lookup and implement)
	Mat new_colormap_gray = gray.clone();

	//(2) get edges by sobel
	int scale = 1;       // optional scale factor for the computed derivative values
	int delta = 0;       // optional delta value that is added to the results prior to storing them in dst
	int ksize = 3;       // the kernel size
	int ddepth = CV_32F; // the output image depth

	double minVal,maxVal;
	Point minLoc, maxLoc;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	Sobel(new_colormap_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT); // Gradient X: show vertical lines
	Sobel(new_colormap_gray, grad_y, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT); // Gradient Y: show horizontal lines
	convertScaleAbs(grad_x, abs_grad_x);
	convertScaleAbs(grad_y, abs_grad_y);

	//(3) generate edges plots with specific directions (according to ravenstate)
	double angle_AB = atan2(model_ptA.y-model_ptB.y,model_ptA.x-model_ptB.x);
	double angle_BC = atan2(model_ptB.y-model_ptC.y,model_ptB.x-model_ptC.x);
	double angle_CD = atan2(model_ptC.y-model_ptD.y,model_ptC.x-model_ptD.x);
	double angle_CE = atan2(model_ptC.y-model_ptE.y,model_ptC.x-model_ptE.x);
	
	Mat grad, theta;
	cv::phase(grad_x,grad_y,theta,false); // angle-to-degree = false
	cv::sqrt(grad_x.mul(grad_x) + grad_y.mul(grad_y),grad);
	threshold(grad, grad, 80, 255, cv::THRESH_TOZERO);	
	//addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);  // not as good
	
	Mat theta_AB = cv::Mat(grad.rows, grad.cols, grad.type(), cv::Scalar(0,0,0));
	Mat theta_BC = cv::Mat(grad.rows, grad.cols, grad.type(), cv::Scalar(0,0,0));
	Mat theta_CD = cv::Mat(grad.rows, grad.cols, grad.type(), cv::Scalar(0,0,0));
	Mat theta_CE = cv::Mat(grad.rows, grad.cols, grad.type(), cv::Scalar(0,0,0));
	

	// Todo: avoid elementwise ... so slow and not working well ... 
	//for (int r = 0; r < gray.rows; r++)
	//for (int c = 0; c < gray.cols; c++)
	//{
	//	theta_AB.at<float>(r,c) = fabs(cos(theta.at<float>(r,c)-angle_AB));  
	//	theta_BC.at<float>(r,c) = fabs(cos(theta.at<float>(r,c)-angle_BC));
	//	theta_CD.at<float>(r,c) = fabs(cos(theta.at<float>(r,c)-angle_CD));
	//	theta_CE.at<float>(r,c) = fabs(cos(theta.at<float>(r,c)-angle_CE));
	//}
	

	//Mat grad_directional_AB = grad.mul(theta_AB);
	//Mat grad_directional_BC = grad.mul(theta_BC);
	//Mat grad_directional_CD = grad.mul(theta_CD);
	//Mat grad_directional_CE = grad.mul(theta_CE);


	//(4) edges to approximate lines (using HoughLine)
	//void HoughLinesP(InputArray image, OutputArray lines, double rho, double theta, int threshold, double minLineLength=0, double maxLineGap=0 )

	//(5) threshold/dilate... other processing tricks?
	
	dilate(grad,grad, Mat(),Point(-1,-1), 1); //anchor point(element center), iterations = 2 

	//(6) find contour and fill contour?

	//vector<vector<cv::Point> > contours;
	//findContours(grad, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//draw the contours
	//for (size_t idx = 0; idx < contours.size(); idx++) 
	//{
	//	drawContours(grad, contours, idx, cv::Scalar(255, 255, 255),CV_FILLED);
	//}


	//(7) color 

	//(8) prepare Q (adjust the brightness)
	minMaxLoc(grad, &minVal, &maxVal, &minLoc, &maxLoc);
	feature_thres = grad*255/maxVal; 	
	feature_thres.convertTo(feature_thres, gray.type());

	feature_prob = feature_thres.clone(); 
	*/
	gray_last = gray.clone();
	LAST_PIXEL_POS = PIXEL_POS[0];
}



void MyTracker_DoSegmentation::prep_reference_shape_U()
{
	// goal: fill in "ravenstate_prob" with ravenstate shape model 
	int ncontours = 1;
	int* npts = new int[ncontours];
	npts[0] = 4;
		
	vector<Point> contour0,contour1,contour2,contour3;
	contour0.push_back(model_ptA1); contour0.push_back(model_ptA2); contour0.push_back(model_ptB2); contour0.push_back(model_ptB1);
	contour1.push_back(model_ptB3); contour1.push_back(model_ptB4); contour1.push_back(model_ptC2); contour1.push_back(model_ptC1);
	contour2.push_back(model_ptC3); contour2.push_back(model_ptD1); contour2.push_back(model_ptD); contour2.push_back(model_ptC);
	contour3.push_back(model_ptC3); contour3.push_back(model_ptE1); contour3.push_back(model_ptE); contour3.push_back(model_ptC);

	// create a pointer to the data as an array of points (via a conversion to a Mat object)
	const cv::Point *pts0 = (const cv::Point*) Mat(contour0).data;
	const cv::Point *pts1 = (const cv::Point*) Mat(contour1).data;
	const cv::Point *pts2 = (const cv::Point*) Mat(contour2).data;
	const cv::Point *pts3 = (const cv::Point*) Mat(contour3).data;

	fillPoly(ravenstate_prob,&pts0,npts,ncontours,cv::Scalar(255,255,255),8,0); // linetype=8, shift=0
	fillPoly(ravenstate_prob,&pts1,npts,ncontours,cv::Scalar(255,255,255),8,0); 
	fillPoly(ravenstate_prob,&pts2,npts,ncontours,cv::Scalar(255,255,255),8,0); 
	fillPoly(ravenstate_prob,&pts3,npts,ncontours,cv::Scalar(255,255,255),8,0); 	
}



void MyTracker_DoSegmentation::prep_disparity_map()
{
	// Disparity 1: Simpler, More Visible, but noisier
	// https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/stereoBM/SBM_Sample.cpp

	static bool first = true;
	cout<<weirdstring; // the code sometimes breaks without it, so do not delete me!
	weirdstring = "";

	int targetX, targetY;
	Vec3f reproj_offset = (0,0,0);
	Vec3f pt2cam = (0,0,0);
	Vec3f pt2tip = (0,0,0);
	Vec3f tooltip_loc = Vec3f(TOOLPOS_WRT_CAM.x,TOOLPOS_WRT_CAM.y,TOOLPOS_WRT_CAM.z);
	Point maxLoc, minLoc;
	double minVal, maxVal;

	// (1) Decide images ROI
	int leftbound = min(max(model_ptC.x-3*depth_roi_width,0),gray.cols-1); // because there are black columns on the left of the image ROI
	int topbound =  min(max(model_ptC.y-2*depth_roi_width,0),gray.rows-1);
	int idealwidth = 5*depth_roi_width;
	int idealheight = 4*depth_roi_width;
	int w = min(idealwidth,gray.cols-leftbound-1);
	int h = min(idealheight,gray.rows-topbound-1);
	
	Rect region_of_interest = Rect(leftbound,topbound,w,h);
	Mat gray_ROI, grayN_ROI, gray_ROI_flip, grayN_ROI_flip, gray_recti, grayN_recti, tmp;
	
	Mat warp_mat( 2, 3, CV_64FC1);
	warp_mat.at<double>(0,0) = 1.0;	warp_mat.at<double>(0,1) = 0.0;	warp_mat.at<double>(0,2) = 0.0;
	warp_mat.at<double>(1,0) = 0.0;	warp_mat.at<double>(1,1) = 1.0;	warp_mat.at<double>(1,2) = rectification_offset;

	switch(image_of_use)
	{
		case 0:
			if(rectify_image) // use rectified image
			{
				if(first)
				{
					initUndistortRectifyMap( intrinsic,  distortion,  rectification,  intrinsic, gray.size(), CV_16SC2, rmap[0][0], rmap[0][1]);
					initUndistortRectifyMap(intrinsicN, distortionN, rectificationN, intrinsicN, gray.size(), CV_16SC2, rmap[1][0], rmap[1][1]);
					first = false;
				}
				remap(gray, gray_recti, rmap[0][0], rmap[0][1], INTER_LINEAR);
				remap(grayN, tmp, rmap[1][0], rmap[1][1], INTER_LINEAR);

				cv::warpAffine(tmp,grayN_recti, warp_mat,tmp.size());

				if(compute_roi_mode == 0)
				{
					gray_ROI = gray_recti;
					grayN_ROI = grayN_recti;
				}
				else
				{
					gray_ROI = gray_recti(region_of_interest);
					grayN_ROI = grayN_recti(region_of_interest); 
				}
			}
			else // use unrectified image 
			{
				if(compute_roi_mode == 0)
				{
					gray_ROI = gray;
					grayN_ROI = grayN;
				}
				else
				{
					gray_ROI = gray(region_of_interest);
					grayN_ROI = grayN(region_of_interest); 
				}
			}
			break;
		case 1:
			gray_ROI = imread("/home/biorobotics/catkin_ws/src/myproject/camera_info/pictures/towerleft.jpeg");
			grayN_ROI = imread("/home/biorobotics/catkin_ws/src/myproject/camera_info/pictures/towerright.jpeg");
			cvtColor(gray_ROI,  gray_ROI,  COLOR_BGR2GRAY);
			cvtColor(grayN_ROI, grayN_ROI, COLOR_BGR2GRAY);
			break;
		case 2:
			gray_ROI = imread("/home/biorobotics/catkin_ws/src/myproject/camera_info/pictures/sculptureleft.png");
			grayN_ROI = imread("/home/biorobotics/catkin_ws/src/myproject/camera_info/pictures/sculptureright.png");
			cvtColor(gray_ROI,  gray_ROI,  COLOR_BGR2GRAY);
			cvtColor(grayN_ROI, grayN_ROI, COLOR_BGR2GRAY);
			break;
		case 3:
			gray_ROI = imread("/home/biorobotics/catkin_ws/src/myproject/camera_info/pictures/aloeleft.png");
			grayN_ROI = imread("/home/biorobotics/catkin_ws/src/myproject/camera_info/pictures/aloeright.png");  
			
			// need to resize for this picture
			resize( gray_ROI, gray_ROI,Size(),0.5,0.5);
			resize(grayN_ROI,grayN_ROI,Size(),0.5,0.5);
			cvtColor(gray_ROI,  gray_ROI,  COLOR_BGR2GRAY);
			cvtColor(grayN_ROI, grayN_ROI, COLOR_BGR2GRAY);
			break;
		case 4:
			gray_ROI = imread("/home/biorobotics/catkin_ws/src/myproject/camera_info/pictures/myimage_left.png");
			grayN_ROI = imread("/home/biorobotics/catkin_ws/src/myproject/camera_info/pictures/myimage_right.png");
			cvtColor(gray_ROI,  gray_ROI,  COLOR_BGR2GRAY);
			cvtColor(grayN_ROI, grayN_ROI, COLOR_BGR2GRAY);
			break;
	}		

	// prepare for left right matcher :)
	flip(gray_ROI,gray_ROI_flip,1);
	flip(grayN_ROI,grayN_ROI_flip,1);

	//zero padding
	copyMakeBorder(      gray_ROI,      gray_ROI,0,0,DISP_NUMDISP,0,cv::BORDER_CONSTANT,cv::Scalar(0));
	copyMakeBorder(     grayN_ROI,     grayN_ROI,0,0,DISP_NUMDISP,0,cv::BORDER_CONSTANT,cv::Scalar(0));
	copyMakeBorder( gray_ROI_flip, gray_ROI_flip,0,0,DISP_NUMDISP,0,cv::BORDER_CONSTANT,cv::Scalar(0));
	copyMakeBorder(grayN_ROI_flip,grayN_ROI_flip,0,0,DISP_NUMDISP,0,cv::BORDER_CONSTANT,cv::Scalar(0));

	// (2) Create the image in which we will save our disparities
	Mat imgDisparity8U = Mat( gray_ROI.rows, gray_ROI.cols, CV_8UC1 );
	Mat imgDisparity16S = Mat( gray_ROI.rows, gray_ROI.cols, CV_16S );
	Mat disp16S_left = Mat( gray_ROI.rows, gray_ROI.cols, CV_16S );
	Mat disp16S_right = Mat( gray_ROI.rows, gray_ROI.cols, CV_16S );

	// (3) Start to compute disparity map
	Ptr<StereoSGBM> sgbm;
	Ptr<StereoBM> sbm;
	Ptr<DisparityWLSFilter> wls_filter;
	Mat imgDisparity16S_tmp, disp16S_left_tmp, disp16S_right_tmp;
	/* disparity algorithm options:
	   0: Call the constructor for StereoBM
           1: Call the constructor for StereoSGBM (first attempt parameters)
           2: Call the constructor for StereoSGBM (magic parameters)
	   3: Call the constructor for StereoSGBM (modified parameters)
	   4: Call the constructor for StereoSGBM (Manually adjust the parameters online)
	*/
	if(disparity_algorithm == 0)
	{
		sbm = StereoBM::create(DISP_NUMDISP,DISP_WINSIZE);  // Call the constructor for StereoBM
		
		sbm->compute(       gray_ROI,     grayN_ROI,  disp16S_left_tmp); // Calculate the disparity image
		sbm->compute( grayN_ROI_flip, gray_ROI_flip, disp16S_right_tmp); 
	}
	else //disparity_algorithm == 1,2,3,4
	{
		sgbm = StereoSGBM::create(DISP_MINDISP,DISP_NUMDISP,DISP_WINSIZE,DISP_P1,DISP_P2,DISP_MAXDIFF,DISP_FLTRCAP,DISP_UNQRATE,DISP_SPKSIZE,DISP_SPKRNGE,DISP_MODE);
		

		sgbm->compute(       gray_ROI,     grayN_ROI,  disp16S_left_tmp); // Calculate the disparity image
		sgbm->compute( grayN_ROI_flip, gray_ROI_flip, disp16S_right_tmp); 
	}

	//undo zero padding
	Rect crop = Rect(DISP_NUMDISP,0,gray_ROI.cols-DISP_NUMDISP,gray_ROI.rows);
	disp16S_left = disp16S_left_tmp(crop);
	disp16S_right = disp16S_right_tmp(crop);
	
	flip(disp16S_right,disp16S_right,1);

	copyMakeBorder(disp16S_right,disp16S_right,0,0,DISP_NUMDISP,0,cv::BORDER_CONSTANT,cv::Scalar(0));
	copyMakeBorder(disp16S_left,disp16S_left,0,0,DISP_NUMDISP,0,cv::BORDER_CONSTANT,cv::Scalar(0));	

	switch(disparity_post_filtering)
	{
		case 0:
			imgDisparity16S = disp16S_left.clone();
			break;
		case 1:
			imgDisparity16S = disp16S_left.clone();
			break;
		case 2:
			imgDisparity16S = max(disp16S_left,disp16S_right);
			break;
		case 3:
			if(disparity_algorithm == 0)
				wls_filter = createDisparityWLSFilter(sbm);
			else
				wls_filter = createDisparityWLSFilter(sgbm);

			wls_filter->setLambda(DISP_LAMBDA);
			wls_filter->setSigmaColor(DISP_SIGMA);

			Mat tmpppp=gray(crop);
			copyMakeBorder(tmpppp,tmpppp,0,0,DISP_NUMDISP,0,cv::BORDER_CONSTANT,cv::Scalar(0));	

			wls_filter->filter(disp16S_left,tmpppp,imgDisparity16S,disp16S_right);
			break;
	}

	imgDisparity16S = imgDisparity16S(crop);

	// from disparity output to real disparity
	Mat imgDisparity16S_nonzero, fullimgDisparity16S_nonzero;
	threshold(imgDisparity16S,imgDisparity16S_nonzero,0,1,THRESH_BINARY);
	
	imgDisparity16S_nonzero.convertTo(imgDisparity16S_nonzero, imgDisparity16S.type());
	imgDisparity16S = DISP_SCALE * imgDisparity16S +DISP_SHIFT * imgDisparity16S_nonzero;	

	fullimgDisparity16S_nonzero = Mat(gray.size(),imgDisparity16S_nonzero.type(),cv::Scalar(0));
	if(compute_roi_mode == 0)
		imgDisparity16S_nonzero.copyTo(fullimgDisparity16S_nonzero);
	else
		imgDisparity16S_nonzero.copyTo(fullimgDisparity16S_nonzero(region_of_interest));

	// (4) Display it as a CV_8UC1 image
	if(normalize_image)
	{
		double minVal, maxVal;
		minMaxLoc( imgDisparity16S, &minVal, &maxVal );
		imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal)); 
	}
	else
		imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 1);

	int i_start, i_end, j_start, j_end;
	Mat fullimgDisparity16S = cv::Mat(gray.rows, gray.cols, imgDisparity16S.type(), cv::Scalar(-1)); // wowowo
	Mat fullimgDisparity8U = cv::Mat(gray.rows, gray.cols, CV_8UC1, cv::Scalar(-1)); 
	Mat disp_null = cv::Mat(gray.rows, gray.cols, CV_8UC1, cv::Scalar(-1));

	if(compute_roi_mode == 0)
	{
		i_start = 0;	i_end = gray.rows;
		j_start = 0;	j_end = gray.cols;

		imgDisparity16S.copyTo(fullimgDisparity16S);
		imgDisparity8U.copyTo(fullimgDisparity8U);
	}
	else
	{
		i_start = topbound;	i_end = topbound+h;
		j_start = leftbound;	j_end = leftbound+w;
	
		imgDisparity16S.copyTo(fullimgDisparity16S(region_of_interest));
		imgDisparity8U.copyTo(fullimgDisparity8U(region_of_interest));
	}
	
	disparity = fullimgDisparity8U.clone();
	if(image_of_use == 0 && compute_roi_mode == 2)
		disp_null.copyTo(disparity, depth_roi_mask);

	// (5) disparity to depth 
	Mat depth_tmp_new, depth_tmp,depth_blk, depth_wht;
	Mat dis2cam(gray.rows, gray.cols, CV_32FC1, cv::Scalar(-1));	
	Mat dis2tool(gray.rows, gray.cols, CV_32FC1, cv::Scalar(-1));	
	Mat inv_dis2tool(gray.rows, gray.cols, CV_32FC1, cv::Scalar(-1));

	depth_tmp = compute_reprojection_to_3D(fullimgDisparity16S);

	for(int i=i_start;i<i_end;i++)
	{
		Vec3f* all_pts_in_row = depth_tmp.ptr<cv::Vec3f>(i);
		for(int j=j_start;j<j_end;j++)
		{
			pt2cam = cv::Vec3f(all_pts_in_row[j][0], all_pts_in_row[j][1], all_pts_in_row[j][2]);
			pt2tip = pt2cam - tooltip_loc;
			dis2cam.at<float>(i,j) = pt2cam[0]*pt2cam[0]+pt2cam[1]*pt2cam[1]+pt2cam[2]*pt2cam[2];
			dis2tool.at<float>(i,j) = pt2tip[0]*pt2tip[0]+pt2tip[1]*pt2tip[1]+pt2tip[2]*pt2tip[2];
		}
	}

	// (6) calibrate/fine tune the depth_tmp matrix

	bool no_good_target_found = false;
	depth_tmp_new = depth_tmp.clone(); // reproj_cali_mode = 0

	if(image_of_use == 0 && !PIXEL_POS.empty())
	{
		switch(reproj_cali_mode)
		{
			case 1:
				targetX = PIXEL_POS[0].x;
				targetY = PIXEL_POS[0].y;

				while((int)disparity.at<unsigned char>(targetY,targetX)==0) // make sure the center point is not a null disparity point
				{
					if(targetX < gray.cols-1)
						targetX ++;
					else
					{
						targetX = PIXEL_POS[0].x;
						no_good_target_found = true;
						break;
					}
				}
				break;
			case 2:
				minMaxLoc(dis2tool, &minVal, &maxVal, &minLoc, &maxLoc);
				targetX = minLoc.x;
				targetY = minLoc.y;
				break;
		}

		if(!no_good_target_found)
		{
			reproj_offset = depth_tmp.at<Vec3f>(targetY,targetX) - tooltip_loc;
			depth_tmp_new = depth_tmp-reproj_offset;

			for(int i=i_start;i<i_end;i++)
			{
				Vec3f* all_pts_in_row = depth_tmp_new.ptr<cv::Vec3f>(i);
				for(int j=j_start;j<j_end;j++)
				{
					pt2cam = cv::Vec3f(all_pts_in_row[j][0], all_pts_in_row[j][1], all_pts_in_row[j][2]);
					pt2tip = pt2cam - tooltip_loc;
					dis2cam.at<float>(i,j) = pt2cam[0]*pt2cam[0]+pt2cam[1]*pt2cam[1]+pt2cam[2]*pt2cam[2];
					dis2tool.at<float>(i,j) = pt2tip[0]*pt2tip[0]+pt2tip[1]*pt2tip[1]+pt2tip[2]*pt2tip[2];
				}
			}
		}
	}
	Mat depth_null = Mat(gray.rows, gray.cols, CV_32FC3, cv::Vec3f(-1,-1,-1));
	reprojection = depth_null.clone();
	if(image_of_use == 0)
	{
		if(compute_roi_mode == 1)
			reprojection(region_of_interest) = depth_tmp_new(region_of_interest);
		else if(compute_roi_mode == 2)
			depth_tmp_new.copyTo(reprojection, depth_roi_mask);
	}

	// (7) prepare the depth map
	
	depth_blk = cv::Mat(gray.rows, gray.cols, dis2tool.type(), Scalar(0));
	depth_wht = cv::Mat(gray.rows, gray.cols, dis2tool.type(), Scalar(255));

	switch(depth_map_mode)
	{
		case 0:
			inv_dis2tool = Scalar::all(DEPTBALL_WIDTH * DEPTBALL_WIDTH) - dis2tool;    // closer: smaller depth, lighter color
			threshold(inv_dis2tool,inv_dis2tool,0,1,THRESH_TOZERO); // get binary mask of whether that pixel is within the sphere
			//normalize(inv_dis2tool, inv_dis2tool, 0, 255, NORM_MINMAX,inv_dis2tool.type(),fullimgDisparity16S_nonzero); // operation mask : fullimgDisparity16S_nonzero
			depth = depth_blk.clone();
			break;
		case 1:
			inv_dis2tool = 1/dis2tool;
			normalize(inv_dis2tool, inv_dis2tool, 0, 255, NORM_MINMAX); // Transform the matrix with float values into a
		                                        	  		    // viewable image form (float between values 0 and 1)
			fullimgDisparity16S_nonzero.convertTo(fullimgDisparity16S_nonzero, inv_dis2tool.type());
			inv_dis2tool = inv_dis2tool.mul(fullimgDisparity16S_nonzero);	
			depth = depth_blk.clone();
			break;
		case 2:
			log(1/dis2tool,inv_dis2tool);
			normalize(inv_dis2tool, inv_dis2tool, 0, 255, NORM_MINMAX); // Transform the matrix with float values into a
		                                        	  		    // viewable image form (float between values 0 and 255)
			fullimgDisparity16S_nonzero.convertTo(fullimgDisparity16S_nonzero, inv_dis2tool.type());
			inv_dis2tool = inv_dis2tool.mul(fullimgDisparity16S_nonzero);
			depth = depth_blk.clone();	
			break;
		case 3:
			cv::sqrt(dis2tool,inv_dis2tool);
			normalize(inv_dis2tool, inv_dis2tool, 0, 255, NORM_MINMAX);	
			depth = depth_wht.clone();
			break;
		
	}

	depth = inv_dis2tool.clone();
	if(image_of_use == 0)
	{
		
		if(compute_roi_mode == 2)
		{
			if(depth_map_mode == 3)
				depth_wht.copyTo(depth, depth_roi_mask);
			else
				depth_blk.copyTo(depth, depth_roi_mask);
		}	
	}
	
	// (8) show the depth values on console 
	static int counterr = -1; 
	counterr += 1;
	if(show_depth_and_reprojection && !PIXEL_POS.empty() && counterr % 10 == 0)
	{
		if(compute_roi_mode == 0)
		{
			static const int arr1[] = {10,240,470};
			static const int arr2[] = {10,320,630};
			vector<int> vec1 (arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]) );
			vector<int> vec2 (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );

			for(std::vector<int>::size_type i = 0; i != vec1.size(); i++) 
			{
				Vec3f* all_pts_in_row = depth_tmp.ptr<cv::Vec3f>(vec1[i]);
				for(std::vector<int>::size_type j = 0; j != vec2.size(); j++) 
				{
					cout<<"pixel ("<<vec1[i]<<","<<vec2[j]<<"): depth^2 = "<<dis2tool.at<float>(vec1[i],vec2[j]);
					cout<<"\t3d point("<<all_pts_in_row[vec2[j]][0]<<", "<<all_pts_in_row[vec2[j]][1]<<", "<<all_pts_in_row[vec2[j]][2]<<")"<<endl;
				}
			}
			cout<<endl<<endl;
		}
		else
		{
			cv::Point pt = PIXEL_POS[0];
			int d = depth_roi_width;
			int arr1[] = {max(0,pt.y-2*d/3), max(0,pt.y-d/3), pt.y, min(gray.rows,pt.y+d/3), min(gray.rows,pt.y+2*d/3)};
			int arr2[] = {max(0,pt.x-2*d/3), max(0,pt.x-d/3), pt.x, min(gray.cols,pt.x+d/3), min(gray.cols,pt.x+2*d/3)}; 
			vector<int> vec1 (arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]) );
			vector<int> vec2 (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );

			if(reproj_cali_mode == 1 || reproj_cali_mode == 2)
				cout<<"center minLoc:("<<targetX<<", "<<targetY<<")"<<endl;

			cout<<"TOOLPOS_WRT_CAM (after rotation) =  "<<TOOLPOS_WRT_CAM<<endl; 
			cout<<"Q matrix (disparity to depth mapping) = "<<endl<<disp2depth<<endl<<endl;

			cout<<"distance to tool^2 (for each point):"<<endl;
			for(std::vector<int>::size_type i = 0; i != vec1.size(); i++) 
			{
				for(std::vector<int>::size_type j = 0; j != vec2.size(); j++) 
					cout<<dis2tool.at<float>(vec1[i],vec2[j])<<" ";
				cout<<endl;
			}
			cout<<endl;
			cout<<"2D coordinate and pixel disparity (for each point):"<<endl;
			for(std::vector<int>::size_type i = 0; i != vec1.size(); i++) 
			{
				for(std::vector<int>::size_type j = 0; j != vec2.size(); j++) 
					cout<<"["<<vec2[j]<<", "<<vec1[i]<<", "<<(int)disparity.at<unsigned char>(vec1[i],vec2[j])<<"] ";
				cout<<endl;
			}
			cout<<endl;
			cout<<"3D reprojection (for each point):"<<endl;
			for(std::vector<int>::size_type i = 0; i != vec1.size(); i++) 
			{
				Vec3f* all_pts_in_row = depth_tmp.ptr<cv::Vec3f>(vec1[i]);
				for(std::vector<int>::size_type j = 0; j != vec2.size(); j++) 
					cout<<"("<<all_pts_in_row[vec2[j]][0]<<", "<<all_pts_in_row[vec2[j]][1]<<", "<<all_pts_in_row[vec2[j]][2]<<")"<<" ";
				cout<<endl;
			}
			cout<<endl;
			if(image_of_use == 0)
			{
				cout<<"3D reprojection adjusted (for each point):"<<endl;
				for(std::vector<int>::size_type i = 0; i != vec1.size(); i++) 
				{
					Vec3f* all_pts_in_row = depth_tmp_new.ptr<cv::Vec3f>(vec1[i]);
					for(std::vector<int>::size_type j = 0; j != vec2.size(); j++) 
						cout<<"("<<all_pts_in_row[vec2[j]][0]<<", "<<all_pts_in_row[vec2[j]][1]<<", "<<all_pts_in_row[vec2[j]][2]<<")"<<" ";
					cout<<endl;
				}
			}

			cout<<endl<<endl;
		}

	}

	// (9) display images 
	if(show_rectified_image)
	{		
		vector<cv::Mat> color_channels(3);
		Mat left_right_image;
		color_channels.at(0) = gray_ROI;  //for blue channel
		color_channels.at(1) = grayN_ROI;  //for green channel
		color_channels.at(2) = grayN_ROI;  //for red channel
		cv::merge(color_channels, left_right_image);
		imshow("rectified stereo pairs",left_right_image);
	}

	if(show_disparity)
	{
		namedWindow("disparity",CV_WINDOW_AUTOSIZE);
		imshow("disparity", disparity);
	}
	
	if(show_depthmap)
	{
		namedWindow("depth",CV_WINDOW_AUTOSIZE);
		imshow("depth", depth);
	}
	
	if(show_rectified_image || show_disparity || show_depthmap)
		cvWaitKey(30);

	/*

	// Disparity 2: Nicely filtered, but doesn't seem very correct
	// http://docs.opencv.org/trunk/d3/d14/tutorial_ximgproc_disparity_filtering.html

	// Prepare the views for matching
	Mat left_for_matcher, right_for_matcher;
	Mat left_disp, right_disp;
	Mat filtered_disp;
	int wsize = 15; 		// window_size: should be positive and odd (3 is default)
	int max_disp = 16*4; 	// max_disparity: should be positive and divisible by 16
	double lambda = 8000;
	double sigma = 1.5;
	double vis_mult = 1.0;
    	resize(colorN,left_for_matcher ,Size(),0.5,0.5);
    	resize(color ,right_for_matcher,Size(),0.5,0.5);
	
	// Perform matching and create the filter instance
	Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
	Ptr<DisparityWLSFilter> wls_filter = createDisparityWLSFilter(left_matcher);
	Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

	cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
	cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

	left_matcher->compute(left_for_matcher, right_for_matcher,left_disp);
	right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
	
	// Perform filtering
        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
        wls_filter->filter(left_disp,color,filtered_disp,right_disp);

	// Visualize the disparity maps
        Mat raw_disp_vis;
        getDisparityVis(left_disp,raw_disp_vis,vis_mult);
        namedWindow("raw disparity", WINDOW_AUTOSIZE);
        imshow("raw disparity", raw_disp_vis);
        Mat filtered_disp_vis;
        getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
        namedWindow("filtered disparity", WINDOW_AUTOSIZE);
        imshow("filtered disparity", filtered_disp_vis);
        cvWaitKey(30);
	disparity = filtered_disp.clone();

	*/
}



Mat MyTracker_DoSegmentation::compute_shifting_result(Mat E_t)
{
	int cx = E_t.cols/2; // move the origin to the image center
	int cy = E_t.rows/2;

	normalize(E_t, E_t, 0.3, 1, NORM_MINMAX); // Transform the matrix with float values into a
                                                // viewable image form (float between values 0 and 1)
	E_t.convertTo(E_t, CV_32FC1);

	Mat q0(E_t, Rect( 0,  0, cx, cy));   // Top-Left - Create a ROI per quadrant
	Mat q1(E_t, Rect(cx,  0, cx, cy));   // Top-Right
	Mat q2(E_t, Rect( 0, cy, cx, cy));   // Bottom-Left
	Mat q3(E_t, Rect(cx, cy, cx, cy));   // Bottom-Right

	Mat tmp;                           
	q0.copyTo(tmp);	q3.copyTo(q0);	tmp.copyTo(q3); // swap quadrants (Top-Left with Bottom-Right)
	q1.copyTo(tmp);	q2.copyTo(q1);	tmp.copyTo(q2); // swap quadrant (Top-Right with Bottom-Left)

	E_t = E_t.mul(PANELTY);

	Mat cropped_E_t, cropped_more_E_t;

	Point top_left =   Point(cx-gray.cols*fourier_resize,cy-gray.rows*fourier_resize);
	Point bott_right = Point(cx+gray.cols*fourier_resize,cy+gray.rows*fourier_resize);	
	Point top_left_smaller =   Point(cx-gray.cols*fourier_resize/8,cy-gray.rows*fourier_resize/8);
	Point bott_right_smaller = Point(cx+gray.cols*fourier_resize/8,cy+gray.rows*fourier_resize/8);

	E_t(Rect(top_left,bott_right)).copyTo(cropped_E_t);
	E_t(Rect(top_left_smaller,bott_right_smaller)).copyTo(cropped_more_E_t);

	Point maxLoc;
	double minVal, maxVal;
	int cx_smaller = cropped_more_E_t.cols/2; // move the origin to the image center
	int cy_smaller = cropped_more_E_t.rows/2;

	minMaxLoc( cropped_more_E_t, &minVal, &maxVal, &minLoc, &maxLoc);
	minLoc.x = (minLoc.x - cx_smaller);
	minLoc.y = (minLoc.y - cy_smaller);	
	return cropped_E_t;
}



Mat MyTracker_DoSegmentation::compute_reprojection_to_3D(Mat fullimgDisparity16S)
{
	static bool first_entry = true;
	static cv::Mat Q;
	cv::Mat_<float> vec_tmp(4,1);
	cv::Mat_<cv::Vec3f> depth_tmp(fullimgDisparity16S.rows,fullimgDisparity16S.cols);  

	switch(reproj_to_3D_method)
	{
		case 0: 
			reprojectImageTo3D(fullimgDisparity16S, depth_tmp, disp2depth,true,-1); // handleMissingValues=false, depth=CV_32F(default)
			break;
		case 1:
			if(first_entry)
			{
				disp2depth.convertTo(Q, CV_32FC1);
				first_entry = false;
			}

			for(int i=0; i<fullimgDisparity16S.rows; ++i) 
		    	for(int j=0; j<fullimgDisparity16S.cols; ++j) 
			{
				cv::Vec3f &point = depth_tmp.at<cv::Vec3f>(i,j);
				vec_tmp(0) = j; 
				vec_tmp(1) = i; 
				vec_tmp(2) = (float)fullimgDisparity16S.at<short>(i,j); 
				vec_tmp(3) = 1;

				if(vec_tmp(2) != -1)
				{
					vec_tmp = Q*vec_tmp;
					vec_tmp /= vec_tmp(3);
					depth_tmp.at<Vec3f>(i,j) = Vec3f(vec_tmp(0),vec_tmp(1),vec_tmp(2));
				}
				else
					depth_tmp.at<Vec3f>(i,j) = Vec3f(vec_tmp(2),vec_tmp(2),vec_tmp(2));
		    	}	
			break;
	}
	
	return depth_tmp;
}



int MyTracker_DoSegmentation::compute_which_direction(Mat tmp_Q) //Todo: look more into why this is happening??
{

	int plus = 0;
	int minus = 0;
	int stay = 0;
	int pixel_coordy, pixel_coordx;


	plus = plus + tmp_Q.at<uchar>((int)(model_ptB.y*fourier_resize+minLoc.y),(int)(model_ptB.x*fourier_resize+minLoc.x));
	plus = plus + tmp_Q.at<uchar>((int)(model_ptC.y*fourier_resize+minLoc.y),(int)(model_ptC.x*fourier_resize+minLoc.x));
	plus = plus + tmp_Q.at<uchar>((int)(model_ptC3.y*fourier_resize+minLoc.y),(int)(model_ptC3.x*fourier_resize+minLoc.x));

	pixel_coordy = model_ptB.y*fourier_resize-minLoc.y;
	pixel_coordx = model_ptB.x*fourier_resize-minLoc.x;
	minus = (pixel_coordy >= 0 && pixel_coordx >=0) ? tmp_Q.at<uchar>(pixel_coordy,pixel_coordx)+minus : 255+minus;
	pixel_coordy = model_ptC.y*fourier_resize-minLoc.y;
	pixel_coordx = model_ptC.x*fourier_resize-minLoc.x;
	minus = (pixel_coordy >= 0 && pixel_coordx >=0) ? tmp_Q.at<uchar>(pixel_coordy,pixel_coordx)+minus : 255+minus;
	pixel_coordy = model_ptC3.y*fourier_resize-minLoc.y;
	pixel_coordx = model_ptC3.x*fourier_resize-minLoc.x;
	minus = (pixel_coordy >= 0 && pixel_coordx >=0) ? tmp_Q.at<uchar>(pixel_coordy,pixel_coordx)+minus : 255+minus;

	stay = stay + tmp_Q.at<uchar>((int)(model_ptB.y*fourier_resize),(int)(model_ptB.x*fourier_resize));
	stay = stay + tmp_Q.at<uchar>((int)(model_ptC.y*fourier_resize),(int)(model_ptC.x*fourier_resize));
	stay = stay + tmp_Q.at<uchar>((int)(model_ptC3.y*fourier_resize),(int)(model_ptC3.x*fourier_resize));

	if(stay > max(plus,minus))
		minLoc = Point(0,0);
	else if(plus > minus) //minLoc should be -minLoc (because the lower the energy the better)
		minLoc = -minLoc;

	return min(stay,min(plus,minus));
}



void MyTracker_DoSegmentation::compute_smooth_traj(Mat tmp_Q, int measure) // Walalaa
{
	static Point2d velocity = Point2d(0,0);
	static Point2d last_pos = Point2d(PIXEL_POS[0].x+minLoc.x,PIXEL_POS[0].y+minLoc.y);
	int pixel_coordy, pixel_coordx, predict;

	Point2d predict_pos = last_pos + velocity;
	Point2d predict_minLoc = predict_pos - PIXEL_POS[0];

	predict = 0;
	pixel_coordy = model_ptB.y*fourier_resize+ predict_minLoc.y;
	pixel_coordx = model_ptB.x*fourier_resize+ predict_minLoc.x;
	predict = (pixel_coordy >= 0 && pixel_coordx >=0 && pixel_coordy<tmp_Q.rows &&  pixel_coordx<tmp_Q.cols) ? tmp_Q.at<uchar>(pixel_coordy,pixel_coordx)+predict : 255+predict;
	pixel_coordy = model_ptC.y*fourier_resize+ predict_minLoc.y;
	pixel_coordx = model_ptC.x*fourier_resize+ predict_minLoc.x;
	predict = (pixel_coordy >= 0 && pixel_coordx >=0 && pixel_coordy<tmp_Q.rows &&  pixel_coordx<tmp_Q.cols) ? tmp_Q.at<uchar>(pixel_coordy,pixel_coordx)+predict : 255+predict;
	pixel_coordy = model_ptC3.y*fourier_resize+ predict_minLoc.y;
	pixel_coordx = model_ptC3.x*fourier_resize+ predict_minLoc.x;
	predict = (pixel_coordy >= 0 && pixel_coordx >=0 && pixel_coordy<tmp_Q.rows &&  pixel_coordx<tmp_Q.cols) ? tmp_Q.at<uchar>(pixel_coordy,pixel_coordx)+predict : 255+predict;

	if(predict < measure)
		minLoc = predict_minLoc;

	// update for next time
	Point2d new_pos = Point2d(PIXEL_POS[0].x+minLoc.x,PIXEL_POS[0].y+minLoc.y);
	velocity = new_pos - last_pos;
	last_pos = new_pos;
}



void MyTracker_DoSegmentation::compute_kalman_filter() // Todo: seems right to me, but not working great! why? Walalaa
{
	static cv::Mat A,B,Q,R,K,k,p,P,x,I,C;
	static cv::Mat action,state,lastState,measurement;

	static ros::Time time = time.now();
	static double TIMESTEP, vx,vy;
	static bool first_time = true;
	if(first_time)
	{
		float identitydata[4] = { 1, 0, 0, 1};
		float zerodata[2] = { 0, 0};

		I = cv::Mat(2, 2, CV_32FC1,identitydata); // Define I matrix
		C = cv::Mat(2, 2, CV_32FC1,identitydata); // Define C matrix
		A = cv::Mat(2, 2, CV_32FC1,identitydata); // Define A matrix
		B = cv::Mat(2, 2, CV_32FC1,identitydata); // Define B matrix
		Q = cv::Mat(2, 2, CV_32FC1,identitydata); // Define Q matrix:  the covariance of the action noise
		R = cv::Mat(2, 2, CV_32FC1,identitydata); // Define R matrix:  the confidence in each of the sensors (key to sensor fusion)
		
		action = cv::Mat(2, 1, CV_32FC1,zerodata);// Initialize "action"
		state = cv::Mat(2, 1, CV_32FC1,zerodata); // Initialize "state"
		state.at<float>(0,0) = PIXEL_POS[0].x + minLoc.x; // got from calculation
		state.at<float>(1,0) = PIXEL_POS[0].y + minLoc.y;

		measurement = cv::Mat(2, 1, CV_32FC1);    // Declare "measurement"
		lastState = cv::Mat(2, 1, CV_32FC1);      // Declare "lastState"
		K = cv::Mat(2, 2, CV_32FC1);		  // Declare K
		k = cv::Mat(2, 2, CV_32FC1); 		  // Declare k	
		p = cv::Mat(2, 2, CV_32FC1); 		  // Declare p
		P = cv::Mat(2, 2, CV_32FC1); 		  // Declare P
		first_time = false;
	}
	TIMESTEP = (time.now()-time).toSec();
	lastState = state;

	state.at<float>(0,0) = lastState.at<float>(0,0); // got from calculation
	state.at<float>(1,0) = lastState.at<float>(1,0);
	action.at<float>(0,0) = vx*TIMESTEP; 
	action.at<float>(1,0) = vy*TIMESTEP;

	measurement.at<float>(0,0) = PIXEL_POS[0].x + minLoc.x; // got from measurement (computer vision)
	measurement.at<float>(1,0) = PIXEL_POS[0].y + minLoc.y;

	// Prediction Equations
	x = A*lastState + B*action; // currently action is always zero
	p = A*P*A.t() + Q;

	// Update Equations
	k = C*p*C.t()+R;
	K = p*C*k.inv();
	state = x + K*(measurement-C*x); // update state

	P = (I - K*C)*p; // update P

	// Differentiate position to get velocity
	vx = (state.at<float>(0,0)-lastState.at<float>(0,0))/TIMESTEP;
	vy = (state.at<float>(1,0)-lastState.at<float>(1,0))/TIMESTEP;

	// Return the result
	minLoc.x = PIXEL_POS[0].x - state.at<float>(0,0);
	minLoc.y = PIXEL_POS[0].y - state.at<float>(1,0);

	// Update time
	time = time.now();

}



void MyTracker_DoSegmentation::show_image()
{
	switch(version)
	{
		case 0:
			show_one_image();
			break;
		case 1:
			show_all_images_ver1();
			break;
		case 2:
			show_one_image();
			break;
		case 3:
			show_all_images_ver3();
			break;
		case 4:
			show_all_images_ver4();
			break;
	}	
}


void MyTracker_DoSegmentation::show_one_image() 
{
	// show the image on window
	if(cvGetWindowHandle(window_name_segm)==0)
		cvNamedWindow(window_name_segm, CV_WINDOW_AUTOSIZE);
	cv::imshow(window_name_segm, ravenstate_mark);
	cvWaitKey(30);
}


void MyTracker_DoSegmentation::show_all_images_ver1() 
{
	/* image layout plan:
	   -------------------------------------------------
	   kinematics           motion        segmentation      --> all these are colored images
	   (mark axis)     (contour/delta)      (result)
	   -------------------------------------------------
	   kinematics           motion         probability     --> all these are gray images
           (probability)    (probability)       (combined)
	   -------------------------------------------------
	*/
	int dstWidth = color.cols*3;
	int dstHeight = color.rows*2;
	cv::Mat dstMat = cv::Mat(dstHeight, dstWidth, CV_8UC3, cv::Scalar(0,0,0));

	cv::Rect roi11(cv::Rect(           0,          0, color.cols, color.rows));
	cv::Rect roi12(cv::Rect(  color.cols,          0, color.cols, color.rows));
	cv::Rect roi13(cv::Rect(2*color.cols,          0, color.cols, color.rows));
	cv::Rect roi21(cv::Rect(           0, color.rows, color.cols, color.rows));
	cv::Rect roi22(cv::Rect(  color.cols, color.rows, color.cols, color.rows));
	cv::Rect roi23(cv::Rect(2*color.cols, color.rows, color.cols, color.rows));
	
	cv::Mat targetROI11 = dstMat(roi11);
	cv::Mat targetROI12 = dstMat(roi12);
	cv::Mat targetROI13 = dstMat(roi13);
	cv::Mat targetROI21 = dstMat(roi21);
	cv::Mat targetROI22 = dstMat(roi22);
	cv::Mat targetROI23 = dstMat(roi23);

	cv::cvtColor(feature_prob,feature_prob,CV_GRAY2BGR);
	cv::cvtColor(ravenstate_prob,ravenstate_prob,CV_GRAY2BGR);
	cv::cvtColor(combined_prob,combined_prob,CV_GRAY2BGR);
	
	// Todo: extend to more... (fill in the blanks)
	ravenstate_mark.copyTo(targetROI11);  // (V) ok: with axis
	  feature_thres.copyTo(targetROI12);  // (V) should be colored image
	          color.copyTo(targetROI13);  // ( ) do segmentation
	ravenstate_prob.copyTo(targetROI21);  // (V) fine tune the parameters. ok: with gaussian blur
	   feature_prob.copyTo(targetROI22);  // (~) do contouring and static prediction.
	  combined_prob.copyTo(targetROI23);  // (~) combine: feature_prob and ravenstate_prob... other strategy?

	// create image window named "My Image"
	if(cvGetWindowHandle(window_name_segm)==0)
		cvNamedWindow(window_name_segm, CV_WINDOW_AUTOSIZE);
	
	// add text to image
	putText(dstMat, "ravenstate info.", Point(5+0*color.cols, 25+0*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "motion detection", Point(5+1*color.cols, 25+0*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "segmented result", Point(5+2*color.cols, 25+0*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "ravenstate prob.", Point(5+0*color.cols, 25+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "motion threshold", Point(5+1*color.cols, 25+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "combined prob.",   Point(5+2*color.cols, 25+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);

	// resize image
	Size size((int)dstWidth*0.7,(int)dstHeight*0.7);
	resize(dstMat,dstMat,size);

	// show the image on window
	cv::imshow(window_name_segm, dstMat);
	cvWaitKey(30);
}


void MyTracker_DoSegmentation::show_all_images_ver3() 
{
	/* image layout plan:
	   ----------------------------------------------------------------------------------------
	   kinematics         refined kinematics   segment_mask     segment_mask1     color
	   (mark axis)         (after DFT)         (result_mask)    (blury shape)     (raw image)
	   ----------------------------------------------------------------------------------------
	   reference shape U   log likelihood Q  overlay U,Q,U'     segment_mask2     segment
           (probability)       (probability)       (combined)       (+color filter)   (foreground)
	   ----------------------------------------------------------------------------------------
	   freq domain U        freq domain Q     E_t result        segment_mask3     segment
           (probability)       (probability)       (combined)       (+binary thres)   (background)
	   -----------------------------------------------------------------------------------------
	*/

	int dstWidth = color.cols*5;
	int dstHeight = color.rows*3;
	cv::Mat dstMat = cv::Mat(dstHeight, dstWidth, CV_8UC3, cv::Scalar(0,0,0));

	cv::Rect roi11(cv::Rect(           0,           0, color.cols, color.rows));
	cv::Rect roi12(cv::Rect(  color.cols,           0, color.cols, color.rows));
	cv::Rect roi13(cv::Rect(2*color.cols,           0, color.cols, color.rows));
	cv::Rect roi14(cv::Rect(3*color.cols,           0, color.cols, color.rows));
	cv::Rect roi15(cv::Rect(4*color.cols,           0, color.cols, color.rows));
	cv::Rect roi21(cv::Rect(           0,  color.rows, color.cols, color.rows));
	cv::Rect roi22(cv::Rect(  color.cols,  color.rows, color.cols, color.rows));
	cv::Rect roi23(cv::Rect(2*color.cols,  color.rows, color.cols, color.rows));
	cv::Rect roi24(cv::Rect(3*color.cols,  color.rows, color.cols, color.rows));
	cv::Rect roi25(cv::Rect(4*color.cols,  color.rows, color.cols, color.rows));
	cv::Rect roi31(cv::Rect(           0,2*color.rows, color.cols, color.rows));
	cv::Rect roi32(cv::Rect(  color.cols,2*color.rows, color.cols, color.rows));
	cv::Rect roi33(cv::Rect(2*color.cols,2*color.rows, color.cols, color.rows));
	cv::Rect roi34(cv::Rect(3*color.cols,2*color.rows, color.cols, color.rows));
	cv::Rect roi35(cv::Rect(4*color.cols,2*color.rows, color.cols, color.rows));

	cv::Mat targetROI11 = dstMat(roi11);
	cv::Mat targetROI12 = dstMat(roi12);
	cv::Mat targetROI13 = dstMat(roi13);
	cv::Mat targetROI14 = dstMat(roi14);
	cv::Mat targetROI15 = dstMat(roi15);
	cv::Mat targetROI21 = dstMat(roi21);
	cv::Mat targetROI22 = dstMat(roi22);
	cv::Mat targetROI23 = dstMat(roi23);
	cv::Mat targetROI24 = dstMat(roi24);
	cv::Mat targetROI25 = dstMat(roi25);
	cv::Mat targetROI31 = dstMat(roi31);
	cv::Mat targetROI32 = dstMat(roi32);
	cv::Mat targetROI33 = dstMat(roi33); 
	cv::Mat targetROI34 = dstMat(roi34);
	cv::Mat targetROI35 = dstMat(roi35);

	cv::cvtColor(ravenstate_prob,ravenstate_prob,CV_GRAY2BGR);
	cv::cvtColor(feature_prob,feature_prob,CV_GRAY2BGR);
	segment_mask = 255*segment_mask;
	segment_mask.convertTo(segment_mask, CV_8UC3); 

	ravenstate_mark.copyTo(targetROI11);  
	  feature_thres.copyTo(targetROI12); 
	   segment_mask.copyTo(targetROI13); 
	  segment_mask1.copyTo(targetROI14); 
	          color.copyTo(targetROI15); 
	ravenstate_prob.copyTo(targetROI21); 
	   feature_prob.copyTo(targetROI22); 
	  combined_prob.copyTo(targetROI23); 
	  segment_mask2.copyTo(targetROI24); 
	   segment_fore.copyTo(targetROI25); 
	ravenstate_freq.copyTo(targetROI31); 
	   feature_freq.copyTo(targetROI32); 
        energyfunc_freq.copyTo(targetROI33); 
	  segment_mask3.copyTo(targetROI34); 
	   segment_back.copyTo(targetROI35); 

	// create image window named "My Image"
	if(cvGetWindowHandle(window_name_segm)==0)
		cvNamedWindow(window_name_segm, CV_WINDOW_AUTOSIZE);
	
	// add text to image
	std::stringstream min_energy;
  	min_energy << "min energy at (" << minLoc.x << "," << minLoc.y << ")" ;
	putText(dstMat, min_energy.str().c_str(),Point(5+2*color.cols, 50+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);

	putText(dstMat, "ravenstate info.",     Point(5+0*color.cols, 25+0*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "edge/color features",  Point(5+1*color.cols, 25+0*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "segmentation mask",    Point(5+2*color.cols, 25+0*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "mask1: post dft shape",Point(5+3*color.cols, 25+0*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "raw image",            Point(5+4*color.cols, 25+0*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(  0,  0,  0), 1.0);

	putText(dstMat, "reference shape: U",   Point(5+0*color.cols, 25+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "log likelihood: Q ",   Point(5+1*color.cols, 25+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(  0,  0,  0), 1.0);
	putText(dstMat, "combined result: E_t", Point(5+2*color.cols, 25+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "mask2: color filter",  Point(5+3*color.cols, 25+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "segment result (tool)",Point(5+4*color.cols, 25+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);

	putText(dstMat, "U freq domain",        Point(5+0*color.cols, 25+2*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "Q freq domain",        Point(5+1*color.cols, 25+2*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "E_t freq domain",      Point(5+2*color.cols, 25+2*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "mask3: binary thres",  Point(5+3*color.cols, 25+2*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "segment result (bg)",  Point(5+4*color.cols, 25+2*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(  0,  0,  0), 1.0);
	// resize image
	Size size((int)dstWidth*0.5,(int)dstHeight*0.5);
	resize(dstMat,dstMat,size);

	// show the image on window
	cv::imshow(window_name_segm, dstMat);
	cvWaitKey(30);
}


void MyTracker_DoSegmentation::show_all_images_ver4() 
{
	/* image layout plan:
	   ----------------------------
	   color          segment
	  (raw image)     (foreground)
	   ----------------------------
	   depth lettice  segment      
                          (background)
	   ----------------------------
	*/

	int dstWidth = color.cols*2;
	int dstHeight = color.rows*2;
	cv::Mat dstMat = cv::Mat(dstHeight, dstWidth, CV_8UC3, cv::Scalar(0,0,0));

	cv::Rect roi11(cv::Rect(           0,           0, color.cols, color.rows));
	cv::Rect roi12(cv::Rect(  color.cols,           0, color.cols, color.rows));
	cv::Rect roi21(cv::Rect(           0,  color.rows, color.cols, color.rows));
	cv::Rect roi22(cv::Rect(  color.cols,  color.rows, color.cols, color.rows));

	cv::Mat targetROI11 = dstMat(roi11);
	cv::Mat targetROI12 = dstMat(roi12);
	cv::Mat targetROI21 = dstMat(roi21);
	cv::Mat targetROI22 = dstMat(roi22);

	          color.copyTo(targetROI11); 
	   segment_fore.copyTo(targetROI12); 
	   segment_back.copyTo(targetROI21); 
	   depth_lattice.copyTo(targetROI22);  


	// create image window named "My Image"
	if(cvGetWindowHandle(window_name_segm)==0)
		cvNamedWindow(window_name_segm, CV_WINDOW_AUTOSIZE);
	
	// add text to image
	putText(dstMat, "raw image",            Point(5+0*color.cols, 25+0*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(  0,  0,  0), 1.0);
	putText(dstMat, "segment result (tool)",Point(5+1*color.cols, 25+0*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);
	putText(dstMat, "segment result (bg)",  Point(5+0*color.cols, 25+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(  0,  0,  0), 1.0);
	putText(dstMat, depthstring,            Point(5+1*color.cols, 25+1*color.rows), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255,255,255), 1.0);


	// resize image
	Size size((int)dstWidth*0.7,(int)dstHeight*0.7);
	resize(dstMat,dstMat,size);

	// show the image on window
	cv::imshow(window_name_segm, dstMat);
	cvWaitKey(30);
}




double MyTracker_DoSegmentation::angle_between(cv::Point v1, cv::Point v2)
{
	float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
	float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

	float dot = v1.x * v2.x + v1.y * v2.y;

	float a = dot / (len1 * len2);

	if (a >= 1.0)
		return 0;

	else if (a <= -1.0)
        	return 180;

    	else if(v2.x < v1.x)
		return acos(a)*180/M_PI; 
	else
        	return 180 - acos(a)*180/M_PI; 
}



Mat MyTracker_DoSegmentation::get_disparity()
{
	Mat result = disparity.clone();
	return result;
}		



Mat MyTracker_DoSegmentation::get_reprojection()
{
	Mat result = reprojection.clone();
	return result;
}



Mat MyTracker_DoSegmentation::get_disp2depth()
{
	Mat result = disp2depth.clone();
	return result;
}
