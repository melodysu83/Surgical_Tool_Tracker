#include "myproject/MyTracker_ImageCalibrator.h"


MyTracker_ImageCalibrator::MyTracker_ImageCalibrator()
{

	string str1 = "/home/biorobotics/catkin_ws/src/myproject/camera_info/camera_extrinsic/3D_points.txt";
	file_name1 = new char[str1.length() + 1];
	strcpy(file_name1, str1.c_str());

	string str2 = "/home/biorobotics/catkin_ws/src/myproject/camera_info/camera_extrinsic/cam_pose_guess.txt";
	file_name2 = new char[str2.length() + 1];
	strcpy(file_name2, str2.c_str());

	string str3 = "/home/biorobotics/catkin_ws/src/myproject/camera_info/camera_extrinsic/cam_pose_offline.txt";
	file_name3 = new char[str3.length() + 1];
	strcpy(file_name3, str3.c_str());

	string str4 = "/home/biorobotics/catkin_ws/src/myproject/camera_info/camera_intrinsics/camera_left.yaml";
	file_name4 = new char[str4.length() + 1];
	strcpy(file_name4, str4.c_str());

	string str5 = "/home/biorobotics/catkin_ws/src/myproject/camera_info/camera_intrinsics/camera_right.yaml";
	file_name5 = new char[str5.length() + 1];
	strcpy(file_name5, str5.c_str());

}


MyTracker_ImageCalibrator::~MyTracker_ImageCalibrator()
{
	
}


void MyTracker_ImageCalibrator::init(int Width,int Height, char* Wname, int camera)
{
	//chess board settings
	CHESSBOARD_WIDTH = Width;
	CHESSBOARD_HEIGHT = Height;
	CHESSBOARD_INTERSECTION_COUNT = CHESSBOARD_WIDTH * CHESSBOARD_HEIGHT;

	window_name_cali = Wname;

	which_camera = camera;

	// Set camera calibration result
	bool everything_good = load_cam_instrinsics();
	if(!everything_good)
		exit(1);
}



void MyTracker_ImageCalibrator::load_image(cv_bridge::CvImagePtr Img)
{
	color = cvCreateImage(cvSize(Img->image.cols,Img->image.rows), IPL_DEPTH_8U, 3);
	gray = cvCreateImage(cvSize(Img->image.cols,Img->image.rows), IPL_DEPTH_8U, 1);

	color_chess = cvCreateImage(cvSize(Img->image.cols,Img->image.rows), IPL_DEPTH_8U, 3);
	color_proj_pnp = cvCreateImage(cvSize(Img->image.cols,Img->image.rows), IPL_DEPTH_8U, 3);
	color_proj_guess = cvCreateImage(cvSize(Img->image.cols,Img->image.rows), IPL_DEPTH_8U, 3);

	// cv_ptr to iplimage*
	color->imageData = (char *) Img->image.data;

	cv::Mat cMat =  cv::cvarrToMat(color);
	IplImage ipltmp=cMat;
	cvCopy(&ipltmp,color_chess);
	cvCopy(&ipltmp,color_proj_pnp);
	cvCopy(&ipltmp,color_proj_guess);

	// convert to gray
	cvCvtColor(color, gray, CV_RGB2GRAY); //source: color image, destination: gray image
}



bool MyTracker_ImageCalibrator::find_chessboard_corners()
{
	corners = new CvPoint2D32f[CHESSBOARD_INTERSECTION_COUNT];
	wasChessboardFound = cvFindChessboardCorners(gray, cvSize(CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT), corners, &corner_count);

	if(wasChessboardFound) {
		// Refine the found corners
		cvFindCornerSubPix(gray, corners, corner_count, cvSize(5, 5), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1));
		cvDrawChessboardCorners(gray, cvSize(CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT), corners, corner_count, wasChessboardFound);
	} 
	return wasChessboardFound;
}



CvPoint2D32f* MyTracker_ImageCalibrator::get_corners()
{
	return corners;
}


int MyTracker_ImageCalibrator::get_corner_count()
{
	return corner_count;
}


void MyTracker_ImageCalibrator::draw_corners()
{
	if(wasChessboardFound)
	{	
		for(int i=0;i<CHESSBOARD_INTERSECTION_COUNT;i++)
		{
			cvCircle(color, cv::Point(corners[i].x, corners[i].y), 3, CV_RGB(255,0,0)); //red
			cvCircle(color_chess, cv::Point(corners[i].x, corners[i].y), 3, CV_RGB(255,0,0)); //red
		}

		int index1 = 0;
		int index2 = CHESSBOARD_WIDTH-1;
		int index3 = CHESSBOARD_INTERSECTION_COUNT-CHESSBOARD_WIDTH;
		int index4 = CHESSBOARD_INTERSECTION_COUNT-1;

		cvCircle(color, cv::Point(corners[index1].x, corners[index1].y), 3, CV_RGB(0,255,0)); //green
		cvCircle(color, cv::Point(corners[index2].x, corners[index2].y), 3, CV_RGB(0,255,0));
		cvCircle(color, cv::Point(corners[index3].x, corners[index3].y), 3, CV_RGB(0,255,0));
		cvCircle(color, cv::Point(corners[index4].x, corners[index4].y), 3, CV_RGB(0,255,0));

		cvCircle(color_chess, cv::Point(corners[index1].x, corners[index1].y), 3, CV_RGB(0,255,0)); //green
		cvCircle(color_chess, cv::Point(corners[index2].x, corners[index2].y), 3, CV_RGB(0,255,0));
		cvCircle(color_chess, cv::Point(corners[index3].x, corners[index3].y), 3, CV_RGB(0,255,0));
		cvCircle(color_chess, cv::Point(corners[index4].x, corners[index4].y), 3, CV_RGB(0,255,0));
	}
	for (int i=0;i<corners_2d_cam.size();i++)
	{
		cvCircle(color, cv::Point(corners_2d_cam[i].x, corners_2d_cam[i].y), 3, CV_RGB(0,0,255)); //blue
		cvCircle(color_proj_pnp, cv::Point(corners_2d_cam[i].x, corners_2d_cam[i].y), 3, CV_RGB(0,0,255)); //blue
	}

	for (int i=0;i<corners_2d_guess.size();i++)
	{
		cvCircle(color, cv::Point(corners_2d_guess[i].x, corners_2d_guess[i].y), 3, CV_RGB(0,155,155)); //gray green
		cvCircle(color_proj_guess, cv::Point(corners_2d_guess[i].x, corners_2d_guess[i].y), 3, CV_RGB(0,155,155)); //gray green
	}

	show_four_images();
	//show_two_images();
}


void MyTracker_ImageCalibrator::list_corners()
{
	CONSOLE.display_2D_points(corners,CHESSBOARD_WIDTH,CHESSBOARD_HEIGHT,wasChessboardFound);
}

void MyTracker_ImageCalibrator::show_two_images()
{
	cv::Mat colorMat =  cv::cvarrToMat(color);
	cv::Mat grayMat =  cv::cvarrToMat(gray);
	cv::cvtColor(grayMat,grayMat,CV_GRAY2BGR);

	int dstWidth = colorMat.cols*2;
	int dstHeight = colorMat.rows;

	cv::Rect roi1(cv::Rect(0, 0, colorMat.cols, colorMat.rows));
	cv::Rect roi2(cv::Rect(colorMat.cols, 0, colorMat.cols, colorMat.rows));

	cv::Mat dstMat = cv::Mat(dstHeight, dstWidth, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat targetROI1 = dstMat(roi1);
	cv::Mat targetROI2 = dstMat(roi2);

	colorMat.copyTo(targetROI1);
	grayMat.copyTo(targetROI2);

	// create image window named "My Image"
	cvNamedWindow(window_name_cali, CV_WINDOW_AUTOSIZE);
	
	// show the image on window
	cv::imshow(window_name_cali, dstMat);
	cvWaitKey(30);
}


void MyTracker_ImageCalibrator::show_four_images()
{
	cv::Mat cMat1 =  cv::cvarrToMat(color);
	cv::Mat cMat2 =  cv::cvarrToMat(color_chess);
	cv::Mat cMat3 =  cv::cvarrToMat(color_proj_pnp);
	cv::Mat cMat4 =  cv::cvarrToMat(color_proj_guess);

	int dstWidth  = cMat1.cols*2;
	int dstHeight = cMat1.rows*2;

	cv::Rect roi1(cv::Rect(         0,          0, cMat1.cols, cMat1.rows)); // top left: all
	cv::Rect roi2(cv::Rect(cMat1.cols,          0, cMat1.cols, cMat1.rows)); // top right: detected 2d chess corners
	cv::Rect roi3(cv::Rect(         0, cMat1.rows, cMat1.cols, cMat1.rows)); // bottom left: proj by pnp
	cv::Rect roi4(cv::Rect(cMat1.cols, cMat1.rows, cMat1.cols, cMat1.rows)); // bottom right: proj by our guess

	cv::Mat dstMat = cv::Mat(dstHeight, dstWidth, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat targetROI1 = dstMat(roi1);
	cv::Mat targetROI2 = dstMat(roi2);
	cv::Mat targetROI3 = dstMat(roi3);
	cv::Mat targetROI4 = dstMat(roi4);

	cMat1.copyTo(targetROI1);
	cMat2.copyTo(targetROI2);
	cMat3.copyTo(targetROI3);
	cMat4.copyTo(targetROI4);

	Size size((int)dstWidth*0.7,(int)dstHeight*0.7);
	resize(dstMat,dstMat,size);
	// create image window named "My Image"
	cvNamedWindow(window_name_cali, CV_WINDOW_AUTOSIZE);
	
	// show the image on window
	cv::imshow(window_name_cali, dstMat);
	cvWaitKey(30);
}

    
void MyTracker_ImageCalibrator::solve_pnp()
{
	vector<Point3d>     world_coords;
	vector<Point2d>     pixel_coords;
	
	
	load_3D_points(); //load 3d points from file
	load_cam_pose(); //load initial guess of camera pose

	int index1 = 0;
	int index2 = CHESSBOARD_WIDTH-1;
	int index3 = CHESSBOARD_INTERSECTION_COUNT-CHESSBOARD_WIDTH;
	int index4 = CHESSBOARD_INTERSECTION_COUNT-1;

	/*
	//pnp with only four points
	// Target rectangle coordinates in 3d space
	world_coords.push_back (Point3d (corners_3d[index1][0],corners_3d[index1][1],corners_3d[index1][2]));
	world_coords.push_back (Point3d (corners_3d[index2][0],corners_3d[index2][1],corners_3d[index2][2]));
	world_coords.push_back (Point3d (corners_3d[index3][0],corners_3d[index3][1],corners_3d[index3][2]));
	world_coords.push_back (Point3d (corners_3d[index4][0],corners_3d[index4][1],corners_3d[index4][2]));

	// Coordinates of rectangle in camera
	pixel_coords.push_back (Point2d (corners[index1].x, corners[index1].y));
	pixel_coords.push_back (Point2d (corners[index2].x, corners[index2].y));
	pixel_coords.push_back (Point2d (corners[index3].x, corners[index3].y));
	pixel_coords.push_back (Point2d (corners[index4].x, corners[index4].y));
	*/
	
	for(int i=0;i<CHESSBOARD_INTERSECTION_COUNT;i++)
	{
		// Target rectangle coordinates in 3d space
		world_coords.push_back(Point3d (corners_3d[i][0],corners_3d[i][1],corners_3d[i][2]));

		// Coordinates of rectangle in camera
		pixel_coords.push_back (Point2d (corners[i].x, corners[i].y));
	}

	cv::projectPoints(world_coords,rotation_vector,translation_vector,intrinsic,distortion,corners_2d_guess);
	
	CONSOLE.display_cam_pose(rotation_vector,translation_vector,cameraRotationVector,cameraTranslationVector,corners_2d_guess,CHESSBOARD_WIDTH,CHESSBOARD_HEIGHT,false);

	// Get vectors for world->camera transform
	solvePnP (world_coords, pixel_coords, intrinsic, distortion, rotation_vector, translation_vector, true, 0);
	//solvePnPRansac (world_coords, pixel_coords, intrinsic, distortion, rotation_vector, translation_vector, true, 100, 8.0, 0.99, inliers, SOLVEPNP_ITERATIVE);

	// Save the result
	//http://stackoverflow.com/questions/19849683/opencv-solvepnp-detection-problems

	cv::Mat R;
	cv::Rodrigues(rotation_vector, R);
	cameraRotationVector=R.t();
	cameraTranslationVector = -R.t()*translation_vector;	
	
	cv::projectPoints(world_coords,rotation_vector,translation_vector,intrinsic,distortion,corners_2d_cam);
	CONSOLE.display_3D_points(corners_3d,corners_2d_cam,CHESSBOARD_WIDTH,CHESSBOARD_HEIGHT,true);
}


void MyTracker_ImageCalibrator::list_pnp_result()
{	
	//http://stackoverflow.com/questions/16265714/camera-pose-estimation-opencv-pnp
	CONSOLE.display_cam_pose(rotation_vector,translation_vector,cameraRotationVector,cameraTranslationVector,corners_2d_guess,CHESSBOARD_WIDTH,CHESSBOARD_HEIGHT,true);
	CONSOLE.display_projection_comparison(corners,corners_2d_guess,corners_2d_cam,CHESSBOARD_INTERSECTION_COUNT);
}


bool MyTracker_ImageCalibrator::load_3D_points()
{
	int points[4][3];
	stringstream buffer;

	//load the text file and put it into a single string:
	ifstream file(file_name1);
	
	buffer << file.rdbuf();
	string data = buffer.str();
	
	std::memset(points, 0, sizeof(points[0][0]) * 4 * 3);
	istringstream iss(data);

	if(file.is_open())
	{	
		for(int i=0;i<4 ;i++)
		for(int j=0;j<3 ;j++)
		{
			int k; 
			iss >> k; 
			points[i][j] = k; 
		}
	}
	else 
		cout<<"File not found !"<<endl;

	//interpret the remaining points
	corners_3d = new double*[CHESSBOARD_INTERSECTION_COUNT];
	for (int i=0;i<CHESSBOARD_INTERSECTION_COUNT;i++)
		corners_3d[i] = new double[3];

	int index1 = 0;
	int index2 = CHESSBOARD_WIDTH-1;
	int index3 = CHESSBOARD_INTERSECTION_COUNT-CHESSBOARD_WIDTH;
	int index4 = CHESSBOARD_INTERSECTION_COUNT-1;

	corners_3d[index1][0] = points[0][0];
	corners_3d[index1][1] = points[0][1];
	corners_3d[index1][2] = points[0][2];

	corners_3d[index2][0] = points[1][0];
	corners_3d[index2][1] = points[1][1];
	corners_3d[index2][2] = points[1][2];

	corners_3d[index3][0] = points[2][0];
	corners_3d[index3][1] = points[2][1];
	corners_3d[index3][2] = points[2][2];

	corners_3d[index4][0] = points[3][0];
	corners_3d[index4][1] = points[3][1];
	corners_3d[index4][2] = points[3][2];

	for(int i=1;i<CHESSBOARD_HEIGHT-1 ;i++)
	for(int j=0;j<3 ;j++)
	{
		corners_3d[i*CHESSBOARD_WIDTH][j]= (corners_3d[index3][j]-corners_3d[index1][j])*i/(CHESSBOARD_HEIGHT-1)+corners_3d[index1][j];
		corners_3d[(i+1)*CHESSBOARD_WIDTH-1][j]= (corners_3d[index4][j]-corners_3d[index2][j])*i/(CHESSBOARD_HEIGHT-1)+corners_3d[index2][j];
	}
	
	for(int i=0;i<CHESSBOARD_HEIGHT; i++)
	for(int j=1;j<CHESSBOARD_WIDTH-1 ;j++)
	for(int k=0;k<3 ;k++)
	{
		corners_3d[i*CHESSBOARD_WIDTH+j][k]= (corners_3d[(i+1)*CHESSBOARD_WIDTH-1][k]-corners_3d[i*CHESSBOARD_WIDTH][k])*j/(CHESSBOARD_WIDTH-1)+corners_3d[i*CHESSBOARD_WIDTH][k];
	}
	
	CONSOLE.display_3D_points(corners_3d,corners_2d_cam,CHESSBOARD_WIDTH,CHESSBOARD_HEIGHT,false);
}


bool MyTracker_ImageCalibrator::load_cam_pose()
{
	string buf,data;
	stringstream buffer;
	vector<string> tokens;

	//load the text file and put it into a single string:
	ifstream file(file_name2);	
	buffer << file.rdbuf();	
	data = buffer.str();
	stringstream ss(data);
	
	cv::Mat R;
	cv::Mat rvec(3,1,cv::DataType<double>::type);
	cv::Mat tvec(3,1,cv::DataType<double>::type);

	if(file.is_open())
	{	
		while (ss >> buf)
			tokens.push_back(buf);
	}
	else 
		cout<<"File not found !"<<endl;

	rvec.at<double>(0) = atof(tokens[0].c_str())*CV_PI/180; 
	rvec.at<double>(1) = atof(tokens[1].c_str())*CV_PI/180; 
	rvec.at<double>(2) = atof(tokens[2].c_str())*CV_PI/180; 
	cv::Rodrigues(rvec, R);

	tvec.at<double>(0) = atof(tokens[3].c_str()); 
	tvec.at<double>(1) = atof(tokens[4].c_str()); 
	tvec.at<double>(2) = atof(tokens[5].c_str()); 

	
	rotation_vector = -rvec.clone();	
	translation_vector = -R.t()*tvec.clone();
}


bool MyTracker_ImageCalibrator::load_cam_pose_offline()
{
	string buf,data;
	stringstream buffer;
	vector<string> tokens;

	//load the text file and put it into a single string:
	ifstream file(file_name3);	
	buffer << file.rdbuf();	
	data = buffer.str();
	stringstream ss(data);

	cv::Mat rvec(3,1,cv::DataType<double>::type);
	cv::Mat tvec(3,1,cv::DataType<double>::type);

	if(file.is_open())
	{	
		while (ss >> buf)
			tokens.push_back(buf);

	}
	else 
		cout<<"File not found !"<<endl;

	rvec.at<double>(0) = atof(tokens[0].c_str())*CV_PI/180; 
	rvec.at<double>(1) = atof(tokens[1].c_str())*CV_PI/180; 
	rvec.at<double>(2) = atof(tokens[2].c_str())*CV_PI/180; 

	tvec.at<double>(0) = atof(tokens[3].c_str()); 
	tvec.at<double>(1) = atof(tokens[4].c_str()); 
	tvec.at<double>(2) = atof(tokens[5].c_str()); 
	
	rotation_vector = rvec.clone();	
	translation_vector = tvec.clone();

	CONSOLE.display_cam_pose_offline(rotation_vector,translation_vector,cameraRotationVector,cameraTranslationVector);
}



bool MyTracker_ImageCalibrator::load_cam_instrinsics()
{
	size_t start,end;
	char* file_name;
	string buf,data,target1,target2,data1,data2;
	stringstream buffer;
	vector<string> tokens1;
	vector<string> tokens2;

	target1 = "data: [";
	target2 = "]\n";

	
	if(which_camera == LEFT_CAMERA)
		file_name = file_name4; // left camera info

	else if(which_camera == RIGHT_CAMERA)
		file_name = file_name5; // right camera info
	else
	{
		CONSOLE.no_correct_camera_set(which_camera);
		return false;
	}

	ifstream file(file_name);

	//load the text file and put it into a single string:
	buffer << file.rdbuf();	
	data = buffer.str();
	cout<<data<<endl;

	start = data.find(target1); // this is where intrinsic is
	if (start!=string::npos)
	{
		end = data.find(target2.c_str(),start+1);
		data1 = data.substr(start+7, end-start-7); // string.substr(starting_index, length_of_sub_string)
	}
	else
		CONSOLE.no_valid_camera_info_file(1,start,end);

	start = data.find(target1.c_str(),end+1); // this is where distortion is
	if (start!=string::npos)
	{
		end = data.find(target2.c_str(),start+1);
		data2 = data.substr(start+7, end-start-7); // string.substr(starting_index, length_of_sub_string)
	}
	else
		CONSOLE.no_valid_camera_info_file(2,start,end);

	stringstream ss1(data1);
	if(file.is_open())
	{	
		while (ss1 >> buf)
			tokens1.push_back(buf.substr(0,buf.length()-1));
	}
	else
		CONSOLE.no_valid_camera_info_file(3,start,end);

	stringstream ss2(data2);
	if(file.is_open())
	{	
		while (ss2 >> buf)
			tokens2.push_back(buf.substr(0,buf.length()-1));
	}
	else
		CONSOLE.no_valid_camera_info_file(3,start,end);

	// (0) prepare to load the values
	distortion = cv::Mat::zeros(1,5,CV_32F);
	intrinsicMat = cv::Mat::zeros(3,3,CV_32F);
	intrinsic = intrinsicMat.clone();

	// (1) load intrinsic matrix
	intrinsic(0,0) = atof(tokens1[0].c_str()); // 492.072810; .....these are values from right camera
	intrinsic(1,1) = atof(tokens1[4].c_str()); // 494.288119; 
	intrinsic(0,2) = atof(tokens1[2].c_str()); // 366.475055; 
	intrinsic(1,2) = atof(tokens1[5].c_str()); // 217.969378; 
	intrinsic(2,2) = 1;
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
	return true;
}



Mat MyTracker_ImageCalibrator::get_camera_rotat()
{
	return cameraRotationVector;
}


Mat MyTracker_ImageCalibrator::get_camera_trans()
{
	return cameraTranslationVector;
}


void MyTracker_ImageCalibrator::set_camera_pose()
{
	load_cam_pose_offline();  

	cv::Mat R;
	cv::Rodrigues(rotation_vector, R);
	cameraRotationVector=R.t();
	cameraTranslationVector = -R.t()*translation_vector;	
}



void MyTracker_ImageCalibrator::getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles){

    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}

vector<cv::Point2d> MyTracker_ImageCalibrator::project(vector<Point3d> world_coords)
{

	vector<cv::Point2d> pixel_coords;
	cv::projectPoints(world_coords,rotation_vector,translation_vector,intrinsic,distortion,pixel_coords);
	return pixel_coords;
	
}
