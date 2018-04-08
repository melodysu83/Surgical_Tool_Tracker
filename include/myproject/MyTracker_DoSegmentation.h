#ifndef MYTRACKER_DOSEGMENTATION_H
#define MYTRACKER_DOSEGMENTATION_H

#include "MyTracker_ImageCalibrator.h"

class MyTracker_DoSegmentation
{
	private:
		cv::Point ptO;   
		cv::Point ptX;
		cv::Point ptY;
		cv::Point ptZ;	

		int version;  // segmentation version
		int tissue_mean_B;
		int tissue_mean_G;
		int tissue_mean_R;
		int which_camera;
		int disparity_algorithm;

		int SEG_BLURY_SCALE;
		int SEG_BLURY_INTENSITY;
		int SEG_THRES_LOWBOUND;
		int SEG_FILL_BLACKSPOT;

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
		int DISP_MODE;		// mode
		double DISP_SHIFT;
		double DISP_SCALE;	// disparity is scaled by 16 (by opencv)
		double DISP_LAMBDA;	// wls filter parameters for the disparity
		double DISP_SIGMA;

		char* file_name4;
		char* file_name5;
		char* window_name_segm;
		double distance_to_cam;
		double distance_to_edge;
		double rectification_offset;
		bool first_access;
		bool static_background;
		bool record_dice_coefficient;
		
		int image_of_use;        // User options for disparity computation
		int compute_roi_mode;    
		int depth_map_mode;      
		int reproj_cali_mode;  
		int reproj_to_3D_method;
		int disparity_post_filtering;  
		bool rectify_image;
		bool normalize_image;
		bool show_rectified_image;
		bool show_disparity;
		bool show_depthmap;
		bool show_depth_and_reprojection; 
		bool show_toolpos_wrt_cam;
		bool show_lattice_points_only;

		string weirdstring;
		string depthstring;

	    	IplImage *color_ipl;
		IplImage *gray_ipl;
	    	IplImage *colorN_ipl;
		IplImage *grayN_ipl;
		cv::Mat  colorwheel;
		cv::Mat  colorwheel_oppo;
		cv::Mat  colorwheel_display;
		cv::Mat  color;
		cv::Mat  gray;
		cv::Mat  colorN;
		cv::Mat  grayN;
		cv::Mat  gray_last;
		cv::Mat  cameraRotationVector;
		cv::Mat  cameraTranslationVector;  // Camera frame origin in respect to Raven base frame
		cv::Mat  disparity;
		cv::Mat  depth;

		cv::Mat distortion;
		cv::Mat intrinsicMat;
		cv::Matx33f intrinsic;
		cv::Mat distortionN;
		cv::Mat intrinsicMatN;
		cv::Matx33f intrinsicN;
		cv::Matx33f rectification;
		cv::Matx33f rectificationN;
		cv::Mat rmap[2][2];
		cv::Mat disp2depth;
		cv::Mat reprojection;

		cv::Mat PANELTY;           // in version 3 (only)
		cv::Size dftSize;          // in version 3 (only)
		int uncertainty_level;     // in version 1 (only)
		double object_size;
 		double fourier_resize;     // in version 3 (only)
		int depth_roi_width;	   // in version 3,4 (only)

		cv::Point model_ptA;       // in version 2,3 (only)
		cv::Point model_ptB;
		cv::Point model_ptC;
		cv::Point model_ptD;
		cv::Point model_ptE;

		cv::Point model_ptA1;      // in version 3 (only)
		cv::Point model_ptA2;
		cv::Point model_ptB1;
		cv::Point model_ptB2;
		cv::Point model_ptB3;
		cv::Point model_ptB4;
		cv::Point model_ptC1;
		cv::Point model_ptC2;
		cv::Point model_ptC3;
		cv::Point model_ptD1;
		cv::Point model_ptE1;

		cv::Point model_dep1;      // in version 3,4 (only)
		cv::Point model_dep2;
		cv::Point model_dep3;
		cv::Point model_dep4;
		cv::Point model_dep5;
		cv::Point model_dep6;
		cv::Point model_dep7;
		cv::Point model_dep8;
		cv::Point model_dep9;

		cv::Point minLoc;	   // in version 3 (only): the optimal translation value
		vector<double> RAVEN_DIS;  // in version 3 (only): the distance to camera
		vector<Point3d> RAVEN_POS; 
		vector<Point2d> PIXEL_POS; // in version 0,1: [0] current pos, [1~3] x~z axes
					   // in version 2,3: [4~8] model_ptA~E
		Point3d TOOLPOS_WRT_CAM;   // current tool tip position with respect to camera 

		cv::Mat  segment_mask;     // in version 3: (only) the final mask of determining segmentation of the surgical tool
		cv::Mat  segment_mask1;    // in version 3: (only) intermediate mask of determining segmentation of the surgical tool
		cv::Mat  segment_mask2;    // in version 3: (only) intermediate mask of determining segmentation of the surgical tool
		cv::Mat  segment_mask3;    // in version 3: (only) intermediate mask of determining segmentation of the surgical tool
		cv::Mat  segment_mask_inv; // in version 3: (only) the final mask of determining the backgound
		cv::Mat  segment_fore;     // in version 3: (only) segmentation result of the surgical tool
		cv::Mat  segment_back;     // in version 3: (only) segmentation result of the backgound
		cv::Mat  depth_lattice;	   // in version 4: (only) the disparity of tool and it's neighboring pixels (Todo: can improve!)

		cv::Mat  depth_roi_mask;   // in version 3,4 (only)
		cv::Mat  ravenstate_freq;  // in version 3: (only) the fourier transform result of u
		cv::Mat  feature_freq;     // in version 3: (only) the fourier transform result of Q
		cv::Mat  energyfunc_freq;  // in version 3: (only) the fourier transform result of E_t	

		cv::Mat  feature_thres;    // in version 1: the motion detection probability
					   // in version 3: Todo!

		cv::Mat  feature_prob;     // in version 1: the motion detection probability
					   // in version 3: the log likelihood Q (based on edge and color?...) Todo!

		cv::Mat  ravenstate_mark;  // in version 0,1: mark the tool tip and orientation
					   // in version 2,3: mark the entire raven model (without/with width estimation)

		cv::Mat  ravenstate_prob;  // in version 1: the probability based on tool tip
					   // in version 3: the reference shape U

		cv::Mat  combined_prob;    // in version 1: merge motion,feature,raven_state info all in here (which may help segmentation)
				           // in version 3: the time domain result of E_t
		

	public:
		MyTracker_DoSegmentation();
		~MyTracker_DoSegmentation();
	    	void init(char*,int,bool,int,int);
		void init_process_colorwheel();
		void init_disparity_param();
		void load_camera_pose(Mat,Mat);
		bool load_cam_intrinsics();
		void load_cam_intrinsic(char*, int);
		void load_image(cv_bridge::CvImagePtr);
		void load_imageN(cv_bridge::CvImagePtr);
		void load_kinematics(vector<cv::Point3d>,vector<cv::Point2d>,vector<double>);
		void load_segmentation_param(vector<int>);
		void load_disparity_param(vector<int>);

		void prep_probability();
		void prep_kinematics_simple();
		void prep_kinematics_with_model();
		void prep_kinematics_with_model_and_width();
		void prep_kinematics_probability();
		void prep_motion_probability();
		void prep_log_likelihood_Q();
		void prep_reference_shape_U();
		void prep_disparity_map();

		void draw_segmentation_result_ver3();
		void draw_segmentation_result_ver4();
		void draw_dft_freqplots(Mat,Mat);
		void draw_dft_result_ver3(Mat);
		void draw_dft_result_ver4();
		void draw_segmentation_colorwheel(bool);

		void start_segmentation();
		void reset_intermediate_images();

		Mat compute_reprojection_to_3D(Mat);
		Mat compute_shifting_result(Mat);
		int compute_which_direction(Mat);
		void compute_kalman_filter();
		void compute_smooth_traj(Mat,int);

		void show_image();
		void show_one_image();
		void show_all_images_ver1();
		void show_all_images_ver3();
		void show_all_images_ver4();
		Mat publish_image();

		double angle_between(cv::Point, cv::Point);
	
		Mat get_disparity();			
		Mat get_reprojection();
		Mat get_disp2depth();
};

#endif


//Todo: do more to the tracking! (~ for in progress, and V for done)
//     *(~) probability map
//     *(~) motion detection
//     *(V) static offset compensation
//     *( ) kalman filter: more accurate probability map
//     *(V) orientation indication : show axis
//     *(V) orientation as a clue for bounding box

