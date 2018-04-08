instructions:
---------------------------
http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration




todo:
---------------------------
1. run "myproject-calibration.launch" file

2. on a new terminal:
   >> rostopic list
   to make sure that the camera frames are published as rostopics

3. then again, on the new terminal:
   >> rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.01156 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left --approximate=0.01



