# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning.

Task MP.1 The data buffer
Implement circular vector.
The first approach is to erase the first element each time a new one over dataBufferSize
So finally:
if (dataBuffer.size() >=dataBufferSize) // wait until at least two images have been processed
{
    dataBuffer.erase(dataBuffer.begin());
}
dataBuffer.push_back(frame);

Implemented in lines 65 to 69 of MidTermProjectCamera_Student.cpp
TASK MP.2 
add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
 HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, FREAK, SIFT
 First we create an array of detector names
 string detectorTypes[]={"HARRIS", "SHITOMASI", "FAST", "BRISK", "ORB", "AKAZE", "FREAK", "SIFT"};
 A name is selected by selecting an array member by number from 0 of Harris to 6 of SIFT
 string detectorType = detectorTypes[4];
Since the name cant be mistyped there are only 3 options Shitomasi, Harris or modern.
  if (detectorType.compare("SHITOMASI") == 0)
  {
      detKeypointsShiTomasi(keypoints, imgGray, bVis);
  }
  else if (detectorType.compare("HARRIS") == 0)
  {
      detKeypointsHarris(keypoints, imgGray, bVis);
  }
  else
  {
      detKeypointsModern(keypoints, imgGray,detectorType, bVis);
  }
TASK MP.3 
only keep keypoints on the preceding vehicle
A rectangle is defined, so points that are inside of the rectanle are inserted.
  vector<cv::KeyPoint> keypointsInside;
  for (auto kp : keypoints) {
      if (vehicleRect.contains(kp.pt)) keypointsInside.push_back(kp);
  }
  keypoints = keypointsInside;
<img src="imgDoc/inCube.png" width="820"  />
Implemented in lines 108 to 112 of MidTermProjectCamera_Student.cpp

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.