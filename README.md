# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning.

## Task MP.1 The data buffer
Implement circular vector.
The first approach is to erase the first element each time a new one over dataBufferSize
So finally:
```c_cpp
if (dataBuffer.size() >=dataBufferSize) // wait until at least two images have been processed
{
    dataBuffer.erase(dataBuffer.begin());
}
dataBuffer.push_back(frame);
```
Implemented in lines 65 to 69 of MidTermProjectCamera_Student.cpp

## Task MP.2 Keypoint Detectors

add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
 HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, FREAK, SIFT
 First we create an array of detector names
 string detectorTypes[]={"HARRIS", "SHITOMASI", "FAST", "BRISK", "ORB", "AKAZE", "FREAK", "SIFT"};
 A name is selected by selecting an array member by number from 0 of Harris to 6 of SIFT
 string detectorType = detectorTypes[4];
Since the name cant be mistyped there are only 3 options Shitomasi, Harris or modern.
```c_cpp
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
```
I have had problems with FREAK, because it needs a third parameters that I unknow. I have use a matrix so the code runs, but I think its not the best way.
Calls implemented in lines 82 to 95 of MidTermProject_Camera_Student.cpp
Methods implemented in lines 135 to 249 of matching2D_Student.cpp
## Task MP.3 Box Filtering 

only keep keypoints on the preceding vehicle
A rectangle is defined, so points that are inside of the rectanle are inserted.
```c_cpp
vector<cv::KeyPoint> keypointsInside;
for (auto kp : keypoints) {
  if (vehicleRect.contains(kp.pt)) keypointsInside.push_back(kp);
}
keypoints = keypointsInside;
```
<img src="imgDoc/inCube.png" width="820"  />

Implemented in lines 108 to 112 of MidTermProjectCamera_Student.cpp

## Task MP.4 Descriptors

Add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
   BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

Like in MP.2 an array is used so cant be mistyped.
```c_cpp
        string descriptorTypes[]={"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
        string descriptorType = descriptorTypes[5];
```
The methods are implemented in matching2D_Student.cpp.
We still use OpenCV build-in descriptors (BRIEF, ORB, FREAK, AKAZE and SIFT) class with default parameters to uniquely identify keypoints. Similar to step 2, we log the descriptor extraction time for performance evaluation.

After all the implementations, I have had some problems with the AKAZE descriptor, since I have seen that it seems that only can be used with the AKAZE key point detector.

## Task MP.5 add FLANN matching 
in file matching2D.cpp from open cv docs
```c_cpp
matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
```
User can choose which matching method to use: Brute-force matcher(MAT_BF) or FLANN matcher(MAT_FLANN). Note that we need to convert our image to CV_32F type due to  a bug in current OpenCV implementation if we choose FLANN matching.

## TASK MP.6 add KNN match selection 
and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
Nearest neighbor or best match is already implemented, so K nearest neighbors (default k=2) are implemented. 
For KNN matching, we filter matches using descriptor distance ratio test to remove some outliers. 
Implemented in matching_2d_Student.cpp lines 36 to 42.

## Task MP.7
Your seventh task is to count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

| Detectors | 01   | 02   | 03   | 04   | 05   | 06   | 07   | 08   | 09   | 10   |Total |
| :-------: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: |
|HARRIS|115|98|113|121|160|383|85|210|171|281|1737|
|SHITOMASI|1370|1301|1361|1358|1333|1284|1322|1366|1389|1339|13423|
|FAST|1824|1832|1810|1817|1793|1796|1788|1695|1749|1770|17874|
|BRISK|2757|2777|2741|2735|2757|2695|2715|2628|2639|2672|27116|
|ORB|500|500|500|500|500|500|500|500|500|500|5000|
|AKAZE|1351|1327|1311|1351|1360|1347|1363|1331|1357|1331|13429|
|SIFT|1438|1371|1380|1335|1305|1370|1396|1382|1463|1422|13862 |


## Task MP.8
Your eighth task is to count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, use the BF approach with the descriptor distance ratio set to 0.8.



* KAZE/AKAZE descriptors will only work with KAZE/AKAZE keypoints.
* SIFT/ORB Segmentation Fault

## Task MP.9
Your ninth task is to log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this information you will then suggest the TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles. Finally, in a short text, please justify your recommendation based on your observations and on the data you collected.
#### Efficiency (matches/ms)

| Det/Desc | 01-02   | 02-03   | 03-04   | 04-05   | 05-06   | 06-07   | 07-08   | 08-09   | 09-10   |Total |Ratio |
| :-------: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: |
|HARRIS/BRISK|12/358.583505 ms|10/351.985806 ms|14/353.041568 ms|15/355.462443 ms|16/368.887559 ms|16/361.308749 ms|15/357.734597 ms|23/355.328097 ms|21/371.312856 ms|142 / 3233.645180 ms|0.043913 matches/ms|
|HARRIS/BRIEF|14/18.102064 ms|11/17.862322 ms|15/18.436232 ms|20/20.096729 ms|24/32.833673 ms|26/15.288008 ms|16/22.931859 ms|24/18.461108 ms|23/26.632139 ms|173 / 190.644134 ms|0.907450 matches/ms|
|HARRIS/FREAK|13/54.882301 ms|13/50.590743 ms|15/54.222338 ms|15/53.351524 ms|17/70.730154 ms|20/53.990369 ms|12/54.215855 ms|21/53.235771 ms|18/61.003241 ms|144 / 506.222296 ms|0.284460 matches/ms|
|HARRIS/SIFT|14/27.921266 ms|11/28.170537 ms|16/33.465681 ms|19/32.749550 ms|22/47.789796 ms|22/26.473935 ms|13/32.923658 ms|24/28.973407 ms|22/32.917205 ms|163 / 291.385035 ms|0.559397 matches/ms|
|HARRIS/ORB|12/14.004330 ms|12/13.166224 ms|15/13.306478 ms|18/13.767561 ms|24/28.241785 ms|20/13.074853 ms|15/15.696748 ms|24/14.608643 ms|22/19.694291 ms|162 / 145.560913 ms|1.112936 matches/ms|
|SHITOMASI/BRISK|95/353.566729 ms|88/351.137353 ms|80/355.906673 ms|90/351.300878 ms|82/351.699733 ms|79/354.194787 ms|85/351.796929 ms|86/352.286720 ms|82/348.432690 ms|767 / 3170.322492 ms|0.241931 matches/ms|
|SHITOMASI/BRIEF|115/12.692633 ms|111/12.982085 ms|104/12.758563 ms|101/12.060618 ms|102/12.131341 ms|102/12.739220 ms|100/12.867592 ms|109/13.771492 ms|100/12.275195 ms|944 / 114.278739 ms|8.260504 matches/ms|
|SHITOMASI/FREAK|86/51.650860 ms|90/52.591746 ms|86/51.183298 ms|88/53.113929 ms|86/53.291516 ms|80/52.370278 ms|81/52.222486 ms|86/52.870479 ms|85/53.306533 ms|768 / 472.601125 ms|1.625049 matches/ms|
|SHITOMASI/SIFT|112/27.277497 ms|109/27.378024 ms|104/27.399334 ms|103/33.664726 ms|99/27.280456 ms|101/27.616008 ms|96/34.519577 ms|106/27.333361 ms|97/27.277180 ms|927 / 259.746163 ms|3.568869 matches/ms|
|SHITOMASI/ORB|106/13.330931 ms|102/15.313251 ms|99/13.566581 ms|102/12.697191 ms|103/16.526812 ms|97/14.717120 ms|98/12.941336 ms|104/13.772613 ms|97/13.925606 ms|908 / 126.791441 ms|7.161367 matches/ms|
|FAST/BRISK|97/336.547478 ms|104/339.786330 ms|101/344.084027 ms|98/339.600155 ms|85/337.973980 ms|107/343.155970 ms|107/338.495235 ms|100/338.432021 ms|100/339.428281 ms|899 / 3057.503477 ms|0.294031 matches/ms|
|FAST/BRIEF|119/2.434331 ms|130/3.033004 ms|118/2.367652 ms|126/2.504139 ms|108/3.165165 ms|123/2.546638 ms|131/2.363226 ms|125/2.523496 ms|119/2.778139 ms|1099 / 23.715790 ms|46.340434 matches/ms|
|FAST/FREAK|98/41.814640 ms|99/43.334236 ms|91/42.538377 ms|98/41.511374 ms|85/40.861985 ms|99/44.452196 ms|102/44.149559 ms|101/41.868910 ms|105/42.219217 ms|878 / 382.750494 ms|2.293923 matches/ms|
|FAST/SIFT|118/24.242793 ms|123/22.030169 ms|110/21.512753 ms|119/28.520187 ms|114/22.004636 ms|119/23.051726 ms|123/27.201531 ms|117/21.702375 ms|103/21.048582 ms|1046 / 211.314752 ms|4.949962 matches/ms|
|FAST/ORB|118/2.949182 ms|123/2.717587 ms|112/2.764936 ms|126/2.895168 ms|106/2.684338 ms|122/2.769690 ms|122/2.778124 ms|123/2.770585 ms|119/2.660915 ms|1071 / 24.990525 ms|42.856243 matches/ms|
|BRISK/BRISK|171/721.023889 ms|176/723.723760 ms|157/723.015584 ms|176/716.744075 ms|174/719.904571 ms|188/723.101655 ms|173/719.579806 ms|171/718.990852 ms|184/721.198315 ms|1570 / 6487.282507 ms|0.242012 matches/ms|
|BRISK/BRIEF|178/384.356865 ms|205/383.852524 ms|185/377.551793 ms|179/376.239730 ms|183/383.498246 ms|195/382.906036 ms|207/383.630312 ms|189/384.189035 ms|183/387.313584 ms|1704 / 3443.538125 ms|0.494840 matches/ms|
|BRISK/FREAK|160/430.368005 ms|177/427.727092 ms|155/425.815670 ms|173/426.166483 ms|161/428.501255 ms|183/421.904260 ms|169/420.368744 ms|178/421.616655 ms|168/420.432444 ms|1524 / 3822.900608 ms|0.398650 matches/ms|
|BRISK/SIFT|182/425.547380 ms|193/423.291462 ms|169/426.870471 ms|183/426.916543 ms|171/425.154147 ms|195/425.557650 ms|194/422.407427 ms|176/421.788275 ms|183/421.056915 ms|1646 / 3818.590270 ms|0.431049 matches/ms|
|BRISK/ORB|162/388.184480 ms|175/386.154592 ms|158/384.460302 ms|167/387.010921 ms|160/385.749710 ms|182/385.198909 ms|167/388.720928 ms|171/386.872324 ms|172/385.154490 ms|1514 / 3477.506656 ms|0.435369 matches/ms|
|ORB/BRISK|73/344.701376 ms|74/348.490854 ms|79/350.870095 ms|85/345.979406 ms|79/344.385952 ms|92/345.754505 ms|90/345.379693 ms|88/351.812254 ms|91/346.168574 ms|751 / 3123.542709 ms|0.240432 matches/ms|
|ORB/BRIEF|49/11.425933 ms|43/8.521480 ms|45/8.431202 ms|59/8.407473 ms|53/8.518387 ms|78/8.586647 ms|68/8.558925 ms|84/8.902980 ms|66/8.799534 ms|545 / 80.152561 ms|6.799533 matches/ms|
|ORB/FREAK|42/51.006592 ms|36/48.713299 ms|44/50.970527 ms|47/47.346796 ms|44/48.220745 ms|51/54.250996 ms|52/47.372652 ms|48/49.196826 ms|56/47.171882 ms|420 / 444.250315 ms|0.945413 matches/ms|
|ORB/SIFT|67/52.988407 ms|79/54.391110 ms|78/55.815629 ms|79/51.769315 ms|82/52.817511 ms|95/55.653549 ms|95/57.400700 ms|94/61.164050 ms|94/59.371298 ms|763 / 501.371569 ms|1.521825 matches/ms|
|ORB/ORB|67/13.030495 ms|70/13.347553 ms|72/12.840003 ms|84/13.235058 ms|91/12.916774 ms|101/13.049212 ms|92/13.109185 ms|93/13.990767 ms|93/13.081711 ms|763 / 118.600758 ms|6.433348 matches/ms|
|AKAZE/BRISK|137/410.909183 ms|125/409.296606 ms|129/418.560965 ms|129/420.101859 ms|131/415.776801 ms|132/430.829913 ms|142/423.337868 ms|146/421.988327 ms|144/424.348542 ms|1215 / 3775.150064 ms|0.321842 matches/ms|
|AKAZE/BRIEF|141/75.232656 ms|134/79.235195 ms|131/77.985390 ms|130/80.237506 ms|134/89.345634 ms|146/86.398827 ms|150/77.227634 ms|148/74.904137 ms|152/87.092974 ms|1266 / 727.659953 ms|1.739824 matches/ms|
|AKAZE/FREAK|126/114.086930 ms|129/118.286230 ms|127/120.158789 ms|121/121.852357 ms|122/117.281480 ms|133/124.309869 ms|144/113.224169 ms|147/114.950587 ms|138/119.084142 ms|1187 / 1063.234553 ms|1.116405 matches/ms|
|AKAZE/SIFT|134/111.867017 ms|134/104.849369 ms|130/104.260214 ms|136/101.086257 ms|137/102.102479 ms|147/101.339269 ms|147/98.357255 ms|154/102.698683 ms|151/103.168202 ms|1270 / 929.728745 ms|1.365990 matches/ms|
|AKAZE/ORB|131/86.348967 ms|129/76.205689 ms|127/75.280194 ms|117/79.439646 ms|130/82.344703 ms|131/88.856928 ms|137/82.076327 ms|135/83.009158 ms|145/86.890271 ms|1182 / 740.451883 ms|1.596322 matches/ms|
|AKAZE/AKAZE|138/149.429672 ms|138/146.884770 ms|133/141.213250 ms|127/139.098680 ms|129/146.808472 ms|146/141.037795 ms|147/147.311975 ms|151/153.557875 ms|150/148.241370 ms|1259 / 1313.583859 ms|0.958447 matches/ms|
|SIFT/BRISK|64/440.719239 ms|66/438.982776 ms|62/443.082140 ms|66/448.838469 ms|59/445.153799 ms|64/445.716086 ms|64/443.996348 ms|67/451.592706 ms|80/470.436968 ms|592 / 4028.518531 ms|0.146952 matches/ms|
|SIFT/BRIEF|86/133.234351 ms|78/139.284875 ms|76/130.992802 ms|85/132.645551 ms|69/130.796479 ms|74/131.725813 ms|76/136.047724 ms|70/133.716922 ms|88/134.077365 ms|702 / 1202.521882 ms|0.583773 matches/ms|
|SIFT/FREAK|65/176.844138 ms|72/174.571560 ms|64/176.190610 ms|66/178.266529 ms|59/168.396981 ms|59/178.617641 ms|64/173.574838 ms|65/174.699998 ms|79/182.718700 ms|593 / 1583.880995 ms|0.374397 matches/ms|
|SIFT/SIFT|82/181.695979 ms|81/192.848225 ms|85/184.498878 ms|93/218.984562 ms|90/183.437311 ms|81/246.735161 ms|82/246.349939 ms|102/259.138301 ms|104/248.385945 ms|800 / 1962.074301 ms|0.407732 matches/ms|

#### TOP3 detector / descriptor combinations

1. FAST + BRIEF
2. FAST + ORB
3. SHITOMASI + BRIEF
  
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
