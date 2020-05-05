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

| Det/Desc | 01-02   | 02-03   | 03-04   | 04-05   | 05-06   | 06-07   | 07-08   | 08-09   | 09-10| 
| :-------: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: |
|HARRIS/BRISK|12/358.385886 ms|10/357.750184 ms|14/372.349805 ms|15/366.250151 ms|16/372.178567 ms|16/362.732477 ms|15/371.763259 ms|23/374.717387 ms|21/373.664856 ms|
|HARRIS/BRIEF|14/19.680785 ms|11/18.532239 ms|15/17.837400 ms|20/25.221728 ms|24/35.153146 ms|26/20.173074 ms|16/21.340864 ms|24/19.378849 ms|23/28.254687 ms|
|HARRIS/FREAK|13/53.512641 ms|13/54.154449 ms|15/55.941066 ms|15/53.958238 ms|17/69.440276 ms|20/51.598194 ms|12/54.483361 ms|21/53.086939 ms|18/59.663430 ms|
|HARRIS/SIFT|14/26.927574 ms|11/27.057455 ms|16/43.582236 ms|19/41.795710 ms|22/54.556842 ms|22/27.513325 ms|13/31.804326 ms|24/28.020547 ms|22/39.650236 ms|
|HARRIS/ORB|12/12.727837 ms|12/13.732632 ms|15/20.884462 ms|18/21.945592 ms|24/35.359262 ms|20/13.145464 ms|15/15.699468 ms|24/14.174654 ms|22/19.269515 ms|
|SHITOMASI/BRISK|95/356.204140 ms|88/350.934162 ms|80/353.562566 ms|90/351.558496 ms|82/348.985293 ms|79/355.879995 ms|85/351.230515 ms|86/347.531573 ms|82/349.296863 ms|
|SHITOMASI/BRIEF|115/13.141982 ms|111/12.976507 ms|104/11.967670 ms|101/13.039128 ms|102/12.582697 ms|102/13.139720 ms|100/12.865170 ms|109/13.105413 ms|100/14.014474 ms|
|SHITOMASI/FREAK|86/54.282881 ms|90/54.536808 ms|86/53.056478 ms|88/52.246690 ms|86/54.283081 ms|80/56.686657 ms|81/61.177342 ms|86/57.106443 ms|85/57.200627 ms|
|SHITOMASI/SIFT|112/27.496691 ms|109/29.344998 ms|104/26.716313 ms|103/45.702761 ms|99/28.590234 ms|101/29.987833 ms|96/33.897440 ms|106/30.094400 ms|97/27.476528 ms|
|SHITOMASI/ORB|106/13.161973 ms|102/14.307307 ms|99/12.962801 ms|102/13.931341 ms|103/13.006185 ms|97/12.931590 ms|98/13.157809 ms|104/13.264390 ms|97/13.509573 ms|
|FAST/BRISK|97/339.036964 ms|104/338.833931 ms|101/333.889382 ms|98/345.105742 ms|85/338.173924 ms|107/343.382660 ms|107/337.737626 ms|100/336.827231 ms|100/345.211161 ms|
|FAST/BRIEF|119/2.374996 ms|130/2.683865 ms|118/2.726323 ms|126/2.563547 ms|108/2.396192 ms|123/2.447021 ms|131/2.371072 ms|125/2.957872 ms|119/2.439846 ms|
|FAST/FREAK|98/41.399170 ms|99/43.370051 ms|91/41.265225 ms|98/42.014962 ms|85/43.378292 ms|99/42.461694 ms|102/41.865934 ms|101/42.648305 ms|105/41.797606 ms|
|FAST/SIFT|118/22.476109 ms|123/22.109257 ms|110/29.750030 ms|119/29.765143 ms|114/22.900172 ms|119/25.025806 ms|123/29.994871 ms|117/24.894792 ms|103/23.226480 ms|
|FAST/ORB|118/3.316375 ms|123/2.882339 ms|112/2.744088 ms|126/3.843339 ms|106/2.809821 ms|122/2.954154 ms|122/2.772463 ms|123/2.822945 ms|119/2.717664 ms|
|BRISK/BRISK|171/714.402386 ms|176/717.959093 ms|157/722.093472 ms|176/718.331455 ms|174/728.953486 ms|188/719.242484 ms|173/723.551577 ms|171/718.757823 ms|184/727.005235 ms|
|BRISK/BRIEF|178/385.697093 ms|205/384.538886 ms|185/389.253928 ms|179/390.356331 ms|183/386.006610 ms|195/385.589458 ms|207/381.947186 ms|189/384.177198 ms|183/381.877338 ms|
|BRISK/FREAK|160/433.135156 ms|177/427.438621 ms|155/430.905930 ms|173/424.505857 ms|161/428.794655 ms|183/418.920874 ms|169/423.310354 ms|178/429.538884 ms|168/422.606991 ms|
|BRISK/SIFT|182/419.117054 ms|193/428.148337 ms|169/442.820847 ms|183/442.853581 ms|171/429.121347 ms|195/429.013766 ms|194/425.431912 ms|176/428.256106 ms|183/418.880163 ms|
|BRISK/ORB|162/383.686976 ms|175/381.541777 ms|158/382.307255 ms|167/383.301172 ms|160/391.364997 ms|182/392.292965 ms|167/390.086240 ms|171/383.130378 ms|172/377.961446 ms|
|ORB/BRISK|73/342.951148 ms|74/340.327489 ms|79/343.759511 ms|85/349.611913 ms|79/355.121244 ms|92/354.689433 ms|90/341.492124 ms|88/342.044532 ms|91/348.181026 ms|
|ORB/BRIEF|49/8.739973 ms|43/8.950136 ms|45/9.067653 ms|59/8.160194 ms|53/9.256885 ms|78/8.323037 ms|68/8.811247 ms|84/8.635854 ms|66/11.897549 ms|
|ORB/FREAK|42/48.660971 ms|36/46.678507 ms|44/47.667351 ms|47/47.243659 ms|44/46.883372 ms|51/48.599383 ms|52/47.646469 ms|48/48.206260 ms|56/51.355483 ms|
|ORB/SIFT|67/52.893129 ms|79/59.087814 ms|78/56.037160 ms|79/54.381028 ms|82/54.211747 ms|95/56.604589 ms|95/63.688477 ms|94/62.096018 ms|94/63.352028 ms|
|ORB/ORB|67/13.543339 ms|70/13.380248 ms|72/12.169460 ms|84/13.310676 ms|91/12.799945 ms|101/13.117907 ms|92/13.143759 ms|93/12.622083 ms|93/12.720182 ms|
|AKAZE/BRISK|137/416.329225 ms|125/410.005330 ms|129/410.182110 ms|129/417.882668 ms|131/424.318740 ms|132/435.867304 ms|142/429.015721 ms|146/424.722338 ms|144/421.783174 ms|
|AKAZE/BRIEF|141/78.623886 ms|134/86.237252 ms|131/91.367707 ms|130/82.034298 ms|134/88.445526 ms|146/90.622980 ms|150/91.828024 ms|148/78.380702 ms|152/81.666420 ms|
|AKAZE/FREAK|126/113.796395 ms|129/124.810393 ms|127/125.608182 ms|121/117.091715 ms|122/128.625004 ms|133/138.377084 ms|144/140.766683 ms|147/134.192121 ms|138/127.414835 ms|
|AKAZE/SIFT|134/104.803453 ms|134/98.688369 ms|130/99.585154 ms|136/112.763822 ms|137/107.626612 ms|147/105.402946 ms|147/103.189962 ms|154/106.816352 ms|151/108.517316 ms|
|AKAZE/ORB|131/82.228464 ms|129/80.458016 ms|127/84.154985 ms|117/75.790423 ms|130/95.935461 ms|131/94.751283 ms|137/90.256413 ms|135/87.209125 ms|145/73.767014 ms|
|AKAZE/AKAZE|138/148.145053 ms|138/152.185504 ms|133/148.318558 ms|127/161.126832 ms|129/144.632524 ms|146/155.416344 ms|147/152.705138 ms|151/149.797987 ms|150/159.306968 ms|
|SIFT/BRISK|64/478.937904 ms|66/475.784425 ms|62/475.337119 ms|66/484.890023 ms|59/470.677821 ms|64/474.854342 ms|64/466.588669 ms|67/474.839084 ms|80/478.099473 ms|
|SIFT/BRIEF|86/131.154342 ms|78/139.247256 ms|76/133.791671 ms|85/136.041640 ms|69/137.420115 ms|74/130.265425 ms|76/135.013901 ms|70/134.285835 ms|88/136.529975 ms|
|SIFT/FREAK|65/176.212215 ms|72/180.719733 ms|64/176.445658 ms|66/192.961412 ms|59/181.400896 ms|59/170.925794 ms|64/177.457854 ms|65/178.589850 ms|79/181.153890 ms|
|SIFT/SIFT|82/180.819280 ms|81/187.446354 ms|85/173.272273 ms|93/187.826023 ms|90/190.375658 ms|81/180.069843 ms|82/191.275399 ms|102/179.682984 ms|104/201.946710 ms|


* KAZE/AKAZE descriptors will only work with KAZE/AKAZE keypoints.
* SIFT/ORB Segmentation Fault

## Task MP.9
Your ninth task is to log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this information you will then suggest the TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles. Finally, in a short text, please justify your recommendation based on your observations and on the data you collected.
#### Efficiency (matches/ms)

| Det/Desc |Total Matching/Total Time |Ratio |
| :-------: | :--: | :--: | 
|HARRIS/BRISK|142 / 3309.792572 ms|0.042903 matches/ms|
|HARRIS/BRIEF|173 / 205.572772 ms|0.841551 matches/ms|
|HARRIS/FREAK|144 / 505.838594 ms|0.284676 matches/ms|
|HARRIS/SIFT|163 / 320.908251 ms|0.507933 matches/ms|
|HARRIS/ORB|162 / 166.938886 ms|0.970415 matches/ms|
|SHITOMASI/BRISK|767 / 3165.183603 ms|0.242324 matches/ms|
|SHITOMASI/BRIEF|944 / 116.832761 ms|8.079925 matches/ms|
|SHITOMASI/FREAK|768 / 500.577007 ms|1.534229 matches/ms|
|SHITOMASI/SIFT|927 / 279.307198 ms|3.318926 matches/ms|
|SHITOMASI/ORB|908 / 120.232969 ms|7.552005 matches/ms|
|FAST/BRISK|899 / 3058.198621 ms|0.293964 matches/ms|
|FAST/BRIEF|1099 / 22.960734 ms|47.864324 matches/ms|
|FAST/FREAK|878 / 380.201239 ms|2.309303 matches/ms|
|FAST/SIFT|1046 / 230.142660 ms|4.545007 matches/ms|
|FAST/ORB|1071 / 26.863188 ms|39.868686 matches/ms|
|BRISK/BRISK|1570 / 6490.297011 ms|0.241900 matches/ms|
|BRISK/BRIEF|1704 / 3469.444028 ms|0.491145 matches/ms|
|BRISK/FREAK|1524 / 3839.157322 ms|0.396962 matches/ms|
|BRISK/SIFT|1646 / 3863.643113 ms|0.426023 matches/ms|
|BRISK/ORB|1514 / 3465.673206 ms|0.436856 matches/ms|
|ORB/BRISK|751 / 3118.178420 ms|0.240846 matches/ms|
|ORB/BRIEF|545 / 81.842528 ms|6.659130 matches/ms|
|ORB/FREAK|420 / 432.941455 ms|0.970108 matches/ms|
|ORB/SIFT|763 / 522.351990 ms|1.460701 matches/ms|
|ORB/ORB|763 / 116.807599 ms|6.532109 matches/ms|
|AKAZE/BRISK|1215 / 3790.106610 ms|0.320571 matches/ms|
|AKAZE/BRIEF|1266 / 769.206795 ms|1.645851 matches/ms|
|AKAZE/FREAK|1187 / 1150.682412 ms|1.031562 matches/ms|
|AKAZE/SIFT|1270 / 947.393986 ms|1.340519 matches/ms|
|AKAZE/ORB|1182 / 764.551184 ms|1.546005 matches/ms|
|AKAZE/AKAZE|1259 / 1371.634908 ms|0.917883 matches/ms|
|SIFT/BRISK|592 / 4280.008860 ms|0.138317 matches/ms|
|SIFT/BRIEF|702 / 1213.750160 ms|0.578373 matches/ms|
|SIFT/FREAK|593 / 1615.867302 ms|0.366986 matches/ms|
|SIFT/SIFT|800 / 1672.714524 ms|0.478265 matches/ms|

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
