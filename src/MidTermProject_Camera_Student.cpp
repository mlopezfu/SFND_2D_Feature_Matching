/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;
//#define createTableDetectors 


/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{   // Changed here to be used in the iteration.
    string detectorTypes[]={"HARRIS", "SHITOMASI", "FAST", "BRISK", "ORB", "AKAZE",  "SIFT"};
    string descriptorTypes[]={"BRISK", "BRIEF", "FREAK",   "SIFT","ORB","AKAZE"};
#ifdef createTableDetectors
    string detectorsTable="| Detectors | 01   | 02   | 03   | 04   | 05   | 06   | 07   | 08   | 09   | 10   |Total |\r\n| :-------: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: |\r\n";
    string detDescTable="| Det/Desc | 01-02   | 02-03   | 03-04   | 04-05   | 05-06   | 06-07   | 07-08   | 08-09   | 09-10| \r\n| :-------: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: |\r\n";
    string detDescTableTime="| Det/Desc |Total Matching/Total Time |Ratio |\r\n| :-------: | :--: | :--: |\r\n";
    
    int detectorsTotal=0;
    int detDescTotal=0;
    double detDescTimeTotal=0;
for (int dt=0;dt<7;dt++)
{// To iterate throug detector types
    cout << endl << endl << endl << endl;
    cout << "Detector: "<< detectorTypes[dt] << endl;
    if(detectorsTotal!=0)
    {   
        detectorsTable+=to_string(detectorsTotal); 
        detectorsTable+="|\n\r";
        detectorsTotal=0;
    }
    detectorsTable+="|";
    detectorsTable+=detectorTypes[dt]; 
    detectorsTable+="|";
    int ndescriptors=5;
    if(dt==5)
        ndescriptors=6;
    if(dt==6)
        ndescriptors=4;

    for(int desct=0;desct<ndescriptors;desct++)
    {
    cout << endl << endl << endl << endl;
    cout << "Detector: "<< detectorTypes[dt] << " Descriptor: "<< descriptorTypes[desct] << endl;
    if(detDescTotal!=0)
    {   
        detDescTableTime+=to_string(detDescTotal)+" / "+to_string(1000*detDescTimeTotal)+" ms|";
        detDescTableTime+=to_string(detDescTotal/(1000*detDescTimeTotal))+" matches/ms|\n\r"; 
        detDescTable+="\n\r";
        detDescTotal=0;
        detDescTimeTotal=0;
    }
    detDescTable+="|"+detectorTypes[dt]+"/"+descriptorTypes[desct]+"|"; 
    detDescTableTime+="|"+detectorTypes[dt]+"/"+descriptorTypes[desct]+"|";
   
#endif
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = true;//false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if (dataBuffer.size() >=dataBufferSize) // wait until at least two images have been processed
        {
            dataBuffer.erase(dataBuffer.begin());
        }
        dataBuffer.push_back(frame);
        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, FREAK, SIFT
        //string detectorTypes[]={"HARRIS", "SHITOMASI", "FAST", "BRISK", "ORB", "AKAZE", "FREAK", "SIFT"};
        #ifdef createTableDetectors
        string detectorType = detectorTypes[dt];
        double t = (double)cv::getTickCount();
        #else
        string detectorType = detectorTypes[0];// 0 Harris
        #endif
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
        #ifdef createTableDetectors
        detectorsTable+=to_string(keypoints.size()); 
        detectorsTable+="|";
        detectorsTotal+=keypoints.size();
        #endif
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            // Create a new vector of keypoints
            vector<cv::KeyPoint> keypointsInside;
            for (auto kp : keypoints) {
                if (vehicleRect.contains(kp.pt)) keypointsInside.push_back(kp);
            }
            keypoints = keypointsInside;
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        //string descriptorTypes[]={"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
        #ifdef createTableDetectors
        string descriptorType = descriptorTypes[desct];
        #else
        string descriptorType = descriptorTypes[0];  //0 BRISK
        #endif
        //string descriptorType = descriptorTypes[0];
        //string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
        
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        

        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1 && !(detectorType=="SIFT" && descriptorType=="ORB")) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorCategory = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
            if (descriptorType.compare("SIFT") == 0)
            {
                descriptorCategory="DES_HOG";
            }
            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
            
            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorCategory, matcherType, selectorType);
            
            #ifdef createTableDetectors
            t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            detDescTable+=to_string(matches.size())+"|"; 
            /*
            detDescTable+="/"; 
            detDescTable+=to_string(1000 * t / 1.0); 
            
            detDescTable+=" ms|";
            */
            detDescTotal+=matches.size();
            detDescTimeTotal+=t;
            #endif
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            
        }

    } // eof loop over all images
#ifdef createTableDetectors   
    }
}
    detectorsTable+=to_string(detectorsTotal); 
    detectorsTable+="|\n\r";    
    ofstream detectorsResults;
    detectorsResults.open("resultadosDetectores.txt");
    detectorsResults << detectorsTable;
    detectorsResults.close();
    //detDescTable+=to_string(detDescTotal);//+" / "+to_string(1000*detDescTimeTotal)+" ms|";
    detDescTableTime+=to_string(detDescTotal/(1000*detDescTimeTotal))+" matches/ms|"; 
    detDescTable+="\n\r";
    ofstream descriptorResults;
    descriptorResults.open("resultadosDescriptores.txt");
    descriptorResults << detDescTable << "\n\r" << detDescTableTime;
    descriptorResults.close();
    
#endif
    return 0;
}
