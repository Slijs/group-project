/* boneCV.cpp
 *
 * Copyright Derek Molloy, School of Electronic Engineering, Dublin City University
 * www.derekmolloy.ie
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that source code redistributions retain this notice.
 *
 * This software is provided AS IS and it comes with no warranties of any type.
 * 
 * Further modifications done by Philip Brink to capture only. Uncomment 
 * `capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));` if your webcam supports this.
 * Currently the resolution for my camera must be set to 1920x1080, so that it will auto select MJPG
 * format, and setting the format manually does not work.
 */

#include<iostream>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main()
{
    // Open the first camera
    VideoCapture capture(0);
    // Set the size and format (format unsupported by this webcam,
    // which is why the size is set to 1920x1080. While this resolution
    // is not supported by my webcam, it is the only way to make it auto
    // select MJPG format.
    capture.set(CV_CAP_PROP_FRAME_WIDTH,1920);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
    //capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    if(!capture.isOpened()){
        cout << "Failed to connect to the camera." << endl;
    }
    // create a mat, and send the capture to it
    Mat frame;
    capture >> frame;
    if(frame.empty()){
	cout << "Failed to capture an image" << endl;
	return -1;
    }
    // write the capture to a file
    imwrite("capture.png", frame);
    return 0;
}
