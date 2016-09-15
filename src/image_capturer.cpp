#include <iostream>
#include <algorithm> 
#include <mutex>
#include <atomic>
#include "pace_setter_class.h"
#include "xml_reader.h"
#include "opencv2/opencv.hpp"
//#include "raspicam/raspicam_cv.h"             //For Raspberry Pi

void CaptureImageThread( cv::Mat *capture,
                         std::mutex *capturemutex,
                         std::atomic<bool> *exitsignal )
{
	std::cout << "Image capturer thread starting!" << std::endl;

    //Create camera
    cv::VideoCapture stream1(0);                //For Laptop
	//raspicam::RaspiCam_Cv Camera;             //For Raspberry Pi

	//Set properties
    //Camera.set(CV_CAP_PROP_FORMAT, CV_16U);   //For Raspberry Pi

    //Open
    if (!stream1.isOpened())                    //For Laptop
    //if (!Camera.open())                       //For Raspberry Pi
	{
		std::cerr<<"Error opening the camera"<<std::endl;
		exit(-1);
	}
	std::cout << "Camera opened succesfully!" << std::endl;
	
	//Create thread variables
	cv::Mat newimage;

	//create pace setter
	PaceSetter camerapacer(std::max(settings::disp::kupdatefps,
		settings::cam::krecfps), "Image capturer");

	//Loop indefinitely
	while( !(*exitsignal) ) {
        stream1.read(newimage);                 //For Laptop
        //(Camera->grab());                       //For Raspberry Pi
		//(Camera->retrieve(newimage));           //For Raspberry Pi
		capturemutex->lock();
		*capture = newimage;
		capturemutex->unlock(); 

		camerapacer.SetPace();
	}

	std::cout << "Exiting image capturer thread!" << std::endl;

}
