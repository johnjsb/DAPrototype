#include <iostream>
#include <algorithm> 
#include <mutex>
#include <atomic>
#include "pace_setter_class.h"
#include "xml_reader.h"
#include "opencv2/opencv.hpp"

#ifdef __arm__									//Detect if compiling for raspberry pi
	#include "raspicam/raspicam_cv.h"             	//For Raspberry Pi
#endif

void CaptureImageThread( cv::Mat *capture,
                         std::mutex *capturemutex,
                         std::atomic<bool> *exitsignal )
{
	std::cout << "Image capturer thread starting!" << std::endl;

    //Create camera
	#ifdef __arm__								//Detect if compiling for raspberry pi
		raspicam::RaspiCam_Cv Camera;             	//For Raspberry Pi
	#else
		cv::VideoCapture stream1(0);                //For Laptop
	#endif

	//Set properties
	#ifdef __arm__								//Detect if compiling for raspberry pi
		Camera.set(CV_CAP_PROP_FORMAT, CV_16U);   	//For Raspberry Pi
	#endif

    //Open
	#ifdef __arm__								//Detect if compiling for raspberry pi
		if (!Camera.open())                       	//For Raspberry Pi
	#else
		if (!stream1.isOpened())                    //For Laptop
	#endif
	{
		std::cerr<<"Error opening the camera"<<std::endl;
		exit(-1);
	}
	std::cout << "Camera opened succesfully!" << std::endl;
	
	//Create thread variables
	cv::Mat newimage;

	//create pace setter
	PaceSetter camerapacer(std::max(std::max(settings::disp::kupdatefps,
		settings::cam::krecfps), settings::ldw::kupdatefps), "Image capturer");

	//Loop indefinitely
	while( !(*exitsignal) ) {
		#ifdef __arm__							//Detect if compiling for raspberry pi
			(Camera.grab());                       	//For Raspberry Pi
			(Camera.retrieve(newimage));           	//For Raspberry Pi
			cv::flip(newimage, newimage, -1);
		#else
			stream1.read(newimage);                 //For Laptop
		#endif
		//resize image
		cv::resize(newimage, newimage, cv::Size(settings::cam::kpixwidth,
			settings::cam::kpixheight));
		capturemutex->lock();
		*capture = newimage;
		capturemutex->unlock(); 

		camerapacer.SetPace();
	}

	std::cout << "Exiting image capturer thread!" << std::endl;

}
