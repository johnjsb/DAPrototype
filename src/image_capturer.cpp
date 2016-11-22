/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

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
	std::cout << "Image capturer thread starting!" << '\n';

    //Create camera
	#ifdef __arm__								//Detect if compiling for raspberry pi
	raspicam::RaspiCam_Cv Camera;             	//For Raspberry Pi
	#else
	cv::VideoCapture stream1( 0 );                //For Laptop
	#endif

	//Set properties
	#ifdef __arm__								//Detect if compiling for raspberry pi
	Camera.set( CV_CAP_PROP_FRAME_WIDTH, settings::cam::kpixwidth );
	Camera.set( CV_CAP_PROP_FRAME_HEIGHT, settings::cam::kpixheight );
	Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
	#endif

    //Open
	#ifdef __arm__								//Detect if compiling for raspberry pi
	if ( !Camera.open() ) {                    	//For Raspberry Pi
	#else
	if ( !stream1.isOpened() ) {                //For Laptop
	#endif
		std::cerr<<"Error opening the camera"<<'\n';
		exit(-1);
	}
	std::cout << "Camera opened succesfully!" << '\n';

	//create pace setter
	PaceSetter camerapacer( std::max(std::max(settings::disp::kupdatefps,
											  settings::cam::krecfps),
									 settings::ldw::kupdatefps),
							"Image capturer");

	//Loop indefinitely
	while( !(*exitsignal) ) {
		cv::Mat newimage;
		#ifdef __arm__							//Detect if compiling for raspberry pi
		(Camera.grab());                       	//For Raspberry Pi
		(Camera.retrieve(newimage));           	//For Raspberry Pi
		cv::flip(newimage, newimage, -1);
		#else
		stream1.read(newimage);                 //For Laptop
		#endif
		//resize image
		#ifndef __arm__
		if ( newimage.rows != settings::cam::kpixheight ) {
			cv::resize( newimage,
						newimage,
						cv::Size(settings::cam::kpixwidth,
								 settings::cam::kpixheight) );
		}
		#endif
		
		capturemutex->lock();
		*capture = newimage;
		capturemutex->unlock(); 

		camerapacer.SetPace();
	}

	std::cout << "Exiting image capturer thread!" << '\n';

}
