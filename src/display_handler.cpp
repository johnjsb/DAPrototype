/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

//Standard libraries
#include <iostream>
#include <mutex>
#include <atomic>
#ifdef __arm__									//Detect if compiling for raspberry pi
	#include <gtk/gtk.h>	
#endif

//3rd party libraries
#include "opencv2/opencv.hpp"
#ifdef __arm__									//Detect if compiling for raspberry pi
	//#include "opencv2/core/opengl.hpp"
#endif

//Project libraries
#include "pace_setter_class.h"
#include "xml_reader.h"

/*****************************************************************************************/
void DisplayUpdateThread( cv::Mat *image,
                          std::mutex *displaymutex,
	                      std::atomic<bool> *exitsignal )
{
	std::cout << "Display handler thread starting!" << '\n';

	//Check xml settings 
	if ( !settings::disp::kenabled ) {
		std::cout << "Display disabled, exiting thread!" << '\n';
		return;
	}
	
	//create pace setter
	PaceSetter displaypacer( settings::disp::kupdatefps, "display handler" );
	
	//Check image is initialized
	while ( image->empty() ) {
		if ( *exitsignal ) {
			return;
		}
		displaypacer.SetPace();
	}

	//Create thread variables
	int sideborderthickness{ (settings::disp::kpixwidth - image->cols) / 2 };
	int topborderthickness{ (settings::disp::kpixheight - image->rows) / 2 };
	
	//Check image sizing to prevent exception
	if ( (sideborderthickness < 0) || (topborderthickness < 0) ) {
		std::cout << "Captured image too large for display, exiting!" << '\n';
		return;
	}
	
	cv::Mat imagetemp{ image->rows, image->cols, image->type(), cv::Scalar(0) };
	
	//Initialize display with first image
	#ifdef __arm__								//Detect if compiling for raspberry pi
	//Check if running headless
	if ( !gtk_init_check(NULL, NULL) ){
		std::cout << "Display unavailable, continuing without..." << '\n';
		return;
	}
	#endif
	std::cout << "Attempting to open display..." << '\n';
	#ifdef __arm__								//Detect if compiling for raspberry pi
	//cv::namedWindow( "Output", cv::WINDOW_OPENGL );
	cv::namedWindow( "Output", cv::WINDOW_NORMAL );
	cv::setWindowProperty( "Output", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN );
	//cv::ogl::Buffer buffer;
	#else
	cv::namedWindow( "Output", cv::WINDOW_NORMAL );
	#endif
	std::cout << "Display opened!" << '\n';

	
	//Loop
	while( !(*exitsignal) ) {
		//Get latest image
		displaymutex->lock();
		cv::copyMakeBorder( *image,
							imagetemp,
							topborderthickness,
							topborderthickness,
							sideborderthickness,
							sideborderthickness,
							cv::BORDER_CONSTANT,
							cv::Scalar(0) );
		displaymutex->unlock();

		#ifdef __arm__
		//OpenGL implementation
		//buffer.copyFrom(imagetemp, cv::ogl::Buffer::ARRAY_BUFFER, true);
		//cv::imshow( "Output", buffer );
		cv::imshow( "Output", imagetemp );
		cv::waitKey( 1 );
		#else
		//Display
		cv::imshow( "Output", imagetemp );
		cv::waitKey( 1 );
		#endif
		
		//Set pace
		displaypacer.SetPace();
	}
	
	std::cout << "Exiting display handler thread!" << '\n';
	return;

}
