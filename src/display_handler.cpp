#include <iostream>
#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"
#include "pace_setter_class.h"
#include "xml_reader.h"

#ifdef __arm__									//Detect if compiling for raspberry pi
	#include <gtk/gtk.h>	
	#include "opencv2/core/opengl.hpp"
#endif

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
		std::cout << "Captured image too large for display, esiting!" << '\n';
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
	cv::namedWindow( "Output", cv::WINDOW_OPENGL );
	cv::setWindowProperty( "Output", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN );
	cv::ogl::Buffer buffer;
	#else
	cv::namedWindow( "Output", CV_WINDOW_NORMAL );
	#endif
	std::cout << "Display opened!" << '\n';

	
	//Loop
	while( !(*exitsignal || (cv::waitKey(1) == 27)) ) {
		//Set pace - at beginning due to waitkey
		displaypacer.SetPace();
		
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
		buffer.copyFrom(imagetemp, cv::ogl::Buffer::ARRAY_BUFFER, true);
		cv::imshow( "Output", buffer );
		#else
		//Display
		cv::imshow( "Output", imagetemp );
		#endif
	}
	
	//In case of escape returned from waitkey, initiate exit
	*exitsignal = true;
	
	std::cout << "Exiting display handler thread!" << '\n';

}
