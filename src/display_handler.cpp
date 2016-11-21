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
	int resizedwidth{ (image->cols*settings::disp::kpixheight) / image->rows };
	int borderthickness{ (settings::disp::kpixwidth - resizedwidth) / 2 };
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
	while( !(*exitsignal) ) {
		//Get latest image
		displaymutex->lock();
		imagetemp = *image;
		displaymutex->unlock();
		
		//Resize if necessary
		if ( imagetemp.rows != settings::disp::kpixheight ) {
			cv::resize( imagetemp,
						imagetemp,
						cv::Size(resizedwidth, settings::disp::kpixheight) );
		}
		
		//Format
		cv::copyMakeBorder( imagetemp,
							imagetemp,
							0,
							0,
							borderthickness,
							borderthickness,
							cv::BORDER_CONSTANT,
							cv::Scalar(0) );

		#ifdef __arm__
		//OpenGL implementation
		buffer.copyFrom(imagetemp, cv::ogl::Buffer::ARRAY_BUFFER, true);
		cv::imshow( "Output", buffer );
		#else
		//Display
		cv::imshow( "Output", imagetemp );
		#endif
		cv::waitKey( 1 );
		//Set pace
		displaypacer.SetPace();
	}
	
	std::cout << "Exiting display handler thread!" << '\n';

}
