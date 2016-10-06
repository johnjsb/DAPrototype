#include <iostream>
#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"
#include "pace_setter_class.h"
#include "xml_reader.h"

#ifdef __arm__									//Detect if compiling for raspberry pi
	#include <gtk/gtk.h>
#endif

void DisplayUpdateThread( cv::Mat *image,
                          std::mutex *displaymutex,
	                      std::atomic<bool> *exitsignal )
{

	std::cout << "Display handler thread starting!" << std::endl;

	//Check xml settings 
	if ( !settings::disp::kenabled ) {
		std::cout << "Display disabled, exiting thread!" << std::endl;
		return;
	}
	
	//Check image is initialized
	while ( image->empty() ) {
		if (*exitsignal) {
			return;
		}	  
	}
	

	//Create thread variables
	int resizedwidth{ ((image->cols*settings::disp::kpixheight)/image->rows) };
	int borderthickness{ (settings::disp::kpixwidth - resizedwidth)/2 };
	cv::Mat imagetemp{ cv::Mat(image->rows, image->cols, image->type(), cv::Scalar(0)) };
	cv::Mat borderedimage{ cv::Mat(settings::disp::kpixheight,
		settings::disp::kpixwidth, image->type(), cv::Scalar(0)) };
	#ifdef __arm__								//Detect if compiling for raspberry pi
		//This checks if the raspberry pi is running headless (for better performance)
		if (!gtk_init_check(NULL, NULL)){
			std::cout << "Display unavailable, continuing without..." << std::endl;
			return;
		}
	#endif
	std::cout << "Attempting to open display..." << std::endl;
	displaymutex->lock();
	imagetemp = *image;
	displaymutex->unlock();
	if ( imagetemp.rows != borderedimage.rows ) {
		cv::resize(imagetemp, imagetemp, cv::Size(resizedwidth, borderedimage.rows));
	}
	imagetemp.copyTo(borderedimage.rowRange(0, imagetemp.rows).colRange(
		borderthickness, settings::disp::kpixwidth - borderthickness));
	#ifdef __arm__								//Detect if compiling for raspberry pi
		cv::namedWindow("Output", cv::WINDOW_OPENGL );
		cv::setWindowProperty("Output", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	#else
		cv::namedWindow("Output", CV_WINDOW_NORMAL);
	#endif
	cv::imshow("Output", borderedimage);
	cv::waitKey(1);
	std::cout << "Display opened!" << std::endl;

	//create pace setter
	PaceSetter displaypacer(settings::disp::kupdatefps, "display handler");
	
	//Loop indefinitely
	while( !(*exitsignal) ) {
		displaymutex->lock();
		imagetemp = *image;
		displaymutex->unlock();
		if ( imagetemp.rows < borderedimage.rows ) {
			cv::pyrUp(imagetemp,imagetemp,cv::Size(resizedwidth, borderedimage.rows));
		} else if ( imagetemp.rows > borderedimage.rows ) {
			cv::pyrDown(imagetemp,imagetemp,cv::Size(resizedwidth, borderedimage.rows));
		}
		imagetemp.copyTo(borderedimage.rowRange(0, imagetemp.rows).colRange(
			borderthickness, settings::disp::kpixwidth - borderthickness));
		cv::imshow("Output", borderedimage);
		cv::waitKey(1);

		displaypacer.SetPace();
	}
	
	std::cout << "Exiting display handler thread!" << std::endl;

}

#undef FULLSCREENMODE
