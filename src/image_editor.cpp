#include <iostream>
#include <iomanip>
#include <algorithm>
#include <math.h>
#include <sstream>
#include <mutex>
#include <atomic>
#include "image_editor.h"
#include "lane_detect_processor.h"
#include "pace_setter_class.h"
#include "xml_reader.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

void ImageEditorThread( cv::Mat *orgimage,
                        std::mutex *capturemutex,
						cv::Mat *displayimage,
						std::mutex *displaymutex,
						ProcessValues *processvalues,
						std::atomic<bool> *exitsignal )
{

	std::cout << "Image editor thread starting!" << std::endl;
		
	//Check image is initialized
	while ( orgimage->empty() ) {
		if (*exitsignal) {
			return;
		}	  
	}
	
	//Thread variables
    time_t now{time(0)};
	std::string timetext{ asctime(localtime(&now)) };
	timetext.pop_back();
	capturemutex->lock();
	float widthscalefactor{ orgimage->cols / 640.0};
	float heightscalefactor{ orgimage->rows / 480.0};
	capturemutex->unlock();
	cv::Point datetimelocation{ cv::Point(395 * widthscalefactor,470 * heightscalefactor) };
	float datetimesize{ 0.5f * heightscalefactor };
	cv::Point speedlocation{ cv::Point(515 * widthscalefactor,30 * heightscalefactor) };
	float speedsize{ 0.75f * heightscalefactor };
	cv::Point latlonglocation{ cv::Point(10 * widthscalefactor, 470 * heightscalefactor) };
	float latlongsize{ 0.5f * heightscalefactor };
	cv::Point followingtimelocation{ cv::Point(10 * widthscalefactor, 455 * heightscalefactor) };
	float followingtimesize{ 0.5f * heightscalefactor };
	cv::Point distancelocation{ cv::Point(100 * widthscalefactor, 455 * heightscalefactor) };
	float distancesize{ 0.5f * heightscalefactor };
	cv::Point diagnosticlocation{ cv::Point(10 * widthscalefactor, 20 * heightscalefactor) };
	float diagnosticsize{ 0.5f * heightscalefactor };
	

	//create pace setter
	PaceSetter editorpacer(std::max(settings::disp::kupdatefps,
		settings::cam::krecfps), "image editor");
	
	//Loop indefinitely
	while( !(*exitsignal) ) {
		//Get original image
		capturemutex->lock();
		cv::Mat modifiedimage{ orgimage->clone() };
		capturemutex->unlock();
        now = time(0);
		timetext = asctime( localtime(&now) );
		timetext.pop_back();
		
		//Resize (if necesssary)
		
		//Show time
		putText( modifiedimage, timetext, datetimelocation, CV_FONT_HERSHEY_COMPLEX,
				datetimesize, cv::Scalar(255,255,0), 1, cv::LINE_8, false );
		//Show speed
		std::stringstream speedtext;
		speedtext << std::fixed << std::setprecision(1) << processvalues->gpsspeed_ << " mph";
		putText( modifiedimage, speedtext.str(), speedlocation, CV_FONT_HERSHEY_COMPLEX,
				speedsize, cv::Scalar(0,255,0), 1, cv::LINE_8, false );
		
		//Show latitude and longitude
		putText( modifiedimage, ConvertLatLong(processvalues->latitude_, processvalues->longitude_),
			latlonglocation, CV_FONT_HERSHEY_COMPLEX, latlongsize, cv::Scalar(255,0,255), 1,
			cv::LINE_8,	false );
			
		//Show following time
		std::stringstream timetext;
		timetext  << std::fixed << std::setprecision(2) << processvalues->timetocollision_ <<
			" s";
		putText( modifiedimage, timetext.str(),	followingtimelocation,
			CV_FONT_HERSHEY_COMPLEX, followingtimesize, cv::Scalar(255,255,255), 1, cv::LINE_8,	false );
		
		//Show following distance
		std::stringstream distancetext;
		distancetext  << std::fixed << std::setprecision(2) << processvalues->forwarddistance_ <<
			" m";
		putText( modifiedimage, distancetext.str(),	distancelocation,
			CV_FONT_HERSHEY_COMPLEX, distancesize, cv::Scalar(255,255,255), 1, cv::LINE_8, false );
			
		//Show diagnostic message
		std::string diagnosticmessage{ GetDiagnosticString(processvalues->ldwstatus_,
			processvalues->fcwstatus_, processvalues-> gpsstatus_) };
		if ( diagnosticmessage.length() != 0 ) {
			putText( modifiedimage, diagnosticmessage, diagnosticlocation,
				CV_FONT_HERSHEY_COMPLEX, diagnosticsize, cv::Scalar(0,0,255), 1, cv::LINE_8,
				false );
		}
		
				
		//Overlay lanes
		Polygon newpolygon = processvalues->GetPolygon();
		cv::Point cvpointarray[4];
		std::copy( newpolygon.begin(), newpolygon.end(), cvpointarray );
		if ( (newpolygon[0] != cv::Point(0,0)) && settings::cam::kshadelanes &&
			(processvalues->ldwstatus_ > 0) ) {
			cv::Mat polygonimage{ modifiedimage.size(), modifiedimage.type(),
				cv::Scalar(0) };
			cv::fillConvexPoly( polygonimage, cvpointarray, 4,  cv::Scalar(0,255,0) );
			OverlayImage( &polygonimage, &modifiedimage, 0.5 );
		}
		
		//Write display image
		displaymutex->lock();
		*displayimage = modifiedimage;
		displaymutex->unlock();
		
		editorpacer.SetPace();
	}
	
	std::cout << "Exiting image editor thread!" << std::endl;

}
/*****************************************************************************************/
void OverlayImage( cv::Mat* overlay,
                   cv::Mat* src,
                   double transparency )
{
    for (int i = 0; i < src->cols; i++) {
        for (int j = 0; j < src->rows; j++) {
            cv::Vec3b &intensity = src->at<cv::Vec3b>(j, i);
            cv::Vec3b &intensityoverlay = overlay->at<cv::Vec3b>(j, i);
            for(int k = 0; k < src->channels(); k++) {
                uchar col = intensityoverlay.val[k];
                if (col != 0){
                    intensity.val[k] = (intensity.val[k]*(1-transparency) +
						(transparency*col));
                }
            }
        }
    }
}
/*****************************************************************************************/
std::string ConvertLatLong ( double latitude, 
                             double longitude )
{
	//Latitude
	bool north{false};
	if ( latitude > 0.0 ) {
		north = true;
	} else {
		latitude = -latitude;
	}
	uint16_t degrees{ static_cast<uint16_t>(latitude) };
	uint16_t minutes{ static_cast<uint16_t>((latitude -
		static_cast<double>(degrees))*60.0) };
	double seconds{ (latitude - static_cast<double>(degrees) -
		(static_cast<double>(minutes)/60.0))*3600.0 };
	std::stringstream secondstext;
	secondstext  << std::fixed << std::setprecision(2) <<  seconds;
	std::string positionstring{ std::to_string(degrees) + "* " + std::to_string(minutes) +
		"' " + secondstext.str()  + "\"" };
	if (north) {
		positionstring += " N";
	} else {
		positionstring += " S";
	}
	positionstring += ", ";
	
	//Longitude
	bool east{false};
	if ( longitude > 0.0 ) {
		east = true;
	} else {
		longitude = -longitude;
	}
	degrees = static_cast<uint16_t>(longitude);
	minutes = static_cast<uint16_t>((longitude - static_cast<double>(degrees))*60.0);
	seconds = (longitude - static_cast<double>(degrees) -
		(static_cast<double>(minutes)/60.0))*3600.0;
	secondstext.str(std::string());
	secondstext  << std::fixed << std::setprecision(2) <<  seconds;
	positionstring += std::to_string(degrees) + "* " + std::to_string(minutes) + "' " +
		secondstext.str()  + "\"";
	if (east) {
		positionstring += " E";
	} else {
		positionstring += " W";
	}
	return positionstring;
}
/*****************************************************************************************/
std::string GetDiagnosticString ( int ldwstatus, int fcwstatus, int gpsstatus )
{
	std::string diagnosticstring{ "" };
	//LDW
	switch ( ldwstatus ) {
		case 1:
			diagnosticstring += "LCW left alarm, ";
			break;
		case 2:
			diagnosticstring += "LCW left warning, ";
			break;
		case 5:
			diagnosticstring += "LCW right warning, ";
			break;
		case 6:
			diagnosticstring += "LCW right alarm, ";
			break;
		case -1:
			diagnosticstring += "LCW lane detection failed, ";
			break;
	}
	//FCW
	switch ( fcwstatus ) {
		case 1:
			diagnosticstring += "FCW warning, ";
			break;
		case 2:
			diagnosticstring += "FCW alarm, ";
			break;
		case 3:
			diagnosticstring += "FCW tailgate warning, ";
			break;
		case 4:
			diagnosticstring += "FCW tailgate alarm, ";
			break;
		case 5:
			diagnosticstring += "FCW tailgate alarm, ";
			break;
		case -1:
			diagnosticstring += "FCW driver pull ahead detected, ";
			break;
	}
	//GPS
	switch ( gpsstatus ) {
		case 1:
			diagnosticstring += "GPS no lock, ";
			break;
		case 4:
			diagnosticstring += "Speed warning, ";
			break;
		case 5:
			diagnosticstring += "Speed alarm, ";
			break;
		case -1:
			diagnosticstring += "GPS sensor failure, ";
			break;
	}
	
	//Drop comma and whitespace on last message
	if ( diagnosticstring.length() > 1 ) {
		diagnosticstring.pop_back();
		diagnosticstring.pop_back();
	}
	return diagnosticstring;
}
