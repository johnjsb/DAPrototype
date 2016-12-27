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
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <mutex>
#include <atomic>
#include <math.h>
#include <exception>
#include <string>

//3rd party libraries
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//Project libraries
#include "image_editor.h"
#include "lane_detect_processor.h"
#include "pace_setter_class.h"
#include "xml_reader.h"

/*****************************************************************************************/
void ImageEditorThread( cv::Mat *orgimage,
                        std::mutex *capturemutex,
						cv::Mat *displayimage,
						std::mutex *displaymutex,
						ProcessValues *processvalues,
						std::atomic<bool> *exitsignal )
{

	std::cout << "Image editor thread starting!" << '\n';

	//create pace setter
	PaceSetter editorpacer( std::max(settings::disp::kupdatefps,
									 settings::cam::krecfps),
							"image editor");
		
	//Check image is initialized
	while ( orgimage->empty() ) {
		if (*exitsignal) {
			return;
		}
		editorpacer.SetPace();
	}
	
	//Thread variables
    time_t now{ time(0) };
	std::string timetext{ asctime(localtime(&now)) };
	timetext.pop_back();
	capturemutex->lock();
	float widthscalefactor{ orgimage->cols / 800.0f };
	float heightscalefactor{ orgimage->rows / 480.0f };
	capturemutex->unlock();
	cv::Point datetimelocation{ cv::Point(550 * widthscalefactor,
										  470 * heightscalefactor) };
	float datetimesize{ 0.5f * heightscalefactor };
	cv::Point speedlocation{ cv::Point(660 * widthscalefactor,
									   30 * heightscalefactor) };
	float speedsize{ 0.75f * heightscalefactor };
	cv::Point latlonglocation{ cv::Point(10 * widthscalefactor,
										 470 * heightscalefactor) };
	float latlongsize{ 0.5f * heightscalefactor };
	cv::Point followingtimelocation{ cv::Point(10 * widthscalefactor,
											   455 * heightscalefactor) };
	float followingtimesize{ 0.5f * heightscalefactor };
	cv::Point distancelocation{ cv::Point(100 * widthscalefactor,
										  455 * heightscalefactor) };
	float distancesize{ 0.5f * heightscalefactor };
	cv::Point diagnosticlocation{ cv::Point(10 * widthscalefactor,
											20 * heightscalefactor) };
	float diagnosticsize{ 0.5f * heightscalefactor };
	
	//Loop indefinitely
	while( !(*exitsignal) ) {
		try {
			//Get original image
			capturemutex->lock();
			cv::Mat modifiedimage{ orgimage->clone() };
			capturemutex->unlock();
			now = time(0);
			timetext = asctime( localtime(&now) );
			timetext.pop_back();
			
			//Show time
			putText( modifiedimage,
					 timetext,
					 datetimelocation,
					 CV_FONT_HERSHEY_COMPLEX,
					 datetimesize,
					 cv::Scalar(255,255,0),
					 1,
					 cv::LINE_8,
					 false );
			//Show speed
			std::stringstream speedtext;
			speedtext << std::fixed <<
						 std::setprecision(1) << processvalues->gpsspeed_ << " mph";
			putText( modifiedimage,
					 speedtext.str(),
					 speedlocation,
					 CV_FONT_HERSHEY_COMPLEX,
					 speedsize,
					 cv::Scalar(0,255,0),
					 1,
					 cv::LINE_8,
					 false );
			
			//Show latitude and longitude
			putText( modifiedimage,
					 ConvertLatLong(processvalues->latitude_, processvalues->longitude_),
					 latlonglocation,
					 CV_FONT_HERSHEY_COMPLEX,
					 latlongsize,
					 cv::Scalar(255,0,255),
					 1,
					 cv::LINE_8,
					 false );
				
			//Show following time
			std::stringstream timetext;
			if (processvalues->fcwstatus_ > 0) {
				timetext  << std::fixed <<
							 std::setprecision(2) << processvalues->timetocollision_ << " s";
			} else {
				timetext  << "-.-- s";
			}
			putText( modifiedimage,
					 timetext.str(),
					 followingtimelocation,
					 CV_FONT_HERSHEY_COMPLEX,
					 followingtimesize,
					 cv::Scalar(255,255,255),
					 1,
					 cv::LINE_8,
					 false );
			
			//Show following distance
			std::stringstream distancetext;
			if ( processvalues->fcwstatus_ > 0 ) {
				distancetext  << std::fixed <<
								 std::setprecision(2) << processvalues->forwarddistance_ << " ft";
			} else {
				distancetext  << "-.-- ft";
			}
			putText( modifiedimage,
					 distancetext.str(),
					 distancelocation,
					 CV_FONT_HERSHEY_COMPLEX,
					 distancesize,
					 cv::Scalar(255,255,255),
					 1,
					 cv::LINE_8,
					 false );
				
			//Show diagnostic message
			std::string diagnosticmessage{ GetDiagnosticString(processvalues->ldwstatus_,
															   processvalues->fcwstatus_,
															   processvalues-> gpsstatus_) };
			if ( diagnosticmessage.length() != 0 ) {
				putText( modifiedimage,
						 diagnosticmessage,
						 diagnosticlocation,
						 CV_FONT_HERSHEY_COMPLEX,
						 diagnosticsize,
						 cv::Scalar(0,0,255),
						 1,
						 cv::LINE_8,
						 false );
			}
			
					
			//Overlay lanes
			Polygon newpolygon = processvalues->GetPolygon();
			cv::Point cvpointarray[4];
			std::copy( newpolygon.begin(), newpolygon.end(), cvpointarray );
			if ( (newpolygon[0] != cv::Point(0,0)) &&
				 settings::cam::kshadelanes &&
				 (processvalues->ldwstatus_ > 0) ) {
				cv::Mat polygonimage{ modifiedimage.size(),
									  CV_8UC1,
									  cv::Scalar(0) };
				cv::fillConvexPoly( polygonimage, cvpointarray, 4,  cv::Scalar(1) );
				OverlayImage( &polygonimage, &modifiedimage );
			}
			
			//Write display image
			displaymutex->lock();
			*displayimage = modifiedimage;
			displaymutex->unlock();
			
			editorpacer.SetPace();
		} catch ( const std::exception& ex ) {
			std::cout << "Image Editor thread threw exception: "<< ex.what() << '\n';
		} catch ( const std::string& str ) {
			std::cout << "Image Editor thread threw exception: "<< str << '\n';
		} catch (...) {
			std::cout << "Image Editor thread threw exception of unknown type!" << '\n';
		}
	}
	
	std::cout << "Exiting image editor thread!" << '\n';
	return;
}
/*****************************************************************************************/
void OverlayImage( cv::Mat* overlay,
                   cv::Mat* src )
{
    for (int i = 0; i < src->cols; i++) {
        for (int j = 0; j < src->rows; j++) {
			if ( overlay->at<uchar>(j, i) != 0 ) {
				cv::Vec3b &intensity = src->at<cv::Vec3b>(j, i);
				intensity.val[1] = (intensity.val[1] + 255) * 0.5f;
			}
        }
    }
	return;
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
	double degrees, minutes, seconds;
	minutes = modf( latitude, &degrees );
	seconds = modf( (minutes * 60.0), &minutes );
	seconds *= 60.0;
	std::stringstream  positionstring{ "" };
	positionstring << std::fixed << std::setprecision(0) << degrees << "* "
				   << minutes << "' " << std::setprecision(2) << seconds << "\"";
	if (north) {
		positionstring << " N";
	} else {
		positionstring << " S";
	}
	positionstring << ", ";
	
	//Longitude
	bool east{false};
	if ( longitude > 0.0 ) {
		east = true;
	} else {
		longitude = -longitude;
	}
	minutes = modf( longitude, &degrees );
	seconds = modf( (minutes * 60.0), &minutes );
	seconds *= 60.0;
	positionstring << std::fixed << std::setprecision(0) << degrees << "* "
				   << minutes << "' " << std::setprecision(2) << seconds << "\"";
	if (east) {
		positionstring << " E";
	} else {
		positionstring << " W";
	}
	return positionstring.str();
}
/*****************************************************************************************/
std::string GetDiagnosticString ( int ldwstatus, int fcwstatus, int gpsstatus )
{
	std::string diagnosticstring{ "" };
	//LDW
	switch ( ldwstatus ) {
		case LDW_LEFT_DEVIATION_ALARM:
			diagnosticstring += "LDW left alarm, ";
			break;
		case LDW_LEFT_DEVIATION_WARNING:
			diagnosticstring += "LDW left warning, ";
			break;
		case LDW_RIGHT_DEVIATION_WARNING:
			diagnosticstring += "LDW right warning, ";
			break;
		case LDW_RIGHT_DEVIATION_ALARM:
			diagnosticstring += "LDW right alarm, ";
			break;
		case LDW_ERROR:
			diagnosticstring += "LDW lane detection failed, ";
			break;
	}
	//FCW
	switch ( fcwstatus ) {
		case FCW_WARNING:
			diagnosticstring += "FCW warning, ";
			break;
		case FCW_ALARM:
			diagnosticstring += "FCW alarm, ";
			break;
		case FCW_TAILGATE_WARNING:
			diagnosticstring += "FCW tailgate warning, ";
			break;
		case FCW_TAILGATE_ALARM:
			diagnosticstring += "FCW tailgate alarm, ";
			break;
		case FCW_PULL_AHEAD_WARNING:
			diagnosticstring += "FCW driver pull ahead detected, ";
			break;
		case FCW_ERROR:
			diagnosticstring += "FCW read error, ";
			break;
	}
	//GPS
	switch ( gpsstatus ) {
		case GPS_NO_LOCK:
			diagnosticstring += "GPS no lock, ";
			break;
		case GPS_SPEED_WARNING:
			diagnosticstring += "Speed warning, ";
			break;
		case GPS_SPEED_ALARM:
			diagnosticstring += "Speed alarm, ";
			break;
		case GPS_ERROR:
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
