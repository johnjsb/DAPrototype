/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

//Header guard
#ifndef LANEDETECTCONSTANTS_H
#define LANEDETECTCONSTANTS_H

//Project libraries
#include "lane_detect_processor.h"

/*****************************************************************************************/
namespace lanedetectconstants {
	//Default polygon
	Polygon defaultpolygon{ cv::Point(0,0),
							cv::Point(0,0),
							cv::Point(0,0),
							cv::Point(0,0) };
							 
	//Image evaluation
	float k_contrastscalefactor{ 0.75f };
	uint16_t k_ystartposition{ 240 };
	
	//Line filtering
	float k_maxvanishingpointangle{ 18.0f };
	uint16_t k_vanishingpointx{ 400 };				//Relative to image size, must change
	uint16_t k_vanishingpointy{ 260 };				//Relative to image size, must change
	uint16_t k_verticallimit{ 280 };				//Relative to image size, must change
	uint16_t k_rho{ 1 };
	float k_theta{ 0.13962634015f };				//Pi / 22.5
	uint16_t k_minimumsize{ 25 };					//Relative to image size, must change
	uint16_t k_maxlinegap{ 5 };						//Relative to image size, must change
	uint16_t k_threshold{ 30 };						//Relative to image size, must change

	//Polygon filtering
	uint16_t k_maxoffsetfromcenter{ 400 };			//Relative to image size, must change
    uint16_t k_minroadwidth{ 540 };					//Relative to image size, must change
    uint16_t k_maxroadwidth{ 800 };					//Relative to image size, must change
	
	//Scoring
	float k_lowestscorelimit{ -400.0f };			//Relative to image size, must change
	float k_weightedheightwidth{ 100.0f };			//Relative to image size, must change
	float k_weightedangleoffset{ -5.0f };
	float k_weightedcenteroffset{ -1.0f };			//Relative to image size, must change

}

#endif // LANEDETECTCONSTANTS_H
