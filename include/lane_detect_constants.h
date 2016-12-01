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
	
	//Image evaluation
	extern float k_contrastscalefactor;
	
	//Segment filtering
	extern uint16_t k_segmentminimumsize;
	extern uint16_t k_verticalsegmentlimit;
	extern float k_segmentminimumangle;
	//extern float k_segmentlengthwidthratio;
	extern float k_segmentsanglewindow;
	
	//Contour filtering
	extern uint16_t k_minimumsize;
	extern float k_minimumangle;
	//extern float k_lengthwidthratio;
	
	//Polygon filtering
    extern uint16_t k_minroadwidth;
    extern uint16_t k_maxroadwidth;
	
	//Scoring
	extern float k_anglefromcenter;
	extern uint16_t k_minimumpolygonheight;
	extern float k_lowestscorelimit;
	extern float k_heightwidthscalefactor;
	
}

#endif // LANEDETECTCONSTANTS_H
