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
	extern float kcontrastscalefactor;
	
	//Polygon filtering
	extern Polygon optimalpolygon;
	extern uint16_t koptimumwidth;
	extern uint16_t kroadwithtolerance;
    extern uint16_t kminroadwidth;
    extern uint16_t kmaxroadwidth;
	
	//Segment filtering
	extern uint16_t ksegmentellipseheight;
	extern uint16_t kverticalsegmentlimit;
	extern float ksegmentminimumangle;
	extern float ksegmentlengthwidthratio;
	extern float ksegmentsanglewindow;
	
	//Contour filtering
	extern uint16_t kellipseheight;
	extern float kminimumangle;
	extern float klengthwidthratio;
	
	//Scoring
	extern float kanglefromcenter;
	extern uint16_t kminimumpolygonheight;
	extern float klowestscorelimit;
	extern float kheightwidthscalefactor;
	
}

#endif // LANEDETECTCONSTANTS_H
