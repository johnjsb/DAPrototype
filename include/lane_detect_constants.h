#ifndef LANEDETECTCONSTANTS_H
#define LANEDETECTCONSTANTS_H

#include "lane_detect_processor.h"

namespace lanedetectconstants {
	//Image evaluation
	extern uint16_t lowercannythreshold;
	
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
	extern float klowestscorelimit;
	
}

#endif // LANEDETECTCONSTANTS_H
