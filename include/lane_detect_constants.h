#ifndef LANEDETECTCONSTANTS_H
#define LANEDETECTCONSTANTS_H

namespace lanedetectconstants {
	
	extern uint16_t ksegmentellipseheight;
	extern float ksegmentanglewindow;
	extern float ksegmentlengthwidthratio;
	
	//Construct from segments filters
	extern float ksegmentsanglewindow;
	
	//Final contour filters
	extern uint16_t kellipseheight;
	extern float kanglewindow;
	extern float klengthwidthratio;
	
	//Scoring variables
    extern double kcommonanglewindow;
    extern uint16_t kminroadwidth;
    extern uint16_t kmaxroadwidth;
	extern uint16_t koptimumwidth;
	//weighting for best grade
	extern double kellipseratioweight;
	extern double kangleweight;
	extern double kcenteredweight;
	extern double kwidthweight;
	extern double klowestpointweight;
	extern double klowestscorelimit;
	
}

#endif // LANEDETECTCONSTANTS_H
