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
#ifndef LANEDETECTPROCESSOR_H
#define LANEDETECTPROCESSOR_H

//Standard libraries
#include <deque>
#include <array>

//3rd party libraries
#include "opencv2/opencv.hpp"

/*****************************************************************************************/
typedef std::array<cv::Point, 4> Polygon;
typedef std::vector<cv::Point> Contour;

struct EvaluatedLine {
    cv::Vec4i line;
	float angle;
	cv::Point center;
};

struct PolygonDifferences {
	Polygon polygon;
	int differencefromaverage;
};

bool CheckAngle( const cv::Point center,
				 const float angle );
void EvaluateLine( const cv::Vec4i& line,
				   std::vector<EvaluatedLine>& evaluatedlines );
void SortLines( const std::vector<EvaluatedLine>& evaluatedlines,
			    const int imagewidth,
			    std::vector<EvaluatedLine>& leftlines,
			    std::vector<EvaluatedLine>& rightlines );
void FindPolygon( Polygon& polygon,
                  const EvaluatedLine& leftevaluatedcontour,
				  const EvaluatedLine& rightevaluatedcontour,
                  const int imagewidth,
                  const int imageheight,
				  bool useoptimaly = false );
float Score( const Polygon& polygon,
             const EvaluatedLine& leftevaluatedline,
			 const EvaluatedLine& rightevaluatedline,
			 const int imagewidth );
void AveragePolygon( Polygon& polygon,
					 std::deque<Polygon>& pastpolygons,
					 const int samplestoaverage,
					 const int samplestokeep );
void ProcessImage( cv::Mat& image,
				   Polygon& polygon );
float FastArcTan2( const float y,
				   const float x );

#endif // LANEDETECTPROCESSOR_H
