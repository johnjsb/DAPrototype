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

struct EvaluatedContour {
    Contour contour;
    //cv::RotatedRect ellipse;
    //float lengthwidthratio;
	float angle;
    cv::Vec4f fitline;
	cv::Point center;
};

struct PolygonDifferences {
	Polygon polygon;
	float differencefromaverage;
};

void EvaluateSegment( const Contour& contour,
	                  std::vector<EvaluatedContour>& evaluatedsegments );
void ConstructFromSegments( const std::vector<EvaluatedContour>& evaluatedsegments,
                            std::vector<Contour>& constructedcontours );
void SortContours( const std::vector<EvaluatedContour>& evaluatedsegments,
				   const int imagewidth,
				   std::vector<EvaluatedContour>& leftcontours,
				   std::vector<EvaluatedContour>& rightcontours );
void FindPolygon( Polygon& polygon,
                  const EvaluatedContour& leftcontour,
				  const EvaluatedContour& rightcontour,
                  const int imageheight,
				  bool useoptimaly = false );
float Score( const Polygon& polygon ,
			 const int imagewidth );
void AveragePolygon( Polygon& polygon,
					 std::deque<Polygon>& pastpolygons,
					 int samplestoaverage,
					 int samplestokeep );
void ProcessImage( cv::Mat& image,
				   Polygon& polygon );
float FastArcTan2( const float y,
				   const float x );

#endif // LANEDETECTPROCESSOR_H
