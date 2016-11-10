#ifndef LANEDETECTPROCESSOR_H
#define LANEDETECTPROCESSOR_H

#include <deque>
#include <array>
#include "opencv2/opencv.hpp"

typedef std::array<cv::Point, 4> Polygon;
typedef std::vector<cv::Point> Contour;

struct EvaluatedContour {
    Contour contour;
    cv::RotatedRect ellipse;
    float lengthwidthratio;
	float angle;
    //cv::Vec4f fitline;
};

struct PolygonDifferences {
	Polygon polygon;
	float differencefromaverage;
};

void EvaluateSegment( const Contour& contour,
					  const int imageheight,
	                  std::vector<EvaluatedContour>&	evaluatedsegments );
void ConstructFromSegments( const std::vector<EvaluatedContour>& evaluatedsegments,
                            std::vector<Contour>& constructedcontours );
void SortContours( const std::vector<EvaluatedContour>& evaluatedsegments,
				   const int imagewidth,
				   std::vector<EvaluatedContour>& leftcontours,
				   std::vector<EvaluatedContour>& rightcontours );
void FindPolygon( Polygon& polygon,
                  const Contour& leftcontour,
				  const Contour& rightcontour,
				  bool useoptimaly = false );
float PercentMatch( const Polygon& polygon,
					const cv::Mat& optimalmat );
int32_t ScorePolygonByPoint( const Polygon& polygon,
							 const Polygon& optimalpolygon );
void AveragePolygon( Polygon& polygon,
					 std::deque<Polygon>& pastpolygons,
					 int samplestoaverage,
					 int samplestokeep );
void ProcessImage( cv::Mat& image,
				   Polygon& polygon );
uint32_t FastSquareRoot( int32_t x );
float FastArcTan2( const float y,
				   const float x );

#endif // LANEDETECTPROCESSOR_H
