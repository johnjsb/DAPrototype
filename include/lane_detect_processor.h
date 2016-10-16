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
    cv::Vec4f fitline;
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
				  const Contour& rightcontour );
float ScoreContourPair( const Polygon& polygon,
                         const int imagewidth,
						 const int imageheight,
						 const EvaluatedContour& leftcontour,
						 const EvaluatedContour& rightcontour );
void AveragePolygon ( Polygon& polygon,
					  std::deque<Polygon>& pastpolygons,
					  int samplestoaverage,
					  int samplestokeep );
void ProcessImage ( cv::Mat& image,
					Polygon& polygon );

#endif // LANEDETECTPROCESSOR_H
