/******************************************************************************************
  Date:    19.09.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      LaneDetectLearning: Machine learning algorithm to determine parameters for consistent
	  lane detection.

  Description:
      This application will process multiple video files to determine the best lande detect
	  parameters for consistent lane detection.  It is a tool to determine the best values
	  for the DAPrototype project's lane detection system.

      OpenCV 3.1.0 -> Compiled with OpenGL support, www.opencv.org

  Other notes:
      Style is following the Google C++ styleguide

  History:
      Date         Author      Description
-------------------------------------------------------------------------------------------
      19.09.2016   N. Greco    Initial creation
******************************************************************************************/

//Standard libraries
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

//3rd party libraries
#include "opencv2/opencv.hpp"

//Project headers
#include "lane_detect_constants.h"
#include "lane_detect_processor.h"
#include "lane_constant_class.h"
#include "result_values_class.h"

/*****************************************************************************************/
//Forward declations
void UpdateLaneConstants(std::vector<LaneConstant> &laneconstants);
void FrameLoaderThread( cv::VideoCapture* videocapture,
						std::mutex* framesmutex,
						std::queue<cv::Mat>* frames,
						std::atomic<bool>* done);
int main(int argc,char *argv[])
{
	//Check arguments passed
	if (argc < 2) {
		std::cout << "No arguments passed, press ENTER to exit..." << std::endl;
		std::cin.get();
		return 0;
	}
	
	
	//Create results file
	std::ofstream resultsfile("resultsfile.csv");
	if (!resultsfile.is_open()) {
		std::cout << "Results file failed to open, press ENTER to exit..." << std::endl;
		std::cin.get();
		return 0;
	}
		
	
	
	//Find total frames in all video files
	uint32_t totalframes{0};
	for (int i = 1; i < argc; i++ ) {
		cv::VideoCapture capture(argv[i]);
		totalframes += capture.get(cv::CAP_PROP_FRAME_COUNT);
		capture.release();
		resultsfile << argv[i] << std::endl;
	}
	resultsfile << std::endl;
	std::cout << (argc - 1) << " files to evaluate with " << totalframes <<
		" total frames" << std::endl;
	//Set how often to message console
	uint32_t messagecount{totalframes/100};	//Every 1%

	//Create variable classes
	double increment{0.5};
	std::vector<LaneConstant> laneconstants;
	//Sort by sequence in code!
	/*
	laneconstants.push_back( LaneConstant( "ksegmentellipseheight",
		lanedetectconstants::ksegmentellipseheight, 0.0, 20.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "kcontrastscalefactor",
		lanedetectconstants::kcontrastscalefactor, 0.17, 0.21, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "klowestscorelimit",
		lanedetectconstants::klowestscorelimit, -100.0, 100.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "kminimumpolygonheight",
		lanedetectconstants::kminimumpolygonheight, 10, 60, 0.05*increment) );
	*/
	laneconstants.push_back( LaneConstant( "ksegmentlengthwidthratio",
		lanedetectconstants::ksegmentlengthwidthratio, 1.0, 5.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "ksegmentminimumangle",
		lanedetectconstants::ksegmentminimumangle, 15.0, 45.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "ksegmentsanglewindow",
		lanedetectconstants::ksegmentsanglewindow, 5.0, 45.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "kellipseheight",
		lanedetectconstants::kellipseheight, 10.0, 50.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "kminimumangle",
		lanedetectconstants::kminimumangle, 20.0, 45.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "klengthwidthratio",
		lanedetectconstants::klengthwidthratio, 3.0, 15.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "kanglefromcenter",
		lanedetectconstants::kanglefromcenter, 5.0, 45.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "kheightwidthscalefactor",
		lanedetectconstants::kheightwidthscalefactor, 200.0, 800.0, 0.05*increment) );
	std::cout << laneconstants.size() << " variables to modify" << std::endl;
	
	//Create header of resultsfile file
	resultsfile << "Iteration" << ",";
	for( int i = 0; i < laneconstants.size(); i++ ) {
		resultsfile << laneconstants[i].variablename_ << ",";
	}
	resultsfile << "average match" << ",";
	resultsfile << "frames detected" << "," << "total frames" << "," << "percent detected";
	resultsfile << "," << "score" << "," << "runtime" << "," << "fps" << "," << std::endl;

	
	//Create resultsfile vector
	ResultValues resultvalues{totalframes};
	int iterationcount{0};
	bool first{true};
	
	//Iterate through each variable
	for ( int i = 0; i < laneconstants.size(); i++ ) {
	//for ( int i = laneconstants.size() - 1; i >= 0; i-- ) {
		if ( !first ) laneconstants[i].Modify();
		first = false;
		resultvalues.NewVariable();
		for(;;) {
			resultvalues.NewIteration();
			UpdateLaneConstants(laneconstants);
			std::chrono::high_resolution_clock::time_point starttime;
			starttime =  std::chrono::high_resolution_clock::now();
			iterationcount++;
			uint32_t frameschecked{0};
			resultsfile << iterationcount << "," << std::fixed << std::setprecision(4);
			for( int j = 0; j < laneconstants.size(); j++ ) {
				resultsfile << laneconstants[j].value_ << ",";
			}
			
			//iterate through each file	
			for (int j = 1; j < argc; j++ ) {
				cv::VideoCapture capture(argv[j]);
				std::mutex framesmutex;
				std::queue<cv::Mat> frames;
				std::atomic<bool> done{false};
				//Multi-threading saves ~20% runtime
				std::thread t_imagequeue( FrameLoaderThread, &capture, &framesmutex, &frames, &done);
				t_imagequeue.detach();
				int framecount{0};
				while ( !(done && frames.empty()) ) {
					framesmutex.lock();
					if ( frames.empty() ) {
						framesmutex.unlock();
						continue;
					}
					framecount++;
					cv::Mat frame{frames.front()};
					frames.pop();
					framesmutex.unlock();
					Polygon polygon;
					ProcessImage( frame, polygon );
					resultvalues.Push( polygon );
					frameschecked++;
					if (frameschecked%messagecount == 0) {
						std::cout << "Iteration " << iterationcount << ", file "
								  << j << ", ";
						std::cout << std::fixed << std::setprecision(0);
						std::cout << ((100.0*framecount)/capture.get(cv::CAP_PROP_FRAME_COUNT));
						std::cout << "% file, " << ((100.0*frameschecked)/totalframes);
						std::cout << "% iteration, variable: ";
						std::cout << laneconstants[i].variablename_ << std::endl;
					}
				}
				capture.release();
			}
			
			//Update
			resultvalues.Update(laneconstants[i]);
			double runtime{std::chrono::duration_cast<std::chrono::microseconds>
				(std::chrono::high_resolution_clock::now() - starttime).count()/1000000.0};
			double fps{totalframes/runtime};
			resultsfile << resultvalues.averagematch_ << ",";
			resultsfile << resultvalues.detectedframes_ << "," << totalframes << ",";
			resultsfile << std::fixed << std::setprecision(2);
			resultsfile << ((resultvalues.detectedframes_ * 100.0) / totalframes) << ",";
			resultsfile << resultvalues.outputscore_ << ",";
			resultsfile << std::fixed << std::setprecision(3) << runtime << ",";
			resultsfile << fps << "," << std::endl;
			if (laneconstants[i].finished_) break;
		}
	}
	resultsfile << "Final" << ",";
	for( int i = 0; i < laneconstants.size(); i++ ) {
		resultsfile << laneconstants[i].value_ << ",";
	}
	resultsfile << std::endl;
	
	//Close up shop
	resultsfile.close();
	return 1;
}

/*****************************************************************************************/
void UpdateLaneConstants(std::vector<LaneConstant> &laneconstants)
{
	
	for ( LaneConstant &l : laneconstants) {
		if (l.variablename_ == "kheightwidthscalefactor" ) {
			lanedetectconstants::kheightwidthscalefactor = l.value_;
		} else if (l.variablename_ == "kcontrastscalefactor" ) {
			lanedetectconstants::kcontrastscalefactor = l.value_;
		} else if (l.variablename_ == "kanglefromcenter" ) {
			lanedetectconstants::kanglefromcenter = l.value_;
		} else if (l.variablename_ == "ksegmentellipseheight" ) {
			lanedetectconstants::ksegmentellipseheight = l.value_;
		} else if (l.variablename_ == "ksegmentlengthwidthratio" ) {
			lanedetectconstants::ksegmentlengthwidthratio = l.value_;
		} else if (l.variablename_ == "ksegmentminimumangle" ) {
			lanedetectconstants::ksegmentminimumangle = l.value_;
		} else if (l.variablename_ == "kellipseheight" ) {
			lanedetectconstants::kellipseheight = l.value_;
		} else if (l.variablename_ == "kminimumangle" ) {
			lanedetectconstants::kminimumangle = l.value_;
		} else if (l.variablename_ == "klengthwidthratio" ) {
			lanedetectconstants::klengthwidthratio = l.value_;
		} else if (l.variablename_ == "kminroadwidth" ) {
			lanedetectconstants::kminroadwidth = l.value_;
		} else if (l.variablename_ == "kmaxroadwidth" ) {
			lanedetectconstants::kmaxroadwidth = l.value_;
		} else if (l.variablename_ == "koptimumwidth" ) {
			lanedetectconstants::koptimumwidth = l.value_;
		} else if (l.variablename_ == "ksegmentsanglewindow" ) {
			lanedetectconstants::ksegmentsanglewindow = l.value_;
		} else if (l.variablename_ == "kminimumpolygonheight" ) {
			lanedetectconstants::kminimumpolygonheight = l.value_;
		} else if (l.variablename_ == "klowestscorelimit" ) {
			lanedetectconstants::klowestscorelimit = l.value_;
		} else {
			std::cout << "Programming error, variable does not exist!" << std::endl;
			std::cin.get();
			exit(0);
		}
	}

	return;
}

/*****************************************************************************************/
void FrameLoaderThread(cv::VideoCapture* videocapture, std::mutex* framesmutex, std::queue<cv::Mat>* frames, std::atomic<bool>* done)
{
	for( int i =0; i < videocapture->get(cv::CAP_PROP_FRAME_COUNT) - 1; i++  ){
		cv::Mat frame;
		for (;;) {			//Don't let queqed frames get too large or program crashes
			framesmutex->lock();
			unsigned long int framesqueqed { frames->size() };
			framesmutex->unlock();
			if (framesqueqed < 1000) {
				break;
			} else {
				std::this_thread::sleep_for(std::chrono::microseconds(500000)); //500ms
			}
		}
		*videocapture >> frame;
		framesmutex->lock();
		frames->push(frame);
		framesmutex->unlock();
	}
	*done = true;
	return;
}
