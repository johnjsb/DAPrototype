#include <iostream>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <queue>
#include <chrono>
#include "video_writer.h"
#include "frame_queue_class.h"
#include "storage_worker_class.h"
#include "pace_setter_class.h"
#include "xml_reader.h"
#include "opencv2/opencv.hpp"

/*****************************************************************************************/
void VideoWriterThread ( cv::Mat *orgimage,
                         std::mutex *capturemutex,
                         cv::Mat *modimage,
                         std::mutex *displaymutex,
                         std::atomic<bool> *exitsignal )
{

	std::cout << "Video writer thread starting!" << std::endl;

	//Check image is initialized
	if ( settings::cam::recordorgimage ) {
		while ( orgimage->empty() ) {
			if (*exitsignal) {
				return;
			}	  
		}		
	} else {
		while ( modimage->empty() ) {
			if (*exitsignal) {
				return;
			}	  
		}		
	}

	std::string filepath;
	cv::Size size;

	if ( settings::cam::recordorgimage ) {
		filepath = settings::cam::filepath + settings::cam::orgfilename;
		size = cv::Size(orgimage->cols, orgimage->rows);
	} else {
		filepath = settings::cam::filepath + settings::cam::modfilename;
		size = cv::Size(modimage->cols, modimage->rows);
	}
				
	//Shift files
	fileShift(filepath, settings::cam::filestokeep);
	
    FrameQueue queue;
	StorageWorker storage{ queue,
			               1,
			               filepath,
						   CV_FOURCC('D', 'I', 'V', 'X'),
						   static_cast<double>(settings::cam::recfps),
						   size,
						   true };

    // And start the worker threads for each storage worker
    std::thread t_storagethread(&StorageWorker::Run, &storage);
	
	//Create thread variables
	std::chrono::high_resolution_clock::time_point startime(
		std::chrono::high_resolution_clock::now());
	int32_t filelengthseconds{60 * settings::cam::minperfile};
	
	//Create pace setter
	PaceSetter videopacer(settings::cam::recfps, "video writer");

	//Loop
	while( !(*exitsignal) ) {
		
		//Normal Execution
		if ( settings::cam::recordorgimage ) {
			capturemutex->lock();
			queue.Push(orgimage->clone());
			capturemutex->unlock();
		} else {
			displaymutex->lock();
			queue.Push(modimage->clone());	
			displaymutex->unlock();
		}
				
		videopacer.SetPace();
		
		//New file changeover
		if (static_cast<long>(std::chrono::duration_cast<std::chrono::seconds>(
			std::chrono::high_resolution_clock::now() - startime).count()) > filelengthseconds) {
			
			//Stop thread
			queue.Stop();
			
			//Shift files
			fileShift(filepath, settings::cam::filestokeep);
			
			//Restart thread
			queue.Restart();

			//Restart timer
			startime = std::chrono::high_resolution_clock::now();
		}
	}

    queue.Cancel();
	t_storagethread.join();
	
	std::cout << "Exiting video writer thread!" << std::endl;

}

/*****************************************************************************************/
int fileShift( std::string filename,
               int numOfFiles )
{
    int i(numOfFiles), j(0), k(0);
    //split extension
    k = filename.find_last_of(".");
    //delete last file if present
    remove(((filename.substr(0,k) + "_" + std::to_string(i)) + filename.substr(k,filename.length() - k)).c_str());
    //rename existing files
    while(i != 1)
    {
        std::string newName = filename.substr(0,k) + "_" + std::to_string(i) + filename.substr(k,filename.length() - k);
        std::string oldName = filename.substr(0,k) + "_" + std::to_string(--i) + filename.substr(k,filename.length() - k);
        j = j + rename(oldName.c_str(), newName.c_str());
    }
    std::string newName = filename.substr(0,k) + "_" + std::to_string(i--) + filename.substr(k,filename.length() - k);
    j = j + i + rename(filename.c_str(), newName.c_str());
    return j;

}
