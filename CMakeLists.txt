#####################################
cmake_minimum_required (VERSION 3.0) 
project (DAPrototype)
include_directories(include)
add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED)
if( ${CMAKE_NOT_RPI_TARGET} )
else ()
	message("Compiling for Raspberry Pi...")
	SET( CMAKE_CXX_FLAGS "-l pthread -l wiringPi -l gps" )
	find_package(raspicam REQUIRED)
	#find_package(GPSD REQUIRED)
	#find_package(WiringPi REQUIRED)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
	include_directories(${GTK2_INCLUDE_DIRS})
	#include_directories(${GPSD_INCLUDE_DIR})
	#add_definitions(${GTK2_CFLAGS_OTHER})
	#include_directories(${WIRINGPI_INCLUDE_DIRS})
endif()
add_library(XML_READER_LIBRARIES src/xml_reader.cpp)
add_library(PROCESS_VALUES_LIBRARIES src/process_values_class.cpp)
add_library(DISPLAY_HANDLER_LIBRARIES src/display_handler.cpp)
add_library(GPIO_HANDLER_LIBRARIES src/gpio_handler.cpp)
add_library(GPS_POLLING_LIBRARIES src/gps_polling.cpp)
add_library(IMAGE_CAPTURER_LIBRARIES src/image_capturer.cpp)
add_library(IMAGE_EDITOR_LIBRARIES src/image_editor.cpp)
add_library(IMAGE_PROCESSOR_LIBRARIES src/image_processor.cpp)
add_library(LIDAR_POLLING_LIBRARIES src/lidar_polling.cpp)
add_library(PACE_SETTER_LIBRARIES src/pace_setter_class.cpp)
add_library(FRAME_QUEUE_LIBRARIES src/frame_queue_class.cpp)
add_library(FCW_TRACKER_LIBRARIES src/fcw_tracker_class.cpp)
add_library(STORAGE_WORKER_LIBRARIES src/storage_worker_class.cpp)
add_library(TINYXML_LIBRARIES src/tinyxml2.cpp)
add_library(VIDEO_WRITER_LIBRARIES src/video_writer.cpp)
add_library(LANE_DETECT_LIBRARIES src/lane_detect_processor.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable (DAPrototype main.cpp)
target_link_libraries(DAPrototype
	XML_READER_LIBRARIES
	${OpenCV_LIBS}
	${raspicam_CV_LIBS}
	${GTK2_LIBRARIES}
#	${WIRINGPI_LIBRARIES}
	DISPLAY_HANDLER_LIBRARIES
	GPIO_HANDLER_LIBRARIES
	IMAGE_CAPTURER_LIBRARIES
	IMAGE_EDITOR_LIBRARIES
	IMAGE_PROCESSOR_LIBRARIES
	LIDAR_POLLING_LIBRARIES
	FCW_TRACKER_LIBRARIES
	PACE_SETTER_LIBRARIES
	TINYXML_LIBRARIES
	VIDEO_WRITER_LIBRARIES
	FRAME_QUEUE_LIBRARIES
	STORAGE_WORKER_LIBRARIES
	LANE_DETECT_LIBRARIES
	PROCESS_VALUES_LIBRARIES
#	GPS_LIBRARIES
	GPS_POLLING_LIBRARIES
)
#####################################
