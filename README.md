/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  Description:
      This project is an attempt to create a standalone, windshield mounted driver assist
      unit with the following functionality:
		  - LDW (Lane Departure Warning)
		  - FCW (Forward Collision Warning)
		  - Driver pull-ahead warning
		  - Dashcam functionality (with GPS & timestamp overlay)

  Target Hardware:
      - Raspberry Pi (v3)
	  - RPi (v2.1) 8MP camera
	  - LidarLite (v3) LIDAR rangefinder
      - Adafruit Ultimate GPS
	  - Custom built HAT w/ power loss GPIO and capacitors to delay power off

  Target Software platform:
      Debian disto (DietPi) running LDXE, with below libraries installed

  3rd Party Libraries:
      OpenCV 3.1.0		-> Compiled with OpenGL support, www.opencv.org
      Raspicam 0.1.3	-> http://www.uco.es/investiga/grupos/ava/node/40
	  WiringPi 2.29		-> http://wiringpi.com/
	  
  License:
	  This software is licensed under GNU GPL v3.0

  Status:
	  Currently under development.  Dashcam w/ GPS and timestamp overlay functional, lane
	  detection semi-functional although not fit for audible alarming.  Forward collision
	  warning is non-functional due to poor performance of lidar sensor outdoors in
	  variable light conditions.
******************************************************************************************/