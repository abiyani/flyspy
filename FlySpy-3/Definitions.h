//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			"I have made this letter longer than usual, because I lack the time to make it short." - Blaise Pascal				//
//																																									//
//					"If you lie to the compiler, it will get its revenge." - Henry Spencer													//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Definitions.h : Purpose of this file is to contain definition of some global variables, structures, class. Also includes commonly used header files.

#pragma once // To force current source file to be included only once in a single compilation : http://en.wikipedia.org/wiki/Pragma_once

#ifndef __DEFINITIONS_H_INCLUDED__ // Include Guard: To safeguard against those compilers which do not support "#pragma once": http://en.wikipedia.org/wiki/Include_guard
#define __DEFINITIONS_H_INCLUDED__


#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <ctime>
#include <map>

//OpenMP
#include <omp.h>

// OpenCV Header files
#include <opencv/cv.h>
//#include "cxcore.h"
#include <opencv/highgui.h>

#include "Utility.h"




using namespace Utility; // For functions in Utility.h header file

/////////////////////////// Debugging macros below /////////////////////////
#define __D 0 ///Macro for allowing Debug prints   
#define __D1 (0 && __D)
#define __D2 (0 && __D1)  

#define __DCORRESPOND (0 && __D) /// Debugging flag for correspondence part
#define __D3D (0&& __D ) /// Start debugging of 3D tracking
#define __DSILH (0 && __D) ///Debugging flag for Silh detector
#define __DKALMAN (0 );

////////////////////////////////////////////////////////////////////////////

#define USE_KALMAN 0 /// Use this switch to turn on and off Kalman Filter use
#define CORRECT3D_ON_BESTVIEW 0 /// Use this switch to turn on correction of 3D based on 2D of bestView
//////////////////////////////////////////////////////////////////////////

///Stores data associated with a single Frame
struct Frame
{
	IplImage *img; //!< The pointer to "actual" image data

	int index; //!< Index number of frame. Not necessarily always initialized/set properly.

	int view_id; //!< The ID of AVI from which the frame is grabbed from. Not necessarily always initialized/set properly.

	Frame(); //!< Constructor for Frame Class. (Initialize "img" to NULL).
	~Frame(); //!< Destructor for Frame Class. Releases "img" if not NULL.
};

typedef std::pair<Point2D, int> points_cluster_pair;


struct contour_dist
{
	int id1,id2;
	double dist;

	///To sort contour_dist vectors in ascending order of distance
	bool operator<(contour_dist& rhs)
	{
		return dist<rhs.dist;
	}
};

//struct ReadCam // Structure for grabbing frames from a PGR Camera
//{
//	FlyCaptureContext context;
//	int BusIndex;
//};





#endif //__DEFINITIONS_H_INCLUDED__
