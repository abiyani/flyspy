#pragma once

#ifndef __SILHDETECTOR_H_INCLUDED__
#define __SILHDETECTOR_H_INCLUDED__

#include "utility.h"

struct contour_data
{
	CvSeq* contour_seq;
	double area;
	Point2D center;
	double radius;

	/// A Counter-initutive definition of "<" because: I want to sort contour_data vectors in *descending* order of area 
	bool operator<(contour_data& tmp)
	{
		return area>tmp.area;
	}

};

struct frameContoursInfo
{
	CvMemStorage* memStore;
	std::vector<contour_data>allContours_data;

	//this structure should change a lil

};

class SilhDetector
{
private:
	inline char pixelDistance(unsigned char *thisFrame, unsigned char *bg, int nChannels);

public:

	IplImage *bgMean;
	bool isInited;
	double alpha;
	char threshold;
	IplImage *denoisedChangeMask;
	std::vector<contour_data> contourList;	

	SilhDetector(void);
	~SilhDetector(void);

	void initialize(IplImage *firstFrame);
	void setThreshold(char thresh)
	{ threshold = thresh; }

	void setAlpha(double a)
	{ alpha = a; }
	frameContoursInfo findChangeMaskContours(IplImage *thisFrame);

	void finish();
	void restartSilhDetect();
	int fc;
};

#endif __SILHDETECTOR_H_INCLUDED__
