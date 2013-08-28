#pragma once
#include "SilhDetector.h"
#include "Definitions.h"
#include <vector>
class CCentroidCalculator2D
{
public:
	CCentroidCalculator2D(void);
	~CCentroidCalculator2D(void);
	void initialize(int npre,int npst, IplImage* firstFrame);
	void finish(void);
	bool getNewFrame(IplImage *thisFrame);
	bool calculateCentroids();

	std::vector<IplImage*> originalFrameWindow;
	std::vector<IplImage*> changeMaskWindow;

	
	//int winSize; //and useless ...
	int nPrevFrames;
	int nNextFrames;
	//int currentWindowSize; //proly useless ...
	SilhDetector silh; ///< Silhouette  detector which detects flies based on the change mask between frames
	bool endOfVideo; //this flag has to be set by view class
	std::vector<frameContoursInfo> windowFramesContours; ///< keeps a window of contour informations in each fram  detector which detects flies based on the change mask between frames
	std::vector<std::vector<Point2D>> windowCentroids; //keeps centroids of the frames in the window ... 
	std::vector<std::vector<Point2D>> windowContoursOriginalCentroids;//keeps original centroids of the frames in the window, means just using founded contours without being smart ...
	bool isInitialized;
	//bool hasEverSeenExactNFlies;
	void restart();
};
