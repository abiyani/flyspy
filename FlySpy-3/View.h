// View.h: Header for implemetation of all functions related to a single view (View class)

#pragma once

#ifndef __VIEW_H_INCLUDED__
#define __VIEW_H_INCLUDED__

#include "Definitions.h"
#include "CentroidCalculator2D.h"
#include "tracker.h"


using namespace Utility;

//! View Class
/*!
This class supports all operation on a single view (file). Including Grabbing frame, writing frame, Silh Detection, Tracking in 2D,etc.
Experiment class will keep a vector<View> member variable to support operation on all views later. Ideally, the programmer using this framework should be
oblivious to presence of this class, and use only Methods of Experiment to perform tasks.
*/
class View
{
public:

	enum InputType
	{
		AVI_ONLY,
		FMF_ONLY,
		SBFMF_ONLY,
		CENTROID_ONLY,
		AVI_AND_CENTROID,
		FMF_AND_CENTROID,
		SBFMF_AND_CENTROID
	};
	
	long currFrameNumberProcessed;

	std::string inpCentroidFileName;
	std::string inpAVIFileName;
	InputType inpType; // To specify what would kind of input this view expects

	/** A structure to store parameters related to Camera Calibration. All these values would be populated using data stored in projection matrix file \n
	The purpose of the structure is to group calibration data logically for better code structure.
	*/
	struct structCamParams
	{
		CvMat *projMatrix; //< Camera Projection Matrix : cvCreateMat(3, 4, CV_64FC1);
		CvMat *invProjMatrix; //< Inverse Camera Projection Matrix : cvCreateMat(4, 3, CV_64FC1);
		Point3D opticalCenter; //< Optical Center for camera
	}camParams;

	Frame currFrame; ///< Stored the data for currentFrame grabbed.
	Frame changeMask; ///< Stores the changeMask for currFrame. To be populated after call to SilhDetect()

	CCentroidCalculator2D centroidCalculator;
	std::vector<Point2D> centroids; ///< Stores Centroid of each fly (to be set by centroidCalculator or read from file). will add "invalid" centroids at the end to make it same size.
	std::vector<Point2D> origCentroidsByCentroidCalculator; //< Stores a backup of centroids before adding "invalid" centroids at the end by processNewFrame()
	bool grabFrameResult;

	bool grabFrame(unsigned long nFrames=-1L);
	
	double returnMinDistBetweenCentroids();
	
	int viewID;///<ID of view
	long maxFramesInFile; ///< Maximum frames in the input video file. To be set by open()

	unsigned __int32 fmfVersion;
	unsigned __int64 bytesPerChunk;
	char *buf;

	unsigned __int32 SizeY, SizeX, nframes, nbytesver, diffmode;

	char *sbfmfVersion;
	unsigned __int64 indexloc;
	unsigned __int64 *ff;
	
	bool isOpen; ///< Indicates weather view has been opened or not
	bool isWrite; ///< Indicates wether output is being saved on a file
	bool is2DTrackingON; ///<Initially false, becomes true as soon as 2D tracker for this view is run at least once

	FILE *fp_Centroid_IN;

	FILE *FMF_In, *SBFMF_In;
	CvCapture *AVI_In; ///< OpenCV structure for reading Video file (Initialized to NULL)
	CvVideoWriter *AVI_Out; ///< OpenCV structure for writing to a video File (Initilized to NULL)
	CvVideoWriter *ChangeMask_AVI_Out; //< For storing changemask output
	IplImage *calculatedInitialBG; //<For storing the background that is being calculated at the very begining of processing

	IplImage *FrameFromCam;
	CvMat *bgcenter, *bgstd;

	CvMat* ConvertMatToSbfmf(double *fr, int row, int col);

	double minCentroidDistance; ///< Minimum straight line distance between any pair of centroids - Used for computing bestView. A larger value => "better" view 

	std::vector<int> lastTrackIndex2D; ///< The LastTrack Order of viewTracker2D.track(), initially: lastTrackIndex2D == {0,1,...nFlies} (assigning order of centroid as fly number for starting)
	
	CTracker viewTracker2D; ///< For tracking the new measurements with previous measurements (and assigning new TrackOrder, s.t., identites of flies are kept)
	
	int framesSinceLastTracked2D; ///< Initially -1, after that number of frames for which 2D tracker has not run consecutively. Incremented in silhDetector and set to zero whenever track2D() is called

	//int nPrevFrames;///< number of previous frames that is being kept before processing current frame
	//int nFutureFrames;///< number of future frames that is being kept before processing current frame

	View();
	void initialize();
	
	void setProjMatrix(CvMat *projMat);
	bool open();

	bool initSavingToFile(std::string outputFileNameAVI,std::string outFileNameChangeMask,CvSize size,double fps);
	void appendToOutputFile();
	void stopSavingToOutputFile();

	void processNewFrame();
	void restart();

	//void reorderCentroidsAndIndex012();
	void track2D();
	
	IplImage *View::grabFrameForSilhDetector(int fIndex);
	void calculateInitialBG(int);
	void close(); // Close the view
	~View();


};

#endif // __VIEW_H_INCLUDED__
