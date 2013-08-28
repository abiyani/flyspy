// View.cpp: Contains implemetation of all functions related to a single view (View class)

#include "View.h"

/********************************/
int nFlies=-1; /// total number of flies - Global variable (should be accessible to all files and always set before start of an experiment). Must be set before doing anything meaningful.
int nViews=-1; /// number of views - lobal variable (should be accessible to all files and always set before start of an experiment). Must be set before doing anything meaningful.
/********************************/
const int NUMBER_OF_SAMPLES_FOR_CALCULATING_INITIAL_BG = 50;

/** Default Constructor. Simply calls initialize(), which does all the job.
\sa initialize()
*/
View::View()
{
	initialize();
}

////////////////////////begin of 'put it in a better place'/////////////////////////////////////////////////////////////
#define PIXEL(img, i, j, k)		(*( ((img)->imageData) + ((img)->widthStep * (i)) + ((img)->nChannels * (j)) + (k) ) )
#define UPIXEL(img, i, j, k)	(*( (unsigned char*) ( ((img)->imageData) + ((img)->widthStep * (i)) + ((img)->nChannels * (j)) + (k)) ))

inline double double_PixelDistance(unsigned char *thisFrame, unsigned char *bg, int nChannels)
{ 	//Can be faster if you don't use double calculations. use ints.
	if(nChannels == 3)
		return (double) ( 0.299 * (((int) thisFrame[2]) - bg[2]) + 0.587 * (((int) thisFrame[1]) - bg[1]) + 0.114 * ( ((int)thisFrame[0]) - bg[0]) );		
	else 
		return (double) (thisFrame[0] - bg[0]);

}
////////////////////////end of 'put it in a better place'/////////////////////////////////////////////////////////////



/** 
This function sets default value for some of the members of View class. To be called each time constructing a new View. \n
See actual implementation to know default values.
\warning Global variable nFlies should be set before calling this function.
*/
void View::initialize()
{
	camParams.projMatrix= NULL;
	camParams.invProjMatrix = NULL;
	
	inpAVIFileName.clear();
	inpCentroidFileName.clear();
	
	isOpen = false;
	isWrite = false;
	
	grabFrameResult = false;

	AVI_In = NULL;
	FMF_In = NULL;
	SBFMF_In = NULL;
	AVI_Out = NULL;
	fp_Centroid_IN = NULL;

	framesSinceLastTracked2D = -1;
	is2DTrackingON = false;

	// Fill in last track Index as 0,1,2 ....
	for(int i=0;i<nFlies;i++)
		lastTrackIndex2D.push_back(i);

	// Initilize the 2D tracker for this View
	viewTracker2D.setTargetNum(nFlies);
	viewTracker2D.setMixingWeight(0.0);
	
	currFrameNumberProcessed = -1L;
}

/** Sets the camParams.projMatrix (Camera Projection Matrix for this view) to the given projection Matrix (projMat)
\param projMat The projection matrix to be set as this View's projection matrix. (should be created using : cvCreateMat(3, 4, CV_64FC1) )
*/
void View::setProjMatrix(CvMat *projMat) // Create projMat using  cvCreateMat(3, 4, CV_64FC1);
{
	// Note I assume that size of projMat is 3x4 : Undefined behavior with wrong input
	
	camParams.projMatrix = cvCloneMat(projMat);

	camParams.invProjMatrix = cvCreateMat(4, 3, CV_64FC1);
	assert(camParams.invProjMatrix != NULL); // If this happens => cvCreateMat could not allocate memory for a 4x3 matrix
	cvInvert(camParams.projMatrix,camParams.invProjMatrix, CV_SVD); // Compute and store the inverse in newly allocated matrix
	camParams.opticalCenter = getOpticalCenter(camParams.projMatrix);
}

CvMat* View::ConvertMatToSbfmf(double *fr, int row, int col)
{
	CvMat *mat = cvCreateMat(row, col, CV_32FC1 );
	
	int k=0;

	//Loop runs backwards to flip image clockwise 90 degrees
	for(int i=0; i<row; i++)
	{
		for(int j=0; j<col; j++)
		{
			//Dividing by 256 since its a 32F image which needs to be scaled to interval [0,1] for display purposes
			//cvmSet(&mat,i,j,fr[k]/255.0);
			cvmSet(mat,i,j,fr[k]);
			k++;
        }
    }

	return mat;
}

/** Opens the "view". And uses already set inpType, inputCentroidFileName,inputAVIFileName variables to open respective file and set value of maxFramesInFile 
\return "true" is file is opened succesfully, "false" otherwise
*/
bool View::open()
{	
	if(inpType == CENTROID_ONLY || inpType == AVI_AND_CENTROID || inpType == FMF_AND_CENTROID || inpType == SBFMF_AND_CENTROID)
	{
		printf("\ninputCentroid file name = %s",inpCentroidFileName.c_str());
		fp_Centroid_IN = fopen(inpCentroidFileName.c_str(),"r");
		if(fp_Centroid_IN==NULL)
		{	
			isOpen = false;
			return false;
		}
	}
	if(inpType == AVI_ONLY || inpType == AVI_AND_CENTROID)
	{
		//printf("\nTrying to open file thissss %s \n",inpAVIFileName.c_str());
		AVI_In= cvCreateFileCapture((const char*)inpAVIFileName.c_str()); // Create the file capture
		if(AVI_In == NULL) // Cannot open File
		{
			if(inpType == AVI_AND_CENTROID) // Then close the already open centroid file
				fclose(fp_Centroid_IN);

			isOpen = false;
			return false;
		}
		maxFramesInFile = (unsigned long int)cvGetCaptureProperty(AVI_In,CV_CAP_PROP_FRAME_COUNT); // Set the maxFramesInFile
		calculateInitialBG(NUMBER_OF_SAMPLES_FOR_CALCULATING_INITIAL_BG); //using 50 frames to calculate model background...
	}

	if(inpType == FMF_ONLY || inpType == FMF_AND_CENTROID)
	{
		FMF_In= fopen((const char*)inpAVIFileName.c_str(), "rb"); // Create the file capture
		if(FMF_In == NULL) // Cannot open File
		{
			if(inpType == FMF_AND_CENTROID) // Then close the already open centroid file
				fclose(fp_Centroid_IN);

			isOpen = false;
			return false;
		}
		
		fread(&fmfVersion, sizeof(unsigned __int32), 1, FMF_In);
		fread(&SizeY, sizeof(unsigned __int32), 1, FMF_In);
		fread(&SizeX, sizeof(unsigned __int32), 1, FMF_In);
		fread(&bytesPerChunk, sizeof(unsigned __int64), 1, FMF_In);
		fread(&nframes, sizeof(unsigned __int64), 1, FMF_In);

		buf = new char[bytesPerChunk];
				
		maxFramesInFile = (unsigned long int)nframes; // Set the maxFramesInFile
		
		calculateInitialBG(NUMBER_OF_SAMPLES_FOR_CALCULATING_INITIAL_BG); //using 50 frames to calculate model background...
	}

	if(inpType == SBFMF_ONLY || inpType == SBFMF_AND_CENTROID)
	{
		SBFMF_In= fopen((const char*)inpAVIFileName.c_str(), "rb"); // Create the file capture
		if(SBFMF_In == NULL) // Cannot open File
		{
			if(inpType == SBFMF_AND_CENTROID) // Then close the already open centroid file
				fclose(fp_Centroid_IN);

			isOpen = false;
			return false;
		}

		fread(&nbytesver, sizeof(unsigned __int32), 1, SBFMF_In);
		sbfmfVersion = new char[nbytesver+1]; 
		fread(sbfmfVersion, sizeof(char), nbytesver, SBFMF_In);
		sbfmfVersion[nbytesver] = '\0';

		if (strcmp(sbfmfVersion, "0.3b")==0)
		{
			fread(&SizeY, sizeof(unsigned __int32), 1, SBFMF_In);
			fread(&SizeX, sizeof(unsigned __int32), 1, SBFMF_In);

			fread(&nframes, sizeof(unsigned __int32), 1, SBFMF_In);
			fread(&diffmode, sizeof(unsigned __int32), 1, SBFMF_In);
			fread(&indexloc, sizeof(unsigned __int64), 1, SBFMF_In);
						
			double *fr = new double[SizeX*SizeY];
			fread(fr, sizeof(double), SizeX*SizeY, SBFMF_In);
			
			bgcenter = ConvertMatToSbfmf(fr, SizeY, SizeX);

			fread(fr, sizeof(double), SizeX*SizeY, SBFMF_In);
			
			bgstd = ConvertMatToSbfmf(fr, SizeY, SizeX);
			
			fseek(SBFMF_In, indexloc, SEEK_SET);
						
			ff = new unsigned __int64[nframes];
			fseek(SBFMF_In, indexloc, SEEK_SET);
			fread(ff, sizeof(unsigned __int64), nframes, SBFMF_In);

			maxFramesInFile = (unsigned long int)nframes; // Set the maxFramesInFile

			calculateInitialBG(NUMBER_OF_SAMPLES_FOR_CALCULATING_INITIAL_BG); //using 50 frames to calculate model background...

		}
	}


	isOpen = true;
	return true; // Success
}

/** 
Close the view (Releases Input and Output AVI CvCaptures, IplImages, CvMats). Would be called by destructor.
\sa ~View()
*/
void View::close()
{
	if(isOpen ==false && isWrite==false) // Already closed
		return; 
	
	// Clear the IplImages stored in View class
	if(currFrame.img != NULL)
		cvReleaseImage(&currFrame.img);
	if(changeMask.img != NULL)
		cvReleaseImage(&changeMask.img);
	currFrame.img = NULL;
	changeMask.img = NULL;
	
	// Release Camera Proj/invProj matrices
	if(camParams.projMatrix!=NULL) 
		cvReleaseMat(&camParams.projMatrix);
	if(camParams.invProjMatrix!=NULL)
		cvReleaseMat(&camParams.invProjMatrix);
	camParams.projMatrix = NULL;
	camParams.invProjMatrix = NULL;

	if(isOpen==true && (inpType==AVI_ONLY || inpType==AVI_AND_CENTROID)&& AVI_In!=NULL) // If InputMode is AVI and File capture is succesfully opened
		cvReleaseCapture(&AVI_In);
	
	if(isOpen==true && (inpType==FMF_ONLY || inpType==FMF_AND_CENTROID)&& FMF_In!=NULL) // If InputMode is FMF and File capture is succesfully opened
		fclose(FMF_In);

	if(isOpen==true && (inpType==SBFMF_ONLY || inpType==SBFMF_AND_CENTROID)&& SBFMF_In!=NULL) // If InputMode is SBFMF and File capture is succesfully opened
		fclose(SBFMF_In);
	
	if(isOpen==true && (inpType==CENTROID_ONLY || inpType==AVI_AND_CENTROID || inpType==FMF_AND_CENTROID || inpType==SBFMF_AND_CENTROID)&& fp_Centroid_IN!=NULL) // If Mode is CENTROID and File capture is succesfully opened
		fclose(fp_Centroid_IN);

	FMF_In = NULL;
	SBFMF_In = NULL;
	AVI_In = NULL;
	fp_Centroid_IN = NULL;

	if(isWrite == true) // Output file is also ON
		stopSavingToOutputFile();
	
	isOpen = false; // "isWrite = false" is taken care by stopSavingToOutputFile()
}

/** 
Stops the saving of output file. Close the VideoWriter AVI_Out safely. Also set flag isWrite = false. \n
Safe to call even if no output file is being written (Does nothing in that case)
*/
void View::stopSavingToOutputFile()
{
	if(isWrite == false)
		return;

	if(isWrite == true && AVI_Out!=NULL)
		cvReleaseVideoWriter(&AVI_Out);

	if(ChangeMask_AVI_Out != NULL)
		cvReleaseVideoWriter(&ChangeMask_AVI_Out);

	ChangeMask_AVI_Out = NULL;
	AVI_Out = NULL;
	isWrite = false;
	return;
}

/** 
Grabs a single frame in AVI from specified location, and save it in this.currFrame.
\warning Releases any IplImage previously present in this.currFrame (irrespective of success of grab() function)

\param frameIndex The frame number to be grabbed.
\return "true" if frame succesfully grabbed (this.currFrame = Stores the IplImage for given frame #, "false" otherwise (this.currFrame = NULL in this case).
*/
bool View::grabFrame(unsigned long frameIndex)
{
	if(isOpen == false) // Cannot grab if view is not open
		return (grabFrameResult = false);
	
	centroids.clear(); 

	IplImage *temp = cvCreateImage(cvSize(SizeX, SizeY), IPL_DEPTH_8U, 1);

	// CurrFrame.img will be changed by processNewFrame() to previous frame in case of inpType = AVI_ONLY, but for other two input type, the currentFrame is actually current!
	if(inpType == AVI_ONLY || inpType == AVI_AND_CENTROID) // If AVI file is opened
	{
		if(currFrame.img!=NULL) // Relese currFrame, if some frame has been previously grabbed.
			cvReleaseImage(&currFrame.img);
		currFrame.img = NULL;

		if((long)frameIndex>=0L && (long)frameIndex<maxFramesInFile) // <==> if(Valid Frame number)
			cvSetCaptureProperty(AVI_In,CV_CAP_PROP_POS_FRAMES,double(frameIndex));
		else
			return (grabFrameResult = false); // Cannot grab .. illegal frame number

		IplImage *temp = cvQueryFrame(AVI_In); // Read the next frame ("temp" points to internal OpenCV memory and should not be freed : See OpenCV doc on cvQueryFrame() )
		if(temp==NULL) // Some error occured in grabbing frame from AVI
			return (grabFrameResult = false);

		currFrame.img = cvCloneImage(temp); // Copy the temp image to currFrame
		
		IplImage* tempFrame;
		tempFrame = cvCloneImage(temp);
		centroidCalculator.originalFrameWindow.push_back(tempFrame);

		if( (int)centroidCalculator.originalFrameWindow.size()> (centroidCalculator.nPrevFrames + centroidCalculator.nNextFrames +1)) //keep the size of the frame vec equal to pr + pst + 1
		{
			cvReleaseImage(&centroidCalculator.originalFrameWindow[0]);
			centroidCalculator.originalFrameWindow.erase(centroidCalculator.originalFrameWindow.begin());
		}

		if (!centroidCalculator.isInitialized)
		{	
			//centroidCalculator.initialize(centroidCalculator.nPrevFrames, centroidCalculator.nNextFrames,currFrame.img);
			centroidCalculator.initialize(centroidCalculator.nPrevFrames, centroidCalculator.nNextFrames,calculatedInitialBG);

			centroidCalculator.isInitialized = true;
			centroidCalculator.getNewFrame(currFrame.img);
		}
		else
			centroidCalculator.getNewFrame(currFrame.img);

	}

	if(inpType == FMF_ONLY || inpType == FMF_AND_CENTROID) // If FMF file is opened
	{
		if(currFrame.img!=NULL) // Relese currFrame, if some frame has been previously grabbed.
			cvReleaseImage(&currFrame.img);
		currFrame.img = NULL;

		if((long)frameIndex>=0L && (long)frameIndex<maxFramesInFile) // <==> if(Valid Frame number)
			fseek (FMF_In, frameIndex*bytesPerChunk + 28 , SEEK_SET );
		else
			return (grabFrameResult = false); // Cannot grab .. illegal frame number

		fread(buf, sizeof(double), 1, FMF_In);
		fread(buf, bytesPerChunk-sizeof(double), 1, FMF_In);
		
		IplImage *temp = cvCreateImage(cvSize(SizeX, SizeY), IPL_DEPTH_8U, 1);

		memmove(temp->imageData, buf, bytesPerChunk);
		
		if(temp==NULL) // Some error occured in grabbing frame from AVI
			return (grabFrameResult = false);

		currFrame.img = cvCloneImage(temp); // Copy the temp image to currFrame
		
		IplImage* tempFrame;
		tempFrame = cvCloneImage(temp);
		centroidCalculator.originalFrameWindow.push_back(tempFrame);

		if( (int)centroidCalculator.originalFrameWindow.size()> (centroidCalculator.nPrevFrames + centroidCalculator.nNextFrames +1)) //keep the size of the frame vec equal to pr + pst + 1
		{
			cvReleaseImage(&centroidCalculator.originalFrameWindow[0]);
			centroidCalculator.originalFrameWindow.erase(centroidCalculator.originalFrameWindow.begin());
		}

		if (!centroidCalculator.isInitialized)
		{	
			//centroidCalculator.initialize(centroidCalculator.nPrevFrames, centroidCalculator.nNextFrames,currFrame.img);
			centroidCalculator.initialize(centroidCalculator.nPrevFrames, centroidCalculator.nNextFrames,calculatedInitialBG);

			centroidCalculator.isInitialized = true;
			centroidCalculator.getNewFrame(currFrame.img);
		}
		else
			centroidCalculator.getNewFrame(currFrame.img);

	}

	if(inpType == SBFMF_ONLY || inpType == SBFMF_AND_CENTROID) // If FMF file is opened
	{
		if(currFrame.img!=NULL) // Relese currFrame, if some frame has been previously grabbed.
			cvReleaseImage(&currFrame.img);
		currFrame.img = NULL;


		if((long)frameIndex>=0L && (long)frameIndex<maxFramesInFile) // <==> if(Valid Frame number)
		{
			if (strcmp(sbfmfVersion, "0.3b")==0)
			{
				unsigned __int32 npixels;
				double stamp;

				IplImage stub;
				CvMat *m = cvCreateMat(SizeY, SizeX, CV_32FC1 );
				CvMat *n = cvCreateMat(SizeY, SizeX, CV_8UC1 );

				fseek (SBFMF_In, ff[frameIndex] , SEEK_SET );
				fread(&npixels, sizeof(unsigned __int32), 1, SBFMF_In);
				fread(&stamp, sizeof(double), 1, SBFMF_In);

				unsigned __int32 *idx = new unsigned __int32[npixels]; 
				fread(idx, sizeof(unsigned __int32), npixels, SBFMF_In);

				unsigned __int8 *val = new unsigned __int8[npixels];
				fread(val, sizeof(unsigned __int8), npixels, SBFMF_In);

				m = cvCloneMat(bgcenter);
					
				for (int j=0; j < npixels; j++)
				{
					int row = floor((double)idx[j]/(double)SizeX);
					int col = (double)idx[j]-(row*(double)SizeX);
				
					//cvmSet(m, row,col,(double)val[j]/255.0);
					cvmSet(m, row,col,(double)val[j]);
				}
				cvConvertScaleAbs(m, n);
				temp = cvGetImage(n, &stub);
			}
		}
		else
			return (grabFrameResult = false); // Cannot grab .. illegal frame number
		
		if(temp==NULL) // Some error occured in grabbing frame from AVI
			return (grabFrameResult = false);

		currFrame.img = cvCloneImage(temp); // Copy the temp image to currFrame

		IplImage* tempFrame;
		tempFrame = cvCloneImage(temp);

		centroidCalculator.originalFrameWindow.push_back(tempFrame);

		if( (int)centroidCalculator.originalFrameWindow.size()> (centroidCalculator.nPrevFrames + centroidCalculator.nNextFrames +1)) //keep the size of the frame vec equal to pr + pst + 1
		{
			cvReleaseImage(&centroidCalculator.originalFrameWindow[0]);
			centroidCalculator.originalFrameWindow.erase(centroidCalculator.originalFrameWindow.begin());
		}

		if (!centroidCalculator.isInitialized)
		{	
			//centroidCalculator.initialize(centroidCalculator.nPrevFrames, centroidCalculator.nNextFrames,currFrame.img);
			centroidCalculator.initialize(centroidCalculator.nPrevFrames, centroidCalculator.nNextFrames,calculatedInitialBG);

			centroidCalculator.isInitialized = true;
			centroidCalculator.getNewFrame(currFrame.img);
		}
		else
			centroidCalculator.getNewFrame(currFrame.img);

	}

	if(inpType == CENTROID_ONLY || inpType == AVI_AND_CENTROID || inpType == FMF_AND_CENTROID || inpType == SBFMF_AND_CENTROID)
	{
		char charPtr_line[10000]={0};
		if(fgets(charPtr_line,10000,fp_Centroid_IN)==NULL) // Reached end of file ?
		{
			puts(charPtr_line);
			return (grabFrameResult = false);
		}
		
		if(strlen(charPtr_line)>0 && charPtr_line[strlen(charPtr_line)-1] == '\n') // Since stupid fgets() also read the '\n' character
			charPtr_line[strlen(charPtr_line)-1] = 0;

		std::string line(charPtr_line);
		
		vector<std::string> tokens;
		Tokenize(line,tokens,",");
		if(tokens.size()==0 || atol(tokens[0].c_str())!=(long int)frameIndex) // The next frame line does not contain 2D centroid for desired frame number
		{
			centroids.clear();
		//	printf("\nI screw up in Tokenizer ... ");
			return (grabFrameResult = false);
		}
		for(int countTokens=3;countTokens<tokens.size();countTokens+=2) // starts from 3, see Centroid file structure for explanation
		{
			Point2D temp;
			if(countTokens==tokens.size()) // Should not be the case => Input centroid file has unmatched pair of X,Y corrdinates
			{
			//	printf("\nTokens do not match");
				centroids.clear();
				return (grabFrameResult = false);
			}
			temp.x = atof(tokens[countTokens].c_str());
			temp.y = atof(tokens[countTokens+1].c_str());
			centroids.push_back(temp);
		}	
		// The code below is not THREAD-SAFE .. stupid strtok() fucks up in mult-threaded env ..learned it the hard way
		
		//char *pch; // used for tokenizing
		///* Read all comma separated values */
		//pch = strtok(line,",");
		//// First one should be the frame number
		//if(atol(pch)!=(long int)frameIndex) // The next frame line does not contain 2D centroid for desired frame number
		//{
		//	centroids.clear();
		//	return false;
		//}
		//
		//while (pch != NULL)
		//{
		//	Point2D temp;
		//	pch = strtok (NULL, ",");
		//	if(pch==NULL)
		//		break;
		//	temp.x = atof(pch);
		//	pch = strtok (NULL, ",");
		//	assert(pch!=NULL); // Should not be the case => Input centroid file has unmatched pair of X,Y corrdinates
		//	temp.y=atof(pch);
		//	centroids.push_back(temp);
		//}
	}
	//printf("\nI return true");
	currFrameNumberProcessed = (long)frameIndex;
	return (grabFrameResult = true); // Success
}

/** Initialize recording to an output file. Set isWrite = true, if output File is succesfully created.
\param outputFileName The path of output file
\param size Size of each Frame of output File. For ex: for 640x480, frame would have size = cvSize(640,480) 
\param fps Frame rate per second for output file (default value = 60.0).
\return "true" if ouutput file is succefully created, "false" otherwise
\sa appendToOutputFile()
*/
bool View::initSavingToFile(std::string outputFileNameAVI,std::string outFileNameChangeMask,CvSize size,double fps)
{
	if(currFrame.img!=NULL)
		size = cvSize(currFrame.img->width,currFrame.img->height);
	
	if(AVI_Out !=NULL && isWrite==true)
		cvReleaseVideoWriter(&AVI_Out);
	AVI_Out = NULL;
	
	printf("\nWill use default FFDS codec for writing %s\n",outputFileNameAVI.c_str());
	AVI_Out = cvCreateVideoWriter(outputFileNameAVI.c_str(),CV_FOURCC('F','F','D','S'),fps,size,false);
	
	// I assume if everything was fine for opening a output AVI file, then it must be true for opening a changemask avi file also :D
	ChangeMask_AVI_Out = cvCreateVideoWriter(outFileNameChangeMask.c_str(),CV_FOURCC('F','F','D','S'),fps,size,false);

	if(AVI_Out == NULL || ChangeMask_AVI_Out == NULL)
	{
		AVI_Out = NULL;
		ChangeMask_AVI_Out = NULL; // Should be checked which one is null and then deallocated ... but being lazy right now :D 

		return false;
	}
	isWrite = true;
	return true;
}

/** Append the IplImage stored by currFrame.img to currently opened output file.
\warning Before calling this function output file must be initialized with initSavingToFile()
\sa initSavingToFile()
*/
void View::appendToOutputFile()
{
	if(isWrite == true && AVI_Out!=NULL && centroidCalculator.changeMaskWindow.size()>centroidCalculator.nPrevFrames + centroidCalculator.nNextFrames && centroidCalculator.originalFrameWindow.size()>centroidCalculator.nPrevFrames + centroidCalculator.nNextFrames)
	{
		IplImage *change_mask_img = cvCloneImage(centroidCalculator.changeMaskWindow[centroidCalculator.nPrevFrames]);
		IplImage *orig_img = cvCloneImage(centroidCalculator.originalFrameWindow[centroidCalculator.nPrevFrames]);

		CvFont font;
		double hScale=.5;
		double vScale=.5;
		int lineWidth=2;
		char str[1000]={0};
		sprintf_s(str,999,"FN = %ld",currFrameNumberProcessed);
		cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth);

		cvPutText(orig_img,str,cvPoint(20,50), &font, cvScalar(255,0,0));
		cvPutText(change_mask_img,str,cvPoint(20,50), &font, cvScalar(255));

		assert(change_mask_img!=NULL); // None of images pushed in "window" should be null
		assert(orig_img!=NULL); // See comment above
		for(int i=0;i<origCentroidsByCentroidCalculator.size();i++) // Draw centroids on change mask
			cvCircle(change_mask_img,cvPoint(origCentroidsByCentroidCalculator[i].x,origCentroidsByCentroidCalculator[i].y),2,cvScalar(90),-1);
	
		cvWriteFrame(AVI_Out,orig_img); //NOT ANY MORE HARDCODED 
		cvWriteFrame(ChangeMask_AVI_Out,change_mask_img);
		
		cvReleaseImage(&orig_img);
		cvReleaseImage(&change_mask_img);
	}
}
/////** Run the Silhouette Detector on currFrame.img (which was grabbed using call to grabFrame() )
////\n After succesful completion of call to this function, following member variables would be set:
////*	-# changeMask.img => Will contain new Change Mask (Output of Background subtraction of current frame and filtering)
////*	-# centroids => Will contain centroid of contours detected after computing changeMask (all "smart" things should go in computing these values as accurately as possible) 
////		Do *NOT* assume any particular order for centroids. Tracker would assign order to them later.
////*	-# framesSinceLastTracked2D => Would be incremented by 1 always, if track2D() is called for this view, it owuld be reset to zero, else keep track of what it is supposed to do ;)
////*	-# minCentroidDistance => this value is set after Centroids are computed by a call to returnMinDistBetweenCentroids()
////\sa returnMinDistBetweenCentroids(),segmentFrame()
////*/


//void View::silhDetect()
//{
//	if(currFrame.img==NULL || isOpen==false)
//		return; // Nothing to do
//
//	if(changeMask.img!=NULL)	// Release the previous changeMask() image
//		cvReleaseImage(&changeMask.img);
//	changeMask.img = NULL;
//
//	framesSinceLastTracked2D++; // will be reset to 0 if track2D() is run, else would be incremented each time to serve it's purpose.
//
//	SilhDetect.beSmart = true;
//
//
//
//#if __DSILH
//	printf("\n ****silh messages for veiw**** %d\n",param.ID);
//#endif
//
//	//centroidCalculator
//
//	changeMask.img = SilhDetect.segmentFrame(currFrame.img);	// Actually compute changeMask() and centroids
//
//	centroids = SilhDetect.silh_Centroids;
//
//	if (centroids.size() >= nFlies) // this part should be in the Silhdetector file not here! 
//		SilhDetect.hasSeenAllFliesinPrevFrame = true;
//	else
//		SilhDetect.hasSeenAllFliesinPrevFrame = false;
//
//	minCentroidDistance = returnMinDistBetweenCentroids(); // This value would be used for computing bestView later.
//}
//
//
void View::processNewFrame()
{
	if(inpType == CENTROID_ONLY || inpType == AVI_AND_CENTROID) // Nothing to "process" since we already have 2-D centroids from file :)
		return;

	// Will process only if inpType == AVI_ONLY
	
	//printf("\nI am in viewprocess1 %d", viewID);
	centroidCalculator.calculateCentroids(); // this 0 here doesn't mean any thing! its just to provide argument for the function ...

		
	//printf("\nI am in viewprocess3");
	/* Since the frame processed for inpType = AVI_ONLY
	is centroidCalculator.originalFrameWindow[centroidCalculator.nPrevFrames], therefore set currFrame.img and changeMask.img to one frame back */
	if(changeMask.img!=NULL)
		cvReleaseImage(&changeMask.img);
	changeMask.img = NULL;

	if(currFrame.img!=NULL)
		cvReleaseImage(&currFrame.img);
	currFrame.img = NULL;
	//printf("\nI am in viewprocess4");
	centroids.clear();
	if(centroidCalculator.changeMaskWindow.size()>=centroidCalculator.nPrevFrames + centroidCalculator.nNextFrames+1 && centroidCalculator.originalFrameWindow.size()==centroidCalculator.changeMaskWindow.size())
	{
		changeMask.img = cvCloneImage(centroidCalculator.changeMaskWindow[centroidCalculator.nPrevFrames]);
		currFrame.img = cvCloneImage(centroidCalculator.originalFrameWindow[centroidCalculator.nPrevFrames]);
		
		centroids = centroidCalculator.windowCentroids[centroidCalculator.nPrevFrames];
		currFrameNumberProcessed-=(long)centroidCalculator.nNextFrames; // since the actual frame processed is "nNextFrames" off from current frame
	}
	else
		currFrameNumberProcessed=-1L;
	//printf("\nI am in viewprocess5");
}

/** Destructor for View class. Simply call close()
\sa close()
*/
View::~View()
{
	close();
}

//TODO: Reza what does this function do ?? should we move it to centroidCalculator now ??
/** Returns Minimum straight line distance between any pair of centroids - Used for computing bestView. A larger value => "better" view
\return The lowest distance value among all possible pair of centroids. See description of function. Returns -1.0 if only 1 or less centroids are present in "centroids"
*/
double View::returnMinDistBetweenCentroids()
{
	double minD=INFINITY_ALEPH0; // == +INFINITY
	if(centroids.size()<=1)
		return -1.0;

	// Simply compute distance among all pair of centroids and return the lowest one.
	// euclidDistance_sq() is used for speed boost (by saving on call to sqrt() each time - since relative order remains same)
	for(int i=0;i<centroids.size();i++)
	{
		for(int j=i+1;j<centroids.size();j++)
			minD = (euclidDistance_sq(centroids[i],centroids[j])<minD ? euclidDistance_sq(centroids[i],centroids[j]):minD );
	}
	return sqrt(minD); // compute sqrt() for the final minimum value (which were all square of actual euclidean distances)
}

/*
void View::reorderCentroidsAndIndex012()
{
	if(centroids.size() != lastTrackIndex2D.size() && lastTrackIndex2D.size()!=0 && lastTrackIndex2D[0]!=-1)
		return;
		
	vector<Point2D> temp;
	for(int i=0;i<lastTrackIndex2D.size();i++)
	{
		assert(lastTrackIndex2D[i] >= 0 && lastTrackIndex2D[i]<centroids.size());
		temp.push_back(centroids[lastTrackIndex2D[i]]);
		lastTrackIndex2D[i] = i;
	}
	centroids.clear();
	centroids = temp;
	temp.clear();
}
*/

/** Run the 2D tracker for particular View().
\n Should be called after each call to silhDetect().
\n Does nothing if number of centroids (measurments) for this.currFrame if not exactly equalt to nFlies
\n After the call to this function following actions would have been performed:
	-# framesSinceLastTracked2D = 0 : since we are already tracking for this particular frame.
	-# lastTrackIndex2D : {lastTrackIndex2D[i] = Fly number for centroids[i] ; (for all 0<=i<centroids.size()) }
	-# is2DTrackingOn : Set as true. (actually should be set only for the first time, and then it will stay that way. But what the heck! set it "true" each time)
*/
void View::track2D()
{
	//if(centroids.size()!=nFlies)
	//	return;


	if(centroids.size()!=nFlies) //NOT ANY MORE HARDCODED
		return;
	
	framesSinceLastTracked2D = 0;
	is2DTrackingON = true;

	viewTracker2D.getNewMeasurement(centroids);//NOT ANY MORE HARDCODED
	viewTracker2D.Track(lastTrackIndex2D);

	lastTrackIndex2D.clear();
	lastTrackIndex2D = viewTracker2D.assignmentVec;
	//reorderCentroidsAndIndex012();
}

void View::restart()
{
		centroidCalculator.restart();//SilhDetect.restartSilhDetect(); //NOT ANY MORE HARDCODED I HAVE TO COME BACK AND WRITE A RESTART FOR CENTER CALCULATOR

		for (int j = 0;j<centroidCalculator.originalFrameWindow.size() + 1;j++) // release originalFrameWindow which has a window of captured frames
		{
			if(centroidCalculator.originalFrameWindow[j] !=NULL)
				cvReleaseImage(&centroidCalculator.originalFrameWindow[j]);
			centroidCalculator.originalFrameWindow[j] = NULL;
		}

		for (int j = 0;j<centroidCalculator.changeMaskWindow.size();j++) // release changeMaskWindow which has a window of captured frames
		{
			if(centroidCalculator.changeMaskWindow[j] !=NULL)
				cvReleaseImage(&centroidCalculator.changeMaskWindow[j]);
			centroidCalculator.changeMaskWindow[j] = NULL;

		}

		// release currFrame and changeMask
		if(currFrame.img!=NULL) 
			cvReleaseImage(&currFrame.img);
		currFrame.img = NULL;

		if(changeMask.img!=NULL)
			cvReleaseImage(&changeMask.img);
		changeMask.img = NULL;



		framesSinceLastTracked2D = -1;

		viewTracker2D.points2D.clear();
		viewTracker2D.points3D.clear();

		lastTrackIndex2D.clear();

		// Fill in last track Index as 0,1,2 ....
		for(int j=0;j<nFlies;j++)
			lastTrackIndex2D.push_back(j);

		viewTracker2D.setTargetNum(nFlies);

		viewTracker2D.assignmentVec.clear();

		viewTracker2D.assignmentVec.resize(nFlies); // put the size of the assignment vector equal to number of the flies
		for (int j=0;j<nFlies;j++)
			viewTracker2D.assignmentVec[j] = j; // this just indicates that the ID's are unknown at the very beginning

		viewTracker2D.readyToCorrespond = false;

		viewTracker2D.setMixingWeight(0.0);
		//		V[i]->centroids.clear(); //HARDCODED I HAVE TO COME BACK AND CLEAR CALC.WINDOWSCETEROIDS
		

		is2DTrackingON = false;

}

bool isDarker(CvScalar x,CvScalar y)
{
	double m1 = (x.val[0]+x.val[1]+x.val[2]);
	double m2 = (y.val[0]+y.val[1]+y.val[2]);
	if(m1<m2)
		return true;
	return false;
}

IplImage *View::grabFrameForSilhDetector(int fIndex)
{
	IplImage *temp = cvCreateImage(cvSize(SizeX, SizeY), IPL_DEPTH_8U, 1);
	if(inpType == AVI_ONLY || inpType == AVI_AND_CENTROID)
	{
		cvSetCaptureProperty(AVI_In,CV_CAP_PROP_POS_FRAMES,double(fIndex));
		temp = cvQueryFrame(AVI_In); // Read the next frame ("temp" points to internal OpenCV memory and should not be freed : See OpenCV doc on cvQueryFrame() )
	}

	if(inpType == FMF_ONLY || inpType == FMF_AND_CENTROID)
	{
		fseek (FMF_In, fIndex*bytesPerChunk + 28 , SEEK_SET );
		fread(buf, sizeof(double), 1, FMF_In);
		fread(buf, bytesPerChunk-sizeof(double), 1, FMF_In);
		memmove(temp->imageData, buf, bytesPerChunk);
	}

	if(inpType == SBFMF_ONLY || inpType == SBFMF_AND_CENTROID)
	{
		if (strcmp(sbfmfVersion, "0.3b")==0)
		{
			unsigned __int32 npixels;
			double stamp;

			IplImage stub;
			CvMat *m = cvCreateMat(SizeY, SizeX, CV_32FC1 );
			CvMat *n = cvCreateMat(SizeY, SizeX, CV_8UC1 );

			fseek (SBFMF_In, ff[fIndex] , SEEK_SET );
			fread(&npixels, sizeof(unsigned __int32), 1, SBFMF_In);
			fread(&stamp, sizeof(double), 1, SBFMF_In);

			unsigned __int32 *idx = new unsigned __int32[npixels]; 
			fread(idx, sizeof(unsigned __int32), npixels, SBFMF_In);

			unsigned __int8 *val = new unsigned __int8[npixels];
			fread(val, sizeof(unsigned __int8), npixels, SBFMF_In);

			m = cvCloneMat(bgcenter);
				
			for (int j=0; j < npixels; j++)
			{
				int row = floor((double)idx[j]/(double)SizeX);
				int col = (double)idx[j]-(row*(double)SizeX);
			
				//cvmSet(m, row,col,(double)val[j]/255.0);
				cvmSet(m, row,col,(double)val[j]);
			}
			cvConvertScaleAbs(m, n);
			temp = cvGetImage(n, &stub);
		}
	}

	if(temp==NULL) // Some error occured in grabbing frame from AVI
	{
		printf("\nError grabbing frame index = %d",fIndex);
		getchar();
		exit(0);
		return NULL;
	}

	IplImage *realImg = cvCloneImage(temp); // Copy the temp image to currFrame
	return realImg;
}

#define RUN_BIGGER_SILH_DETECTOR_VERSION 0

void View::calculateInitialBG(int nFramesBG)
{
#if !RUN_BIGGER_SILH_DETECTOR_VERSION
	
	IplImage *x;
	int maxFramesInFile;
	
	if(inpType == AVI_ONLY || inpType == AVI_AND_CENTROID)
	{
		x = grabFrameForSilhDetector(1);
		maxFramesInFile = (int)cvGetCaptureProperty(AVI_In,CV_CAP_PROP_FRAME_COUNT); // Set the maxFramesInFile
	
	}
	else if(inpType == FMF_ONLY || inpType == FMF_AND_CENTROID)
	{
		x = grabFrameForSilhDetector(1);
		maxFramesInFile = nframes;
	}
	else if(inpType == SBFMF_ONLY || inpType == SBFMF_AND_CENTROID)
	{
		x = grabFrameForSilhDetector(1);
		maxFramesInFile = nframes;
	}

	calculatedInitialBG = cvCloneImage(x);
	cvReleaseImage(&x);
	
	cvZero(calculatedInitialBG);
	
	
	/*for(int i=0;i<calculatedInitialBG->height;i++)
		for(int j=0;j<calculatedInitialBG->width;j++)
			cvSet2D(calculatedInitialBG,i,j,cvScalar(0,0,0));*/

	int step = (maxFramesInFile/nFramesBG);
	for(int i=1;i<maxFramesInFile;i+=step)
	{
		IplImage *x = grabFrameForSilhDetector(i);
		for(int i=0;i<calculatedInitialBG->height;i++)
			for(int j=0;j<calculatedInitialBG->width;j++)
			{
				if(!isDarker(cvGet2D(x,i,j),cvGet2D(calculatedInitialBG,i,j)))
					cvSet2D(calculatedInitialBG,i,j,cvGet2D(x,i,j));
			}
		cvReleaseImage(&x);
	}
	char fnameCurr[10000];

	string filename = inpAVIFileName;

	string outputPath = filename.substr( 0, filename.find_last_of( '\\' ) +1 );

	sprintf(fnameCurr,"%sChangeMask-BG-View-%d.jpg",outputPath.c_str(), viewID);
	
	cvSaveImage(fnameCurr,calculatedInitialBG);
#endif
	////////////// ////////////////
#if RUN_BIGGER_SILH_DETECTOR_VERSION
		//if(!isOpen)
		//{
		//	printf("\nits not open, I'm going to call open function");
		//	open();
		//}


		IplImage *tmp_frame = cvQueryFrame(AVI_In); // Read the next frame ("temp" points to internal OpenCV memory and should not be freed : See OpenCV doc on cvQueryFrame() )
	if (tmp_frame == NULL)
	{
		printf("\nproblem in opening the file");
		getchar();
	}
	IplImage *currbgMean = cvCloneImage(tmp_frame);
	cvZero (currbgMean);

	CvPoint pt1,pt2;
	pt1.x = 0;	pt1.y = 0;
	pt2.x = tmp_frame->height;
	pt2.y = tmp_frame->width;

	IplImage *bgcurFrame = cvCloneImage(tmp_frame);
	cvZero (bgcurFrame);
	IplImage  *prevFrame;

	prevFrame = cvCloneImage(bgcurFrame);

	double bgIniThreshold = 10; // this threshold is HARDCODED for now
	double alpha;

	unsigned long int maxFramesInFile = (unsigned long int)cvGetCaptureProperty(AVI_In,CV_CAP_PROP_FRAME_COUNT); // Set the maxFramesInFile
	// I assume maxFramesInFile > 500 ...

	int n = 0;
	char mycaption1[100];
	char mycaption2[100];

	sprintf(mycaption1,"curFrame View%d",viewID);
	sprintf(mycaption2,"currbgMean View%d",viewID);

	for (int myfc = 0;myfc<maxFramesInFile;myfc+=(int)(maxFramesInFile/nFramesBG))
	{
		n++;
		printf("\n%d/%d",n, nFramesBG);

		cvSetCaptureProperty(AVI_In,CV_CAP_PROP_POS_FRAMES,double(myfc));
		tmp_frame = cvQueryFrame(AVI_In); // Read the next frame ("temp" points to internal OpenCV memory and should not be freed : See OpenCV doc on cvQueryFrame() )

		if(tmp_frame==NULL) // Some error occured in grabbing frame from AVI
		{
			printf("\n an error occured in background modeling part");
			break;
		}

		//	if (!bgcurFrame)
		cvReleaseImage(&bgcurFrame);

		bgcurFrame = cvCloneImage(tmp_frame);

		//cvWaitKey(-1);

		if (myfc ==0)
		{
			cvReleaseImage(&prevFrame);
			prevFrame = cvCloneImage(bgcurFrame);
			continue;
		}
		else
		{

			alpha = 1.0/(n+1);
			////////////////////////for one frame/////////////////////////////////
			int i,j; // Defined above so as to unable parallelization (OpenMP)
#pragma omp parallel shared(bgcurFrame,currbgMean, prevFrame) private(i,j)		//SHOULD BE SHARING BGMEAN TOO. BUT IS THROWING AN ERROR :'( WHY??? Dunnooo. Default access clause is sharing though.
			{
#pragma omp for schedule(dynamic)
				for( j= std::min(pt1.y, pt2.y)+1; j<std::max(pt1.y, pt2.y)-1; ++j) //just look for silhouettes in ROI
				{
					for( i=std::min(pt1.x, pt2.x)+1; i<std::max(pt1.x, pt2.x)-1; ++i)

					{
						// printf("\n i = %d, j = %d, pt1.x = %d, pt2.x = %d",i,j, pt1.x, pt2.x);
						//if( pixelDistance( &UPIXEL(thisFrame, i, j, 0), &UPIXEL(bgMean, i, j, 0), 3 ) < -threshold )
						if( fabs((double_PixelDistance( &UPIXEL(bgcurFrame, i, j, 0), &UPIXEL(prevFrame, i, j, 0), 3 ))) < bgIniThreshold )
						{
							for(int k = 0; k < currbgMean->nChannels; ++k)
							{

								UPIXEL(currbgMean, i, j, k) = (unsigned char) (alpha * UPIXEL(bgcurFrame, i, j, k) + (1-alpha) * UPIXEL(currbgMean, i, j, k));
								//PIXEL(thisFrame, i, j, 3) = 0;
								//PIXEL(thisFrameClone, i, j, 3) = 0;
							}
						}
					}
				}
				/////////////////////end of for one frame/////////////////////////////////
			}


		}




		cvShowImage(mycaption1,bgcurFrame);
		cvShowImage(mycaption2,currbgMean);
		cvReleaseImage(&prevFrame);
		prevFrame = cvCloneImage(bgcurFrame);
		cvWaitKey(1);
	}

	calculatedInitialBG = cvCloneImage(currbgMean);

	cvDestroyWindow(mycaption1);
	cvDestroyWindow(mycaption2);

	cvReleaseImage(&bgcurFrame);
	cvReleaseImage(&prevFrame);
	cvReleaseImage(&currbgMean);

	char fnameCurr[10000];
	sprintf(fnameCurr,"ChangeMask-BG-View-%d.jpg",viewID);
	
	cvSaveImage(fnameCurr,calculatedInitialBG);
	//cvReleaseCapture(&inVideoCapture); I should not Release the capture because I am using it later ..
#endif
}