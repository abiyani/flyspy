// Sample command line parameters (add in Project properties -> Debugging -> Command Line) : 3 DFS d:\vg\a\2DCentroid-View0.txt d:\vg\long_exp\projmatrix2.txt d:\vg\a\TrackFile.txt
#include "Experiment.h"

using namespace std;

extern int nFlies;
extern int nViews;

////////////////////////////////////// Selective compilation directives below ////////////////////////////

#define __SAVE 1 // For saving the output or not

#define __CM_DISPLAY 0 // For displaying changemask images
#define __DISPLAY 0 // For turning ON/OFF display

#define __3DTRACKING 1 // To start and display 3D tracking

#define __WRITE2DCENTROID 0

#define MINIMUM_VIEWS_FOR_TRACKING (int(e.V.size())-6) // Right now we track if we have less than one view than total views 

// Flags for 2D centroid file
// //Current stucture for line in 2D centroid file is
//{<Frame Number-integer>,<Flag-integer value>,<Quality score - double value>,<X1-double>,<Y1-double>,<X2>,<Y2>,.....} where Xi,Yi = X and Y coordinates of i-th centroid
#define CENTROID_COMPUTER_GENERATED 0
#define CENTROID_HUMAN_FIXED 1
#define CENTROID_EXTRAPOLATED 2

const double correspondenceErrorThresholdForFlagging = 13.0;

enum en_CorrespondenceMethod
{
	STABLE_POLYAMORY,
	DFS,
	HUNGARIAN
}CorrespondenceMethod;

void printUsage()
{
	printf("\nFlySpy usage help : \n");
	printf("\nFlySpy-3 <Num_Flies> <Num_Views> <INP_METHOD=AVI|FMF|SBFMF|CENTROID> <HUNGARIAN|DFS|STABLE_POLYAMORY> <Path for first input file> \
		   <Path for projection matrix file> <num_frames_to_process> [<Ground Truth 3D TrackFile> <Threshold> (only for AVI case)]");
	getchar();
}

//////////////////////////////////////////////////////////////////////////////////////////////
////////////////// CVPR Specific code - for analysis of simulated data ///////////////////////
//USed for analysis of simulated data  - CVPR paper
double computeMinCorrespondenceError(map<FlyType,vector<ViewCentroidPair> > &inpMap)
{
	map<FlyType,vector<ViewCentroidPair> >::iterator it;
	int minMisMatch = -1; // This is greater than upper bound of max possible value

	for(int countView=0;countView<nViews;countView++) // find minimum among all views
	{
		int totalMismatch = 0;
		for(it=inpMap.begin();it!= inpMap.end();it++) // For each flytype
		{
			//Computer error for this flytype this view
			// First find out what is the centroid number for this flytype
			int refViewCentroidNo=-1;

			for(int i=0;i<(*it).second.size();i++)
			{
				if((*it).second[i].first==countView)
				{
					refViewCentroidNo = (*it).second[i].second;
					break;
				}
			}
			assert(refViewCentroidNo != -1);

			for(int i=0;i<(*it).second.size();i++)
				totalMismatch += ((*it).second[i].second!=refViewCentroidNo) ? 1 : 0;
		}
		if(minMisMatch == -1)
			minMisMatch = totalMismatch;
		else
			minMisMatch = min(minMisMatch,totalMismatch);
	}
	return (double)minMisMatch/(double)nFlies; // Average number of wrongly corresponded centroids per real centroid
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc,char *argv[])
{
	Experiment e; // The main experiment class object - all action takes place thru this

	char inpCentroidFileName[10000] = {0}; // For simulated 2D centroids data
	char projMatFilePath[10000] = {0}; // Projection matrix file - to be given in either case: simulated data or real video
	string outputTrackFileName;
	int lastFramesToProcessIndex = 999999;
	
	string outputPath;			// same as input file

	string outputSortedTrackFileName;
	string outputForCorrespondenceMapFileName;
	string outputErrorFileName;
	FILE *fp_SortedTrackFile = NULL, *fp_CorrespondenceMap = NULL, *fp_ErrorOut=NULL;

	FILE *fp_ComparisonWithGroundTruth=NULL;

	FILE *fp_groundTruth3DTrackFile = NULL;
	/////////////////// Input file names/window names, etc ////////////////////////////////
	char outCentroidFileName[10000] = "Out_2DCentroid_View0.txt";

	//////// Input AVI file name - for default (no command line argument) case //////// (Not required if CENTROID_ONLY case)
	char inpAVIName[10000] = "C:\\FreeFlightCam0.fmf";
	
	// The parameter below is used for analyzing efficiency in simulated data (for CVPR paper).
	double Threshold_For_False_Track = INFINITY_ALEPH1; // If the computed 3D point is at a distance greater than this threshold, then discard that point  - For simulated data (CVPR)

	if(argc == 1) // Default case: No command line arguments
	{
		CorrespondenceMethod = HUNGARIAN;
		
		e.setNFlies(5); // Set number of flies - VERY IMPORTANT
		e.setNViews(4); // Set number of views - VERY IMPORTANT
		
		//e.setInputType(View::AVI_ONLY);
		e.setInputType(View::FMF_ONLY);
		//e.setInputType(View::SBFMF_ONLY);
		//e.setInputType(View::CENTROID_ONLY);
		
		//strcpy(inpCentroidFileName,"D:\\vg\\exp4\\Julia Exp\\Chunk0\\new 2D centroid\\centroid-mod3View2.txt"); // Will be used if 2D centroid file is already present

		//////// Projection Matrix file name - required in all case
		strcpy(projMatFilePath,"C:\\projmat.txt");
		outputTrackFileName = "C:\\tracks.txt";
	}

	else // Process command line params !! 
	{

		int param1 = atoi(argv[1]); // Number of flies;
		e.setNViews(atoi(argv[2])); // Number of views
		if(param1<=0 || param1 >= 100000)
		{
			printf("\nInavlid value (%d) for num_flies",param1);
			printUsage();
			return 0;
		}
		if(nViews<=0 || nViews >= 100000)
		{
			printf("\nInavlid value (%d) for num_flies",nViews);
			printUsage();
			return 0;
		}

		if(_strcmpi(argv[3],"AVI") == 0)
		{
			if(argc!=8)
			{
				printUsage();
				return 0;
			}
			e.setInputType(View::AVI_ONLY);
			printf("\nWill use AVI file as input .....");
		}
		else if (_strcmpi(argv[3],"FMF") == 0)
		{
			if(argc!=8)
			{
				printUsage();
				return 0;
			}
			e.setInputType(View::FMF_ONLY);
			printf("\nWill use FMF file as input .....");
		}
		else if (_strcmpi(argv[3],"SBFMF") == 0)
		{
			if(argc!=8)
			{
				printUsage();
				return 0;
			}
			e.setInputType(View::SBFMF_ONLY);
			printf("\nWill use SBFMF file as input .....");
		}
		else
		{
			if(argc!=10)
			{
				printUsage();
				return 0;
			}
			e.setInputType(View::CENTROID_ONLY);
			printf("\nWill use Centroid file as input .....");
		}
		e.setNFlies(param1);
		// Process second parameter == Method for correspondence
		if(_strcmpi(argv[4],"HUNGARIAN") == 0)
		{
			CorrespondenceMethod = HUNGARIAN;
		}
		else
		{
			if(_strcmpi(argv[4],"DFS") == 0)
			{
				CorrespondenceMethod = DFS;
			}
			else
			{
				if(_strcmpi(argv[4],"STABLE_POLYAMORY")==0)
				{
					CorrespondenceMethod = STABLE_POLYAMORY;
				}
				else
				{
					printf("\nInvalid Correspondence method = %s",argv[4]);
					printUsage();
					return 0;
				}
			}
		}

		// Process next 3 params - centroid file name, proj mat file name, trackfile name
		if(_strcmpi(argv[3],"AVI") == 0)
		{
			strcpy(inpAVIName,argv[5]);
			strcat(inpAVIName, "\\FreeFlightCam0.avi");
		}
		else if(_strcmpi(argv[3],"FMF") == 0)
		{
			strcpy(inpAVIName,argv[5]);
			strcat(inpAVIName, "\\FreeFlightCam0.fmf");
		}
		else if(_strcmpi(argv[3],"SBFMF") == 0)
		{
			strcpy(inpAVIName,argv[5]);
			strcat(inpAVIName, "\\FreeFlightCam0.sbfmf");
		}
		else
			strcpy(inpCentroidFileName,argv[5]);

		strcpy(projMatFilePath,argv[6]);

		string filename = inpAVIName;

		////// Create string for output Files ////////

		outputPath = filename.substr( 0, filename.find_last_of( '\\' ) +1 );

		outputTrackFileName = outputPath + "Out_TrackFile.txt";
		outputSortedTrackFileName = outputPath + "Out_SortedTrackFile.txt";
		outputForCorrespondenceMapFileName = outputPath + "Out_CorrespondenceMap.txt";
		outputErrorFileName = outputPath + "Out_Error.txt";
		
		fp_SortedTrackFile = fopen(outputSortedTrackFileName.c_str(),"w");
		fp_CorrespondenceMap = fopen(outputForCorrespondenceMapFileName.c_str(),"w");
		fp_ErrorOut = fopen(outputErrorFileName.c_str(),"w");
		
		if(!(fp_SortedTrackFile && fp_CorrespondenceMap && fp_ErrorOut))
		{
			printf("\nCould not create output correspondence ( %s ) or SortedTrackFile ( %s ) or ErrorFile( %s )",outputForCorrespondenceMapFileName.c_str(),outputSortedTrackFileName.c_str(),outputErrorFileName.c_str());
			return 0;
		}
		
		lastFramesToProcessIndex = atoi(argv[7]); // Last frame to process
		
		//if(lastFramesToProcessIndex < 1) // Use default value for absurd case
		//	lastFramesToProcessIndex = 999999;
		
		string temp = outputPath + "Out_2DCentroid_View0.txt";

		strcpy(outCentroidFileName,temp.c_str());

		if(_strcmpi(argv[3],"CENTROID") == 0)// Input type =  centroids file
		{	
			fp_groundTruth3DTrackFile = fopen(argv[8],"r");
			if(fp_groundTruth3DTrackFile == NULL)
			{
				printf("\nCannot open Ground truth 3D track file %s .. Terminating",argv[8]);
				getchar();
				return 0;
			}
			string comparisonWithGroundTruthfname = "ComparisonWithGroundTruth.txt";
			fp_ComparisonWithGroundTruth = fopen(comparisonWithGroundTruthfname.c_str(),"w");
			if(fp_ComparisonWithGroundTruth==NULL)
			{
				printf("\nError creating output file %s ",comparisonWithGroundTruthfname.c_str());
				return 0;
			}
			Threshold_For_False_Track = atof(argv[9]);
		}
	}

	vector<string> wname; // Name for AVI output opencv window
	vector<string> cname; // Change mask opencv windows
	vector<string> correspondName;  // correspond output opencv windows
	for(int countView=0;countView<nViews;countView++)
	{
		char num[100];
		sprintf(num,"%d",countView);
		string num_str(num);
		wname.push_back(outputPath + "File-" + num_str);
		cname.push_back(outputPath + "ChangeMask-File" + num_str);
		correspondName.push_back(outputPath + "Corr-" + num_str);
	}

	//char wname[4][20] = {"File0","File1","File2","File3"};
	//char cname[4][20] = {"ChangeMask-File0","ChangeMask-File1","ChangeMask-File2","ChangeMask-File3"};
	//char correspondName[4][20] = {"Corr-0","Corr-1","Corr-2","Corr-3"}; // To show output of corresponder
	
	//////////////////// Initialization /////////////////////////

	FILE *fp_OutTrackFile; // For saving 3D tracks


	vector<FILE*> fp_OutCentroid(nViews);

	if( fopen_s(&fp_OutTrackFile,outputTrackFileName.c_str(),"w") !=0 )
	{
		printf( "\nThe output file %s could not be created\n", outputTrackFileName);
		getchar();
		return 0;
	}
#if __WRITE2DCENTROID

	for(int i=0;i<fp_OutCentroid.size();i++)
	{
		if(fopen_s(&fp_OutCentroid[i],outCentroidFileName,"w")!=0)
		{
			printf("\nError creating centroid file %s ",outCentroidFileName);
			getchar();
			return 0;
		}
		outCentroidFileName[strlen(outCentroidFileName)-5]++; // To create next file name: 2DCentroid-View0.txt -> 2DCentroid-View1.txt -> 2DCentroid-View2.txt ...and so on
	}
#endif

	// Set properties for each view 
	for(int countView=0;countView<nViews;countView++)
	{

		e.addView();
		e.V[countView]->viewID = countView;

		int curr_cam_num; // To identify which projection matrix entry should be used for particular AVI file
		// The method below used for identifying "curr_cam_num" is to use the digit just before .avi (or .txt) in the file name. For ex, if file name: AVIFileCam0.avi ...
		// ... then curr_cam_num = 0 , if file name: 2dCentroidView3.txt, then curr_cam_num = 3; This technique obviously does not work for #views > 9. 
		switch(e.inpType) // Must be called afrer addView();
		{
		case View::AVI_ONLY:
			curr_cam_num = inpAVIName[strlen(inpAVIName)-5]-'0';
			e.setInputAVIFileName(countView,inpAVIName);
			break;
		case View::FMF_ONLY:
			curr_cam_num = inpAVIName[strlen(inpAVIName)-5]-'0';
			e.setInputAVIFileName(countView,inpAVIName);
			break;
		case View::SBFMF_ONLY:
			curr_cam_num = inpAVIName[strlen(inpAVIName)-7]-'0';
			e.setInputAVIFileName(countView,inpAVIName);
			break;
		case View::CENTROID_ONLY:
			curr_cam_num = inpCentroidFileName[strlen(inpCentroidFileName)-5]-'0';
			e.setInputCentroidFileName(countView,inpCentroidFileName);
			break;
		case View::AVI_AND_CENTROID:
			assert(inpCentroidFileName[strlen(inpCentroidFileName)-5]-'0' == inpAVIName[strlen(inpAVIName)-5]-'0');
			curr_cam_num = inpCentroidFileName[strlen(inpCentroidFileName)-5]-'0';
			e.setInputAVIFileName(countView,inpAVIName);
			e.setInputCentroidFileName(countView,inpCentroidFileName);
			break;
		case View::FMF_AND_CENTROID:
			assert(inpCentroidFileName[strlen(inpCentroidFileName)-5]-'0' == inpAVIName[strlen(inpAVIName)-5]-'0');
			curr_cam_num = inpCentroidFileName[strlen(inpCentroidFileName)-5]-'0';
			e.setInputAVIFileName(countView,inpAVIName);
			e.setInputCentroidFileName(countView,inpCentroidFileName);
			break;
		case View::SBFMF_AND_CENTROID:
			assert(inpCentroidFileName[strlen(inpCentroidFileName)-5]-'0' == inpAVIName[strlen(inpAVIName)-7]-'0');
			curr_cam_num = inpCentroidFileName[strlen(inpCentroidFileName)-5]-'0';
			e.setInputAVIFileName(countView,inpAVIName);
			e.setInputCentroidFileName(countView,inpCentroidFileName);
			break;
		}
		assert(curr_cam_num>=0 && curr_cam_num <=9); // Valid digit is extracted

		CvMat* temp = NULL;

		temp = readProjMatrixFromFile(projMatFilePath,curr_cam_num);
		if(temp==NULL)
		{
			printf("\nCannot open Projection Matrix file @ %s\nAborting...",projMatFilePath);
			getchar();
			return 0;
		}
		e.V[countView]->setProjMatrix(temp);

		cvReleaseMat(&temp);

		if(!e.openView(countView))
		{
			printf("\nError opening input files: one or both from -> %s , %s",inpAVIName,inpCentroidFileName);
			getchar();
			cvDestroyAllWindows();
			return 0;
		}
		if(e.inpType == View::SBFMF_ONLY || e.inpType == View::SBFMF_AND_CENTROID)
			inpAVIName[strlen(inpAVIName)-7] += 1; // Increase to next file name
		else
			inpAVIName[strlen(inpAVIName)-5] += 1; // Increase to next file name
		
		inpCentroidFileName[strlen(inpCentroidFileName)-5] +=1;

	//	cvNamedWindow(correspondName[countView]); // G-Corresponder
#if __DISPLAY
		cvNamedWindow(wname[countView].c_str());
#endif
#if __CM_DISPLAY
		cvNamedWindow(cname[countView].c_str());
#endif
	} // End for Loop

	char video_output_fn[1000];
	
	strcpy(video_output_fn, outputPath.c_str());

	switch(CorrespondenceMethod)
	{
	case STABLE_POLYAMORY:
		strcat(video_output_fn,"Out_STABLE_POLY_");
		break;
	case HUNGARIAN:
		strcat(video_output_fn,"Out_HUNGARIAN_");
		break;
	case DFS:
		strcat(video_output_fn,"Out_DFS_");
		break;
	}

#if __SAVE
	assert(e.inpType!=View::CENTROID_ONLY); // Should not happen, since __SAVE makes no sense for centroid_only case
	printf("\nCurrently recording Change Masks (remeber to udpate View class for normal recording again)\n");
	
	int outVideoWidth=-1,outVideoHeight=-1, fps=-1;
	
	if(e.V[0]->AVI_In!=NULL) // If View file has been opened, then get dimension from view file (otherwise assume it to be default (640x480))
	{
		outVideoWidth = (int)cvGetCaptureProperty(e.V[0]->AVI_In,CV_CAP_PROP_FRAME_WIDTH);
		outVideoHeight = (int)cvGetCaptureProperty(e.V[0]->AVI_In,CV_CAP_PROP_FRAME_HEIGHT);
		fps = (int)cvGetCaptureProperty(e.V[0]->AVI_In,CV_CAP_PROP_FPS);
	}
	
	if(e.V[0]->FMF_In!=NULL) // If View file has been opened, then get dimension from view file (otherwise assume it to be default (640x480))
	{
		outVideoWidth = e.V[0]->SizeX;
		outVideoHeight = e.V[0]->SizeY;
		fps = 60;
	}

	if(e.V[0]->SBFMF_In!=NULL) // If View file has been opened, then get dimension from view file (otherwise assume it to be default (640x480))
	{
		outVideoWidth = e.V[0]->SizeX;
		outVideoHeight = e.V[0]->SizeY;
		fps = 60;
	}

	if(e.initSavingToFile(video_output_fn,cvSize(outVideoWidth,outVideoHeight),fps)==false)
	{
		printf("\nCould not start writing ... ");
		getchar();
	}
#endif


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////// Real processing starts after this point /////////////////////////////////
	int FIndex; // Starting frame number (initilized in for loop below)
	long time_start = clock();
	long f_processed=0; // total number of frames processed (for calculating cumulative processing FPS - "speed of program")
	

	FILE *fp_numCentroids;
	string fn_numCentroids = outputPath + "numCentroids.csv";
	fp_numCentroids = fopen(fn_numCentroids.c_str(),"w");

	// Variables below are added for analysis required for CVPR paper - simulated data part //
	vector<map<int,bool> > IsPresent(nFlies);
	int pointsAboveThreshold=0; // Count total number of points above Threshold - added for analyzing simluated data (for CVPR)
	int ActualFrameProcessed=0; // counter for actual number of frames processed - useful inc omputing average values at end - added for analyzing simluated data (for CVPR)
	double correspondenceError = 0.0; // added for analyzing simluated data (for CVPR)
	double totalCorrespondenceErrorValue = 0.0;
	double totalDistanceBetween3Dpoints = 0.0;
	vector<double> totalDistanceBetween3DpointsVec;
	int countScrollPressed = 0;
	
	assert(nViews == e.V.size()); // These values should match, because we use them interchangeably. BAD things will happen if they do not ;)

	// these 3 variables are used for finding stats about effect of doing correspondence of all pair of sequences
	int num_corr_sequences=0;
	int num_unique_initial_corr_sequence = 0;
	int total_mismatch=0,total_MinimumSequences=0;
	FILE *fp_temp;
	string fn_temp = outputPath + "distrib.txt";
	fp_temp = fopen(fn_temp.c_str(),"w");

	int lastTimeHowManyFlies=0; // Keep note of how many flies were tracked in last frame - used for FLAGGING frames

	for(ActualFrameProcessed=0,FIndex = 1;e.grabFrame((unsigned long)FIndex)  && FIndex < lastFramesToProcessIndex ;FIndex++,ActualFrameProcessed++) // && FIndex < n => will process only frames upto "n"
	{

		if(GetAsyncKeyState(VK_SCROLL)) // will be true is "scroll lock" is pressed (unfortunately in any program - even in background, therefore a buffer of 4 time)
		{
			if(countScrollPressed<4)
				countScrollPressed++;
			else
				break;
		}

		//cvWaitKey();
		long curr_time = clock();

		//printf("\nI am here 1 ");
		vector<int> allFliesVisibleViewList;
		allFliesVisibleViewList = e.processNewFrame(); // Runs Silh detetcion, and stores list of view with exactly "nFlies" blobs

		//printf("\n\n############## nFlies = %d, Grabbing frame  = %d, processing frame = %ld ############\n",nFlies,FIndex,e.currFrameNumberProcessed);
		printf("\n\n++++++++++++++ nFlies = %d, # Frame grabbed  = %d  +++++++++++++++",nFlies,FIndex);

		//vector<int> allFliesVisibleViewList = e.silhDetect(); // Runs Silh detetcion, and stores list of view with exactly "nFlies" blobs

		
		//printf("\nI am here 2 ");
#if __WRITE2DCENTROID
		/* Write 2D centroids to file if flag is ON */
		for(int i=0;i<e.V.size() && e.currFrameNumberProcessed!=-1L;i++)
		{
			double quality_score = 1.0; // This value should ideally be spitted out by SilhDetector for each view. Currently a dummy value
			fprintf(fp_OutCentroid[i],"%d,%d,%.2lf,",(int)e.currFrameNumberProcessed,(int)CENTROID_COMPUTER_GENERATED,quality_score);
			for(int j=0;j<e.V[i]->origCentroidsByCentroidCalculator.size();j++)
				fprintf(fp_OutCentroid[i],"%.2lf,%.2lf,",e.V[i]->origCentroidsByCentroidCalculator[j].x,e.V[i]->origCentroidsByCentroidCalculator[j].y);

			fprintf(fp_OutCentroid[i],"\n");

		}
#endif

			fprintf(fp_numCentroids,"\n%d", FIndex);		
			for (int i = 0;i<e.V.size();i++)	
				if(e.V[i]->centroidCalculator.windowContoursOriginalCentroids.size()>2)
					fprintf(fp_numCentroids,",%d,%d",e.V[i]->centroidCalculator.windowContoursOriginalCentroids[1].size(),e.V[i]->centroidCalculator.windowCentroids[1].size());

#if __3DTRACKING
		int corr_mismatch=0,corr_MinimumSequences=0;
		if(allFliesVisibleViewList.size()>=MINIMUM_VIEWS_FOR_TRACKING) // Atleast 2 views should have == nFlies measurment to do something meaningful .. else just skip to next frame
		{	
		/*	printf("\nsafe view list,numcentroids = ");
			for(int ii=0;ii<allFliesVisibleViewList.size();ii++)
				printf("(v = %d,#c = %d) ",allFliesVisibleViewList[ii],e.V[allFliesVisibleViewList[ii]]->origCentroidsByCentroidCalculator.size());*/

			switch(CorrespondenceMethod)
			{
			case STABLE_POLYAMORY:
				e.correspondViewsUsingStablePolyamory(allFliesVisibleViewList,corr_mismatch,corr_MinimumSequences);
				break;
			case HUNGARIAN:
				e.correspondViewsUsingHungarian(allFliesVisibleViewList,corr_mismatch,corr_MinimumSequences);
				break;
			case DFS:
				e.correspondViewsUsingDFS(allFliesVisibleViewList);
				break;
			}
			
			num_corr_sequences += factorial(allFliesVisibleViewList.size());
			num_unique_initial_corr_sequence += ( ( allFliesVisibleViewList.size()*(allFliesVisibleViewList.size()-1) )/2);
			
			total_MinimumSequences += corr_MinimumSequences;
			total_mismatch += corr_mismatch;
			
			printf("\nTotal Min seq = %.5lf %%\nTotal Mismatches in initial same views = %.5lf %%",\
				(double(total_MinimumSequences)/(num_corr_sequences))*100.0,(double(total_mismatch)/num_unique_initial_corr_sequence)*100.0);
			
			
			//printMap(e.correspondOutput);
			//getchar();
			e.clearCorresponderOutput(); // Strip off any dummy points
		//	printMap(e.correspondOutput);
			printf("\nReal error = %.3lf",e.computeTotalErrorForOutput(e.correspondOutput,false));
			
			////////////////////////////////////////////////////////////
			/// Check output of G-correspoinder by plotting it //////
			/////////////////////////////////////////////////////////////
			vector<IplImage*> tempCorrespondImg(e.V.size(),NULL);
			bool flag=true;
			for(int countView=0;countView<e.V.size() && flag;countView++)
			{
				if(e.V[countView]->currFrame.img==NULL)
				{
					flag=false;
					break;
				}
				tempCorrespondImg[countView] = cvCloneImage(e.V[countView]->currFrame.img);
			}
			//printf("\nFlag = %s",(flag)?"true":"false");
			map<FlyType,vector<ViewCentroidPair> >::iterator it;
			int i=0;
			CvFont font;
			double hScale=.5;
			double vScale=.5;
			int lineWidth=2;
			char str[10];
			cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth);

			for(i=0,it = e.correspondOutput.begin();it!=e.correspondOutput.end() && flag;i++,++it)
			{
				sprintf_s(str,9,"%d",(*it).first);
				for(int j=0;j<(*it).second.size();j++)
				{
					int currView = (*it).second[j].first;
					int currCentroid = (*it).second[j].second;
					cvPutText(tempCorrespondImg[currView],str,cvPoint(e.V[currView]->centroids[currCentroid].x,e.V[currView]->centroids[currCentroid].y), &font, cvScalar(255,0,0));
					//printf("(V = %d, C = %d), ",(*it).second[j].first,(*it).second[j].second);

				}
			}

			for(int countView=0;countView<e.V.size() && flag;countView++)
			{
				//cvShowImage(correspondName[countView].c_str(),tempCorrespondImg[countView]);
			}
			

			for(int countView=0;countView<e.V.size() && flag;countView++)
			{
				if(tempCorrespondImg[countView]!=NULL)
					cvReleaseImage(&tempCorrespondImg[countView]);
			}

			//printf("\n******************************");
			/*printf("\n\n******** current MAP *************");
			map<FlyType,vector<ViewCentroidPair> >::iterator it;
			int i=0;
			for(i=0,it = currMap.begin();it!=currMap.end();i++,it++)
			{
			printf("\nFlyType %d -> ",i);
			for(int j=0;j<(*it).second.size();j++)
			printf("(V = %d, C = %d), ",(*it).second[j].first,(*it).second[j].second);
			}
			printf("\n******************************");*/
			//printf("\nI am here 3 ");
			//// For writing corresponder's output
			//map<FlyType,vector<ViewCentroidPair> >::iterator it;
			////printf("\n=====stable orgy===");
			//for(it = e.correspondOutput.begin();it!=e.correspondOutput.end();it++)
			//{
			//	fprintf(fp_corresfile,	"\n %d, %d,", (int)e.currFrameNumberProcessed,(*it).first);	
			//	for (int i = 0;i<(*it).second.size();i++)
			//		fprintf(fp_corresfile,"(%d, %d%d)",(*it).second[i].first,(*it).second[i].second,(*it).second[i].second);
			//}
			////////////////////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////////////////////////////////////////
			//continue;
			double err = e.computeTotalErrorForOutput(e.correspondOutput,false);
			// To scale the value (so that it is no longer dependent on number of flies). the factor, (e.V.size()*(e.V.size()-1)/2)  = #of views chose 2 (vC2)
			if(allFliesVisibleViewList.size() >1)
				err /= ( double((int(allFliesVisibleViewList.size()) *( int(allFliesVisibleViewList.size())-1 ))/2.0) * double(nFlies) );
			bool corr_flag = false;
			if(err > correspondenceErrorThresholdForFlagging)
				corr_flag = true;
			
			vector<Point3D> out3D;
			int howManyTrackedInThisFrame=0; // How many flies are actually tracked in this frame - used for flagging frames
			if((e.howManyCentroidsCorresponded>=nFlies-1 && nFlies!=1) || (e.howManyCentroidsCorresponded==nFlies))
			{	
				out3D = e.Track3D();
				for(int countFlies=0;countFlies<out3D.size();countFlies++)
					if(!isInvalidPoint(out3D[countFlies]))
						howManyTrackedInThisFrame++;
			
				if(e.inpType!=View::CENTROID_ONLY) // A possible source of vector subscript out of range errors (was earlier trying to write in case of centorid only - fatal error)
					e.draw3DPoints(out3D);
			}
			//	printf("\nI am here 4 ");
			fprintf(fp_OutTrackFile,"%d,",(corr_flag==false && howManyTrackedInThisFrame==lastTimeHowManyFlies) ? 0 : 1); // If number of lfies tracked changes, then flag this frame
			lastTimeHowManyFlies = howManyTrackedInThisFrame;
			fprintf(fp_OutTrackFile,"%d,",(int)e.currFrameNumberProcessed);
			//printf("\nAll flies visible view list size = %d 3dPoints \n",allFliesVisibleViewList.size());
			for(int i=0;i<out3D.size();i++)
			{
				//printf("%.2lf,%.2lf,%.2lf\n",out3D[i].x,out3D[i].y,out3D[i].z);
				if(i<int(out3D.size())-1)
					fprintf(fp_OutTrackFile,"%.2lf,%.2lf,%.2lf,",out3D[i].x,out3D[i].y,out3D[i].z);
				else
					fprintf(fp_OutTrackFile,"%.2lf,%.2lf,%.2lf",out3D[i].x,out3D[i].y,out3D[i].z);
			}
			fprintf(fp_OutTrackFile,"\n");
			//Write correspondence error to file
	
			

				//	printf("\nI am here 5 ");
			/////////// Print out the SortedOutputFile => only make sense for simulated data (checking correspondence) //////
			//printf("\nCheckpoint 1");
			
			
			// Check if there are any dummy 3D points (in case of simulated data it will happen only for first 3 frames - when we do not track).
			bool hasDummy3DPoint = false;
			for(int count3DPoints = 0;count3DPoints<out3D.size() && !hasDummy3DPoint;count3DPoints++)
				if(isInvalidPoint(out3D[count3DPoints]))
					hasDummy3DPoint = true;

		
			//////////////////////////////////// Code added for Analysis of simulated data for Journal paper ////////////////
			///////////////// Output the sorted correspondence map => to check the simulated data ///////////
			if(fp_ErrorOut !=NULL && fp_CorrespondenceMap!=NULL)
			{
				//printf("\n\n******** current MAP *************");
				map<FlyType,vector<ViewCentroidPair> > sortedMap = e.correspondOutput;
				//printf("\n....pos 1");
				sortCorrespondenceOutput(sortedMap,allFliesVisibleViewList[0]);
				//printf("\n....pos 2");
				
				//printf("\n....pos 3");
				fprintf(fp_ErrorOut,"%d,%.3lf\n",(int)e.currFrameNumberProcessed,err); // Write the scaled correspondence error.
		

				totalCorrespondenceErrorValue += err; //(this error is the value we seek to minimze in our correspondence function): this is final error after minimization.

				map<FlyType,vector<ViewCentroidPair> >::iterator it;
				fprintf(fp_CorrespondenceMap,"\n%d,Err = %.2lf,",(int)e.currFrameNumberProcessed,err);
				int i=0;
				for(i=0,it = sortedMap.begin();it!=sortedMap.end();i++,it++)
				{
					//printf("\n....pos 4");
					fprintf(fp_CorrespondenceMap,"FT %d,",i);
					for(int j=0;j<(*it).second.size();j++)
						fprintf(fp_CorrespondenceMap,"{V= %d,C= %d},",(*it).second[j].first,(*it).second[j].second);
					//printf("\n....pos 5");
				}
			}
			//	printf("\nCheckpoint 3");
				////////////////////////////////////////////////////////////////////////////////////////////////////////////////

				

			if(fp_groundTruth3DTrackFile!=NULL)
			{
				char line[100000];
				fgets(line,99999,fp_groundTruth3DTrackFile);
				vector<Point3D> gt = readFlyTrackFromSingleFrame(line,1);

				assert(gt.size() == out3D.size()); // Should be same == nFlies

				vector<int> a1,a2;
				hungCorrespondOf2Sets(gt,out3D,a1,a2);

				// Add those points which are less than threshold in track formation list
				for(int k=0;k<a1.size();k++)
				{
					totalDistanceBetween3Dpoints += euclidDistance(gt[k],out3D[a1[k]]);
					fprintf(fp_temp,"%.2lf\n",euclidDistance(gt[k],out3D[a1[k]]));
					totalDistanceBetween3DpointsVec.push_back(euclidDistance(gt[k],out3D[a1[k]]));
					//printf("\nDist between pts = %.2lf",euclidDistance(gt[k],out3D[a1[k]]));
					if(euclidDistance_sq(gt[k],out3D[a1[k]])<Threshold_For_False_Track*Threshold_For_False_Track)
						IsPresent[a1[k]][k] = true; // Set this path used as true
					else
						pointsAboveThreshold++;
				}
				correspondenceError += computeMinCorrespondenceError(e.correspondOutput);
			}
			else
				ActualFrameProcessed--; // If this frame is not used for analysis just do not count it in processed frame too.

			if(fp_SortedTrackFile != NULL)
			{
				sort(out3D.begin(),out3D.end(),ComparePoint3DLessThan);
				fprintf(fp_SortedTrackFile,"%d,",(allFliesVisibleViewList.size()==e.V.size()) ? 0 : 1); // Trivial flagging for now
				fprintf(fp_SortedTrackFile,"%d,",(int)e.currFrameNumberProcessed);
				for(int i=0;i<out3D.size();i++)
				{
					if(i<out3D.size()-1)
						fprintf(fp_SortedTrackFile,"%.2lf,%.2lf,%.2lf,",out3D[i].x,out3D[i].y,out3D[i].z);
					else
						fprintf(fp_SortedTrackFile,"%.2lf,%.2lf,%.2lf\n",out3D[i].x,out3D[i].y,out3D[i].z);
				}
			}
		//	printf("\nCheckpoint 4");
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		else
		{
			lastTimeHowManyFlies=0; // Nothing was tracked
		}
#endif

#if __SAVE
		e.appendToOutputFile();
#endif
		vector<IplImage*>change_mask_img(e.V.size(),NULL);
		
#if __CM_DISPLAY || __DISPLAY
		
		// This loop displays the output and Change Mask
		for(int countView=0;countView<e.V.size();countView++)
		{
#if __CM_DISPLAY
			//int temp = 1;
			if(e.V[countView]->centroidCalculator.changeMaskWindow.size()>e.V[countView]->centroidCalculator.nPrevFrames)
			{
				change_mask_img[countView] = cvCloneImage(e.V[countView]->centroidCalculator.changeMaskWindow[e.V[countView]->centroidCalculator.nPrevFrames]);
				for (int j = 0;j<e.V[countView]->origCentroidsByCentroidCalculator.size();j++)
					cvDrawCircle(change_mask_img[countView],cvPoint((int)e.V[countView]->origCentroidsByCentroidCalculator[j].x,(int)e.V[countView]->origCentroidsByCentroidCalculator[j].y),2,cvScalar(50),-1);

				//for (int j = 0;j<e.V[countView]->centroidCalculator.windowCentroids[temp].size();j++)
				//	{
				//		printf("\n cv = %d, j = %d", countView, j);	
				//		printf("\n changeMaskWindow[1].size() = %d", e.V[countView]->centroidCalculator.changeMaskWindow.size());
				//		cvDrawCircle(e.V[countView]->centroidCalculator.changeMaskWindow[1],cvPoint((int)e.V[countView]->centroidCalculator.windowCentroids[temp][j].x,(int)e.V[countView]->centroidCalculator.windowCentroids[temp][j].y),2,cvScalar(50),-1);

				//	}

				cvShowImage(cname[countView].c_str(),change_mask_img[countView]);
				//	cvShowImage(cname[countView].c_str(),e.V[countView]->centroidCalculator.changeMaskWindow[1]);
			}

#endif
#if __DISPLAY
			if(e.V[countView]->centroidCalculator.originalFrameWindow.size()>e.V[countView]->centroidCalculator.nPrevFrames && \
				e.V[countView]->centroidCalculator.originalFrameWindow[e.V[countView]->centroidCalculator.nPrevFrames]!=NULL)
				cvShowImage(wname[countView].c_str(),e.V[countView]->centroidCalculator.originalFrameWindow[e.V[countView]->centroidCalculator.nPrevFrames]);
#endif
			/*if(e.V[countView]->originalFrameWindow.size()==e.V[countView]->nPrevFrames + e.V[countView]->nFutureFrames + 1)
			if(e.V[countView]->originalFrameWindow[1] !=NULL)
			cvShowImage(wname[countView].c_str(),e.V[countView]->originalFrameWindow[e.V[countView]->nPrevFrames]);*/
			
		}	
		for(int k=0;k<change_mask_img.size();k++)
			if(change_mask_img[k]!=NULL)
				cvReleaseImage(&change_mask_img[k]);

#endif
		f_processed++; // Increment frames processed (used for keeping track of FPS).
		printf("\nCumulative Frame Processing speed (FPS) = %.2lf fps ... Current FPS = %.2lf fps",f_processed/(((double)(clock()-time_start))/(CLOCKS_PER_SEC)),\
			1.0/(((double)(clock()-curr_time))/(CLOCKS_PER_SEC)));

#if __CM_DISPLAY || __DISPLAY
		cvWaitKey(((int)e.currFrameNumberProcessed>25300) ?  -1 : 1); // For setting appropriate delay.
		
#endif

	}
	

	FILE *fp_finalAnalysis;
	string fname = outputPath + "Out_FinalAnalysis.txt";
	fp_finalAnalysis = fopen(fname.c_str(),"w");
	assert(fp_finalAnalysis != NULL);
	int total = 0;
	for(int i=0;i<IsPresent.size();i++)
		total += (int)IsPresent[i].size();
	
	sort(totalDistanceBetween3DpointsVec.begin(),totalDistanceBetween3DpointsVec.end());
	//double ans123=0.0;
	//for(int i=0;i<totalDistanceBetween3DpointsVec.size();i++)
	//	ans123+=totalDistanceBetween3DpointsVec[i];
	//	

	if(fp_groundTruth3DTrackFile!=NULL)
	{
		//fprintf(fp_finalAnalysis,"Average number of paths used to reconstruct a single path = %.3lf\n",double(total)/double(nFlies));
		//fprintf(fp_finalAnalysis, "Average number of 3D points matched with ground truth points with cost above threshold (per fly) = %.3lf\n",double(pointsAboveThreshold)/double(ActualFrameProcessed));
		//fprintf(fp_finalAnalysis, "Correspondence error (average number of wrongly corresponded centroids per fly type) = %.3lf\n",correspondenceError/double(ActualFrameProcessed));
		// 5th value = totalDistanceBetween3Dpoints/((double)nFlies*ActualFrameProcessed) == Average distance between ground truth 3d point and tracked 3d point per frame per fly.
		// 6th value = totalCorrespondenceErrorValue/ActualFrameProcessed == Average correspondence error (scale invarient w.r.t. number of views or flies).
		// 7th value = % of sequence of views which have minimum correspondence error
		// 8th value = % of initial sequence (first 2) views with differing correspondence error values (mismatch)
		// 9th value = Medain of average fistance between ground truth and 3d poitns (medain for 5th value in the list)
		// The last value (f_processed/(((double)(clock()-time_start))/(CLOCKS_PER_SEC))) ) is FPS, for first threee columns see above
		fprintf(fp_finalAnalysis,"%d,%.5lf,%.5lf,%.5lf,%.5lf,%.5lf,%.5lf,%.5lf,%.5lf,%.5lf",nFlies,\
			double(total)/double(nFlies),\
			double(pointsAboveThreshold)/((double(nFlies))*ActualFrameProcessed), \
			correspondenceError/double(ActualFrameProcessed),\
			totalDistanceBetween3Dpoints/((double)nFlies*ActualFrameProcessed),\
			totalCorrespondenceErrorValue/ActualFrameProcessed,\
			(double(total_MinimumSequences)/(num_corr_sequences))*100.0,\
			(double(total_mismatch)/num_unique_initial_corr_sequence)*100.0,\
			totalDistanceBetween3DpointsVec[ ((int)totalDistanceBetween3DpointsVec.size())/2],
			f_processed/(((double)(clock()-time_start))/(CLOCKS_PER_SEC)));
	}

	fclose(fp_finalAnalysis);
	fclose(fp_temp);
	if(fp_SortedTrackFile != NULL)
		fclose(fp_SortedTrackFile);
	if(fp_CorrespondenceMap != NULL)
		fclose(fp_CorrespondenceMap);
	if(fp_ErrorOut != NULL)
		fclose(fp_ErrorOut);

	fclose(fp_OutTrackFile);
#if __WRITE2DCENTROID
	for(int i=0;i<fp_OutCentroid.size();i++)
		fclose(fp_OutCentroid[i]);
#endif

#if __SAVE
	e.stopSavingToOutputFile();
#endif
	printf("\nTracking successfully completed.");

	cvDestroyAllWindows();

	fclose(fp_numCentroids);

	return 0;
}
