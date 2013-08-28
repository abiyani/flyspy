// Experiment.cpp: Contains implemetation of all functions related to an Experiment
#include "Experiment.h"

extern int nFlies;
extern int nViews;

namespace DFS_Vars // Global variables required for running DFS for correspondence
{
	int nRuns;
	double leastError;
	vector<int> safeViewList;
	int nCentroids;
}
//extern double distance(Point3D a,Point3D b);
//extern double distance(Point2D a,Point2D b);

////////////////// function below are added for analyzing the output of simulated data /////////////////////////////////

// Sort correspondence map (w.r.t, centroid of View "vIndex") - useful for comparing correspondence in simulated data
void sortCorrespondenceOutput(map<FlyType,vector<ViewCentroidPair> > &toSort,int vIndex)
{
	map<FlyType,vector<ViewCentroidPair> > sortedMap;

	map<FlyType,vector<ViewCentroidPair> >::iterator it;
	for(it = toSort.begin();it!=toSort.end();++it)
	{
		for(int j=0;j<(*it).second.size();j++)
		{
			if((*it).second[j].first==vIndex)
			{
				sortedMap[(*it).second[j].second] = (*it).second;
				sort(sortedMap[(*it).second[j].second].begin(),sortedMap[(*it).second[j].second].end(),CompareViewCentroidPairLessThan);
				break;
			}
		}
	}
	toSort = sortedMap;
}
bool CompareViewCentroidPairLessThan(ViewCentroidPair &a,ViewCentroidPair &b)
{
	if(a.first<b.first)
		return true;
	else
		if(a.first == b.first)
			return a.second < b.second;
	return false;
}

bool ComparePoint3DLessThan(Point3D &a,Point3D &b) // Comparison function for sorting Point3D (to have same tracks (fly number) for same video always)
{
	if(a.x<b.x)
		return true;
	else
		if(a.x == b.x)
			if(a.y<b.y)
				return true;
			else
				if(a.y == b.y)
					return (a.z<b.z);
				else
					return false;
		else
			return false;
}

/////////////////////////////////////////////////////////////////////////

// This function is for debugging purposes (prints the "map" of type : corresponderOutput);
void printMap(map<FlyType,vector<ViewCentroidPair> > &currMap)
{
	printf("\n\n******** current MAP *************");
	map<FlyType,vector<ViewCentroidPair> >::iterator it;
	int i=0;
	for(i=0,it = currMap.begin();it!=currMap.end();i++,it++)
	{
		printf("\nFlyType %d -> ",i);
		for(int j=0;j<(*it).second.size();j++)
			printf("(V = %d, C = %d), ",(*it).second[j].first,(*it).second[j].second);
	}
	printf("\n******************************");
}

/**
Default Constructor for Experiment. Simply calls initialize()
*/
Experiment::Experiment()
{
	initialize();
}

/** Set default values for various parameters. To be called by constructor. See im,plementation for details.
*/
void Experiment::initialize()
{
	currFrameNumberProcessed = -1L;
	Max_DFS_Runs = 2<<(sizeof(int)*8 - 1); // Maximum value of 
	Max_Tolerable_Error_Each_Centroid = 1.0/EPSILON; // == INFINITY
	Max_Tolerable_Error_Corresponder = 1.0/EPSILON; // == INFINITY

	nFlies = -1; // To avoid running any experiment without setting this number.
}

/**
Set number of flies (nFlies) for experiment. Also set number of flies for 3D Tracker. Should be called before doing anything meaningful.
\param numFlies : Number of flies in experiment
*/
void Experiment::setNFlies(int numFlies)
{
	if(numFlies>0)
		nFlies = numFlies;

	for(int countView=0;countView<V.size();countView++) // Reset the LastTrackIndex2D
	{
		V[countView]->lastTrackIndex2D.clear();
		for(int j=0;j<nFlies;j++)
			V[countView]->lastTrackIndex2D.push_back(j);
	}

	Tracker3D.setTargetNum(nFlies); // Set target number for 3D tracker
}

void Experiment::setNViews(int numViews)
{
	if(numViews>0)
		nViews = numViews;
}

/** Destructor for Experiment Class. Simply calls close() function.
\sa close()
*/
Experiment::~Experiment()
{
	close();
}


/**
Close the current experiment. It effectively deallocates ("delete") each "View" allocated (with "new") for experiment. Destructor also calls this function.
\sa ~Experiment()
*/
void Experiment::close()
{
	for(int i=0;i<V.size();i++)
	{
		V[i]->close();
		if(V[i]!=NULL)
			delete V[i];
	}
	V.clear();
}

/**
Initialize the kalman filter for an experiment. Number of flies should be set before calling this function
*/
void Experiment::initKalman() // Call after all views and nFlies has been setup
{
	float dt = 1.0f;
	const float F[] = 
	{ 1,  0,  0, dt,  0,  0,\
	0,  1,  0,  0, dt,  0,\
	0,  0,  1,  0,  0, dt,\
	0,  0,  0,  1,  0,  0,\
	0,  0,  0,  0,  1,  0,
	0,  0,  0,  0,  0,  1};

	for(int i=0;i<nFlies;i++)
	{
		kalmanArr.push_back(new CvKalman);
		kalmanArr[i] = cvCreateKalman( 6, 3, CV_64FC1);

		//CvMat* state = cvCreateMat(6, 1, CV_32FC1 );
		memcpy( kalmanArr[i]->transition_matrix->data.fl, F, sizeof(F));
		cvSetIdentity( kalmanArr[i]->measurement_matrix, cvRealScalar(.5) );
		cvSetIdentity( kalmanArr[i]->process_noise_cov, cvRealScalar(1e-5) );
		cvSetIdentity( kalmanArr[i]->measurement_noise_cov, cvRealScalar(1e-1) );
		cvSetIdentity( kalmanArr[i]->error_cov_post, cvRealScalar(1));
	}

}

/** Reinitialize kalman filter for an experiment. (required when there is a gap or 2 or more frame in consecutive measurements). 
\n Number of flies should be set before calling this function too.
*/
void Experiment::reInitKalman()
{
	printf("\n\n ************************ Reinitializing kalman .............. ************************ ");
	//getchar();
	printf("\n\n");
	for(int i=0;i<kalmanArr.size();i++)
	{
		cvReleaseKalman(&kalmanArr[i]);
	}
	kalmanArr.clear();
	initKalman();
}

/** Run kalman filter on a particular input 3D point and returns teh output 3D point
\param index Index of kalman filter to be used. (Each kalman[] is for a single fly and associated with it throughout an experiment).
\param inp Previously Observed 3D Point
\param vx Velocity in X direction based on past 2 measurements.
\param vy Velocity in Y direction based on past 2 measurements.
\param vz Velocity in Z direction based on past 2 measurements.
\param observedPt The correct point for previous run of kalman filter. To be used iff isCorrect == true
\param isCorrect If true, then correct based on previous observed point.
\return Predicted 3D Point
*/
Point3D Experiment::runKalman(int index,Point3D inp,double vx,double vy,double vz,Point3D observedPt,bool isCorrect)
{
	if(isCorrect) // correct kalman based on observation
	{
		CvMat* measurement = cvCreateMat( 3, 1, CV_32FC1 );
		cvmSet(measurement,0,0,observedPt.x);
		cvmSet(measurement,1,0,observedPt.y);
		cvmSet(measurement,2,0,observedPt.z);
		cvKalmanCorrect( kalmanArr[index], measurement);
		//printf("\nLocation = %.2lf,%.2lf",temp.x,temp.y);
	}

	cvmSet(kalmanArr[index]->state_post,0,0,inp.x);
	cvmSet(kalmanArr[index]->state_post,1,0,inp.y);
	cvmSet(kalmanArr[index]->state_post,2,0,inp.z);
	cvmSet(kalmanArr[index]->state_post,3,0,vx);
	cvmSet(kalmanArr[index]->state_post,4,0,vy);
	cvmSet(kalmanArr[index]->state_post,5,0,vz);


	const CvMat* prediction = cvKalmanPredict( kalmanArr[index], 0 );
	Point3D temp;

	temp.x = cvmGet(prediction,0,0);
	temp.y = cvmGet(prediction,1,0);
	temp.z = cvmGet(prediction,2,0);

	return temp;
}

/**
Grabs a given frame number for each view, and store it in "currFrame" of ViewClass. Parallelized using OpenMP.
\param frameIndex The frame number to be grabbed for each view
\return "true" If *ALL* Views could grab the given frame succesfully, "false" otherwise
*/
bool Experiment::grabFrame(unsigned long frameIndex)
{ 
	if(V.size()==0) //Nothing to do
		return false;
	int countView; // Loop counter: required to be initilized at top due to OpenMP usage
	//printf("\n in e.grabFrame and frameindex = %d",frameIndex);

	// Had to let go of idea of keeping a vector of grabFrameresult () due to RACE conditions
	//vector<bool> grabFrameResult; // To store output of each call made to grabFrame() for individual Views, do not want to use something fancy like "&&" or "||" in conjuction because of OpenMP loop
	omp_set_num_threads(int(V.size())); // Number of threads to be used by OpenMP
#pragma omp parallel shared(countView) // Beware of RACE conditions - if not taken care can return "false" at strange moments (different each time u run!_
	{
#pragma omp for schedule(dynamic)
		//Parallelize here
		for(countView=0;countView<V.size();countView++)
			V[countView]->grabFrame(frameIndex);
	}

	// Check weather any of the calls failed (currFrame of that view would be NULL) - Could not be checked while grabbing itself, because of OpenMP structured block (no jump allowed)
	for(int i=0;i<V.size();i++)
	{
		if(V[i]->grabFrameResult == false)
			return false;
		V[i]->grabFrameResult = false;
	}
	currFrameNumberProcessed = V[0]->currFrameNumberProcessed; // Update currFrameNumberPrcessed from view class
	return true;
}

/** Creates and initialize output AVI files for each view (all starting with a common prefix described by "fname"),

\param fname : if fname="AVIFile" then files : "AVIFileA.avi","AVIFileB.avi" .. and so on would be created for View 0,1, ...
\param size : Size of frame
\param fps : Frame rate per second for output AVI. Default = 60.0;
\return : "true" if creation of output file(s) is succesful, "false" otherwise
*/

bool Experiment::initSavingToFile(std::string fname,CvSize size,double fps)
{
	for(int i=0;i<V.size();i++)
	{
		char str[2] = {i+'A',0};
		string temp = str;
		string s = fname + temp + ".avi";
		string sCM = fname + temp + "-ChangeMask" + ".avi";
		if(V[i]->initSavingToFile(s,sCM,size,fps) == false)
			return false;
	}
	return true;
}

/** Appends currFrame.img for each view to their respective output AVI Files. Parallelized using OpenMP.
\warning initSavingToFile() should be called before appending any frame.
*/
void Experiment::appendToOutputFile()
{
	int nView=0;
#pragma omp parallel shared(nView)
	{
#pragma omp for schedule(dynamic)
		//Parallelize here
		for(nView=0;nView<V.size();nView++)
			V[nView]->appendToOutputFile();
	}
}

/** Stop saving output file. Safely close the CvVideoWriters associated with each output file.
*/
void Experiment::stopSavingToOutputFile()
{
	int nView=0;
	for(nView=0;nView<V.size();nView++)
		V[nView]->stopSavingToOutputFile();
}

/** Add a view to experiment. Should be called required number of times before starting an experiment.
*/
void Experiment::addView()
{
	View *temp = new View();
	printf("\n inexperiment add view.. inpType = %d",inpType);
	temp->inpType = inpType; // Copy the input type for view from experiment => All views in an experiment have same input type
//	temp->calculateInitialBG(200); // pick 200 frames uniformely from the video and make the initial bg model ..

	V.push_back(temp);
}

void Experiment::setInputType(View::InputType iType)
{
	inpType = iType;
}

bool Experiment::setInputAVIFileName(int ViewID, std::string aviFName)
{
	if(ViewID<0 || ViewID >= (int)V.size())
		return false;
	V[ViewID]->inpAVIFileName = aviFName;
	return true;
}

bool Experiment::setInputCentroidFileName(int ViewID,std::string centroidFName)
{
	if(ViewID<0 || ViewID >= (int)V.size())
		return false;
	V[ViewID]->inpCentroidFileName = centroidFName;
	return true;
}
/** Open the specified file as input AVI for specified view
\param ViewID : The ID of view to which the given input file should be associated. 
\param inpFile : Complete path for input file
\return : "true" if File is sucesfully opened, "false" otherwise
\warning Before calling openView() for a particular ViewID, user must have called addView() at least that number of times.
*/

bool Experiment::openView(int ViewID)
{
	if(ViewID<0 || ViewID >= (int)V.size())
		return false;
	return (V[ViewID]->open());
}


int Experiment::bestViewIndex()
{
	int maxIndex=-1;
	double maxVal = -1.0;

	int min_last = V[0]->framesSinceLastTracked2D;

	bool DetectStarted=false;
	for(int i=1;i<V.size();i++)
	{
		if(V[i]->framesSinceLastTracked2D !=-1)
		{
			DetectStarted = true;
			min_last = (min_last>V[i]->framesSinceLastTracked2D) ? V[i]->framesSinceLastTracked2D : min_last;
		}
	}
	if(DetectStarted == false) // This is the first frame .. tracking not yet started even
		return -1;

	// Now among those which had tracked flies at same point of time, use the MaxMin distance heuristic
	for(int i=0;i<V.size();i++)
	{
		if(V[i]->framesSinceLastTracked2D==min_last)
		{
			double newVal = V[i]->returnMinDistBetweenCentroids(); //NOT ANY MORE HARDCODED
			//if(newVal > maxVal && V[i]->centroids.size()==nFlies)
			if(newVal > maxVal && V[i]->centroids.size()==nFlies) //NOT ANY MORE HARDCODED
			{
				maxIndex = i;
				maxVal = newVal;
			}
		}
	}
	if(maxIndex == -1)
	{
		for(int i=0;i<V.size();i++)
		{
			double newVal = V[i]->returnMinDistBetweenCentroids(); //NOT ANY MORE HARDCODED FOR VALUE 1
			//if(newVal > maxVal && V[i]->centroids.size()==nFlies)
			if(newVal > maxVal && V[i]->centroids.size()==nFlies) //NOT ANY MORE HARDCODED
			{
				maxIndex = i;
				maxVal = newVal;
			}
		}
	}

	if(maxIndex == -1)
		maxIndex=0;
#if __D1
	printf("\nMaximum Min val for this frame = %.2lf",maxVal);
#endif
	return maxIndex;
}

int Experiment::Track2D() // Returns which view is best and tracks in all possible views
{
	int bestView = bestViewIndex();

	if(bestView == -1)
		return -1; // Cannot track;

	for(int i=0;i<V.size();i++)
		V[i]->track2D();

	return bestView;
}


void Experiment::draw3DPoints(vector<Point3D> pt)
{
	for(int countView=0;countView<V.size();countView++)
	{
		//if(V[countView]->currFrame.img == NULL)
		//	continue; // Nothing to do if there is no image grabbed

		if(V[countView]->centroidCalculator.originalFrameWindow[V[countView]->centroidCalculator.nPrevFrames] == NULL)
			continue;

		for(int countPt=0;countPt<pt.size();countPt++)
		{
			if(!isInvalidPoint(pt[countPt]))
			{
				Point2D out_pt = get2Dfrom3D(pt[countPt],V[countView]->camParams.projMatrix);
				CvFont font;
				double hScale=.5;
				double vScale=.5;
				int lineWidth=2;
				char str[10];
				sprintf_s(str,9,"%d",countPt);
				cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth);

				//cvPutText(V[countView]->currFrame.img,str,cvPoint((int)out_pt.x,(int)out_pt.y), &font, cvScalar(255,0,0));
				cvPutText(V[countView]->centroidCalculator.originalFrameWindow[V[countView]->centroidCalculator.nPrevFrames]\
					,str,cvPoint((int)out_pt.x,(int)out_pt.y), &font, cvScalar(255,0,0));
				char frameNum[100];
				sprintf(frameNum,"Frame# %d",(int)currFrameNumberProcessed);
				cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, .5,.5,0,1);
				//cvPutText(V[countView]->currFrame.img,frameNum,cvPoint(V[countView]->currFrame.img->width-120,50),&font,cvScalar(0,0,0));
				cvPutText(V[countView]->centroidCalculator.originalFrameWindow[V[countView]->centroidCalculator.nPrevFrames]\
					,frameNum,cvPoint(V[countView]->currFrame.img->width-120,50), &font, cvScalar(0,0,0));

			}
		}
	}
}

/** Runs Silh Detection for all views (parallelized using OpenMP)
\return A vector containing index of views where number of blobs detected == nFlies (i.e., safeViews for correspond and tracking operations). 
*/
//vector<int> Experiment::silhDetect()
//{
//	int i;
//	vector<int> toReturn;
//
//	omp_set_num_threads(int(V.size()));
//#pragma omp parallel shared(i)
//	{
//		#pragma omp for schedule(dynamic)
//		for(i=0;i<V.size();i++)
//			V[i]->silhDetect();
//	}
//	for(int countView=0;countView<V.size();countView++)
//		if(V[countView]->centroids.size() == nFlies)
//			toReturn.push_back(countView);
//	return toReturn;
//}


///////////////////////////////


vector<int> Experiment::processNewFrame()
{
	int i;
	vector<int> toReturn;
	//printf("\nI am in process 1");
	omp_set_num_threads(int(V.size()));
#pragma omp parallel shared(i)
	{
#pragma omp for schedule(dynamic)
		for(i=0;i<V.size();i++)
		{
			V[i]->processNewFrame();
		}
	}
	//printf("\nI am in process 2");
	currFrameNumberProcessed = V[0]->currFrameNumberProcessed;
	
	int countHowManyViewsHaveNFliesMeasurement = 0;
	// Now compute the safeViewList (those frames which have exactly nFlies measurements)
	for(int countView=0;countView<V.size();countView++)
	{
		
		V[countView]->origCentroidsByCentroidCalculator = V[countView]->centroids; // Keep a backup of orginal centroids - added for G-Correspondence

		assert(currFrameNumberProcessed == V[countView]->currFrameNumberProcessed); // Should not be false, since, each view should process the same frame at same time
		if((V[countView]->origCentroidsByCentroidCalculator.size()>=nFlies-1 && nFlies!=1) || (V[countView]->origCentroidsByCentroidCalculator.size()==nFlies)) // Changed for G-corresponder (initially condition was ==nFlies)
		{
			//printf("\nView = %d, centroids = %d",countView,V[countView]->origCentroidsByCentroidCalculator.size());
			toReturn.push_back(countView);
		}
		if(V[countView]->origCentroidsByCentroidCalculator.size() == nFlies)
			countHowManyViewsHaveNFliesMeasurement++;
		////////////// Added for G-Correspondence //////////////////
		// Add dummy (invalid) points to make each centroid vector size exactly equal to nFlies
		if(V[countView]->centroids.size()>nFlies)
			V[countView]->centroids.clear();
		while(V[countView]->centroids.size()<nFlies)
			V[countView]->centroids.push_back(getInvalidPoint());
		
		
		printf("\nV[%d]->centroids.size() = %d ",countView,V[countView]->centroids.size());
		
		assert(V[countView]->centroids.size() == nFlies); // At the end of this loop each centroid vector should contain exactly nFlie smeasurment (after appending invalid pts).
		////////////////////////////////////////////////////////////
		//printf("\nI am in process 3");
	}
	
	//for(int countView=0;countView<V.size();countView++)
	//{
	//	for(int i=0;i<V[countView]->centroids.size();i++)
	//	{
	//		printf("View%d cent%d = %.2lf,%.2lf\n",countView,i,V[countView]->centroids[i].x,V[countView]->centroids[i].y);
	//	}
	//	printf("\n\n");
	//}

	if(countHowManyViewsHaveNFliesMeasurement == nViews-1) // If 3 views have 5, do not include the one with <5 mesauements
	{
		for(int i=0;i<toReturn.size();i++)
			if(V[toReturn[i]]->origCentroidsByCentroidCalculator.size()!=nFlies)
				toReturn.erase(toReturn.begin()+i);
	}
	return toReturn;
}

/////////////////////////////
vector<Point3D> Experiment::Track3D()
{
	std::vector<Point3D> pts_3D;

	// Compute 3D points by back projecting now - We assume correspond() had been run before and Experiment::correspondOutput is populated
#if __D3D
	printf("\n*************************** 3-D Points ************************\n");
#endif
	map<FlyType,vector<ViewCentroidPair> >::iterator it;

	for(it = correspondOutput.begin();it!=correspondOutput.end();++it)
	{
		FlyType countFlyType = (*it).first;

		Point3D avg_pt;
		avg_pt.x = 0.0;
		avg_pt.y = 0.0;
		avg_pt.z = 0.0;

		int total =0;

		for(int countView1=0;countView1<(int)correspondOutput[countFlyType].size()-1;countView1++) //LACK OF (int) will put us in an (semi)infinite loop! (0-1) uint will be max_int ..
		{
			for(int countView2=countView1+1;countView2<(int)correspondOutput[countFlyType].size();countView2++)
			{

				int v1 = correspondOutput[countFlyType][countView1].first;
				int v2 = correspondOutput[countFlyType][countView2].first;

				Point2D p1_v1 = V[v1]->centroids[correspondOutput[countFlyType][countView1].second]; // NOT ANY MORE HARDCODED I HAVE TO COME BACK AND CLEAN THIS

				Point2D p1_v2 = V[v2]->centroids[correspondOutput[countFlyType][countView2].second]; // NOT ANY MORE HARDCODED I HAVE TO COME BACK AND CLEAN THIS

				
			
				// Compute 3D point for blob #countFlies in View1 and View 2
				Point3D p2_v1 = backProject(p1_v1,V[v1]->camParams.invProjMatrix);
				Point3D p2_v2 = backProject(p1_v2,V[v2]->camParams.invProjMatrix);

				Point3D pa,pb;
				double mua,mub;
				LineLineIntersect(V[v1]->camParams.opticalCenter,p2_v1, V[v2]->camParams.opticalCenter,p2_v2,&pa,&pb,&mua,&mub);

				/*printf("\nPa-Pb = %.3lf .",sqrt( (pa.x-pb.x)*(pa.x-pb.x) + (pa.y-pb.y)*(pa.y-pb.y) + (pa.z-pb.z)*(pa.z-pb.z) ));
				for(int i=0;i<safeViewList.size();i++)
				printf("%d ",safeViewList[i]);*/
				// Put a threshold here (lenght of line pa-pb)  - if you want to check authenticity of 3D points

				Point3D mean;
				mean = meanOfPoints(pa,pb);
				avg_pt.x += mean.x;
				avg_pt.y += mean.y;
				avg_pt.z += mean.z;
				total++;
#if __D3D
				printf("\n 3-Dpoint for fly %d amount %d and %d = (%.2lf,%.2lf,%.2lf)",countFlyType,v1,v2,mean.x,mean.y,mean.z);
#endif
			}
		}
		avg_pt.x /= (double)total;
		avg_pt.y /= (double)total;
		avg_pt.z /= (double)total;

		pts_3D.push_back(avg_pt);
	}
	if(Tracker3D.readyToCorrespond == false) // Just sort the first measurement, so that we always get same numbering of flies when we run FlySpy multiple time
		sort(pts_3D.begin(),pts_3D.end(),ComparePoint3DLessThan);

	
	//for(int i=0;i<pts_3D.size();i++)
	//{
	//	printf("\n3D point %d  = %.2lf,%.2lf,%.2lf",i,pts_3D[i].x,pts_3D[i].y,pts_3D[i].z);
	//}
	//printf("\ni should track now ...");

	Tracker3D.getNewMeasurement(pts_3D);

	if(lastTrackIndex3D.size()==0)
	{
		for(int i=0;i<pts_3D.size();i++) // to make it work for less than nFlies situation. g-correspondence
			lastTrackIndex3D.push_back(i);

	}

	//printf("\n&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");

	//for(int i=0;i<lastTrackIndex3D.size();i++)
	//	printf("ass[%d] = %d  ",i,lastTrackIndex3D[i]);

	
	Tracker3D.Track(lastTrackIndex3D);

	//printf("\nnow Assignment:");
	//for(int i=0;i<Tracker3D.assignmentVec.size();i++)
	//	printf("ass[%d] = %d  ",i,Tracker3D.assignmentVec[i]);

	//printf("\nend Of Assignment:");
	//printf("\nout of Track3D.track");
	vector<Point3D> output;//(nFlies);
	Point3D dummy3D;
	dummy3D.x = INFINITY_ALEPH0;
	dummy3D.y = INFINITY_ALEPH0;
	dummy3D.z = INFINITY_ALEPH0;
	
	for(int i=0;i<nFlies;i++)
		output.push_back(dummy3D);
	//printf("\nafter pushing back dummies");

#if __D3D
	printf("\n******** Assignment vector returned for this 3-D tracker ******************\n");
#endif




	for(int i=0;i<Tracker3D.assignmentVec.size();i++)
	{
#if __D3D

#endif
		//printf("\n%d",Tracker3D.assignmentVec[i]);
		output[Tracker3D.assignmentVec[i]] = pts_3D[i]; ////////// TOFIX TODO: POSITION OF ERROR /////////////////////
		// Runw ith DEBUG mode on, and put a breakpoint on this line - we see in Frame #15(first frame we track) that Tracker3D.assignmentVec[] contains an a
		// absurd value (-8210454 something like that). Why ? Interestingly error does not occur when we sample just 10 frames for Silh!!
		// Search for this line "for (int i = 0;i<assCurr.size();i++) // can be wrong ..." - it is in Tracker3D.Track() function !! - Legendary comment!! :P
	}
	lastTrackIndex3D = Tracker3D.assignmentVec;
	return output;



}

void Experiment::restartExperiment()
{
	// Re-init each view first
	for(int i=0;i<V.size();i++)
		V[i]->restart();

	Tracker3D.points2D.clear();
	Tracker3D.points3D.clear();
	lastTrackIndex3D.clear();
}

// usingDFS is to differentiate the calls made by StablePolyamory and DFS versions. Since DFS would like to prune the result.
double Experiment::computeTotalErrorForOneFlyType(vector<ViewCentroidPair> inp,bool usingDFS=false)
{
	double totalError=0.0;
	// Compute all pair of 2 views (each element in "inp" refer to a unique view, so basically generate all pairs {(m,n) |-> 0 <= m<n <= inp.size() }
	for(int i=0;i<inp.size()-1;i++)
	{
		for(int j=i+1;j<inp.size();j++)
		{
			int rowView = inp[i].first;
			int colView = inp[j].first;
			int rowViewCentroid = inp[i].second;
			int colViewCentroid = inp[j].second;
			
			//printf("\nheree 1");
		//printf("\ncount = %d %d %d",errorTable.table.count(make_PairOfView(rowView,colView)),rowView,colView);
			double currError = errorTable.table[make_PairOfView(rowView,colView)].backProjectionErrorTable[rowViewCentroid][colViewCentroid].value();
			//printf("\nhere 2");
			if(usingDFS==true && currError>Max_Tolerable_Error_Each_Centroid) // If we are using DFS then prune if value is greater than threshold
				return -1.0; // -1.0 => Prune

			//printf("\n%.2lf", errorTable.table[make_PairOfView(rowView,colView)].backProjectionErrorTable[rowViewCentroid][colViewCentroid].value());
			totalError += currError;
		}
	}
	return totalError;
}

// With DFS we may get -1.0 if this correspondence map exceed the thresholds anywhere. To get true cost, use with "usingDFS" = false
double Experiment::computeTotalErrorForOutput(map<FlyType,vector<ViewCentroidPair> > inpMap,bool usingDFS=false)
{
	map<FlyType,vector<ViewCentroidPair> >::iterator it;
	double totalError = 0.0;

	// Now iterate over all key values in the map, and add their errors (computed by another sub-function : computeTotalErrorForOneFlyType() );
	for(it=inpMap.begin();it!= inpMap.end();it++)
	{
		double currError = computeTotalErrorForOneFlyType((*it).second,usingDFS);
		if(usingDFS==true && areRealsEqual(totalError,-1.0)) // If we are using DFS, then prune if required
			return -1.0; //-1.0 => Prune

		totalError +=currError;
	}

	return totalError;
}

void Experiment::fillExperimentErrorTable(vector<int> safeViewList)
{
	errorTable.table.clear();


	for(int i=0;i<safeViewList.size()-1;i++)
	{
		for(int j=i+1;j<safeViewList.size();j++)
		{
			ViewPairErrorTable_struct tempViewErrTable,tempTransposeErrTable;
			int rowView = safeViewList[i];
			int colView = safeViewList[j];
			//	printf("\nError Table for views : %d and %d:\n",rowView,colView);
			tempViewErrTable.fillTable(rowView,colView,V[rowView],V[colView]);
			//	tempViewErrTable.print();
			tempViewErrTable.makeTranspose(tempTransposeErrTable);
			errorTable.table[make_PairOfView(rowView,colView)] = tempViewErrTable;
			errorTable.table[make_PairOfView(colView,rowView)] = tempTransposeErrTable;
		}
	}
}


void Experiment::correspondViewsUsingDFS(vector<int> safeViewList)
{
	if(safeViewList.size()==0) // Nothing to do
		return;

	DFS_Vars::leastError = INFINITY_ALEPH0; // == INFINITY
	DFS_Vars::nRuns = 0;
	DFS_Vars::safeViewList = safeViewList;
	//DFS_Vars::nCentroids = (int)V[safeViewList[0]]->centroids.size(); // nCentroids should be same for all pair of views

	//DFS_Vars::nCentroids = (int)V[safeViewList[0]]->centroidCalculator.windowCentroids[2].size(); // HARDCODED I HAVE TO COME BACK AND CHECK IT AGAIN
	DFS_Vars::nCentroids = (int)V[safeViewList[0]]->centroids.size(); // HARDCODED I HAVE TO COME BACK AND CHECK IT AGAIN
	// CHECKIT AGAIN 


	correspondOutput.clear();

	fillExperimentErrorTable(safeViewList);
	map<FlyType,vector<ViewCentroidPair> > tempMap;
	runDFS(tempMap,0);
	printf("\nTotal iteration = %d",DFS_Vars::nRuns);
}

void Experiment::runDFS(map<FlyType,vector<ViewCentroidPair> > tempMap,int viewIndexToProcess=0) // viewIndexToProcess = View index in safeViewList to process in this call of DFS
{
	DFS_Vars::nRuns++; // Increase the count by 1 (to count # recursive calls)

	if(viewIndexToProcess >= DFS_Vars::safeViewList.size()) // Base cases - Return if all views are done
	{
		DFS_Vars::leastError = computeTotalErrorForOutput(tempMap,true);
		if(areRealsEqual(DFS_Vars::leastError,-1.0)) // If final tempMap is not a valid correspondence (as per thresholds)
			DFS_Vars::leastError = INFINITY_ALEPH0; // == INFINITY
		else
			correspondOutput = tempMap;

		return;
	}

	// Step 1 : Add all centroids in safeViewList.centroids(correspondOutput.count())
	vector<int> centroidOrder;
	for(int i=0;i<DFS_Vars::nCentroids;i++) // nCentroids should be same for all views, 
	{													//  condition could have been i<V[safeViewList[viewIndexToProcess]]->centroids.size() too
		centroidOrder.push_back(i);
	}

	// Step 2 : Do all permutation of it ... (prune if not good) ... runDFS() if good ...
	bool firstTime=true; // To test when the do{ }while() loop is run for first time
	do
	{
		//Add the current order to correspondOutput;

		for(int i=0;i<DFS_Vars::nCentroids;i++) // DFS_Vars::nCentroids == nFlies
		{
			if(firstTime) // If it is the first time, simply push the view,centroid pair.
				tempMap[i].push_back(make_pair(DFS_Vars::safeViewList[viewIndexToProcess],centroidOrder[i]));
			else
			{ // If it is not the first time, then we need to find previous position of particular view for each fly type and change centroid to new one
				bool isBreak = false;
				for(int countViewCentroidPair=0;countViewCentroidPair<tempMap[i].size();countViewCentroidPair++)
				{
					if(tempMap[i][countViewCentroidPair].first==DFS_Vars::safeViewList[viewIndexToProcess])
					{
						tempMap[i][countViewCentroidPair].second = centroidOrder[i];
						isBreak = true;
						break;
					}
				}
				assert(isBreak == true);
			}
		}
		firstTime = false;
		double currError = computeTotalErrorForOutput(tempMap,false);
		if(areRealsEqual(currError,-1.0) || currError>DFS_Vars::leastError) // Prune
			continue;
		else
			runDFS(tempMap,viewIndexToProcess+1);

	}while(next_permutation(centroidOrder.begin(),centroidOrder.end()));
}

void Experiment::correspondViewsUsingStablePolyamory(vector<int> safeViewList,int &totalMisMatches,int &totalMinimumSequences)
{
	double currError = INFINITY_ALEPH2; // == INFINITY
	bool firstTime = true;

	map<pair<int,int> , vector<double> > corrErrorForEachInitialPairOfView; // I assume, whenever I insert value pair<int,int> is sorted, s.t., left <= right
	vector<double> total_correspondErrorVec; // Just to check how many permutation have similar errors;

	fillExperimentErrorTable(safeViewList);

	do
	{
		//	printf("\nDoing correspond for safeViewList %d: ",safeViewList[0]);
		map<FlyType,vector<ViewCentroidPair> > tempMap;
		stablePolyamory(safeViewList,tempMap);
		double tempError = computeTotalErrorForOutput(tempMap,false);
		//	printf("\nTempError  = %.2lf",tempError);
		
		///////// For analysis of effect of "sequential correspondence" /////////////
		total_correspondErrorVec.push_back(tempError);
		int v1 = safeViewList[0];
		int v2 = safeViewList[1];
		if(v1>v2)
			swap(v1,v2);
		corrErrorForEachInitialPairOfView[make_pair(v1,v2)].push_back(tempError);
		/////////////////////////////////////////////////////////////////////////////

		if(firstTime == true || tempError < currError)
		{
			firstTime = false;
			correspondOutput.clear();
			correspondOutput = tempMap;
			currError = tempError;
			//	printf("\nNew Map : ");
			//	printMap(correspondOutput);

		}

		tempMap.clear();
	}while(next_permutation(safeViewList.begin(),safeViewList.end()));

	// Count how many sequences lead to minimum correspondence error.
	int local_howManyMinimum = 0;
	for(int i=0;i<total_correspondErrorVec.size();i++)
	{	
		/*printf("\n{");
		for(int j=0;j<x.size();j++)
			printf(" %d, ",x[j]);
		printf(" }   ");*/
		if(areRealsEqual(total_correspondErrorVec[i],currError,.1))
			local_howManyMinimum++;
		//printf("%.4lf, %.4lf\n",total_correspondErrorVec[i],total_correspondErrorVec[i]-currError);
		/*if(!next_permutation(x.begin(),x.end()))
			break;*/
	}
	
	// Compute how many different error values are produced if initial two views in sequential correspondence are same.
	map<pair<int,int> , vector<double> >::iterator it;
	int local_total_mismatches = 0;
	for(it = corrErrorForEachInitialPairOfView.begin();it!=corrErrorForEachInitialPairOfView.end();++it)
	{
		 // Sort so that we can easily find is there had been a mismatch .
		// Sorting is DEFINITELY an overkill with this deinition of mismatch (since just a linear pass will tell us max and min value, which we can comaprE)
		// But if we want to count how many different values exist, then sorting can be used ... right now we don't give a damn :D
		sort((*it).second.begin(),(*it).second.end());
		
		int mismatch=0;
		if(!areRealsEqual((*it).second[0],(*it).second[(*it).second.size()-1],.1))
			mismatch =1;
		/*for(int i=1;i<(*it).second.size();i++)
			if(!areRealsEqual((*it).second[i-1],(*it).second[i]))
				mismatch++;*/
		local_total_mismatches += mismatch;
	}
	totalMisMatches = local_total_mismatches;
	totalMinimumSequences = local_howManyMinimum;


	//map<FlyType,vector<ViewCentroidPair> >::iterator it;
	//printf("\n=====stable orgy===");
	//for(it = correspondOutput.begin();it!=correspondOutput.end();it++)
	//{
	//	printf(	"\n  %d,", (*it).first);	
	//	for (int i = 0;i<(*it).second.size();i++)
	//		printf("(%d, %d)",(*it).second[i].first,(*it).second[i].second);
	//}

	printf("\nFinal total error of stable orgy = %.2lf",currError);
}


// We assume that every view is going to be paired with last view(==element) in "candidates"
// currMap is passed as a refrence, so that we dont have to copy data again and again
void Experiment::prepareForPolyamory(vector<int> candidates,map<FlyType,vector<ViewCentroidPair> > &currMap,pair<vector<vector<int> >,vector<vector<int> > > &choiceMatrices) 
{
	// We will arbitrarily call last element of "candidates" as "FEMALE" candidate and all others as MALE candidates
	//printf("\nIn prepare polyamory ... ");
	assert(candidates.size()>1); // Must have atleast 2 candidates for a Polyamory (or marriage)
	//Note we also assume all Views present in "candidates" have same number of centroids

	//int nCentroids = (int)V[candidates[0]]->centroids.size(); // Should be same for all element of candidates //HARDCODED

	int nCentroids = (int)V[candidates[0]]->centroids.size(); // Should be same for all element of candidates //NOT ANY MORE HARDCODED 1 SHOULD BE PRE



	assert(nCentroids == nFlies); // Should be equal to number of flies 

	// The "s" in males is because of the fact that their can be more than 1 male candidate.
	vector<vector<ErrorAndFlyType> > rankMatrixForMales(nFlies,vector<ErrorAndFlyType>(nFlies)); // Declare a vector of Size nFlies X nFlies 
	vector<vector<ErrorAndFlyType> > rankMatrixForFemale(nFlies,vector<ErrorAndFlyType>(nFlies)); // Declare a vector of Size nFlies X nFlies 

	const int femaleViewIndex = (const int)(candidates.size()-1);

	map<FlyType,vector<ViewCentroidPair> >::iterator it;
	int countFlyType = 0;
	/*printf("\n\nCandidates : ");
	for(int i=0;i<candidates.size();i++)
	printf(" %d ",candidates[i]);*/
	for (countFlyType = 0, it=currMap.begin();it != currMap.end();it++,countFlyType++) // This loop is for iterating over all fly types
	{
		for(int countFemaleCentroids=0;countFemaleCentroids<nCentroids;countFemaleCentroids++) // This loop is for iterating over all centroids in the "FEMALE" View
		{
			double err=0.0;
			for(int countCentroidInFlyType=0;countCentroidInFlyType<(*it).second.size();countCentroidInFlyType++) // This loop is for iterating over all centroids for a "FlyType"
			{
				int rowView = (*it).second[countCentroidInFlyType].first;
				/*int rowView_position_in_candidate = find_in_vector(candidates,rowView);
				if(rowView_position_in_candidate == -1 || rowView_position_in_candidate == femaleViewIndex)
				continue;*/

				int colView = candidates[femaleViewIndex];

				int rowViewCentroid = (*it).second[countCentroidInFlyType].second;
				int colViewCentroid = countFemaleCentroids;

				/*	printf("\nRowView centroid = %d rowView = %d countCentroidInFlyType = %d %d", rowViewCentroid,rowView,countCentroidInFlyType,(*it).second.size());
				printf("\nCol view centroid = %d colView = %d",colViewCentroid,colView);*/

				err += errorTable.table[make_PairOfView(rowView,colView)].backProjectionErrorTable[rowViewCentroid][colViewCentroid].value();

			}
			rankMatrixForMales[countFlyType][countFemaleCentroids].error = err;
			rankMatrixForMales[countFlyType][countFemaleCentroids].flyType = countFemaleCentroids;

			rankMatrixForFemale[countFemaleCentroids][countFlyType].error = err;
			rankMatrixForFemale[countFemaleCentroids][countFlyType].flyType = countFlyType;

		}
	}

	//printf("\n******* One polyamory \n");
	// Note that size of rankMatrixForFemale rankMatrixForMales both have size == nFlies x nFlies ... and nFlies == nCentroids

	//printf("\n rankMatrixforMale: \n");
	//for (int i = 0;i<rankMatrixForMales.size();i++)
	//{
	//	printf("[");
	//	for (int j = 0;j<rankMatrixForMales[i].size();j++)
	//	{
	//		printf(" %.2lf, ",rankMatrixForMales[i][j].error);


	//	}
	//	printf("]\n");

	//}


	//printf("\n rankMatrixforFemale: \n");
	//for (int i = 0;i<rankMatrixForFemale.size();i++)
	//{
	//	printf("[");
	//	for (int j = 0;j<rankMatrixForFemale[i].size();j++)
	//	{
	//		printf(" %.2lf, ",rankMatrixForFemale[i][j].error);


	//	}
	//	printf("]\n");

	//}

	for(int i=0;i<nFlies;i++)
	{
		sort(rankMatrixForMales[i].begin(),rankMatrixForMales[i].end());
		sort(rankMatrixForFemale[i].begin(),rankMatrixForFemale[i].end());
		/*	printf("\n\ni = %d .. candidates.size = %d ",i,candidates.size());
		for(int j=0;j<rankMatrixForMales[i].size();j++)
		{
		printf("\n(%.2lf,%d)  (%.2lf,%d)",rankMatrixForMales[i][j].error,rankMatrixForMales[i][j].flyType,rankMatrixForFemale[i][j].error,rankMatrixForFemale[i][j].flyType);
		}*/
	}


	//	vector<vector<ErrorAndFlyType> > rankMatrixForMales(nFlies,vector<ErrorAndFlyType>(nFlies)); // Declare a vector of Size nFlies X nFlies 
	//	vector<vector<ErrorAndFlyType> > rankMatrixForFemale(nFlies,vector<ErrorAndFlyType>(nFlies)); // Declare a vector of Size nFlies X nFlies 




	//First clear the choice matrix and then fill it
	for(int i=0;i<choiceMatrices.first.size();i++)
	{
		choiceMatrices.first.clear();
		choiceMatrices.second.clear();
	}
	choiceMatrices.first.clear();
	choiceMatrices.second.clear();

	//Now fill in the choice matrices
	for(int i=0;i<nFlies;i++)
	{
		vector<int> tempVec1;
		vector<int> tempVec2;

		for(int j=0;j<nFlies;j++)
		{
			tempVec1.push_back(rankMatrixForMales[i][j].flyType);
			tempVec2.push_back(rankMatrixForFemale[i][j].flyType);
		}
		choiceMatrices.first.push_back(tempVec1); // First element of Choice matrix if choiceMatrix for MALES
		choiceMatrices.second.push_back(tempVec2); // Second element is choice Matrix for females
	}
	//	printf("\nOut of prepare polyampry");

	//printf("\n choiceMatrices.first: \n");
	//for (int i = 0;i<choiceMatrices.first.size();i++)
	//{
	//	printf("[");
	//	for (int j = 0;j<choiceMatrices.first[i].size();j++)
	//	{
	//		printf(" %d, ",choiceMatrices.first[i][j]);


	//	}
	//	printf("]\n");
	//}

	//printf("\n choiceMatrices.second: \n");
	//for (int i = 0;i<choiceMatrices.second.size();i++)
	//{
	//	printf("[");
	//	for (int j = 0;j<choiceMatrices.second[i].size();j++)
	//	{
	//		printf(" %d ",choiceMatrices.second[i][j]);


	//	}
	//	printf("]\n");
	//}
}




vector<int> Experiment::findStableMarriage(pair<vector<vector<int> >,vector<vector<int> > > &choiceMatrices)
{
	vector<int> malesNextProposalIndex;
	vector<int> malesPartnerList;
	vector<int> femalesPartnerList;

	const int TOTAL_MALES = (int)choiceMatrices.first.size(); // Should be equal to = choiceMatrices.second.size() (both should be square matrix of same size)
	const int FREE = -1;
	for(int i=0;i<TOTAL_MALES;i++)
	{
		malesNextProposalIndex.push_back(0);
		malesPartnerList.push_back(FREE);
		femalesPartnerList.push_back(FREE);
	}

	//	printf("\nin Stable marriage");

	int nextFreeMale=-1;

	while((nextFreeMale = find_in_vector(malesPartnerList,FREE))!=-1) // While there are free males execute the loop
	{		
		// Now male would propose highest ranking woman he has not yet propsed. So Proposes woman # = maleNextProposalIndex[nextFreeMale]
		// Then two possible cases:
		int femaleProposed = choiceMatrices.first[nextFreeMale][malesNextProposalIndex[nextFreeMale]];
		malesNextProposalIndex[nextFreeMale]++; // Increase the proposal index for next time

		// Case 1: Woman being proposed is free herself. In that case both get engaged.
		if(femalesPartnerList[femaleProposed] == FREE)
		{
			femalesPartnerList[femaleProposed] = nextFreeMale;
			malesPartnerList[nextFreeMale] = femaleProposed;
			continue;
		}

		//Case 2: Woman being proposed is engaged to somebody else: Two further sub cases
		int currentFiance = femalesPartnerList[femaleProposed];
		for(int i=0;true;i++) // Should come out of loop with "break" statements always
		{
			if(choiceMatrices.second[femaleProposed][i]==currentFiance) // Case 2a : If this is true, then woman prefer current fiance over new one. So she wont break the enagagement.
				break;

			if(choiceMatrices.second[femaleProposed][i]==nextFreeMale) // Case 2b: If this is true, then woman prefer new male over already enagaged one. So she gets engaged to new one.
			{
				malesPartnerList[currentFiance] = FREE;
				femalesPartnerList[femaleProposed] = nextFreeMale;
				malesPartnerList[nextFreeMale] = femaleProposed;
				break;
			}
		}
	}
	/*printf("\ndone with stable marriage : ");
	for(int i=0;i<femalesPartnerList.size();i++)
	printf(" %d ",femalesPartnerList[i]);*/

	return femalesPartnerList;
}

void Experiment::stablePolyamory(vector<int> orderViews,map<FlyType,vector<ViewCentroidPair> > &toReturn)
{
	toReturn.clear();
	assert(V[orderViews[0]]->centroids.size() == nFlies); // This should be true for infact all values of orderViews (0,1,2,3....) 

	// Initially assign "FlyType" according to centroid order in first view
	for(int i=0;i<nFlies;i++)
	{
		vector<ViewCentroidPair> tempVec(1);
		tempVec[0].first = orderViews[0];
		tempVec[0].second = i;

		toReturn[i] = tempVec;
	}

	pair<vector<vector<int> >,vector<vector<int> > > choiceMatrices;

	vector<int> orderForMarriage;
	orderForMarriage.push_back(orderViews[0]);
	orderForMarriage.push_back(orderViews[1]);

	map<FlyType,vector<ViewCentroidPair> >::iterator it;
	for(int i=1;i<orderViews.size();i++)
	{
		//	printf("\ndoing marriage for grup of size = %d",orderForMarriage.size());
		prepareForPolyamory(orderForMarriage,toReturn,choiceMatrices);

		vector<int> femaleMatching = findStableMarriage(choiceMatrices);

		for(int countFemaleCentroids=0;countFemaleCentroids<femaleMatching.size();countFemaleCentroids++)
		{
			it=toReturn.find(femaleMatching[countFemaleCentroids]);
			//			assert(it!=map::end); // Should not happen that a particular type does not exist
			(*it).second.push_back(make_pair(orderForMarriage[orderForMarriage.size()-1],countFemaleCentroids));
		}
		if(i!=orderViews.size()-1)
			orderForMarriage.push_back(orderViews[i+1]);
	}
	// At the end of this loop, "toReturn" will have required output. .. So just return
}


//

/** Coressponds centroids of 2D views using Hungarian
\n since the order of views in the process of correspondance matters, this function calls hangarianViewCorresponding with all permutation of safeView list and finds the one with Minimum error

\param safeViewList is a vector of ID of views that are 'safe' which means they have nFlies measurement
\sa hungarianViewCorresponding(vector<int> orderViews,map<FlyType,vector<ViewCentroidPair> > &toReturn), calculateCostMatrixForHungarian(vector<int> candidates,map<FlyType,vector<ViewCentroidPair> > &currMap,int* costVec)
OUTDATED comment above

*/

// totalMisMatches return how many different error values arose when initial tqo views were same.
// totalMinimuSequences return how many sequences lead to minimum error.
void Experiment::correspondViewsUsingHungarian(vector<int> safeViewList,int &totalMisMatches,int &totalMinimumSequences)
{
	//this function has been written similar to correspondViewsUsingStablePolyamory(), so most of the comments here would apply for that function too...
	
	map<pair<int,int> , vector<double> > corrErrorForEachInitialPairOfView; // I assume, whenever I insert value pair<int,int> is sorted, s.t., left <= right

	// the commented code below is to check permutation effect on hungarian
	/*vector<int> x;
	for(int i=0;i<safeViewList.size();i++)
		x.push_back(safeViewList[i]);
	sort(x.begin(),x.end());*/

	double currError = INFINITY_ALEPH2; // == INFINITY
	fillExperimentErrorTable(safeViewList); // calculates the error table for the experiment once
	
	bool firstTime = true;
	vector<double> total_correspondErrorVec; // Just to check how many permutation have similar errors;

	do
	{
		map<FlyType,vector<ViewCentroidPair> > tempMap; // this is to keep the return map(correspondence) from hungarianViewCorresponding which is being called in next line ...

		
		hungarianViewCorresponding(safeViewList, tempMap);
		//printMap(tempMap);
		
		double tempError = computeTotalErrorForOutput(tempMap,false); // caclulate current correspondance error
		
		///////// For analysis of effect of "sequential correspondence" /////////////
		total_correspondErrorVec.push_back(tempError);
		int v1 = safeViewList[0];
		int v2 = safeViewList[1];
		if(v1>v2)
			swap(v1,v2);
		corrErrorForEachInitialPairOfView[make_pair(v1,v2)].push_back(tempError);
		/////////////////////////////////////////////////////////////////////////////

		if(firstTime == true || tempError < currError)
		{
			firstTime = false;
			correspondOutput.clear();
			correspondOutput = tempMap;
			currError = tempError;
		}
		tempMap.clear();
	}while(next_permutation(safeViewList.begin(),safeViewList.end()));
	//sort(total_correspondErrorVec.begin(),total_correspondErrorVec.end());
	//printf("\nCorrespond error .... ");
	
	//// the commented code below is to check permutation effect on hungarian

	// Count how many sequences lead to minimum correspondence error.
	int local_howManyMinimum = 0;
	for(int i=0;i<total_correspondErrorVec.size();i++)
	{	
		/*printf("\n{");
		for(int j=0;j<x.size();j++)
			printf(" %d, ",x[j]);
		printf(" }   ");*/
		if(areRealsEqual(total_correspondErrorVec[i],currError,.1))
			local_howManyMinimum++;
		//printf("%.4lf, %.4lf\n",total_correspondErrorVec[i],total_correspondErrorVec[i]-currError);
		/*if(!next_permutation(x.begin(),x.end()))
			break;*/
	}
	
	// Compute how many different error values are produced if initial two views in sequential correspondence are same.
	map<pair<int,int> , vector<double> >::iterator it;
	int local_total_mismatches = 0;
	for(it = corrErrorForEachInitialPairOfView.begin();it!=corrErrorForEachInitialPairOfView.end();++it)
	{
		 // Sort so that we can easily find is there had been a mismatch .
		// Sorting is DEFINITELY an overkill with this deinition of mismatch (since just a linear pass will tell us max and min value, which we can comaprE)
		// But if we want to count how many different values exist, then sorting can be used ... right now we don't give a damn :D
		sort((*it).second.begin(),(*it).second.end());
		
		int mismatch=0;
		if(!areRealsEqual((*it).second[0],(*it).second[(*it).second.size()-1],.1))
			mismatch =1;
		/*for(int i=1;i<(*it).second.size();i++)
			if(!areRealsEqual((*it).second[i-1],(*it).second[i]))
				mismatch++;*/
		local_total_mismatches += mismatch;
	}
	totalMisMatches = local_total_mismatches;
	totalMinimumSequences = local_howManyMinimum;
	//printf("\nFinal total error of hungarian = %.2lf",currError);
	//getchar();
}

/** calculates cost matrix to use as Hungarian algorithm input
\n 
\param candidates is a vector of ID of views that are being corresponded
\param currMap is the current map of correspondence
\return costVec as an 1-D array of size (nFlies x nFlies) of int is being returned to be used as input to Hugarian algorithm

*/

// We assume that every view is going to be paired with last view(==element) in "candidates"
// currMap is passed as a refrence, so that we dont have to copy data again and again, 
//just to clarify, we are not changing anything in currMap in this function and so its not a returned parameter

//void Experiment::calculateCostMatrixForHungarian(vector<int> candidates,map<FlyType,vector<ViewCentroidPair> > &currMap,int* costVec)
//{
//	// this function has been written using prepareForPolyamory(), so most of the comments here would apply there too...
//	// to calculate the cost matrix, error between each centroid of new view and each map line (we have nFlies line in map)
//	// has to be calculated. since the errortable between all views and their centroids has been filled before calling the correspondece
//	// so for example here if we want to calculate errorMatrix[2,3], we find the error for correspondence of centroid #3 of newAddedView 
//	// and the centroids of all previous views that are as Fly #2 or in other words line 2 of current map. these errors will be sumed up and gives errorMatrix[2,3] as result
//	// the only thing that we have to do here is to add the errors between centroid ith of last view and 
//	assert(candidates.size()>1); // Must have atleast 2 candidates for correspondance
//
//	int nCentroids = (int)V[candidates[0]]->centroids.size(); // Should be same for all element of candidates 
//
//	assert(nCentroids == nFlies); // Should be equal to number of flies 
//
//	vector<vector<ErrorAndFlyType> > errorMatrix(nFlies,vector<ErrorAndFlyType>(nFlies)); // Declare a vector of Size nFlies X nFlies to save the error
//	// this basically means the last element in vector, 
//	// since before calling this function we are pushing back the ID of new view that we want to add to map in candidates vector, 
//	// we always want to look at the last added view and find the cost of adding that view to the current map
//	const int newAddedViewIndex = (const int)(candidates.size()-1); 
//
//	map<FlyType,vector<ViewCentroidPair> >::iterator it; // to iterate over the map
//	int countFlyType = 0;
//	for (countFlyType = 0, it=currMap.begin();it != currMap.end();it++,countFlyType++) // This loop is for iterating over all fly types
//	{
//		for(int newAddedViewCentroids=0;newAddedViewCentroids<nCentroids;newAddedViewCentroids++) // This loop is for iterating over all centroids in the NewAddedView
//		{
//			double err=0.0;
//			//////////////// Code added for G-correspondence //////////////////
//			bool isDummySequence = false; // Does the sequence of centroid we are iterating has any dummy point in it ?
//			bool isTargetCentroidDummy = false; // If the target centroid is invalid
//			///////////////////////////////////////////////////////////////
//
//			for(int countCentroidInFlyType=0;countCentroidInFlyType<(*it).second.size();countCentroidInFlyType++) // This loop is for iterating over all centroids for a "FlyType"
//			{
//				int rowView = (*it).second[countCentroidInFlyType].first;
//				/*int rowView_position_in_candidate = find_in_vector(candidates,rowView);
//				if(rowView_position_in_candidate == -1 || rowView_position_in_candidate == femaleViewIndex)
//				continue;*/
//				int colView = candidates[newAddedViewIndex];
//				int rowViewCentroid = (*it).second[countCentroidInFlyType].second;
//				int colViewCentroid = newAddedViewCentroids;
//
//				err += errorTable.table[make_PairOfView(rowView,colView)].backProjectionErrorTable[rowViewCentroid][colViewCentroid].value();
//
//				////////////////////////// Code added for G-correspondence /////////////////////
//				if (isInvalidPoint(V[rowView]->centroids[rowViewCentroid]))
//					isDummySequence = true;
//				if(isInvalidPoint(V[colView]->centroids[colViewCentroid]))
//					isTargetCentroidDummy = true;
//				///////////////////////////////////////////////////////////////////////////////////
//			}
//
//			//////////////// Code added for G-correspondence //////////////////
//			int howManyInvalid = ((isDummySequence) ? 1 : 0) + ((isTargetCentroidDummy) ? 1 : 0); 
//			if(howManyInvalid == 1) // When exactly one of the centroid is dummy, then error should be +Infinity
//				err = INFINITY_POSITIVE_double;
//			if(howManyInvalid ==2) // When both the centroids are dummy then error should be -Infinity
//				err = INFINITY_NEGATIVE_double;
//			// if howManyInvalid == 0 then keep the err value calculated.
//			if(howManyInvalid==0)
//				assert(err<=1e12&&err>-EPSILON); // Just a sanity check, that no value contained +-Infinity somewhere
//			////////////////////////////////////////////////////////////////////
//			errorMatrix[countFlyType][newAddedViewCentroids].error = err;
//			errorMatrix[countFlyType][newAddedViewCentroids].flyType = newAddedViewCentroids; // we are not using this to calculate cost function ..
//		}
//	}
//	for (int i = 0;i<errorMatrix.size();i++) // fill up t he cost Vector using the errors in errorMatrix
//		for (int j = 0;j<errorMatrix[i].size();j++)
//			costVec[nFlies *i + j] = (int)(errorMatrix[i][j].error * 100.00); //magnifying error to makes sure we are not loosing any important information by casting from double to int
//
//}
//
///** this function simply calls hungCorrespondOf2SetsCostFunctionVersion()
//\param inputCostVec is a pointer to the cost vector that have been calculated in calculateCostMatrixForHungarian()
//\return vector of assignment as output of Hungarian algorithm
//*/
//
////vector<int> Experiment::findHungarianCorrespondence(int *inputCostVec)
////{
////
////	vector<int> asss1, asss2;
////	hungCorrespondOf2SetsCostFunctionVersion(nFlies,nFlies,inputCostVec,asss1,asss2); // this function uses inputCostVec as cost function for Hungarian and returns assignment vectors
////	return asss2;
////
////}


void Experiment::calculateCostMatrixForHungarian(vector<int> candidates,map<FlyType,vector<ViewCentroidPair> > &currMap,Matrix <double> &m)
{
	// this function has been written using prepareForPolyamory(), so most of the comments here would apply there too...
	// to calculate the cost matrix, error between each centroid of new view and each map line (we have nFlies line in map)
	// has to be calculated. since the errortable between all views and their centroids has been filled before calling the correspondece
	// so for example here if we want to calculate errorMatrix[2,3], we find the error for correspondence of centroid #3 of newAddedView 
	// and the centroids of all previous views that are as Fly #2 or in other words line 2 of current map. these errors will be sumed up and gives errorMatrix[2,3] as result
	// the only thing that we have to do here is to add the errors between centroid ith of last view and 
	assert(candidates.size()>1); // Must have atleast 2 candidates for correspondance

	int nCentroids = (int)V[candidates[0]]->centroids.size(); // Should be same for all element of candidates 

	assert(nCentroids == nFlies); // Should be equal to number of flies 

	vector<vector<ErrorAndFlyType> > errorMatrix(nFlies,vector<ErrorAndFlyType>(nFlies)); // Declare a vector of Size nFlies X nFlies to save the error
	// this basically means the last element in vector, 
	// since before calling this function we are pushing back the ID of new view that we want to add to map in candidates vector, 
	// we always want to look at the last added view and find the cost of adding that view to the current map
	const int newAddedViewIndex = (const int)(candidates.size()-1); 

	map<FlyType,vector<ViewCentroidPair> >::iterator it; // to iterate over the map
	int countFlyType = 0;
	for (countFlyType = 0, it=currMap.begin();it != currMap.end();it++,countFlyType++) // This loop is for iterating over all fly types
	{
		for(int newAddedViewCentroids=0;newAddedViewCentroids<nCentroids;newAddedViewCentroids++) // This loop is for iterating over all centroids in the NewAddedView
		{
			double err=0.0;
			//////////////// Code added for G-correspondence //////////////////
			bool isDummySequence = false; // Does the sequence of centroid we are iterating has any dummy point in it ?
			bool isTargetCentroidDummy = false; // If the target centroid is invalid
			///////////////////////////////////////////////////////////////

			for(int countCentroidInFlyType=0;countCentroidInFlyType<(*it).second.size();countCentroidInFlyType++) // This loop is for iterating over all centroids for a "FlyType"
			{
				int rowView = (*it).second[countCentroidInFlyType].first;
				/*int rowView_position_in_candidate = find_in_vector(candidates,rowView);
				if(rowView_position_in_candidate == -1 || rowView_position_in_candidate == femaleViewIndex)
				continue;*/
				int colView = candidates[newAddedViewIndex];
				int rowViewCentroid = (*it).second[countCentroidInFlyType].second;
				int colViewCentroid = newAddedViewCentroids;

				err += errorTable.table[make_PairOfView(rowView,colView)].backProjectionErrorTable[rowViewCentroid][colViewCentroid].value();

				////////////////////////// Code added for G-correspondence /////////////////////
				if (isInvalidPoint(V[rowView]->centroids[rowViewCentroid]))
					isDummySequence = true;
				if(isInvalidPoint(V[colView]->centroids[colViewCentroid]))
					isTargetCentroidDummy = true;
				///////////////////////////////////////////////////////////////////////////////////
			}

			//////////////// Code added for G-correspondence //////////////////
			int howManyInvalid = ((isDummySequence) ? 1 : 0) + ((isTargetCentroidDummy) ? 1 : 0); 
			if(howManyInvalid == 1) // When exactly one of the centroid is dummy, then error should be +Infinity
				err = INFINITY_ALEPH0;
			if(howManyInvalid ==2) // When both the centroids are dummy then error should be -Infinity
				err = -1.0*INFINITY_ALEPH0;
			// if howManyInvalid == 0 then keep the err value calculated.
			if(howManyInvalid==0)
				assert(err<=INFINITY_ALEPH0 && err>=(-EPSILON)); // Just a sanity check, that no value contained +-Infinity somewhere
			////////////////////////////////////////////////////////////////////
			errorMatrix[countFlyType][newAddedViewCentroids].error = err;
			errorMatrix[countFlyType][newAddedViewCentroids].flyType = newAddedViewCentroids; // we are not using this to calculate cost function ..
		}
	}
	
	for (int i = 0;i<errorMatrix.size();i++) // fill up t he cost Vector using the errors in errorMatrix
	{
	
		for (int j = 0;j<errorMatrix[i].size();j++)
		{
			m(i,j) = errorMatrix[i][j].error;
			
		}
		
	}
			//costVec[nFlies *i + j] = (int)(errorMatrix[i][j].error * 100.00); //magnifying error to makes sure we are not loosing any important information by casting from double to int


}

/** this function simply calls hungCorrespondOf2SetsCostFunctionVersion()
\param inputCostVec is a pointer to the cost vector that have been calculated in calculateCostMatrixForHungarian()
\return vector of assignment as output of Hungarian algorithm
*/


vector<int> Experiment::findHungarianCorrespondence(Matrix<double> inputCostMatrix)
{
	//printf("\n I am at  Experiment::findHungarianCorrespondence(Matrix<double> inputCostMatrix)");

	vector<int> asss1, asss2;
	hungCorrespondOf2SetsCostFunctionVersion(nFlies,nFlies,inputCostMatrix,asss1,asss2); // this function uses inputCostVec as cost function for Hungarian and returns assignment vectors

	//printf("\n I am at  end of Experiment::findHungarianCorrespondence(Matrix<double> inputCostMatrix)");
	return asss2;

}


/** this function is the main function of corresponding views using hungarian which basically calls other functions
\n and returns corresponding map
\param orderViews is a permutation of safeviewList,
\return map of corresponding (toReturn parameter)
\sa stablePolyamory()
*/
void Experiment::hungarianViewCorresponding(vector<int> orderViews,map<FlyType,vector<ViewCentroidPair> > &toReturn)
{
	// this function has been written using stablePolyamory (), so comments here would apply mostly for that function too
	toReturn.clear();
	assert(V[orderViews[0]]->centroids.size() == nFlies); // This should be true for infact all values of orderViews (0,1,2,3....) 

	// Initially assign "FlyType" according to centroid order in first view
	for(int i=0;i<nFlies;i++)
	{
		vector<ViewCentroidPair> tempVec(1);
		tempVec[0].first = orderViews[0];
		tempVec[0].second = i;

		toReturn[i] = tempVec;
	}

	vector<int> viewListinMap;

	viewListinMap.push_back(orderViews[0]); // we start with first and second view in the permutation, since we need at least two view to start correspondence
	viewListinMap.push_back(orderViews[1]);

	map<FlyType,vector<ViewCentroidPair> >::iterator it;
	for(int i=1;i<orderViews.size();i++)
	{
		//int* theCost;
		//theCost = new int [nFlies*nFlies]; // Allocate memory for cost matrix
		//calculateCostMatrixForHungarian(viewListinMap,toReturn,theCost);

		//vector<int> assignmentResult = findHungarianCorrespondence(theCost);

		//delete [] theCost; // free the memory for cost matrix

		Matrix<double> theCost(nFlies, nFlies);
		calculateCostMatrixForHungarian(viewListinMap,toReturn,theCost);
		vector<int> assignmentResult = findHungarianCorrespondence(theCost);
		

		for(int countFemaleCentroids=0;countFemaleCentroids<assignmentResult.size();countFemaleCentroids++)
		{
			it=toReturn.find(assignmentResult[countFemaleCentroids]);
			(*it).second.push_back(make_pair(viewListinMap[viewListinMap.size()-1],countFemaleCentroids));
		}
		if(i!=orderViews.size()-1)
			viewListinMap.push_back(orderViews[i+1]); // add the next view in permutation and continue filling the Map
	}
	//printf("\nend of looooop");
	// At the end of this loop, "toReturn" will have required output. .. So just return
	}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////// Added for G-correspondence //////////////////////
bool Experiment::containDummyPoint(vector<ViewCentroidPair> vec)
{
	for(int i=0;i<vec.size();i++)
	{
		//printf("\nView centroid pair = %d,%d",vec[i].first,vec[i].second);
		if(isInvalidPoint(V[vec[i].first]->centroids[vec[i].second]))
		{
			//printf("\nI was invalid ... ");
			return true;
		}
	}
	return false;
}

void Experiment::clearCorresponderOutput()
{
	if(!correspondOutput.empty())
	{
		map<FlyType,vector<ViewCentroidPair> >::iterator it;
		vector<map<FlyType,vector<ViewCentroidPair> >::iterator > toRemoveit;
		int count=0;
		for(it=correspondOutput.begin();it!=correspondOutput.end();++it,count++)
		{
			if(containDummyPoint((*it).second))
				toRemoveit.push_back(it);
		}
		count-= toRemoveit.size();
		howManyCentroidsCorresponded = count;
		//printf("\nhowmanycentroid = %d",howManyCentroidsCorresponded);
		for(int i=0;i<toRemoveit.size();i++)
			correspondOutput.erase(toRemoveit[i]); // Remove it from corresponder if it contains a dummy point
	}
}