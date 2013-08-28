// Experiment.cpp: Header for all functions related to an Experiment

#pragma once

#ifndef __EXPERIMENT_H_INCLUDED__
#define __EXPERIMENT_H_INCLUDED__

#include "View.h"
#include "Error.h"

using namespace std;

typedef int FlyType; //< "FlyType" is a euphemism to allow corresponder to tag all centroids (in different views) as same type (what it believes correspond to same fly in real world).
typedef pair<int,int> ViewCentroidPair; //< The purpose of this redefinition is to store as <View #,Centroid #> as a pair. ex : <0,1> => View 0's Centroid 1

// Sort correspondence map (w.r.t, centroid of View "vIndex") - useful for comparing correspondence in simulated data
void sortCorrespondenceOutput(map<FlyType,vector<ViewCentroidPair> >&toSort,int vIndex);
bool ComparePoint3DLessThan(Point3D &a,Point3D &b); // A comparison operator for Point3D
bool CompareViewCentroidPairLessThan(ViewCentroidPair &a,ViewCentroidPair &b); // A comparison operator for View centroid pair

struct ErrorAndFlyType
{
	int flyType;
	double error;

	bool operator<(const ErrorAndFlyType& rhs)
	{
		return (error<rhs.error);
	}
};

void printMap(map<FlyType,vector<ViewCentroidPair> > &currMap);
/**
This is the main class for any project using FlySpy-3 framework. Ideally, main() should create an object of type Experiment, 
and then use it to perform all the tasks supported by FlySpy-3 framework (SilhDetection, Tracking, ... everything).
*/
class Experiment
{
public:
	// Code for kalman below : All functions related to Kalman are no longer in use now : OBSOLETE
	vector<CvKalman*> kalmanArr;
	void initKalman();
	Point3D runKalman(int index,Point3D inp,double vx,double vy,double vz,Point3D observedPt,bool isCorrect);
	void reInitKalman();
	
	long currFrameNumberProcessed; // Current frame # actually processed (will be different than current frame number for inpType == AVI_ONLY)

	View::InputType inpType; // Input Type for all views -> we assume that all view will have same Input Type

	std::vector<View*> V; ///< Each View is created and pushed into this vector. So finally, V.size() == Number of views in  the experiment.
	CTracker Tracker3D; ///< Tracker for tracking flies based on their 3D positions recreated using two or more views.
	
//	ErrorTable **ViewTable; // A zagged array (lower triangular form) ViewTable[0].size() = 0, ViewTable[1].size() = 1, ... ViewTable[i].size() = i-1
//	Order_struct BestOrder;
	
	std::vector<int> lastTrackIndex3D; ///< Last Track Index for 3D tracker (a technicality for CTracker). See also: lastTrackIndex2D in View class.
	long Max_DFS_Runs;
	double Max_Tolerable_Error_Each_Centroid;
	double Max_Tolerable_Error_Corresponder;
//	TrackData data;
	
	Experiment();
	void initialize();
	
	//vector<int> silhDetect();
	std::vector<int>processNewFrame();


	~Experiment();
	void close();
	
	std::vector<Point3D> Track3D();

	void setNFlies(int num_flies);
	void setNViews(int numViews);
	void setInputType(View::InputType iType); // Set input type for experiment. Will copy it to all views which are then created using addView(). Should be called before creating any view

	bool grabFrame(unsigned long frameIndex=-1); // Grabs a single frame from each view
	void addView(); // Should call setNFlies() and setInputType() before calling this function
	
	bool setInputAVIFileName(int ViewID,std::string aviFName);
	bool setInputCentroidFileName(int ViewID,std::string centroidFName);

	bool openView(int ViewID);
	
	bool initSavingToFile(string fname,CvSize size,double fps=60.0);
	void appendToOutputFile();
	void stopSavingToOutputFile();

	ExperimentErrorTable_struct errorTable;
	
	void fillExperimentErrorTable(vector<int> safeViewList);
	map<FlyType,vector<ViewCentroidPair> > correspondOutput;

	double computeTotalErrorForOneFlyType(vector<ViewCentroidPair> inp,bool usingDFS);
	double computeTotalErrorForOutput(map<FlyType,vector<ViewCentroidPair> > inpMap,bool usingDFS);
	vector<int> findStableMarriage(pair<vector<vector<int> >,vector<vector<int> > > &choiceMatrices);

	void correspondViewsUsingStablePolyamory(vector<int> safeViewList,int &totalMisMatches,int &totalMinimumSequences);
	void stablePolyamory(vector<int> orderViews,map<FlyType,vector<ViewCentroidPair> > &toReturn);
	void prepareForPolyamory(vector<int> candidates,map<FlyType,vector<ViewCentroidPair> > &currMap,pair<vector<vector<int> >,vector<vector<int> > > &choiceMatrices) ;

	void correspondViewsUsingDFS(vector<int> safeViewList);
	void runDFS(map<FlyType,vector<ViewCentroidPair> > tempMap,int viewIndexToProcess);
	//double get_cost(vector<int> List1,vector<int> List2, int View1, int View2);
	//void runDFS(Order_struct List,vector<int> &safeViewList,double pruning_threshold);

	//void initViewTable();
	//bool correspondViews(vector<int> &safeViewList,int focal_view=0,int max_dfs_run=-1);

	int bestViewIndex(); // Obsolete in new version
	int Track2D();
	//void renumberBestOrder(int focal_view,std::vector<int> safeViewList);

	void draw3DPoints(vector<Point3D> pt);
	
	void restartExperiment();

	void correspondViewsUsingHungarian(vector<int> safeViewList,int &totalMisMatches,int &totalMinimumSequences);

//	void calculateCostMatrixForHungarian(vector<int> candidates,map<FlyType,vector<ViewCentroidPair> > &currMap,int* costVec);
	void calculateCostMatrixForHungarian(vector<int> candidates,map<FlyType,vector<ViewCentroidPair> > &currMap,Matrix <double> &m);



	//	vector<int> findHungarianCorrespondence(int *inputCostVec);
	vector<int> findHungarianCorrespondence(Matrix<double> inputCostMatrix);

	void hungarianViewCorresponding(vector<int> orderViews,map<FlyType,vector<ViewCentroidPair> > &toReturn);
	void clearCorresponderOutput(); // Strip off dummy points - added for G-correspondence.
	bool containDummyPoint(vector<ViewCentroidPair> vec); // Tell if a vector of ViewCentroid Pair contains a dummy point - added for G-correspondence

	int howManyCentroidsCorresponded; // G-correspondence
};

#endif // __EXPERIMENT_H_INCLUDED__