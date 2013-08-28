#include "Tracker.h"

extern int nFlies;
extern int nViews;

double calculateDir(Point2D p1, Point2D p2)
{
	double m;
	m = (180*atan2((p2.x - p1.x) ,(p2.y - p1.y)))/PI;
	return m;
}


CTracker::CTracker(void):mixingWeight(0),\
targetNum(nFlies),\
readyToCorrespond(false),\
minDisThreshold(20)
{
	if(nFlies<=0) // This means nFlies has not been set up yet
		targetNum = 3; // Default value for nFlies (should be resized later using setTargetNum later)
	//assignmentVec.resize(targetNum); // put the size of the assignment vector equal to number of the flies
	//for (int i=0;i<targetNum;i++)
	//	assignmentVec[i] = i; // this just indicates that the ID's are unknown at the very beginning
}

CTracker::~CTracker(void)
{
}

void CTracker::setTargetNum(int a)
{
	targetNum = a;

	//assignmentVec.resize(targetNum); // put the size of the assignment vector equal to number of the flies
	//for (int i=0;i<targetNum;i++)
	//	assignmentVec[i] = i; // this just indicates that the ID's are unknown at the very beginning

}

double CTracker::getMixingWeight()
{
	return mixingWeight;
}

void CTracker::setMixingWeight(double mw)
{
	mixingWeight = mw;
}

//int** CTracker::calculateCostMatrix(void)
//{
//	if(points3D.size()==0)
//	{
//		vector <Point2D> pt_2;
//		vector <Point2D> pt_1;
//		vector <Point2D> pt;
//		pt_2.resize(targetNum);
//		pt_1.resize(targetNum);
//		pt.resize(targetNum);
//		pt_2 = points2D[0];
//		pt_1 = points2D[1];
//		pt = points2D[2];
//
//		vector <int> origDir;
//		vector <int> newDir;
//		origDir.resize(targetNum);
//		newDir.resize(targetNum);
//		origDir.empty();
//		newDir.empty();
//
//		for(int i = 0;i<targetNum;i++)
//		{
//			origDir[i] = (int)calculateDir(pt_2[i],pt_1[i]);
//			newDir[i] = (int)calculateDir(pt_1[i],pt[i]);
//		}
//
//		int *cost_vec = new int [targetNum*targetNum];
//
//		for (int i = 0;i <targetNum;i++)
//			for (int j = 0;j < targetNum; j++)
//				cost_vec[targetNum*i + j] = (int) ( ( (1.0-mixingWeight)*euclidDistance(pt[i],pt_1[j]) + mixingWeight*abs(origDir[i]-newDir[j]) ) * 10000.00);
//
//
//		int** cost_mtrx = array_to_matrix(cost_vec,targetNum,targetNum);
//		delete cost_vec; //Memory leak due to lack of this line
//
//		return cost_mtrx;
//	}
//	else // for 3D points
//	{
//		assert(points2D.size()==0);
//
//		vector <Point3D> pt_2;
//		vector <Point3D> pt_1;
//		vector <Point3D> pt;
//		pt_2.resize(targetNum);
//		pt_1.resize(targetNum);
//		pt.resize(targetNum);
//		pt_2 = points3D[0];
//		pt_1 = points3D[1];
//		pt = points3D[2];
//
//		//vector <int> origDir;
//		//vector <int> newDir;
//		//origDir.resize(targetNum);
//		//newDir.resize(targetNum);
//		//origDir.empty();
//		//newDir.empty();
//
//		//for(int i = 0;i<targetNum;i++)
//		//{
//		//	origDir[i] = (int)calculateDir(pt_2[i],pt_1[i]);
//		//	newDir[i] = (int)calculateDir(pt_1[i],pt[i]);
//		//}
//
//		int *cost_vec = new int [targetNum*targetNum];
//
//		for (int i = 0;i <targetNum;i++)
//			for (int j = 0;j < targetNum; j++)
//				cost_vec[targetNum*i + j] = (int)(((1-mixingWeight)*euclidDistance(pt[i],pt_1[j])) * 10000.00);// + (mixingWeight)*abs(origDir[i]-newDir[j]));
//
//		int** cost_mtrx = array_to_matrix(cost_vec,targetNum,targetNum);
//
//		delete cost_vec; // Anurag: Memory leak due to lack of this line
//
//		return cost_mtrx;
//	}
//	return NULL; // Cannot be reached
//}

void CTracker::getNewMeasurement(vector<Point2D> measurements)
{
	if (points2D.size()<3)
	{
		/*for(int i=0;i<targetNum;i++)
		assignmentVec[i] = -1;*/
		//	printf("\nPoint2d. size() = %d",points2D.size());
		points2D.push_back(measurements);
		readyToCorrespond = false;
	}
	else
	{
		readyToCorrespond = true;
		points2D.push_back(measurements);
		points2D.erase(points2D.begin());
	};

	if(points2D.size()>3)
		printf("\n SOME THING IS WRONG IN GETNEWMEASUREMENT FUNCTION!"); // proly an assert would work better
}


void CTracker::getNewMeasurement(vector<Point3D> measurements)
{
	//printf("\n*** in get measurement ****\n");
	//for(int i=0;i<measurements.size();i++)
	//	printf("(%.2lf,%.2lf,%.2lf)\n",measurements[i].x,measurements[i].y,measurements[i].z);

	if (points3D.size()<3)
	{
		readyToCorrespond = false;
		points3D.push_back(measurements);
	}
	else
	{
		readyToCorrespond = true;
		points3D.push_back(measurements);
		points3D.erase(points3D.begin());
	};

	if(points3D.size()>3)
		printf("\n SOME THING IS WRONG IN GETNEWMEASUREMENT FUNCTION!"); // proly an assert would work better
}

void CTracker::findEasyTargets(vector<Point3D> targetPos, vector<bool> &isAnEasyTarget)
{
	// this function checks the minimum distance between each target and others and declares it as an easy target if the distance is big enough
	for (int i = 0;i<targetPos.size();i++)
	{
		double minDis = INT_MAX;
		for (int j = 0;j<targetPos.size();j++)
		{
			if(i != j) //to not check a point against itself
				if (euclidDistance(targetPos[i],targetPos[j]) < minDis)
					minDis = euclidDistance(targetPos[i],targetPos[j]); 
		}
		isAnEasyTarget.push_back((bool)(minDis > minDisThreshold));
	}
}

void CTracker::findEasyTargets(vector<Point2D> targetPos, vector<bool> &isAnEasyTarget)
{
	// this function checks the minimum distance between each target and others and declares it as an easy target if the distance is big enough
	for (int i = 0;i<targetPos.size();i++)
	{
		double minDis = INT_MAX;
		for (int j = 0;j<targetPos.size();j++)
		{
			if(i != j) //to not check a point against itself
				if (euclidDistance(targetPos[i],targetPos[j]) < minDis)
					minDis = euclidDistance(targetPos[i],targetPos[j]); 
		}
		isAnEasyTarget.push_back((bool)(minDis > minDisThreshold));
	}
}

//void CTracker::addNewTarget()
//{ 
//	printf("\n addNewTarget");
//	// find which one is not there
//	int notIncluded = -1;
//	for (int j = 0;j<targetNum;j++)
//	{
//		if (find_in_vector(assignmentVec,j) != -1)
//		{
//			notIncluded = j;
//			break;
//		}
//	}
//	assignmentVec.push_back(notIncluded);
//}

void CTracker::Track(vector<int> currOrder)
{ 	
	assignmentVec.clear();
	if (!readyToCorrespond)
	{
		std::cout<<"not ready to correspond yet, needs at least 3 measurements \n";
		return ;
	}
	else
	{

		//vector <Point3D> pt_2;
		vector <Point3D> pt_1;
		vector <Point3D> pt;
		//pt_2.resize(targetNum);
		//pt_1.resize(targetNum);
		//pt.resize(targetNum);
		if (points3D.size()<3)
		{
			printf("\nshould not come here at all. CTracker::Track function");
			getchar();
			return;
		}

		//pt_2 = points3D[0];
		pt_1 = points3D[1];
		pt = points3D[2];

		//printf("\npt_1 = ");
		//for (int i = 0;i<pt_1.size();i++)
		//	printf("\n(%.2lf, %.2lf,%.2lf)",pt_1[i].x,pt_1[i].y,pt_1[i].z);

		//printf("\npt = ");
		//for (int i = 0;i<pt.size();i++)
		//	printf("\n(%.2lf, %.2lf,%.2lf)",pt[i].x,pt[i].y,pt[i].z);


		std::vector<int> assPre, assCurr;//,assignmentVec2;

		hungCorrespondOf2Sets(pt_1,pt,assPre,assCurr);

		assignmentVec.clear(); // to make sure the size of the returned assignmentVec is same as the number of points...
		if (pt_1.size() == pt.size()) // the case of same number of measurements and flies ..
		{
			assignmentVec.clear(); // to make sure the size of the returned assignmentVec is same as the number of points...

			//printf("\nsize of current order is %d", currOrder.size());
			for (int i = 0;i<pt.size();i++)
				assignmentVec.push_back(currOrder[assCurr[i]]);

			return;
		}
	
		// here we are considering just cases that number of measurements is nFlies -1 or nFlies ...
		if(pt_1.size() - pt.size() ==1)
		{
			printf("\n%%%%%%%%%MERGEEEEE%%%%%%%%%%%%!"); 

			for (int i = 0;i<pt_1.size();i++) // can be wrong ...
				if (assCurr[i] == -1)
					continue;
				else
					assignmentVec.push_back(currOrder[assCurr[i]]);

			return;

		}
	
		if(pt.size() - pt_1.size() ==1)
		{
			printf("\n^^^^^^^^^^^^^SPLITTTT^^^^^^^^^^^!"); 
			int missedTarget=-1;// = find_in_vector(currOrder,
			for (int j = 0;j<targetNum;j++)
			{
				if (find_in_vector(currOrder,j) == -1)
				{
					missedTarget = j;
					break;
				}
			}
			// Can be buggy still
			if(missedTarget == -1) // This is a fix for very beginning frames (although this line is unaware of this fact - so it can be fix any thing in the world!).
				return; // Take care of the situtaion when we have 5,4,5 measurments as first 3 measurments (so it think it is a split, but currOrder doesn't have any -1).


			for (int i = 0;i<assCurr.size();i++) // can be wrong ...
			{
				if (assCurr[i] == -1)
					assignmentVec.push_back(missedTarget);
				else
					assignmentVec.push_back(currOrder[assCurr[i]]);
			}
			//addNewTarget(); 
			return;

		}

	}


}
