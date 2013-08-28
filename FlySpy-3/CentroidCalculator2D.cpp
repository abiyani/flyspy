#include "CentroidCalculator2D.h"
#include "utility.h"
#define MAXITERATIONS 50 //for k-means
#define MAXEPS 1.0	
#define MAXTRIES 5

#define MAX_ACCEPTED_DIST_BETWEEN_TWO_MEASUREMENTS 30  //this is the max distance between two consecutive measurements that we won't call it as a false jump 

#define MAX_DISTANCE_OF_PAIR 50


//#define __DSILH 1

#define __TEST_NEW_SILH 0

extern int nFlies;
extern int nViews;

////////////////////////////////////
int findMostTypicalPattern(std::vector<std::vector<int> > v/*,vector<bool> &isMostTypicalType*/,std::string &mostTypicalString)
{
	std::vector<std::string> output;
	if(v.size()==0)
		return 0;

	for(int i=0;i<v.size();i++)
	{
		std::string temp_string;
		std::map<int,char> hash_table;
		char nextChar = '0';
		for(int j=0;j<v[i].size();j++)
		{
			if(hash_table.count(v[i][j])>0) // Already exist in hash table
				temp_string.push_back(hash_table[v[i][j]]);
			else
			{
				hash_table[v[i][j]] = nextChar++;
				temp_string.push_back(hash_table[v[i][j]]);
			}
		}
		output.push_back(temp_string);
		//cout<<output[i]<<endl;
	}
	//Find out the most repeated string
	std::map<std::string,int> freq;
	for(int i=0;i<output.size();i++)
	{
		if(freq.count(output[i])==0)
			freq[output[i]] = 1;
		else
			freq[output[i]] = freq[output[i]]+1;
	}

	int repeat_value=freq[output[0]];
	mostTypicalString = output[0];
	for(int i=1;i<output.size();i++)
	{
		repeat_value = (freq[output[i]]>repeat_value) ? freq[output[i]] : repeat_value;
		mostTypicalString = (freq[output[i]]==repeat_value) ? output[i] : mostTypicalString;
	}

	//for(int i=0;i<v.size();i++)
	//	isMostTypicalType.push_back((output[i]==mostTypicalString)?true:false);
	//
	return repeat_value;
}



double differenceBetweenTwoGroupOfPoints_firstVersion( std::vector<Point2D> pts1,std::vector<Point2D> pts2)
{
	// I use a simple test to realize if there is jump between pts1 and pts2, this can fail but heuristically should work, for solving this problem more accurately  
	//Hungarian is needed here ... too much! I'm not going for that right now ...
	//(1) choose randomly a point from set 1, (2)calculate distance between that point and other points in that set
	//(3) find the closest point from set 2 (4) do step 2 for this point (5) compare the difference, if points scatter similary this difference shouldn't be that large.


	if(pts1.size() != pts2.size())
	{
#if __DSILH
		printf("\n for difference between two groups, size is not the same, so I'm returning zero");
#endif

		return 0.0;    // this is for the case that we don't have same number of measurements in two consecutive frame .. so distance information is not useful
	}


	double dist1 = 0;
	double dist2 = 0;
	int indx1 = (int)(rand()%pts1.size());

	for (int i = 0;i<pts1.size();i++)
		dist1= dist1 + sqrt(euclidDistance_sq(pts1[indx1],pts1[i]));

	//find the closet correspondence in other set...
	double mymin = INT_MAX;	int indx2 = -1;

	for (int i = 0;i<pts2.size();i++)
	{
		double tmp = sqrt(euclidDistance_sq(pts1[indx1],pts2[i]));
		if (tmp < mymin)
		{
			indx2 = i;
			mymin = tmp;
		}
	}

	for (int i = 0;i<pts2.size();i++)
		dist2 = dist2 + sqrt(euclidDistance_sq(pts2[indx2],pts2[i]));
#if __DSILH
	printf("\n dist1 = %.2lf , dist2 = %.2lf",dist1,dist2);
#endif
	return fabs(dist1-dist2);


}

















Point2D calculateCenterofContour (CvSeq *c) //returns center of a contour, should go to utility file ..
{
	Point2D center;
	double totalX=0.0,totalY=0.0;
	double total=0.0;
	for( int k=0; k<c->total;k++)
	{
		CvPoint* p = (CvPoint*)cvGetSeqElem(c,k);
		totalX += p->x;
		totalY += p->y;
		total++;
	}
	center.x = totalX/total;
	center.y = totalY/total;

	return center;
}


void clusterVectorOfPoints(std::vector<points_cluster_pair> &points, int numCluster)  //this function is not being used yet ...
{


	std::vector<contour_dist> distance_list;
	contour_dist temp;

	for (int i = 0 ; i<points.size();i++)
		for (int j = i+1;j<points.size();j++)
		{
			temp.id1 = i;
			temp.id2 = j;
			temp.dist = euclidDistance(points[i].first,points[j].first);
			distance_list.push_back(temp);
		}

		std::sort(distance_list.begin(),distance_list.end());

		if(fabs(distance_list[0].dist-distance_list[1].dist)<10) // in this case there is no hope that you can solve the problem correctly ...
			return;



		Point2D tempPt;
		int nRepeat = 1;
		std::vector<Point2D> repPoints;
		for (int i = 0;i<points.size();i++)
			for (int j = 0;j<nRepeat;j++)
			{
				tempPt.x = points[i].first.x;
				tempPt.y = points[i].first.y;
				repPoints.push_back(tempPt);
			}

			//	printf("\n rep points is");


			CvMat* allThePoints = cvCreateMat((int)repPoints.size(), 1, CV_32FC2);
			CvMat* clusters = cvCreateMat((int)repPoints.size(), 1, CV_32SC1);
#if __DSILH
			printf("\ncluster verctor of points");
			printf("\npoints.size() = %d",repPoints.size());

#endif
			//	CvMat* allThePoints = cvCreateMat(points.size(), 1, CV_32FC2);
			//	CvMat* clusters = cvCreateMat(points.size(), 1, CV_32SC1);
			allThePoints->rows = 0;	
#if __DSILH
			printf("\ncluster verctor of points");
#endif
			//////////////////////////////////////////////////////////////////////////////////////
			//for( int k=0; k<points.size();k++)
			//{
			////	CvPoint tmp;
			//	allThePoints->data.fl[(allThePoints->rows)*2]   = (float)points[k].first.x;
			//	allThePoints->data.fl[(allThePoints->rows)*2+1] = (float)points[k].first.y;
			//	(allThePoints->rows)++;	
			//	//printf("\n k = %d",k);
			//}

			/////////////////////////////////////////////////////////////////////////////////////
			////////////////////SHUFFLE THE SAMPLE////////////////////////////////////////

			for (int i = 0;i<repPoints.size()/2;i++)
			{		
				int k1 =  rand()%(int)repPoints.size();
				int k2 =  rand()%(int)repPoints.size();
				tempPt = repPoints[k1];
				repPoints[k1] = repPoints[k2];
				repPoints[k2] = tempPt;

			}

			/////////////////////////////////////////////



			for( int k=0; k<repPoints.size();k++)
			{
				allThePoints->data.fl[(allThePoints->rows)*2]   = (float)repPoints[k].x;
				allThePoints->data.fl[(allThePoints->rows)*2+1] = (float)repPoints[k].y;
				(allThePoints->rows)++;	
			}

			int tr = 0;
			std::vector <int> prev_clust;
			prev_clust.resize(points.size());
			std::vector<std::vector<int>> allResults;
			std::string a;
			char aa;
			while (tr<10)//(!sameResultsofTwoRun && tr<MAXTRIES) //we repleat kmeans and 10 times, and hope we find a solution that all the centers are inside of the contour 
			{
				clusters->rows = allThePoints->rows;
				cvKMeans2( allThePoints, numCluster, clusters, cvTermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, MAXITERATIONS, MAXEPS ));

				for (int i = 0; i<prev_clust.size();i++)
				{
					prev_clust[i] = clusters->data.i[i*nRepeat];
				}

#if __DSILH
				printf("\n ****clustering result****");
				printf("\nsize of prev_clust is %d",prev_clust.size());
#endif
				allResults.push_back(prev_clust);

#if __DSILH		
				for (int i = 0;i<repPoints.size();i++)
					printf("\n (%.2lf, %.2lf) , C is : %d",repPoints[i].x , repPoints[i].y,clusters->data.i[i]);


				printf("\n****HHHHHHHHHHHHHHHHHHHHHHH****");
				for( int i = 0; i <prev_clust.size(); i++ )
					printf(" %d " ,prev_clust[i]); 
#endif

				findMostTypicalPattern(allResults,a);
				printf(a.c_str());

				tr++;
				printf("\n here tr = %d",tr);
			}

			for (int i =0;i<points.size();i++)
			{
				aa = a[i];
#if __DSILH
				printf("%c , ",aa);
				printf("%d , ",atoi(&aa));
#endif

				points[i].second = atoi(&aa);

			}

#if __DSILH
			for (int i = 0;i<points.size();i++)

				printf("\npoint[%d] is (%.2lf, %.2lf), tr is %d, cluster for this point is %d",i,points[i].first.x,points[i].first.y,tr,points[i].second);

			printf("\n*******");
#endif

}




void clusterVectorOfPoints_distance_version(std::vector<points_cluster_pair> &points, int detectedNc)
{
	assert(false); // NEVER COME HERE , SINCE THIS CODE WILL NOT WORK FOR nFlies == 1 .. IS NOT BEING CALLED NOW!

	// I'm writing this function in the way to be compatible with clustering version for detecting merging ...
	// this is a hard coded function for 2 and 3 merges, more general function is the function above which is not functional yet, some weird output of 
	std::vector<contour_dist> distance_list;
	contour_dist temp;

	for (int i = 0 ; i<points.size();i++)
		for (int j = i+1;j<points.size();j++)
		{
			temp.id1 = i;
			temp.id2 = j;
			temp.dist = euclidDistance(points[i].first,points[j].first);
			distance_list.push_back(temp);
		}

		std::sort(distance_list.begin(),distance_list.end());
#if __DSILH
		printf ("\n distance 0  = %.2lf, distance 1 = %2lf ", distance_list[0].dist,distance_list[1].dist);
#endif	
		if ((detectedNc == nFlies - 1) && (fabs(distance_list[0].dist-distance_list[1].dist)>10)) // we just have one missing measurement 
		{
			points[distance_list[0].id1].second = 0;
			points[distance_list[0].id2].second = 0;

			int label = 1;
			for (int i = 0;i<points.size();i++)
			{
				//if( (i != distance_list[0].id1) && (i != distance_list[0].id2) )
				if(points[i].second != 0)
				{
					points[i].second = label;
					label++;
				}
			}
#if __DSILH
			printf("\ncase 2111:");
			for (int i = 0;i<points.size();i++)
				printf("\npoint[%d] is (%.2lf, %.2lf),cluster for this point is %d",i,points[i].first.x,points[i].first.y,points[i].second);

#endif	

		}

		if (detectedNc == nFlies - 2) // we just have one missing measurement
		{
			bool is311Case = true;				
			int sum=0;
			std::vector<int> lst;
			for(int i=0;i<=2;i++)
			{
				lst.push_back(distance_list[i].id1);
				lst.push_back(distance_list[i].id2);

			}
			for(int i=0;i<lst.size() && is311Case==true;i++)
			{
				int sum=0;
				for(int j=0;j<lst.size();j++)
					sum+=((lst[i]-lst[j])!=0) ? 1 : 0;
				if(sum!=2)
					is311Case = false;
			}

			if (is311Case)
			{

				for (int i = 0; i<2;i++)
				{
					points[distance_list[i].id1].second = 0;
					points[distance_list[i].id2].second = 0;
				}
				// now for rest ...
				int nlabel = 1;
				for (int i = 0;i<points.size();i++)
					if(points[i].second !=0)
					{
						points[i].second = nlabel;
						nlabel++;
					}
#if __DSILH
					printf("\ncase 311:");
					for (int i = 0;i<points.size();i++)
						printf("\npoint[%d] is (%.2lf, %.2lf),cluster for this point is %d",i,points[i].first.x,points[i].first.y,points[i].second);
#endif				

			}

			else //not 3SSS but its 22SSS //do blah blah ...
			{
				points[distance_list[0].id1].second = 0; 
				points[distance_list[0].id2].second = 0;
				points[distance_list[1].id1].second = 1;
				points[distance_list[1].id2].second = 1;
				int lbl = 2;
				for (int i = 0;i<points.size();i++)
					if ((points[i].second != 0) && (points[i].second != 1))
					{
						points[i].second = lbl;
						lbl++;
					}

#if __DSILH
					printf("\ncase 221:");

					for (int i = 0;i<points.size();i++)
						printf("\npoint[%d] is (%.2lf, %.2lf),cluster for this point is %d",i,points[i].first.x,points[i].first.y,points[i].second);
#endif

			}
		}


}


void findCentersOfContour(CvSeq* contour, int numCluster, std::vector <Point2D> &contourkmResults) //returns center of mass of a contour, should go to utility file ..
{
	printf("\n beginning of findCentersofContour");
	contourkmResults.clear();
	CvMat* allThePoints = cvCreateMat(contour->total, 1, CV_32FC2);
	CvMat* clusters = cvCreateMat(contour->total, 1, CV_32SC1);
	allThePoints->rows = 0;	

	for( int k=0; k<contour->total;k++)
	{
		CvPoint* tmp = (CvPoint*)cvGetSeqElem(contour,k);
		allThePoints->data.fl[(allThePoints->rows)*2]   = (float)tmp->x;
		allThePoints->data.fl[(allThePoints->rows)*2+1] = (float)tmp->y;
		(allThePoints->rows)++;	
	}

	printf("\n Find CentersOfContour after loading allpoints");
	double *centroids=new double[2*numCluster]; //2 is because of x and y
	int *numPointsInCentroid=new int[numCluster];
	bool allCentersAreInside = false;
	int tr = 0;
	while (!allCentersAreInside && tr<MAXTRIES) //we repleat kmeans and MAXTRIES times, and hope we find a solution that all the centers are inside of the contour 
	{
		//kmResults.clear();
		//kmResults.resize(numCluster);
		tr++;
		clusters->rows = allThePoints->rows;
		cvKMeans2( allThePoints, numCluster, clusters, cvTermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, MAXITERATIONS, 0.1 ));

		//Calculate the centroids

		for(int i=0; i<numCluster; i++)
		{
			centroids[i*2+0]=centroids[i*2+1]=0;
			numPointsInCentroid[i] = 0;
		}

		for(int i=0; i<clusters->rows; i++)
		{
			centroids[ (clusters->data.i[i])*2 + 0] += allThePoints->data.fl[i*2];
			centroids[ (clusters->data.i[i])*2 + 1] += allThePoints->data.fl[i*2 + 1];
			numPointsInCentroid [ clusters->data.i[i] ] ++;
		}

		for(int i=0; i<numCluster; i++)
		{
			centroids[i*2 + 0] /= numPointsInCentroid[i];
			centroids[i*2 + 1] /= numPointsInCentroid[i];
		}


		CvPoint2D32f tmp; bool tmpbool = true; // I don't need this tmp bool, just to make code more clear ...

		for (int  i=0; i<numCluster; i++)
		{
			tmp.x = (float)centroids[2*i];
			tmp.y = (float)centroids[2*i + 1];
			tmpbool=tmpbool&&(cvPointPolygonTest(contour, tmp,false)>0); //if both points are inside of the contour then stop trying kmeans again
		}
		if(tmpbool)
			allCentersAreInside = true;

	}

	for (int i = 0;i<numCluster;i++) //write results in kmresults vector ...
	{
		Point2D tmp;
		tmp.x = centroids[2*i];
		tmp.y = centroids[2*i + 1];
		contourkmResults.push_back(tmp);
	}


#if __DSILH
	//for (int i = 0;i<contourkmResults.size();i++)
	//	printf("\npoint[%d] is (%.2lf, %.2lf), tr is %d",i,contourkmResults[i].x,contourkmResults[i].y,tr);
#endif
	delete [] centroids;
	delete [] numPointsInCentroid;
	cvReleaseMat(&allThePoints);
	cvReleaseMat(&clusters);



}


//void correctBasedonPreviousFrame(std::vector<frameContoursInfo> currentWindowFramesContours)
//{
//
//
//	std::vector<Point2D> prev_Centroids;
//	// here I'm going to assume I'm using the last frame information and not more than that ... and also I assume size of window is 3 //
//	// this part can be changed easily as soon as we make sure what we are gonna do!
//	for (int i = 0;i<currentWindowFramesContours[0].allContours_data.size())
//		prev_Centroids = 
//
//
//	prev_Centroids = silh_Centroids;
//	//if (contourList.size() > nFlies)
//	//	hasSeenAllFliesinPrevFrame = true; // WE SHOULD GET ATLEAST 5 MEASUREMENT AT THE BEGINNING TO BE SMART!
//
//
//#if __DSILH
//	printf("contourList.size() = %d\n",contourList.size());
//#endif
//	int Nc = (int)contourList.size(); // here I just want to consider contours that have area>minarea
//
//	//	if ((beSmart && hasSeenAllFliesinPrevFrame) && (contourList.size() != nFlies))
//
//
//	if(Nc == nFlies)
//	{
//#if __DSILH
//		printf("\n I'm lucky coz Nc = %d",Nc);
//#endif
//		//hasSeenAllFliesinPrevFrame = true;
//		silh_Centroids.clear();
//		for (int i = 0;i<contourList.size();i++)
//			silh_Centroids.push_back(contourList[i].center);
//		//check distance between previous Centroids and calculated silh_Centroids
//		if (differenceBetweenTwoGroupOfPoints( silh_Centroids,prev_Centroids)>MAX_ACCEPTED_DIST_BETWEEN_TWO_MEASUREMENTS)// && (prev_Centroids.size() == nFlies))
//		{
//			// it might be better to repeat previous measurement ,  ... silh_Centroids = prev_Centroids; .. will try it..
//#if __DSILH
//			printf("\n size of prev_centroids is %d",prev_Centroids.size());
//			printf("\n I have nFlies measurement but the distance between prev centroids and currents is\
//				   big\nI I'm returning you zero measurement");
//
//#endif
////			printf("\n  u bastard!");
//			//silh_Centroids = prev_Centroids;
//			//silh
//			return;
//		}
//	}
//
//
//	if(!beSmart || prev_Centroids.size() <nFlies)
//	{
//#if __DSILH
//		printf("\nbe careful, size of contourList is %d, I'm not being smart or prev_ Centroids.size() is %d",contourList.size(),prev_Centroids.size());
//		printf("\nso I give up! I just return the centroids of current contours");
//#endif
//
//		silh_Centroids.clear();
//		for (int i = 0;i<contourList.size();i++)
//			silh_Centroids.push_back(contourList[i].center);
//
//		return;
//	}
//
//	// from here I try to be smart
//	if (beSmart)
//	{
//		int Nc = (int)contourList.size();
//
//#if __DSILH
//		printf("\n Im trying to be smart ...with Nc = %d", Nc);
//#endif
//
//		if (Nc > nFlies)
//		{
//			//THIS PART NOT BE THAT HARD, KEEP FIRST nFlies CONTOURS THAT ARE BIGGER
//			//CALCULATE THEIR CENTER OF MASS AND URE GOOD TO GO!
//			std::sort(contourList.begin(),contourList.end()); // I have overloaded the sort to sort contourList based on their area from large to smal
//			contourList.erase(contourList.begin()+nFlies,contourList.end()); 		//remove the extra elements from end of list, keep nFlies largest elements 
//			silh_Centroids.clear();
//			for( int i = 0;i<contourList.size();i++) 
//				silh_Centroids.push_back(contourList[i].center);
//
//			return;
//		}
//
//		
//	}
//#if __DSILH
//	printf("\n after all the silh_Centroids.size() = %d", silh_Centroids.size());
//#endif
//}
//
//
//
//

bool inferContoursCentroids (std::vector<Point2D> knownCentroids,std::vector<contour_data> frameContourList,std::vector<Point2D> &resultCentroids)
{
		printf("\n %%%%%%%%%%%%%%%%%%%%%%%%%%I am in inferCntoursCentroids function%%%%%%%%%%%%%%%%%%%%%%%%%%555");
		printf("\nknownCentroids.size() = %d frameContourList.size() = %d", knownCentroids.size(),frameContourList.size());
	int Nc = (int)frameContourList.size();

	//for the case that we have more than nFlies measurement ...
	if(Nc>knownCentroids.size()) //here basically I expect knownCentroids.size() is equal to nFlies and I'm checking the condition that I have more than nFlies measurement
	{
		// put all countour_data to a vector
		std::vector<Point2D> all_Contours_centers;
		for (int i = 0;i<frameContourList.size();i++)
			all_Contours_centers.push_back(frameContourList[i].center);

		std::vector<int> cont_center_ass,known_center_ass; 
		// keeps assignment of all_Contours_centers and knownCentroids resulting from hungCorrespondOf2Sets
		// assigns current measurement with previous meas, and keeps nFlies of them which are closer to the prev. measurements using hungarian

		hungCorrespondOf2Sets(all_Contours_centers,knownCentroids,cont_center_ass,known_center_ass); 

		for(int i = 0;i<cont_center_ass.size();i++)
			if (cont_center_ass[i] != -1) // assignment -1 means no assignment 
				resultCentroids.push_back(all_Contours_centers[i]);

		printf("\nCaseNc>knownCentroids, resultCnetorids. size() = %d", resultCentroids.size());
		return true;
	}

#if __OLDVERSIONOFINFERRING


#if __DSILH
	//printf("\n I'm in Nc < nFlies t\n");
	//printf("\nbe smart using the past centroids ...");
	printf("\nnFlies - Nc = %d",nFlies - Nc);
#endif


	std::vector<Point2D> prev_Centroids;
	prev_Centroids = knownCentroids;

	std::vector<points_cluster_pair> prev_Points_Cluster;
	prev_Points_Cluster.resize(prev_Centroids.size()); // prev_Centroids.size() should always be equal to nFlies ...
	for (int i = 0 ; i < prev_Points_Cluster.size(); i++)
	{
		prev_Points_Cluster[i].first = prev_Centroids[i];
		prev_Points_Cluster[i].second = -1; // has not been clustered yet;
	}
#if __DSILH
	for (int i = 0;i<prev_Points_Cluster.size();i++)
		printf("\n p%d = (%.2lf, %.2lf)",i,prev_Points_Cluster[i].first.x,prev_Points_Cluster[i].first.y);
#endif
	clusterVectorOfPoints(prev_Points_Cluster,Nc); // this function returns prev centroids with their cluster indices (we want to find those that are merged)


	//clusterVectorOfPoints_distance_version(prev_Points_Cluster,Nc); // this is a temporary substitution 


	//for (int k = 0;k<prev_Points_Cluster.size();k++)
	//	printf("(%.2lf, %.2lf) , id = %d ----",prev_Points_Cluster[k].first.x,prev_Points_Cluster[k].first.y,prev_Points_Cluster[k].second);

	std::vector <std::pair <Point2D,int>> prevPointsClusterCenters;
#if __DSILH
	printf("\nprev_Points_Cluster.size() = %d",prev_Points_Cluster.size());
#endif
	int count = 0;Point2D temp;
	for (int indx = 0;indx<Nc;indx++)
	{
		printf("\n indx = %d",indx);
		temp.x = 0;temp.y = 0;				
		for (int i = 0; i < prev_Points_Cluster.size(); i++)
		{
			if (prev_Points_Cluster[i].second == indx)
			{
				temp.x += prev_Points_Cluster[i].first.x;
				temp.y += prev_Points_Cluster[i].first.y;
				count++;
			}
		}
#if __DSILH
		printf("\n count = %d",count);
#endif
		if(count != 0)
		{
			temp.x/=count;
			temp.y/=count;
			prevPointsClusterCenters.push_back(std::make_pair(temp,count));
		}
		count = 0;
	}
#if __DSILH
	printf("\nprevPointsClusterCenters.size() = %d",prevPointsClusterCenters.size());
#endif

	// now we should find the contours in new measurements that have the most closest centers to these temp points;
	// at this point size of prevPoinstclusterscenter and contourList should be same;


	printf("\n here are the contourList Centers...");
	for (int i = 0 ; i<frameContourList.size(); i++)
		printf("\n contourList[%d].center = (%.2lf, %.2lf)", i,frameContourList[i].center.x,frameContourList[i].center.y );

	printf("\n here are the prevPointsClusterCenters...");
	for (int i = 0 ; i<prevPointsClusterCenters.size(); i++)
		printf( "\n prevPointsClusterCenters[%d] = (%.2lf, %.2lf)",i,prevPointsClusterCenters[i].first.x,prevPointsClusterCenters[i].first.y);

	std::vector<int>cindx;
	int tmp = 0;

	printf("\nprevPointsClusterCenters.size() = %d",prevPointsClusterCenters.size());
	for (int i = 0;i<prevPointsClusterCenters.size();i++)
	{
		double minDist = euclidDistance(prevPointsClusterCenters[i].first,frameContourList[0].center);
		tmp = 0;
		for (int j = 1;j<frameContourList.size();j++)
		{
			if (euclidDistance(prevPointsClusterCenters[i].first,frameContourList[j].center)<minDist)
			{
				tmp = j;
				minDist = euclidDistance(prevPointsClusterCenters[i].first,frameContourList[j].center);
			}			
		}
		cindx.push_back(tmp);
	}

	printf("\ncindx.size() = %d",cindx.size());

	for (int i = 0;i<cindx.size();i++)
#if __DSILH
		printf("\ncindx[%d] = %d",i,cindx[i]);
#endif
	for (int i = 0;i<cindx.size();i++)
		for (int j = i+1;j<cindx.size();j++)
			if (cindx[i] ==cindx[j])
#if __DSILH
				printf("\ncindx has some problems, not solvable check this situation ....! Idealy I shouldn't see this msg!");
#endif
	// here we cound the contours, now we need to do clustering on merged contours and find centroid n populate silh_Centroid

	for ( int i = 0;i<prevPointsClusterCenters.size();i++)
	{
		std::vector <Point2D> kmTemp;
		if (prevPointsClusterCenters[i].second>1) //two or more points have been assigned to the same cluster
		{
#if __DSILH
			printf("\ncontourList.size() = %d",frameContourList.size());
			printf("\ncindx[i] = %d",cindx[i]);
#endif
			findCentersOfContour(frameContourList[cindx[i]].contour_seq, prevPointsClusterCenters[i].second , kmTemp);
#if __DSILH
			printf("\ncontourList[%d].center = (%.2lf, %.2lf)",cindx[i], frameContourList[cindx[i]].center.x,frameContourList[cindx[i]].center.y);
#endif
			for(int j = 0;j<kmTemp.size();j++)
				resultCentroids.push_back(kmTemp[j]);
		}
		if ( prevPointsClusterCenters[i].second == 1)
			resultCentroids.push_back(frameContourList[cindx[i]].center);
	}

	//check distance between previous Centroids and calculated silh_Centroids
	if (differenceBetweenTwoGroupOfPoints( resultCentroids,prev_Centroids)>MAX_ACCEPTED_DIST_BETWEEN_TWO_MEASUREMENTS)
	{
		// it might be better to repeat previous measurement ,  ... silh_Centroids = prev_Centroids; .. will try it..
#if __DSILH
		printf("\n I realized I can't solve the case since the distance between prev centroids and currents is\
				 huge\nI just return the centroid of contours...");
#endif
		//				silh_Centroids = prev_Centroids;  // here its proly better to return the centroids of current contours ...
		// I'll return nothing!
		// silh_Centroids.clear();

	}



	return true;

#endif
	
	else
	{
		// put all countour_data to a vector
		std::vector<Point2D> all_Contours_centers;
		for (int i = 0;i<frameContourList.size();i++)
			all_Contours_centers.push_back(frameContourList[i].center);

		std::vector<int> cont_center_ass,known_center_ass; 
		// keeps assignment of all_Contours_centers and knownCentroids resulting from hungCorrespondOf2Sets
		// assigns current measurement with previous meas, and keeps nFlies of them which are closer to the prev. measurements using hungarian
		
		hungCorrespondOf2Sets(knownCentroids,all_Contours_centers,known_center_ass,cont_center_ass); 
		int indx_1=-1;
		printf("\n known_center_ass.size() = %d",known_center_ass.size());

		for (int i = 0;i<known_center_ass.size();i++)
		{
			if (known_center_ass[i] == -1)
			{
				indx_1 = i;
				break;
			}

		}
		int mergedContindx;
		mergedContindx = closestPoint(all_Contours_centers,knownCentroids[indx_1]); //trying to idenitfy one of the points that is merging ..
		if (mergedContindx == -1) // in this case all_Contours_centers.size() == 0, so no contour has been detected, can't do any infering .. 
			return false;
		for (int i = 0;i<all_Contours_centers.size();i++) 
		{
			if (i != mergedContindx)
				resultCentroids.push_back(all_Contours_centers[i]);
		}

		std::vector <Point2D> kmTemp;

		//printf("\nmegedInded = %d",mergedContindx);
		findCentersOfContour(frameContourList[mergedContindx].contour_seq, 2 , kmTemp); // this is hard coded for one merge ...can be changed easily...
		for (int i = 0;i<kmTemp.size();i++)
			resultCentroids.push_back(kmTemp[i]);


		// now check it the resulted centroids are good enough! what does it mean? it means if inferred points are close to previous points ..

		double totDist, maxDist;
		differenceBetweenTwoGroupOfPoints(knownCentroids,resultCentroids,totDist,maxDist);
		//printf("\n ---knownCentroids---\n");
		//for (int i = 0;i<knownCentroids.size();i++)
		//	printf("(%.2lf , %.2lf), ", knownCentroids[i].x,knownCentroids[i].y);

		//printf("\n ---resultCentroids---\n");
		//for (int i = 0;i<resultCentroids.size();i++)
		//	printf("(%.2lf , %.2lf), ",resultCentroids[i].x, resultCentroids[i].y );


		//printf("\n totdist = %.2lf , maxdist = %.2lf\n", totDist,maxDist);

		if (maxDist>MAX_DISTANCE_OF_PAIR)
			resultCentroids = all_Contours_centers; // if the distance is too much, just retun the centroids ...

		return true;
	}

}


CCentroidCalculator2D::CCentroidCalculator2D(void):isInitialized(false)
{
	//set number of previous and future frames that Centroid Calculator wants to use for inference of 2D centroids
	nPrevFrames = 1;  //CAUTION: minimum value for this variable is 1, since we are using atleast ONE previous frame to infer centroids for the current frame 
	nNextFrames = 1;
}

CCentroidCalculator2D::~CCentroidCalculator2D(void)
{
}

void CCentroidCalculator2D::initialize(int npre,int npst, IplImage* firstFrame)
{
	nPrevFrames = npre;
	nNextFrames = npst;
	//windowFramesContours.clear(); //clear all informations in window ...
	endOfVideo = false; 
	silh.initialize(firstFrame);
	//hasEverSeenExactNFlies =false;


}

void CCentroidCalculator2D::finish(void)
{

	for (int i = 0;i<nPrevFrames+nNextFrames+1;i++)
		cvReleaseMemStorage(&windowFramesContours[i].memStore);
	//don't forget to release images of contour_info (for keeping color ...)

}
std::vector<Point2D> returnRawCentorids (frameContoursInfo currentFrameInfo)
{
	std::vector<Point2D> tmp;
	for (int i = 0;i<currentFrameInfo.allContours_data.size();i++)
		tmp.push_back(currentFrameInfo.allContours_data[i].center);
	return tmp;

}
bool CCentroidCalculator2D::calculateCentroids()
{
	if(windowFramesContours.size()<nPrevFrames + nNextFrames + 1)
	{
		windowCentroids = windowContoursOriginalCentroids; // this is for the beginning of the video, there are not enough frames to do anything useful, so I just return Rawcentriods which are windowContoursOriginalCentroids
		return false;

	}
	//if(!hasEverSeenExactNFlies && windowContoursOriginalCentroids.size() == nFlies)
	//	hasEverSeenExactNFlies = true;

	else
	{
			//// this whole if then else structure now can be changed , not need to be smart ...!
		if (windowFramesContours[nPrevFrames+nNextFrames].allContours_data.size() ==nFlies) //if I'm getting nFlies measurement in current frame, proly I just need to check if there is a jump
		{
			//		printf("\n I am in ---if (windowFramesContours[nPrevFrames].allContours_data.size() ==nFlies)---");

			//here more things can be added ...for example check if in M previous frames (M <=npre:numb of prev frame that we keep) we had nFlies measurement .. if so just return it, if not
			//some weird situation has happened ...// good thing is that we can even check the size of raw centriods ... wherever we have weird number of measurements we can try to look at that frame more carefully

			//just check for jumps, if windowFramesContours[nPrevFrames-1].allContours_data.size() == nFlies
			// and if it's not then just return the centroids of the contours ...
			// and also we have future information, we will check with future too ...interesting!


			//	if (differenceBetweenTwoGroupOfPoints( windowCentroids[nPrevFrames-1],prev_Centroids)>MAX_ACCEPTED_DIST_BETWEEN_TWO_MEASUREMENTS)

			windowCentroids.push_back(windowContoursOriginalCentroids[nPrevFrames+nNextFrames]); // windowCentroid had first 2 frames, it needs one more to have size 3, but should get it from the last original frames ...
			//				printf("\n windowsCentroids.size = %d", windowCentroids.size());
			if(windowCentroids.size()>nPrevFrames + nNextFrames +1)
				windowCentroids.erase(windowCentroids.begin()); //to keep the size of windowCentroids constant

						//printf("\n I am in ---if (windowFramesContours[nPrevFrames].allContours_data.size() ==nFlies)---");

			return true;

		}



		if (windowFramesContours[nPrevFrames+nNextFrames].allContours_data.size() ==nFlies-1 && nFlies!=1) //if I'm getting nFlies measurement in current frame, proly I just need to check if there is a jump
		{
			// hardcoded for recent experiments ...

			windowCentroids.push_back(windowContoursOriginalCentroids[nPrevFrames+nNextFrames]);
			//				printf("\n windowsCentroids.size = %d", windowCentroids.size());
			if(windowCentroids.size()>nPrevFrames + nNextFrames +1)
				windowCentroids.erase(windowCentroids.begin()); //to keep the size of windowCentroids constant

				//printf("\n I am in ---if (windowFramesContours[nPrevFrames].allContours_data.size() ==nFlies-1)---");

			return true;

		}

		if (windowFramesContours[nPrevFrames+nNextFrames].allContours_data.size() < nFlies)
		{
			////RECENTLY CEOMMENTED
			windowCentroids.push_back(windowContoursOriginalCentroids[nPrevFrames+nNextFrames]); //just return centroids ...
		if(windowCentroids.size()>nPrevFrames + nNextFrames +1)
			windowCentroids.erase(windowCentroids.begin()); //to keep the size of windowCentroids constant
			return true;
		}
		////END OF RECENTLY COMMENTED


/////Recently added ...
		//if (windowFramesContours[nPrevFrames].allContours_data.size() < nFlies)
		//{

		//	windowCentroids = windowContoursOriginalCentroids;
		//	return false;
		//}
//////end of recently added ...

/////recently commented

		//if (windowFramesContours[nPrevFrames].allContours_data.size() < nFlies)
		//{

		//	//printf("\n*******windowFramesContours[nPrevFrames].allContours_data.size() < nFlies*********");
		//	std::vector<Point2D> tmpCentroids;
		//	//				if (windowFramesContours.size() > nPrevFrames + nNextFrames +1) // if window size is >3 you can have future frame, otherwise u can't do nothing , then use future information
		//	if (windowFramesContours.size() >3) // if window size is >3 you can have future frame, otherwise u can't do nothing , then use future information

		//	{

		//		if (windowFramesContours[nPrevFrames+1].allContours_data.size() == nFlies) //correction using future ...
		//		{
		//			//					printf("\n(windowFramesContours[nPrevFrames+1].allContours_data.size() == nFlies)");
		//			//the jump condition should also be checked with this and windowFramesContours[nPrevFrames-1] ...
		//			// 

		//			// if next prev_frame and next_frame are okey (nFlies measurement with reasonable distance, we will ignore this frame ..
		//			// HERE WE ARE JUST PASSING WHATEVER WE GET FROM SILH'S , PROLY IT WON'T BE BAD SINCE IN NEXT FRAME WE ARE GONNA GET MEASUREMENTS ..
		//			//BETTER IDEA WOULD BE TO RETURN MEAN OF PREV AND POST FRAME IF DISTANCE BETWEEN THEM IS OKEY...

		//			//					windowCentroids.push_back(windowContoursOriginalCentroids[nPrevFrames]); // WHY NOT RETURNING NEW MEASUREMENTS ...?
		//			windowCentroids.push_back(windowContoursOriginalCentroids[nPrevFrames+1]); // WHY NOT RETURNING NEW MEASUREMENTS ...?

		//			if(windowCentroids.size()>nPrevFrames + nNextFrames +1)
		//				windowCentroids.erase(windowCentroids.begin()); //to keep the size of windowCentroids constant

		//			return true;				
		//		}
		//	}



		//	if (windowCentroids[nPrevFrames-1].size() == nFlies ) // correction using past ...
		//		//WE HAVE TO MAKE SURE TWO FLIES ARE MERGING! WE WILL CHECK PREVIOUS CONTOURES... INSTEAD OF PREVIOUS MEASUREMENTS
		//		//BUT THEN THE PROBLEM WOULD BE WHEN THEY ARE MERGED AND CONSTANTLY MERGED WE WON'T GET MEASUREMENTS
		//		// IF WINDOWCENTROIDS AND ORIGINALWINDOWCENTRODS [nPrevFrames-1] BOTH HAVE NFLIES SIZE FOR SURE WE HAVE 
		//	{
		//		//printf("\n(windowCentroids[nPrevFrames-1].size() == nFlies )");

		//		inferContoursCentroids(windowCentroids[nPrevFrames-1],windowFramesContours[nPrevFrames].allContours_data,tmpCentroids);
		//		windowCentroids.push_back(tmpCentroids);
		//		if(windowCentroids.size()>nPrevFrames + nNextFrames +1)
		//			windowCentroids.erase(windowCentroids.begin()); //to keep the size of windowCentroids constant
		//		return true;
		//	}
		//	////RECENTLY CEOMMENTED
		//						windowCentroids.push_back(windowContoursOriginalCentroids[nPrevFrames+1]); //just return centroids ...
		//						if(windowCentroids.size()>nPrevFrames + nNextFrames +1)
		//							windowCentroids.erase(windowCentroids.begin()); //to keep the size of windowCentroids constant
		//	////END OF RECENTLY COMMENTED

		//	//printf("\n*****************windowCentroids.size() = %d*******",windowCentroids.size());
		//	return true;
		//}
////end of recently commented

		if (windowFramesContours[nPrevFrames+nNextFrames].allContours_data.size() > nFlies && windowCentroids[nPrevFrames].size()==nFlies)
		{
			std::vector<Point2D> tmpCentroids;
			
			//				printf("\n windowCentroids[nPrevFrames-1].size() = %d",windowCentroids[nPrevFrames-1].size());
			inferContoursCentroids(windowCentroids[nPrevFrames],windowFramesContours[nPrevFrames+nNextFrames].allContours_data,tmpCentroids);
			//				printf("\n tmpCentroids.size() = %d", tmpCentroids.size());
			windowCentroids.push_back(tmpCentroids);
			if(windowCentroids.size()>nPrevFrames + nNextFrames +1)
				windowCentroids.erase(windowCentroids.begin()); //to keep the size of windowCentroids constant

			//printf("\n tmpCentroids.size() = %d", tmpCentroids.size());
			//printf("\n more than nFlies ...");

			return true;
		}

		

	}
	
	return true;
}

bool CCentroidCalculator2D::getNewFrame(IplImage* thisFrame) //this function gets new frame and calculates the contours of the change mask and push it back to the vector ..
{
	frameContoursInfo  currentFrameContours;
	currentFrameContours = silh.findChangeMaskContours(thisFrame);
	IplImage *temp_cm = cvCloneImage(silh.denoisedChangeMask);
	changeMaskWindow.push_back(temp_cm); // Push back change mask
	if(changeMaskWindow.size()>nPrevFrames + nNextFrames+1) // keep the size of changemask vector correct ...
	{
		cvReleaseImage(&changeMaskWindow[0]);
		changeMaskWindow.erase(changeMaskWindow.begin());
	}

	windowFramesContours.push_back(currentFrameContours);
	//std::vector<Point2D> temp;
	//temp = returnRawCentorids(currentFrameContours);
	//printf("enter a rand value : ");
	//int ra;
	//scanf("%d",&ra);

	//srand(ra);
	//for (int i = 0;i<temp.size();i++)
	//{
	//	temp[i].x +=(double)(rand()%1000)/250.0;
	//	temp[i].y +=(double)(rand()%1000)/250.0;
	//	printf("\nrand value = %.2lf",(double)(rand()%1000)/250.0);
	//}
	if (windowFramesContours.size()>nPrevFrames + nNextFrames + 1)
	{
		cvReleaseMemStorage(&windowFramesContours[0].memStore);
		windowFramesContours.erase(windowFramesContours.begin());
		
	}

	windowContoursOriginalCentroids.push_back (returnRawCentorids(currentFrameContours));

	if (windowContoursOriginalCentroids.size()>nPrevFrames + nNextFrames + 1)
		windowContoursOriginalCentroids.erase(windowContoursOriginalCentroids.begin());
	


	if (windowFramesContours.size()>nPrevFrames + nNextFrames + 1)
	{
		printf ("\n this should not happen, size of windows should not be greater than nPrevFrames+nPostFrame");
		return false;
	}
	

	//printf("\nwindowContoursOriginalCentroids size = %d",windowContoursOriginalCentroids.size());

	return true;


}

void CCentroidCalculator2D::restart()
{

	for (int i = 0;i<nPrevFrames + nNextFrames;i++)
		if (&windowFramesContours[i].memStore != NULL)
			cvReleaseMemStorage(&windowFramesContours[0].memStore);
	windowFramesContours.clear();
	windowCentroids.clear();
	windowContoursOriginalCentroids.clear();
	silh.restartSilhDetect();

}