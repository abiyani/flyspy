#pragma once
#include "Utility.h"
//#include "hungarian.h"

#include "munkres.h"
Point2D Utility::getInvalidPoint() // Returns a "invalid" point - a "sentinel" point2d to signify that it is not actually a valid point2D, but rather a placeholder
{
	Point2D p;
	p.x = INFINITY_ALEPH0;
	p.y = INFINITY_ALEPH0;
	return p;
}
int Utility::factorial(int x)
{
	int fact=1;
	for(int i=2;i<=x;i++)
		fact*=i;
	return fact;
}


bool Utility::isInvalidPoint(Point2D p) // Checks wether a given point is "invalid" or not 
{
	if( (fabs(p.x)+fabs(p.y))>=(INFINITY_ALEPH0-1.0))
		return true;
	return false;

	/*if( (p.x>=INFINITY_POSITIVE_double-1.0) || (p.x<=INFINITY_NEGATIVE_double+1.0) || (p.y>=INFINITY_POSITIVE_double-1.0) || (p.y<=INFINITY_NEGATIVE_double+1.0))
		return true;
	return false;*/
}



bool Utility::isInvalidPoint(Point3D p)
{
	if( (fabs(p.x)+fabs(p.y)+fabs(p.z))>=(INFINITY_ALEPH0-1.0))
		return true;
	return false;

	/*if( (p.x>=INFINITY_POSITIVE_double-1.0) || (p.x<=INFINITY_NEGATIVE_double+1.0) || (p.y>=INFINITY_POSITIVE_double-1.0) || (p.y<=INFINITY_NEGATIVE_double+1.0) \
		|| (p.z>=INFINITY_POSITIVE_double-1.0) || (p.z<=INFINITY_NEGATIVE_double+1.0) )
		return true;
	return false;*/
}
std::vector<Point3D> Utility::readFlyTrackFromSingleFrame(std::string line,int commasToLeave)
{
	std::vector<Point3D> toReturn;
	std::vector<std::string> tokens;
	Tokenize(line,tokens,",");
	for(int i = commasToLeave;i<=tokens.size()-3;i+=3) // Leave values before specified number of commas
	{
		Point3D temp;
		temp.x = atof(tokens[i].c_str());
		temp.y = atof(tokens[i+1].c_str());
		temp.z = atof(tokens[i+2].c_str());
		toReturn.push_back(temp);
	}
	return toReturn;
}

/** A string Tokenizer. Code taken from: http://oopweb.com/CPP/Documents/CPPHOWTO/Volume/C++Programming-HOWTO-7.html (trivial to write)
strtok() is not THREAD-SAFE: learned it hard way, so should use some Tokenizer which does not have this kind of problem!
*/
void Utility::Tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ")
{
	// Skip delimiters at beginning.
	std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

/** Checks wether or not two double values are equal to each other (since simply checking a==b can be misleading for floating point values)
* @param a : First double value
* @param b : Second double value
* @return true if a==b (with maximum error == EPSILON), false otherwise
*/
bool Utility::areRealsEqual(double a,double b,double precision)
{
	return (fabs(a-b)<precision);
}

/** Checks wether or not two float values are equal to each other (since simply checking a==b can be misleading for floating point values)
* @param a : First float value
* @param b : Second float value
* @return true if a==b (with maximum error == EPSILON), false otherwise
*/
bool Utility::areRealsEqual(float a,float b,float precision)
{
	return ( fabs(a-b)<float(precision) );
}

/** Returns distance (computed using euclidean norm) between 2D points "a" and "b". Overloaded for 3D points as well.
\overload
* @param a : Point a
* @param b : Point b
* @return: euclidean length of line joining points a and b
*/
double Utility::euclidDistance(Point2D a,Point2D b)
{
	return (sqrt( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) ) );
}

/** Returns distance (computed using euclidean norm) between 3D points "a" and "b".
\overload
* @param a : Point a
* @param b : Point b
* @return: euclidean length of line joining points a and b
*/
double Utility::euclidDistance(Point3D a,Point3D b)
{
	return (sqrt( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z) ) );
}

/** Returns square of distance (computed using euclidean norm) between 2D points "a" and "b". To be used for speeded up comparison of 
* distance between a pair of points, since it lacks a "sqrt()" call over computing "actual" euclidean distance.
\overload
* @param a : Point a
* @param b : Point b
* @return: square of euclidean length of line joining points a and b
*/
double Utility::euclidDistance_sq(Point2D a,Point2D b)
{
	return ((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

/** Returns square of distance (computed using euclidean norm) between 3D points "a" and "b". To be used for speeded up comparison of 
* distance between a pair of points, since it lacks a "sqrt()" call over computing "actual" euclidean distance.
\overload
* @param a : Point a
* @param b : Point b
* @return: square of euclidean length of line joining points a and b
*/
double Utility::euclidDistance_sq(Point3D a,Point3D b)
{
	return ((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

/** Computes the center of line joining 2D points "a" and "b".
\overload
* @param a : Point a
* @param b : Point b
* @return: Center point of line a-b.
*/
Point3D Utility::meanOfPoints(Point3D a,Point3D b)
{
	Point3D mean;
	mean.x = (a.x+b.x)/2.0;
	mean.y = (a.y+b.y)/2.0;
	mean.z = (a.z+b.z)/2.0;
	return mean;
}

/** Computes the center of line joining 3D points "a" and "b".
\overload
* @param a : Point a
* @param b : Point b
* @return: Center point of line a-b.
*/
inline Point2D Utility::meanOfPoints(Point2D a,Point2D b)
{
	Point2D mean;
	mean.x = (a.x+b.x)/2.0;
	mean.y = (a.y+b.y)/2.0;
}

/** Computes the optical center of camera from camera projection matrix (http://en.wikipedia.org/wiki/Camera_matrix )
* @param pM : Camera projection matrix (3x4).
* @return : Optical center of camera.
*/
Point3D Utility::getOpticalCenter( CvMat* pM )
{
	CvMat *A = cvCreateMat(3, 3, CV_64FC1);
	CvMat *Ainv = cvCreateMat(3, 3, CV_64FC1);
	CvMat *b = cvCreateMat(3, 1, CV_64FC1);
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
			cvmSet(A, i, j, cvmGet(pM,i,j));
		cvmSet(b, i, 0, cvmGet(pM, i,3));
	}
	cvInvert(A, Ainv);
	CvMat *oc = cvCreateMat(3, 1, CV_64FC1);
	cvMatMul(Ainv, b, oc);
	Point3D toRet;
	toRet.x = -1 * cvmGet(oc, 0, 0);				//NULL SPACE OF MATRIX pM
	toRet.y = -1 * cvmGet(oc, 1, 0);
	toRet.z = -1 * cvmGet(oc, 2, 0);

	cvReleaseMat(&A);
	cvReleaseMat(&Ainv);
	cvReleaseMat(&b);
	cvReleaseMat(&oc);
	return toRet;
}

/** Computes an ARBITRARY 3D point on the ray (originating at camera optical center) which is backprojected from given point "v",
and inverse camera projection matrix is given to be: "invPM".  Do *NOT* make any assumption on the depth of this point.
@param v : The point "v" which is backProjected.
@param invPm : Inverse projection matrix for camera.
@return : Some arbitrary point on ray originating from optical center and having 2D projection = v.
*/

Point3D Utility::backProject(Point2D v, CvMat *invPm)
{
	CvMat *temp = cvCreateMat(3, 1, CV_64FC1);
	cvmSet(temp, 0, 0, v.x);
	cvmSet(temp, 1, 0, v.y);
	cvmSet(temp, 2, 0, 1);
	CvMat *pt3D = cvCreateMat(4, 1, CV_64FC1);

	cvMatMul(invPm, temp, pt3D);
	Point3D toRet;
	toRet.x = cvmGet(pt3D, 0, 0) / cvmGet(pt3D, 3, 0);
	toRet.y = cvmGet(pt3D, 1, 0) / cvmGet(pt3D, 3, 0);
	toRet.z = cvmGet(pt3D, 2, 0) / cvmGet(pt3D, 3, 0);
	cvReleaseMat(&temp);
	cvReleaseMat(&pt3D);
	return toRet;
}

/** Returns the 2D projection of a 3D point when viewed from teh camera having projection matrix = "projMat"
@param input : The actual 3D point.
@param projMat : Camera Projection matrix.
@return : 2D Projection of point "input" when viewed using camera having projection matrix = "prjMat"
*/
Point2D Utility::get2Dfrom3D(Point3D input,CvMat* projMat)
{
	Point2D output;
	CvMat *p2D = cvCreateMat(3, 1, CV_64FC1);
	CvMat *p3D = cvCreateMat(4, 1, CV_64FC1);
	cvmSet(p3D, 3, 0, 1);

	cvmSet(p3D, 0, 0, input.x);
	cvmSet(p3D, 1, 0, input.y);
	cvmSet(p3D, 2, 0, input.z);
	cvMatMul(projMat, p3D, p2D);

	output.x = cvmGet(p2D,0,0)/cvmGet(p2D,2,0);
	output.y = cvmGet(p2D,1,0)/cvmGet(p2D,2,0);
	cvReleaseMat(&p2D);
	cvReleaseMat(&p3D);
	return output;

}

/** 
* Calculate the line segment PaPb that is the shortest route between two lines P1-P2 and P3-P4.
* \par Calculate also the values of "mua" and "mub" where
* \n Pa = P1 + mua (P2 - P1)
* \n  Pb = P3 + mub (P4 - P3)
\n
*	- Credit: This function was written by Paul Bourke.
*	- See more Explanation & Code at: http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline3d/
@param p1 : See explanation above
@param p2 : See explanation above
@param p3 : See explanation above
@param p4 : See explanation above
@param pa : See explanation above
@param pb : See explanation above
@param mua : See explanation above
@param mub : See explanation above

@return : Return false if no solution exists, otherwise true; Note: pa,pb,mua,mub are also part of "return" value.
*/

bool Utility::LineLineIntersect(Point3D p1,Point3D p2,Point3D p3,Point3D p4,Point3D *pa,Point3D *pb,double *mua, double *mub)
{
	Point3D p13,p43,p21;
	double d1343,d4321,d1321,d4343,d2121;
	double numer,denom;

	p13.x = p1.x - p3.x;
	p13.y = p1.y - p3.y;
	p13.z = p1.z - p3.z;
	p43.x = p4.x - p3.x;
	p43.y = p4.y - p3.y;
	p43.z = p4.z - p3.z;
	if (fabs(p43.x)  < EPSILON && fabs(p43.y)  < EPSILON && fabs(p43.z)  < EPSILON)
		return(false);
	p21.x = p2.x - p1.x;
	p21.y = p2.y - p1.y;
	p21.z = p2.z - p1.z;
	if (fabs(p21.x)  < EPSILON && fabs(p21.y)  < EPSILON && fabs(p21.z)  < EPSILON)
		return(false);

	d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
	d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
	d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
	d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
	d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

	denom = d2121 * d4343 - d4321 * d4321;
	if (fabs(denom) < EPSILON)
		return(false);
	numer = d1343 * d4321 - d1321 * d4343;

	*mua = numer / denom;
	*mub = (d1343 + d4321 * (*mua)) / d4343;

	pa->x = p1.x + *mua * p21.x;
	pa->y = p1.y + *mua * p21.y;
	pa->z = p1.z + *mua * p21.z;
	pb->x = p3.x + *mub * p43.x;
	pb->y = p3.y + *mub * p43.y;
	pb->z = p3.z + *mub * p43.z;

	return(true);
}

/** Returns the Projection matrix for a particular camera number (in the order they appear in specified file).
@param fileName : The File containing projection matrix
@param camIndex : The Index of camera for which projection matrix is to be read. ("Index" is determined by appearence of their projection matrix in file)
@return : Projection matrix. (cvMat : 3x4, 64bit float values)
*/

CvMat* Utility::readProjMatrixFromFile(std::string fileName,int camIndex)
{
	CvMat *projMat = cvCreateMat(3, 4, CV_64FC1);
	FILE *fp;

	if(projMat == NULL) // Failed to allocate memory : return NULL value to indicate failure
		return projMat;

	fopen_s(&fp,fileName.c_str(),"r");

	if(fp==NULL)  // Failed to open file : return NULL value to indicate failure
	{
		printf("\nProjection matrix file %s could not be opened",fileName.c_str());
		cvReleaseMat(&projMat);
		projMat = NULL;
		return projMat;
	}

	int equal_Signs_ToSkip = camIndex * 3 * 4; // A single projection matrix is defined completly in 3*4 = 12 "=" signs, skip those to read next
	int i=0;
	for(i=0;!feof(fp) && i<equal_Signs_ToSkip;)
		if(fgetc(fp)== int('='))
			i++;

	if(i!=equal_Signs_ToSkip) // File does not has the specified camera index : error
	{
		cvReleaseMat(&projMat);
		fclose(fp);
		projMat = NULL;
		return projMat;
	}

	// Get the projection Matrix values now
	for(int row=0;row<3;row++)
	{
		for(int col=0;col<4;col++)
		{
			// Keep skipping to next '=' sign and keep reading next float value
			int ch=!EOF; // Initialize with something !=EOF, so that it does loop condition does not fail on first instance

			while(ch!=EOF && ch!=int('='))
				ch = fgetc(fp);
			if(ch==EOF) // File does not has complete projection matrix : ERROR
			{
				cvReleaseMat(&projMat);
				fclose(fp);
				projMat = NULL;
				return projMat;
			}
			double temp;
			fscanf_s(fp,"%lf",&temp); // Read the next value after '=' sign
			cvmSet(projMat,row,col,temp);
		}
	}
	fclose(fp);
	return projMat;
}

/**
Finds the index of first occurence of a particular value (==key) in the gien vector). Can be replaced with STL's find() too. It's a Generic function.
@param V : The vector in which "key" is to be searched.
@param key : The value to be searched in vector "V"
@return : Minimum index "i", such that V[i] == key, -1 if key is not present in V.
*/

template <typename T> int Utility::find_in_vector(std::vector<T> V,T key) // Return the index of particular key in a vector, return -1 if not found
{
	for(int i=0;i<(int)V.size();i++)
		if(V[i]==key)
			return i;
	return -1;
}

void Utility::makeVectorsConfirm(std::vector<std::vector<int> > &v,int focal_row, std::vector<int> &safeViewList)
{
	if(v.size()==0 || focal_row<0 || focal_row>=v.size()) // Invalid input
		return;

	int MAX = (int)v[focal_row].size()*10; // Some large value (to be added and later subtracted) - ideally v[0].size() should be enough .. just for precaution

	//ADD MAX to each element to make them unique
	for(int col=0;col<v[focal_row].size();col++) // One loop for ech element in row == "index"
	{
		for(int row=0;row<v.size();row++)
		{
			if(row!=focal_row && find_in_vector(safeViewList,row)!=-1) // If row==focal_view ... nothing to do
			{
				int index = find_in_vector(v[row],v[focal_row][col]);
				assert(index!=-1); // If this happens => Not even all safeViews have same number of elements - Error (check input)
				v[row][index] = col+MAX;
			}
		}
		v[focal_row][col] = col+MAX;
	}

	//Now subtract MAX from each element
	for(int row=0;row<v.size();row++)
		for(int col=0;col<v[row].size() && find_in_vector(safeViewList,row)!=-1;col++)
			//printf("\nv[%d][%d] = %d",row,col,v[row][col]);
			v[row][col]-=MAX;
	/*
	for(int col=0;col<v[focal_row].size();col++)
	for(int row=0;row<v.size();row++)
	if()
	v[row][col]-=MAX;*/
}

/** 
Returns the velocity of object, assuming it moves from point p1 to point p2 in 1 unit time
@param p2 : final position of object.
@param p1 : Initial position of object.
@return : vx,vy,vz = Velocity in X,Y, and Z directions (assuming one unit of time passed between initial and final location).
*/

void Utility::computeVelocity(Point3D p2,Point3D p1,double &vx,double &vy,double &vz)
{
	vz = p2.z-p1.z;
	vy = p2.y-p1.y;
	vx = p2.x-p1.x;
}

/**
Returns the "index" of point in candidatePointsVec, closest to given "targetPoint". Used by Kalman filter right now.
\overload
@param candidatePointsVec : The pool of point from which closest point is to be found.
@param targetPoint : The target point (from which distance is to be minimized : common for all)
@return : Index of point in vector closest to "targetPoint". -1 if there is no point in vector.
*/
int Utility::closestPoint(std::vector<Point2D> candidatePointsVec,Point2D targetPoint) // return the index of point closest to target point among all candidatePoints
{
	int min_index=0;
	double min_dist;
	if(candidatePointsVec.size()==0)
		return -1;
	min_dist = euclidDistance_sq(candidatePointsVec[0],targetPoint);
	for(int i=1;i<candidatePointsVec.size();i++)
	{
		double temp;
		temp=euclidDistance_sq(candidatePointsVec[i],targetPoint);
		if(temp<min_dist)
		{
			min_index = i; 
			min_dist = temp;
		}
	}
	return min_index;
}

/** 
A bloddy dirty overload for original closestPoint. Keep the first parameter in pair "int" intact . Rest all is same as original closestPoint
\overload
*/

int Utility::closestPoint(std::vector<std::pair<int,Point2D> > candidatePointsVec,Point2D targetPoint)
{
	int min_index=0;
	double min_dist;
	if(candidatePointsVec.size()==0)
		return -1;
	min_dist = euclidDistance_sq(candidatePointsVec[0].second,targetPoint);
	for(int i=1;i<candidatePointsVec.size();i++)
	{
		double temp;
		temp = euclidDistance_sq(candidatePointsVec[i].second,targetPoint);
		if(temp<min_dist)
		{
			min_index = i; 
			min_dist = temp;
		}
	}
	return min_index;
}





/** Returns the cost vector of two sets of points and usestheir euclidance distance as cost
\param pointsSet1 : first set of points
\param pointsSet2 : second set of points
\return : cost vector of two sets of points (*dist_vector)
*/
void Utility::another_hung_distance(std::vector<Point2D> pointsSet1, std::vector<Point2D> pointsSet2, int *dist_vector) // this is for the case that number of points are not same ...
{

	int s1 = (int)pointsSet1.size();
	int s2 = (int)pointsSet2.size();

	//printf("\n s1 = %d",s1);
	//printf("\n s2 = %d",s2);

	//assert(s1 == s2);

	for (int i = 0;i <s1;i++)
		for (int j = 0;j < s2; j++)				
			dist_vector[s2*i + j] = (int)euclidDistance(pointsSet1[i],pointsSet2[j]);
}


/** Returns the cost vector of two sets of points and usestheir euclidance distance as cost
\param pointsSet1 : first set of points
\param pointsSet2 : second set of points
\return : cost vector of two sets of points (*dist_vector)
*/
void Utility::another_hung_distance(std::vector<Point3D> pointsSet1, std::vector<Point3D> pointsSet2, int *dist_vector) // this is for the case that number of points are not same ...
{

	int s1 = (int)pointsSet1.size();
	int s2 = (int)pointsSet2.size();

	//printf("\n s1 = %d",s1);
	//printf("\n s2 = %d",s2);

	//assert(s1 == s2);


	for (int i = 0;i <s1;i++)
		for (int j = 0;j < s2; j++)				
			dist_vector[s2*i + j] = (int)(10000.00*euclidDistance(pointsSet1[i],pointsSet2[j]));
}



/** Returns assignment of two sets of points using Hungarian algorithm
\param set1 : first set of points
\param set2 : second set of points
\return : assignment results(assignment1, assignment2)
*/
//void Utility::hungCorrespondOf2Sets (std::vector<Point2D> set1, std::vector<Point2D>set2, std::vector<int>&assignment1,std::vector<int>&assignment2)
//{
//	int size1 = (int)set1.size();
//	int size2 = (int)set2.size();
//	int *distance_vec = new int [size1*size2];
//	another_hung_distance(set1,set2,distance_vec);
//
//	hungarian_problem_t p;
//	int** m = array_to_matrix((int *)distance_vec,(int)set1.size(),(int)set2.size());
//
//	/* initialize the gungarian_problem using the cost matrix*/
//	int matrix_size = hungarian_init(&p, m ,(int)set1.size(),(int)set2.size(), HUNGARIAN_MODE_MINIMIZE_COST) ;
//
//	//hungarian_print_costmatrix(&p);
//	hungarian_solve(&p);
//
//
//	int maxSize = std::max(size1,size2);
//
//	assignment1.resize(maxSize);
//	assignment2.resize(maxSize);
//
//	for (int i = 0;i<maxSize;i++)
//	{
//		assignment1[i] = -1;
//		assignment2[i] = -1;
//	}
//
//	for (int i1 = 0;i1<maxSize;i1++)
//		for(int i2 = 0;i2<maxSize;i2++)
//		{
//			if(p.assignment[i1][i2] ==1)
//			{
//				if (i1<size1 && i2<size2)
//				{
//					assignment1[i1] = i2;
//					assignment2[i2] = i1;
//				}
//			}
//		}
//
//		delete [] distance_vec;
//		hungarian_free(&p);
//
//		for(int i=0; i<size1; i++)  // p.num_rows == targetNum == p.num_cols
//		{
//			free(m[i]);
//		}
//		free(m);
//
//}


// an overload of Hungarian for two sets of points for 3D case

void Utility::hungCorrespondOf2Sets (std::vector<Point2D> set1, std::vector<Point2D>set2, std::vector<int>&assignment1,std::vector<int>&assignment2)
{
		int nrows = set1.size();
		int ncols = set2.size();

		int maxSize = std::max(nrows,ncols);
		
		assignment1.resize(maxSize);
		assignment2.resize(maxSize);


	for (int i = 0;i<maxSize;i++)
	{
		assignment1[i] = -1;
		assignment2[i] = -1;
	}


		Matrix<double> costMatrix(maxSize, maxSize);

		for (int i = 0;i<nrows;i++)
			for (int j = 0;j<ncols;j++)
				costMatrix(i,j) = euclidDistance(set1[i],set2[j]);

		Munkres myMunkres;
		myMunkres.solve(costMatrix);
		// solution is back stored in costMatrix variable, 0 for matches, otherwise -1;
		// now fill in assignment
		for (int i = 0;i<maxSize;i++)
		{
			for (int j = 0;j<maxSize;j++)
			{
				if (costMatrix(i, j) ==0)
				{
					if (i<nrows && j<ncols)
					{
					assignment1[i] = j;
					assignment2[j] = i;
					}
				}
			}
		}




}



void Utility::hungCorrespondOf2Sets (std::vector<Point3D> set1, std::vector<Point3D>set2, std::vector<int>&assignment1,std::vector<int>&assignment2)
{
		int nrows = set1.size();
		int ncols = set2.size();

		int maxSize = std::max(nrows,ncols);
		
		assignment1.resize(maxSize);
		assignment2.resize(maxSize);


	for (int i = 0;i<maxSize;i++)
	{
		assignment1[i] = -1;
		assignment2[i] = -1;
	}


		Matrix<double> costMatrix(maxSize, maxSize);

		for (int i = 0;i<nrows;i++)
			for (int j = 0;j<ncols;j++)
				costMatrix(i,j) = euclidDistance(set1[i],set2[j]);

		Munkres myMunkres;
		myMunkres.solve(costMatrix);
		// solution is back stored in costMatrix variable, 0 for matches, otherwise -1;
		// now fill in assignment
		for (int i = 0;i<maxSize;i++)
		{
			for (int j = 0;j<maxSize;j++)
			{
				if (costMatrix(i, j) ==0)
				{
					if (i<nrows && j<ncols)
					{
					assignment1[i] = j;
					assignment2[j] = i;
					}
				}
			}
		}




}


void Utility::hungCorrespondOf2SetsCostFunctionVersion (int size1, int size2, Matrix<double> costMat , std::vector<int>&assignment1,std::vector<int>&assignment2)
{
	int nrows = size1;
		int ncols = size2;
	
		

		int maxSize = std::max(nrows,ncols);

		assignment1.resize(maxSize);
		assignment2.resize(maxSize);

	for (int i = 0;i<maxSize;i++)
	{
		assignment1[i] = -1;
		assignment2[i] = -1;
	}

		assignment1.resize(maxSize);
		assignment2.resize(maxSize);

		Munkres myMunkres;
		myMunkres.solve(costMat);
		// solution is back stored in costMatrix variable, 0 for matches, otherwise -1;
		// now fill in assignment

		
		for (int i = 0;i<nrows;i++)
		{
			for (int j = 0;j<ncols;j++)
			{
							
					if (costMat(i, j) ==0)
				{
					
					assignment1[i] = j;
					assignment2[j] = i;
				}
			}
		}

		
		
}




//void Utility::hungCorrespondOf2SetsCostFunctionVersion (int size1, int size2, int*distance_vec , std::vector<int>&assignment1,std::vector<int>&assignment2)
//{
//	hungarian_problem_t p;
//	int** m = array_to_matrix((int *)distance_vec, size1,size2);
//
//	/* initialize the gungarian_problem using the cost matrix*/
//	int matrix_size = hungarian_init(&p, m , size1,size2, HUNGARIAN_MODE_MINIMIZE_COST) ;
//	hungarian_solve(&p);
//
//	int maxSize = std::max(size1,size2);
//
//	assignment1.resize(maxSize);
//	assignment2.resize(maxSize);
//
//	for (int i = 0;i<maxSize;i++)
//	{
//		assignment1[i] = -1;
//		assignment2[i] = -1;
//	}
//
//	for (int i1 = 0;i1<maxSize;i1++)
//		for(int i2 = 0;i2<maxSize;i2++)
//		{
//			if(p.assignment[i1][i2] ==1)
//			{
//				if (i1<size1 && i2<size2)
//				{
//					assignment1[i1] = i2;
//					assignment2[i2] = i1;
//				}
//			}
//		}
//
//		hungarian_free(&p);
//
//		for(int i=0; i<size1; i++)  // p.num_rows == targetNum == p.num_cols
//			free(m[i]);
//
//		free(m);
//}
//
//
//
//

bool Utility::differenceBetweenTwoGroupOfPoints( std::vector<Point2D> pnts1,std::vector<Point2D> pnts2, double &totalDist, double &maxDistofPair)
{
	totalDist = 0;	maxDistofPair = -1; //initialize ...
	if(pnts1.size() != pnts2.size())
	{
		printf("\n for difference between two groups, size is not the same, so I'm returning -1");
		return false;    // this is for the case that we don't have same number of measurements in two consecutive frame .. so distance information is not useful
	}
	std::vector<int> ass1,ass2;
	hungCorrespondOf2Sets(pnts1,pnts2,ass1,ass2);

	for (int i = 0;i<ass1.size();i++)
	{
		double temp = euclidDistance(pnts1[i],pnts2[ass1[i]]);
		totalDist+=temp;
		if (temp>maxDistofPair)
			maxDistofPair = temp;
	}
	return true;

}






