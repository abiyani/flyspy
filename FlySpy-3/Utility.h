#pragma once

#ifndef __UTILITY_H_INCLUDED__
#define __UTILITY_H_INCLUDED__

#include <opencv/cv.h> // Required just for the definition of CvPoint3D64f and CvPoint2D64f
#include "Matrix.h"

#define Point3D CvPoint3D64f /// Redefinition of CvPoint3D64f to a more human readable name
#define Point2D CvPoint2D64f /// Redefinition of CvPoint2D64f to a more human readable name

/// PI: Computed as PI = 2.0*acos(0.0)
const double PI=(2.0*acos(0.0));

/// Precision for floating point value comparisons.
const double EPSILON = 1e-12;

const double INFINITY_ALEPH0 = 1e13;
const double INFINITY_ALEPH1 = 1e90;
const double INFINITY_ALEPH2 = 1e200;

/** This Namespace contains several miscellaneous functions for tasks like: reading projection matrices from file,
computing distance between points, 3D lines, and any other random function. There is no particular criteria 
for a function to be in this namespace. This is to be globally used by complete project (I added "using namespace Utility;" to Definitions.h for now)
*/
namespace Utility
{
	bool areRealsEqual(double a,double b,double precision=EPSILON);
	bool areRealsEqual(float a,float b,float precision=1e-3f);

	void Tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters);

	bool LineLineIntersect(Point3D p1,Point3D p2,Point3D p3,Point3D p4,Point3D *pa,Point3D *pb,double *mua, double *mub);

	Point3D getOpticalCenter( CvMat* pM );

	Point2D get2Dfrom3D(Point3D input,CvMat* projMat);

	Point3D backProject(Point2D v, CvMat *invPm);

	CvMat* readProjMatrixFromFile(std::string fileName,int camIndex);

	double euclidDistance(Point3D a,Point3D b);
	double euclidDistance_sq(Point3D a,Point3D b);

	double euclidDistance(Point2D a,Point2D b);
	double euclidDistance_sq(Point2D a,Point2D b);

	Point2D meanOfPoints(Point2D a,Point2D b);
	Point3D meanOfPoints(Point3D a,Point3D b);

	int factorial(int x); // Return factorial of x;

	template <typename T> int find_in_vector(std::vector<T> V,T key);

	void makeVectorsConfirm(std::vector<std::vector<int> > &v,int focal_row, std::vector<int> &safeViewList);

	int closestPoint(std::vector<Point2D> candidatePointsVec,Point2D targetPoint);
	int closestPoint(std::vector<std::pair<int,Point2D> > candidatePointsVec,Point2D targetPoint);

	// Required for kalman
	void computeVelocity(Point3D p2,Point3D p1,double &vx,double &vy,double &vz); // essntially just return difference of x y and z values as velocities
	bool differenceBetweenTwoGroupOfPoints( std::vector<Point2D> pnts1,std::vector<Point2D> pnts2, double &totalDist, double &maxDistofPair);

	void hungCorrespondOf2Sets (std::vector<Point2D> set1, std::vector<Point2D>set2, std::vector<int>&assignment1,std::vector<int>&assignment2);
	void hungCorrespondOf2Sets (std::vector<Point3D> set1, std::vector<Point3D>set2, std::vector<int>&assignment1,std::vector<int>&assignment2);

	void another_hung_distance(std::vector<Point2D> pre, std::vector<Point2D> curr, int *dist_vector);
	void another_hung_distance(std::vector<Point3D> pre, std::vector<Point3D> curr, int *dist_vector);

//	void hungCorrespondOf2SetsCostFunctionVersion (/*std::vector<Point2D> set1*/int size1, int size2, int*distance_vec /*std::vector<Point2D>set2*/, std::vector<int>&assignment1,std::vector<int>&assignment2);//, int **theCost, bool calculateTheCost, int set1Size, int set2Size);
	
	void hungCorrespondOf2SetsCostFunctionVersion (/*std::vector<Point2D> set1*/int size1, int size2, Matrix<double>costMat /*std::vector<Point2D>set2*/, std::vector<int>&assignment1,std::vector<int>&assignment2);//, int **theCost, bool calculateTheCost, int set1Size, int set2Size);



	// The function readFlyTrackFromSingleFrame() read 3D Points from a line (after leaving specified number of commas)
	// Ex: line = 0,212,34.323,32.534,432.34,32.22,32.,32.32 ... then if commasToLeave = 2, then return = {(34.323,32.534,432.34),(32.22,32.,32.32)}
	// Useful for reading 3D Track file from file
	std::vector<Point3D> readFlyTrackFromSingleFrame(std::string line,int commasToLeave);

	//////// The functions below are added for G-correspondence //////////
	Point2D getInvalidPoint();
	bool isInvalidPoint(Point2D p);
	bool isInvalidPoint(Point3D p);
}

#endif //  __UTILITY_H_INCLUDED__