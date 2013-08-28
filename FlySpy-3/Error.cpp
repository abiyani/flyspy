#include "Error.h"

/// Returns a PairOfView type structure
PairOfView make_PairOfView(int rowView,int colView)
{
	PairOfView temp;
	temp.rowView = rowView;
	temp.colView = colView;
	return temp;
}

/** Returns the BackProjectionError_struct when two points in different views are intersected
* (here "intersection" means the center of line with minimum distance between 3D lines joining optical center and 3D projection os these points in their respective views).
* \n Here is a step by step working of this function:
*	-# Construct the 2 lines (in 3D) pasing through optical center (of their respective views) and intersecting the ViewPlane at given 2D points.
*	-# Find the equation of shortest line (in 3D) joining these two lines. This would be a single point, if the lines actually intersect.
*	-# Compute the center of this shortest line, and assume it to be "true" 3D point for stereo pair: "pt2D_view1" and "pt2D_view2".
*	-# Now to see the validity of "true" 3D point, backProject this 3D point to each of views again, and call these 2D points as : new_pt_view1 , new_pt_view2
*	-# Compute backPojection error as Manhattan distance (|x2-x1| + |y2-y1|)  between original 2D points (passed as input) and backProjection of "true" 3D points. And also the 
length of shortest line joining two backprojected 3D lines.
*	-# Return the BackProjection_struct: Manhattan distance for View1, Manhattan Distance for View2 and Length of shortest line joining two 3D lines as the error. 
(If the input 2D points, weere actually the same point, the error should be very less for all values in the structure).

* \param : pt2D_view1 2D Point (input) in View 1
* \param : pt2D_view2 2D Point (input) in View 2
* \param : *v1 Pointer to View 1, so that we can access Camera Parameters (a simple technicality).
* \param : *v2 Pointer to View 2, so that we can access Camera Parameters (a simple technicality).
* \return : BackProjectionError_struct. See the description above for details.

\sa LineLineIntersect(), backProject(), get2Dfrom3D()
\relates View
*/
BackProjectionError_struct computeBackProjectionError(Point2D pt2D_view1, Point2D pt2D_view2,View *v1,View *v2)
{
	BackProjectionError_struct toReturn;
	//////////////////////////////////////////////////////////////////////////////////////////////////
	// Case for dummy points - added for G-correspondence
	int howManyInvalid = ( (isInvalidPoint(pt2D_view1))?1:0 ) + ( (isInvalidPoint(pt2D_view2))?1:0 );
	if(howManyInvalid == 2) // Both the points are dummy point, then error between them = -Infinity
	{
		toReturn.errV1 = -1.0*INFINITY_ALEPH0;
		toReturn.errV2 = -1.0*INFINITY_ALEPH0;
		toReturn.shortestLineLength = -1.0*INFINITY_ALEPH0;
		return toReturn;
	}
	if(howManyInvalid == 1) // Exactly one of them is dummy (invalid) point, then error = +Infinity
	{
		toReturn.errV1 = INFINITY_ALEPH0;
		toReturn.errV2 = INFINITY_ALEPH0;
		toReturn.shortestLineLength = INFINITY_ALEPH0;
		return toReturn;
	}
	// If execution reach here => None of the point is "invalid" (dummy), then proceed as usual
	///////////////////////////////////////////////////////////////////////////////////
	
	Point3D pt3D_view1 = backProject(pt2D_view1,v1->camParams.invProjMatrix); // Get an arbitrary 3D point lying on line joining optical center and intersecting View 1 plane at pt2D_view1
	Point3D pt3D_view2 = backProject(pt2D_view2,v2->camParams.invProjMatrix); // Get an arbitrary 3D point lying on line joining optical center and intersecting View 2 plane at pt2D_view2

	double mua,mub;
	Point3D Pa,Pb;
	LineLineIntersect(v1->camParams.opticalCenter,pt3D_view1,v2->camParams.opticalCenter,pt3D_view2,&Pa,&Pb,&mua,&mub); // Find the shortest distance line (Pa-Pb) among two 3D lines

	Point3D center; // This is supposedly the 3D point computed from stereo pair pt2D_view1 and pt2D_view2
	center = meanOfPoints(Pa,Pb); // Compute center of Pa,Pb

	Point2D new_pt_view1 = get2Dfrom3D(center,v1->camParams.projMatrix); // BackProject "center" to view 1
	Point2D new_pt_view2 = get2Dfrom3D(center,v2->camParams.projMatrix); // BackProject "center" to view 2

	// Compute the Manhattan distances between original and backProjected 2D points
	toReturn.errV1 = fabs(pt2D_view1.x-new_pt_view1.x) + fabs(pt2D_view1.y-new_pt_view1.y); 
	toReturn.errV2 = fabs(pt2D_view2.x-new_pt_view2.x) + fabs(pt2D_view2.y-new_pt_view2.y);

	//Compute the length of shortest line joining the backprojected 3D lines.
	toReturn.shortestLineLength = euclidDistance(Pa,Pb);

	return toReturn;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void ViewPairErrorTable_struct::clear()
{
	for(int i=0;i<backProjectionErrorTable.size();i++)
		backProjectionErrorTable[i].clear();
	backProjectionErrorTable.clear();

	viewID.rowView = -1;
	viewID.colView = -1;
}

ViewPairErrorTable_struct& ViewPairErrorTable_struct::operator=(const ViewPairErrorTable_struct &rhs)
{
	// Check for self-assignment!
	if (this == &rhs)      // Same object?
		return *this;        // Yes, so skip assignment, and just return *this.

	clear();
	viewID = rhs.viewID;
	for(int i=0;i<rhs.backProjectionErrorTable.size();i++)
		backProjectionErrorTable.push_back(rhs.backProjectionErrorTable[i]);

	return *this;
}

void ViewPairErrorTable_struct::print()
{
	for(int i=0;i<backProjectionErrorTable.size();i++)
	{
		for(int j=0;j<backProjectionErrorTable[i].size();j++)
		{
			printf("%.2lf\t",backProjectionErrorTable[i][j].value());
		}
		printf("\n");
	}
}
void ViewPairErrorTable_struct::makeTranspose(ViewPairErrorTable_struct &dest)
{
	dest.clear();
	dest.viewID.rowView = viewID.colView;
	dest.viewID.colView = viewID.rowView;

	if(backProjectionErrorTable.size()<=0) // Empty Table
		return;

	for(int col=0;col<backProjectionErrorTable[0].size();col++) // Assuming number of columns are same in each row.
	{
		std::vector<BackProjectionError_struct> tempVec;
		for(int row=0;row<backProjectionErrorTable.size();row++) // Push the values [col][0],[col][1],[col][2] ... into a temp vector
		{
			tempVec.push_back(backProjectionErrorTable[row][col]);
		
		}

		dest.backProjectionErrorTable.push_back(tempVec); // Now push the temp vector as dest's errorTable row# = value of variable "col"
	}
}

void ViewPairErrorTable_struct::fillTable(int rowViewIndex,int colViewIndex,View *rowView,View *colView)
{
	viewID.rowView = rowViewIndex;
	viewID.colView = colViewIndex;
	if(rowViewIndex == colViewIndex)
		return;
	assert(rowView->centroids.size() == colView->centroids.size()); // Should not happen, infact size() == nFlies for both the views

//	int dimension = (int)rowView->centroids.size(); // should be always == colView->centroids.size() == nFlies
	int dimension = (int)rowView->centroids.size(); // NOT ANY MORE HARDCODED should be always == colView->centroids.size() == nFlies HARDCODED INDEX FOR WINDOWCENTEROIDS SHOULD BE PRE
	for(int row=0;row<dimension;row++)
	{
		vector<BackProjectionError_struct> tempVec;
		for(int col=0;col<dimension;col++)
		{
			tempVec.push_back(computeBackProjectionError(rowView->centroids[row],colView->centroids[col],rowView,colView)); ///NOT ANY MORE HARDCODED
				//printf("\n row = %d, col = %d",row,col);
		}
			//tempVec.push_back(computeBackProjectionError(rowView->centroidCalculator.windowCentroids[1][row],colView->centroidCalculator.windowCentroids[1][col],rowView,colView));//HARDCODED

		
		backProjectionErrorTable.push_back(tempVec);
	}
	
}

ViewPairErrorTable_struct::~ViewPairErrorTable_struct()
{
	for(int i=0;i<backProjectionErrorTable.size();i++)
		backProjectionErrorTable[i].clear();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

