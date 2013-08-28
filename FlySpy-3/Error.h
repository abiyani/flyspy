// Error.h : The aim of this file is to collect all the data structures required to keep BackProjection Errors (esp, for corresponding between views)
#pragma once

#ifndef __ERROR_H_INCLUDED__
#define __ERROR_H_INCLUDED__

#include "View.h"

struct PairOfView
{
	int rowView;
	int colView;
	bool operator==(const PairOfView &rhs)
	{
		return (rowView==rhs.rowView && colView==rhs.colView);
	}
	bool operator!=(const PairOfView &rhs)
	{
		return !(*this == rhs);
	}
	bool operator<(const PairOfView &rhs) const // Define some arbitrary order on keys, so that it can be used as  a "key" for STL map
	{
		if(rowView<rhs.rowView)
			return true;
		else
			if(rowView>rhs.rowView)
				return false;

		if(colView<rhs.colView)
			return true;
		return false;
	}
	bool operator>(const PairOfView &rhs) // Defined as a>b, iff, a<b or a==b
	{
		return !((*this == rhs) && (*this<rhs));
	}

};

/** A structure to store Error assuming two points in different view correspond to same point in real world
\n Higher the values of all 3 parameters in this structure, Higher is the error
\sa computeBackProjectionError()
*/
struct BackProjectionError_struct
{
	double errV1; //< Absolute sum of errors(=difference from original point) in X and Y coordinates of backprojected point in View 1 
	double errV2; //< Absolute sum of errors(=difference from original point) in X and Y coordinates of backprojected point in View 2
	double shortestLineLength; //< The "shortest distance" between 3D lines backprojected from optical centeres and the given points

	BackProjectionError_struct()
	{
		errV1 = 0.0;
		errV2 = 0.0;
		shortestLineLength = 0.0;
	}

	/**Return the value (measure) of this error. : If we want to assign a weight to shortestLineLength, it should be done in this function
	\return : The value of total error*/
	double value()
	{
		return (errV1 + errV2); // For now shortestLineLength is given NO weight.
	}
	bool operator<(BackProjectionError_struct &rhs)
	{
		return (this->value() < rhs.value());
	}
	bool operator>(BackProjectionError_struct &rhs)
	{
		return (this->value() > rhs.value());
	}
	bool operator==(BackProjectionError_struct &rhs)
	{
		return areRealsEqual(value(),rhs.value());
	}
};

struct ViewPairErrorTable_struct
{
	vector<vector<BackProjectionError_struct> > backProjectionErrorTable; 
	PairOfView viewID;

	void clear();
	ViewPairErrorTable_struct& operator=(const ViewPairErrorTable_struct &rhs);
	void makeTranspose(ViewPairErrorTable_struct &dest);
	void fillTable(int rowViewIndex,int colViewIndex,View *rowView,View *colView);
	void print();

	~ViewPairErrorTable_struct();

};

// Error table for complete Experiment. Stores error table for all pair of views.
struct ExperimentErrorTable_struct
{
	std::map<PairOfView,ViewPairErrorTable_struct> table;
};

BackProjectionError_struct computeBackProjectionError(Point2D pt2D_view1, Point2D pt2D_view2,View *v1,View *v2);
PairOfView make_PairOfView(int rowView,int colView);

#endif