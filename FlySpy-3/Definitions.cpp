// Definitions.cpp : To define constructor and destructors for some common structures
#include "Definitions.h"

Frame::Frame()
{
	img = NULL;
}

Frame::~Frame()
{
	if(img!=NULL) // If img structure is initialized then free the memory. (Note: Do not ever point "img" to output of cvQueryImage() - since the memory it use cannot be dealloc)
		cvReleaseImage(&img);
}


