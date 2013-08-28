
#include "SilhDetector.h"

#define PIXEL(img, i, j, k)		(*( ((img)->imageData) + ((img)->widthStep * (i)) + ((img)->nChannels * (j)) + (k) ) )
#define UPIXEL(img, i, j, k)	(*( (unsigned char*) ( ((img)->imageData) + ((img)->widthStep * (i)) + ((img)->nChannels * (j)) + (k)) ))


//#define MINAREAFORCONTOURS 15
#define MINAREAFORCONTOURS 5
//#define FRAMESFORBACKGROUND 10000

frameContoursInfo SilhDetector::findChangeMaskContours(IplImage *thisFrame) // I want to keep it void
{
#if __DSILH
	printf("\n*****begiining of segmentFrames*****\n");
#endif
	if(isInited == false || bgMean == NULL) // So the first frame is automatically initilaized as bgMean
		initialize(thisFrame);

	IplImage* changeMask;
	changeMask = cvCreateImage( cvGetSize(thisFrame),IPL_DEPTH_8U, 1); //Reza: these can be single channel, coz we just care about changes/ later: now it is! :D
	cvZero(changeMask);

	IplImage* thisFrameClone;
	thisFrameClone = cvCloneImage(thisFrame);


	fc++;
	//printf("\n fc = %d",fc);
	//////////////////////FINDING CHANGE MASK////////////////////////////////
	int i,j; // Defined above so as to unable parallelization (OpenMP)
#pragma omp parallel shared(thisFrame, changeMask) private(i,j)		//SHOULD BE SHARING BGMEAN TOO. BUT IS THROWING AN ERROR :'( WHY??? Dunnooo. Default access clause is sharing though.
	{
#pragma omp for schedule(dynamic)
		for(i= 0; i<thisFrame->height; ++i)
		{
			for(j=0; j<thisFrame->width; ++j) // Throwing error in Release mode (distance () function) ... dunno why ?? :(
			{
				if(pixelDistance( &UPIXEL(thisFrame, i, j, 0), &UPIXEL(bgMean, i, j, 0), 1) > -threshold )
				{
					for(int k = 0; k < bgMean->nChannels; ++k)
					{
						//if (fc< FRAMESFORBACKGROUND)
							 UPIXEL(bgMean, i, j, k) = (unsigned char) (alpha * UPIXEL(thisFrame, i, j, k) + (1.0-alpha) * UPIXEL(bgMean, i, j, k));
					}
				}
				else
					PIXEL(changeMask,i, j, 0) |= 0xFF; //Set all bits to 1 (White color)
			}
		}
	}

	////////////////////////BEGINING OF DENOISE/////////////
	IplImage* myTemp;
	//denoisedChangeMask = cvCreateImage( cvGetSize(thisFrame),IPL_DEPTH_8U, 1); //THIS CAUSED MEMORY LEAK
	myTemp = cvCreateImage( cvGetSize(thisFrame),IPL_DEPTH_8U, 1);

	cvErode(changeMask,myTemp,NULL,1);
	cvDilate(myTemp,denoisedChangeMask,NULL,2);//changed from 2 (last aprameter)

	CvMemStorage* storage = cvCreateMemStorage();
	CvSeq* first_contour = NULL;

	int Nc = cvFindContours(denoisedChangeMask,storage,&first_contour,sizeof(CvContour),CV_RETR_EXTERNAL);//	CV_RETR_LIST);

	//CONTOURS WITH AREA < AREATHRESH SHOULD BE FILTERED OUT AT THE VERY BEGINNINGGGGGGGGG! DO IT!
#if __DSILH
	printf("Nc = %d\n",Nc);
#endif

	contourList.clear();

	for( CvSeq* c=first_contour; c!=NULL; c=c->h_next ) //make the contourlist I just want to include contours with Area > MINAREAOFCONTOURS;
	{
		contour_data temp;
		CvPoint2D32f tmppt;
		Point2D tmppt2;
		float r;
		
		if (fabs(cvContourArea(c)) >  MINAREAFORCONTOURS)
		{
			temp.area = fabs(cvContourArea(c));
			temp.contour_seq = c;
			cvMinEnclosingCircle(c,&tmppt,&r);
			tmppt2.x = tmppt.x; tmppt2.y = tmppt.y;
			temp.center = tmppt2;
			temp.radius = r;
			contourList.push_back(temp);
		}
	}

	cvZero(denoisedChangeMask);

	for ( int i = 0;i<contourList.size();i++)
		cvDrawContours(denoisedChangeMask,contourList[i].contour_seq, cvScalar(255,255,255), cvScalar(255), -1, CV_FILLED, 8);

	//	cvMorphologyEx( denoisedChangeMask, denoisedChangeMask, 0, 0, CV_MOP_CLOSE, 1);//CVCLOSE_ITR );

	cvReleaseImage(&myTemp);
	cvReleaseImage(&thisFrameClone);
	 
	cvReleaseImage(&changeMask);
	frameContoursInfo temp;
	temp.memStore = storage;
   //cvReleaseMemStorage(&storage);

	temp.allContours_data = contourList;
	return temp;
}

void SilhDetector::finish()
{
	if (bgMean != NULL)
		cvReleaseImage(&bgMean);
	bgMean = NULL;
	isInited = false;	
}

void SilhDetector::restartSilhDetect()
{
	if(bgMean!=NULL)
		cvReleaseImage(&bgMean);
	bgMean = NULL;
	
	contourList.clear();
	SilhDetector();
}


SilhDetector::~SilhDetector(void)
{
	if (bgMean != NULL)
		cvReleaseImage(&bgMean);
	isInited = false;
}
SilhDetector::SilhDetector(void)
{
	isInited = false;
	bgMean = NULL;
	alpha = (double)0.05;
	threshold = 7;
	fc = 0;

}

void SilhDetector::initialize(IplImage *firstFrame)
{
	if(bgMean != NULL)
	{
		cvReleaseImage(&bgMean);
		bgMean = NULL;
	}
	bgMean = cvCloneImage(firstFrame);		//Take first frame as mean.
	denoisedChangeMask = cvCreateImage( cvGetSize(firstFrame),IPL_DEPTH_8U, 1);
	cvZero(denoisedChangeMask);


	isInited = true;
}

inline char SilhDetector::pixelDistance(unsigned char *thisFrame, unsigned char *bg, int nChannels)
{ 	//Can be faster if you don't use double calculations. use ints.
	if(nChannels == 3)
		return (char) ( 0.299 * (((int) thisFrame[2]) - bg[2]) + 0.587 * (((int) thisFrame[1]) - bg[1]) + 0.114 * ( ((int)thisFrame[0]) - bg[0]) );		
	else 
		return (char) (thisFrame[0] - bg[0]);

}

