///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2017 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#ifndef SAMPLE_H
#define SAMPLE_H
/*
// Include some generic header files
#if defined(WIN32)
	#include <SDKDDKVer.h>
	#include <windows.h>
	#include <stdio.h>
	#include <conio.h>
#elif defined(__linux__) || defined(__APPLE__)
	#include <stdio.h>
	#include <termios.h>
	#include <unistd.h>
	#include <fcntl.h>
	static struct termios _old, _new;
	/* Initialize new terminal i/o settings */
/*	void initTermios(int echo) 
	{
	  tcgetattr(0, &_old); // grab old terminal i/o settings 
	  _new = _old; /* make new settings same as old settings 
	  _new.c_lflag &= ~ICANON; /* disable buffered i/o 
	  _new.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode 
	  tcsetattr(0, TCSANOW, &_new); /* use these new terminal i/o settings now 
	}
	/* Restore old terminal i/o settings 
	void resetTermios(void) 
	{
	  tcsetattr(0, TCSANOW, &_old);
	}
	/* Read 1 character - echo defines echo mode 
	char getch_(int echo) 
	{
	  char ch;
	  initTermios(echo);
	  ch = getchar();
	  resetTermios();
	  return ch;
	}
	/* Read 1 character without echo 
	char _getch(void) 
	{
	  return getch_(0);
	}
	int _kbhit(void)
	{
	  struct termios oldt, newt;
	  int ch;
	  int oldf;
	 
	  tcgetattr(STDIN_FILENO, &oldt);
	  newt = oldt;
	  newt.c_lflag &= ~(ICANON | ECHO);
	  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	 
	  ch = getchar();
	 
	  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	  fcntl(STDIN_FILENO, F_SETFL, oldf);
	 
	  if(ch != EOF)
	  {
		ungetc(ch, stdin);
		return 1;
	  }
	  return 0;
	}
#endif
*/
// Include DUO API header file
#include <DUOLib.h>
#include "opencv2/opencv.hpp"
#include "pcl/common/common_headers.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/visualization/cloud_viewer.h>
#include "boost/thread/thread.hpp"


#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;


struct Rectification
{
	Mat rmap[2][2];
	Mat K;
	double B;
};

// Some global variables
static DUOInstance _duo = NULL;
static PDUOFrame _pFrameData = NULL;

//#if defined(WIN32)
//	static HANDLE _evFrame = CreateEvent(NULL, FALSE, FALSE, NULL);
//#elif defined(__linux__) || defined(__APPLE__)
	#include <pthread.h>
	#include <stdlib.h>
	#define WAIT_OBJECT_0	0
	struct event_flag
	{
		pthread_mutex_t mutex;
		pthread_cond_t  condition;
		unsigned int flag;
	};

	event_flag *CreateEvent(void *lpEventAttributes, bool bManualReset, bool bInitialState, char *name)
	{
		struct event_flag* ev = (struct event_flag*) malloc(sizeof(struct event_flag));
		pthread_mutex_init(&ev->mutex, NULL);
		pthread_cond_init(&ev->condition, NULL);
		ev->flag = 0;
		return ev;
	}

	void SetEvent(struct event_flag* ev)
	{
		pthread_mutex_lock(&ev->mutex);
		ev->flag = 1;
		pthread_cond_signal(&ev->condition);
		pthread_mutex_unlock(&ev->mutex);
	}

	int WaitForSingleObject(struct event_flag* ev, int timeout)
	{
		pthread_mutex_lock(&ev->mutex);
		while (!ev->flag)
			pthread_cond_wait(&ev->condition, &ev->mutex);
		ev->flag = 0;
		pthread_mutex_unlock(&ev->mutex);
		return WAIT_OBJECT_0;
	}

	static event_flag *_evFrame = CreateEvent(NULL, 0, 0, NULL);
#endif

// One and only duo callback function
// It sets the current frame data and signals that the new frame data is ready
static void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
	_pFrameData = pFrameData;
	SetEvent(_evFrame);
}

// Opens, sets current image format, fps and start capturing
static bool OpenDUOCamera(int width, int height, float fps)
{
	if(_duo != NULL)
	{
		// Stop capture
		StopDUO(_duo);
		// Close DUO
		CloseDUO(_duo);
		_duo = NULL;
	}

	// Find optimal binning parameters for given (width, height)
	// This maximizes sensor imaging area for given resolution
	int binning = DUO_BIN_NONE;
	if(width <= 752/2) 
		binning += DUO_BIN_HORIZONTAL2;
	else if(width <= 752/4) 
		binning += DUO_BIN_HORIZONTAL4;
	if(height <= 480/4) 
		binning += DUO_BIN_VERTICAL4;
	else if(height <= 480/2) 
		binning += DUO_BIN_VERTICAL2;

	// Check if we support given resolution (width, height, binning, fps)
	DUOResolutionInfo ri;
	if(!EnumerateDUOResolutions(&ri, 1, width, height, binning, fps))
		return false;

	if(!OpenDUO(&_duo))
		return false;

	char tmp[260];
	// Get and print some DUO parameter values
	GetDUODeviceName(_duo,tmp);
	printf("DUO Device Name:      '%s'\n", tmp);
	GetDUOSerialNumber(_duo, tmp);
	printf("DUO Serial Number:    %s\n", tmp);
	GetDUOFirmwareVersion(_duo, tmp);
	printf("DUO Firmware Version: v%s\n", tmp);
	GetDUOFirmwareBuild(_duo, tmp);
	printf("DUO Firmware Build:   %s\n", tmp);

	// Set selected resolution
	SetDUOResolutionInfo(_duo, ri);

	// Start capture
	if(!StartDUO(_duo, DUOCallback, NULL))
		return false;

	return true;
}

// Waits until the new DUO frame is ready and returns it
static PDUOFrame GetDUOFrame()
{
	if(_duo == NULL) 
		return NULL;

	if(WaitForSingleObject(_evFrame, 1000) == WAIT_OBJECT_0)
		return _pFrameData;
	else
		return NULL;
}

// Stops capture and closes the camera
static void CloseDUOCamera()
{
	if(_duo == NULL)
		return;

	// Stop capture
	StopDUO(_duo);
	// Close DUO
	CloseDUO(_duo);
	_duo = NULL;
}

static void SetExposure(float value)
{
	if(_duo == NULL)
		return;
	SetDUOExposure(_duo, value);
}

static void SetGain(float value)
{
	if(_duo == NULL)
		return;
	SetDUOGain(_duo, value);
}

static void SetLed(float value)
{
	if(_duo == NULL)
		return;
	SetDUOLedPWM(_duo, value);
}

//#endif // SAMPLE_H






class StereoPair {
private:
	//Atributes
    
    static DUOInstance duoCam;      // DUO3D camera instance
    static PDUOFrame   duoFrame;      // DUO3D stereo frame
    int exposure;       // From 0 to 100
    int ledIntensity;   // From 0 to 100
    
   
    bool    useColorImages;
    bool    flippedUpsideDown;
    bool    swapped;
    bool    canRectify;
   
    int     imageWidth;
    int     imageHeight;
    
	Rectification	rectification;			// Rectification maps
	StereoSGBM		semiGlobalBlobMatch;	// Disparity computation method
	Mat				Q;                      // camera matrix from stereoRectify(..., Q, ...);
    
    string          calibration_ParametersFileName;
    string          sgbm_ParametersFileName;
    
     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3D;
    
public:
    ///////Public objects and variables/////////
    string          dataDirectory;
    // Calibration parameters
     bool    rectify;
    Size            calibration_boardSize;
    float           calibration_squareSize;
    int             calibration_numberOfImages;
    
    Mat				leftImage;			// Rectified left image
    Mat				rightImage;			// Rectified right image
	Mat				disparityMap;       // Disparity map (not normalized)
	Mat				image3D;            // Depth map
    float           maximumDepth;
    
	// Constructors and destructors
    StereoPair();
	StereoPair(int width, int height, int fps, string _dataDirectory);

    // Updating and displaying
    void rectifyImages(bool doRectify);
    void updateImages();
    void updateDisparityImg(float scaleFactor = 1.0);
    void updateImage3D();
    
    void displayImages(bool drawLines);
    void displayDisparityMap();
    void displayImage3D();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> getPointCloudVisualizer();
    void updatePointCloudVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);
    
	// Initialization methods
	void setupRectification();
    void setupDisparityParameters();
    void saveDisparityParameters();

	// Utilities
	void calibrate();	//Calibrate camera intrinsics and extrinsics
    Point3f getPixel3Dcoords(int pixX, int pixY, double disp);      // TODO: implement
    void resizeImages(float scaleFactor);
    Mat glueTwoImagesHorizontal(Mat Img1, Mat Img2);
    Mat glueTwoImagesVertical(Mat Img1, Mat Img2);
    bool saveImage(Mat image, string imageName, string outputDirectory);
    void flipUpsideDown();
    float computeFieldOfView();
    void autoTuneExposure();


	// Get methods
	Mat getDisparityImageNormalised();
};


//#endif /* STEREOPAIR_H_ */




