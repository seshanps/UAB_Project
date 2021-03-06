///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#include "Sample.h"
#include "CommonMethods.h"



#define WIDTH	320
#define HEIGHT	240
#define FPS	30
#define FOV     70
#define MAX_DEPTH 1.8
// Scale fator for adjusting the scale of the 3D reconstruction
#define SCALE_FACTOR_3D_RECONSTRUCTION 50.0
const string DATA_DIRECTORY = "/home/adas/Projects/CL-DUO3D-LIN-1.1.0.30/Docs/Snaps/"; 






//————————————————————————————————————————————————————————————————————
// Default initializer
//————————————————————————————————————————————————————————————————————

StereoPair::StereoPair(){
    StereoPair(640, 480, 30, "");
}


//————————————————————————————————————————————————————————————————————
// Initializer
//————————————————————————————————————————————————————————————————————

StereoPair::StereoPair(int width, int height, int fps, string _dataDirectory){

    dataDirectory = _dataDirectory;
    imageWidth = width;
    imageHeight = height;
    maximumDepth = -1.0; // Default: not apply filter when displaying point cloud;
    
    // Setting some defaults
    useColorImages = false;
    flippedUpsideDown = false;
    swapped = false;
    canRectify = false;
    rectify = false;
    calibration_ParametersFileName = "stereo_calibration_parameters.xml";
    sgbm_ParametersFileName = "semiglobal_block_match_parameters.xml";
    calibration_boardSize = Size(9, 6);
    calibration_squareSize = 0.022;
    calibration_numberOfImages = 16;
    
    
# ifdef DUO3D
    if(!OpenDUOCamera(width, height, fps)){
        cout << "\n*******CAMERA INITIALIZATION ERROR******" << endl;
        exit(EXIT_FAILURE);
    }
    // Set exposure and LED brightness
    exposure = 20;
    ledIntensity = 0;
    SetExposure(exposure);
    SetLed(ledIntensity);

    }
# endif
    
    // Setup rectification parameters and rectification maps
    //setupRectification();
	//Setup Semi Global Block Matching object
 //   setupDisparityParameters();
printf("hello1234%d",rectify);
}


void StereoPair::rectifyImages(bool doRectify){
    rectify = doRectify;}



//————————————————————————————————————————————————————————————————————
//  setupRectification
//————————————————————————————————————————————————————————————————————

void StereoPair::setupRectification() {
    Mat R1, R2, P1, P2;
    Mat rmap[2][2];
    Rect validRoi[2];
    
    // Open calibration file
    string calibration_parametersFile = dataDirectory + calibration_ParametersFileName;
    FileStorage fs(calibration_parametersFile, FileStorage::READ);
    
    // Read calibration file
    if(!fs.isOpened()){
        cout << "READ ERROR: Calibration file was not found" << endl;
        cout << "The provided file path is: " + dataDirectory + calibration_parametersFile << endl;
        cout << "Do you want to calibrate now? (y/n): " << endl;
        return;
    }
    else{
        Size imageSize;
        Mat cameraMatrix0, cameraMatrix1, distCoeffs0, distCoeffs1, imgSize, RInitial, TInitial;
        fs["Size"] >> imageSize;
        fs["K1"] >> cameraMatrix0;
        fs["distCoeffs1"] >> distCoeffs0;
        fs["K2"] >> cameraMatrix1;
        fs["distCoeffs1"] >> distCoeffs1;
        fs["R"] >> RInitial;
        fs["T"] >> TInitial;
        
        // Compute rectification mappings
        stereoRectify(cameraMatrix0, distCoeffs0, cameraMatrix1, distCoeffs1, imageSize, RInitial, TInitial, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, imageSize, &validRoi[0], &validRoi[1]);
        initUndistortRectifyMap(cameraMatrix0, distCoeffs0, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
        initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
        
        
        // Save the rectification mappings
        rectification.K = P1;
        rectification.B = P2.at<double>(0, 3) / P2.at<double>(0, 0);
        for(int i=0; i<2; i++)
            for(int j=0; j<2; j++)
                rectification.rmap[i][j] = rmap[i][j];
        canRectify = true;
    }
}









Mat StereoPair::glueTwoImagesHorizontal(Mat Img1, Mat Img2){
    Mat LR(Img1.rows, Img1.cols+Img2.cols, Img1.type());
    
    //Place each image horizontally adjacent to each other
    Mat left_roi(LR, Rect(0, 0, Img1.cols, Img1.rows));
    Img1.copyTo(left_roi);
    Mat right_roi(LR, Rect(Img1.cols, 0, Img2.cols, Img2.rows)); // Copy constructor
    Img2.copyTo(right_roi);
    
    return LR;
}


//————————————————————————————————————————————————————————————————————
//  glueTwoImagesVertical
//————————————————————————————————————————————————————————————————————

Mat StereoPair::glueTwoImagesVertical(Mat Img1, Mat Img2){
    Mat LR(Img1.rows+Img2.rows, Img1.cols, Img1.type());
    
    //Place each image vertically adjacent to each other
    Mat top_roi(LR, Rect(0, 0, Img1.cols, Img1.rows));
    Img1.copyTo(top_roi);
    Mat bottom_roi(LR, Rect(0, Img1.rows, Img2.cols, Img2.rows)); // Copy constructor
    Img2.copyTo(bottom_roi);
    
    return LR;
}



void StereoPair::updateImages() {
    Mat newFrameL, newFrameR;
	
    

    // Capture DUO frame
   /* PDUOFrame pFrameData = GetDUOFrame();
    if(pFrameData == NULL) return;
    IplImage *left =  cvCreateImageHeader(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 1);
    IplImage *right = cvCreateImageHeader(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 1);
    // Set the image data
    left->imageData =  (char*)pFrameData->leftData;
    right->imageData = (char*)pFrameData->rightData;
    // DUO3D seems to have a bug and gives left image as right!
    newFrameL = right;
    newFrameR = left;
*/


// Create OpenCV windows
//		cvNamedWindow("Left");
//		cvNamedWindow("Right");

// Capture DUO frame
		//Create image headers for left & right frames
		IplImage *left, *right;
		PDUOFrame pFrameData = GetDUOFrame();
		if(pFrameData == NULL) {
		
		return;
		}
		printf("UpdateImages working\n");
		// Set the image data
	
		{
			//first = false;
			left = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
			right = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

		}
		left->imageData = (char*)pFrameData->leftData;
		right->imageData = (char*)pFrameData->rightData;

		// Process images here (optional)
		newFrameL = right;
   		newFrameR = left;
		// Display images
				
		// Run capture loop until <Esc> key is pressed
	/*	while((cvWaitKey(1) & 0xff) != 27)
		{
		cvShowImage("Left", left);
		cvShowImage("Right", right);
		}

		cvDestroyAllWindows();
*/
		

    if (flippedUpsideDown) {
        Mat tmpL, tmpR;
        flip(newFrameL, tmpL, -1);  // Flip vertically
        flip(newFrameR, tmpR, -1);  // Flip vertically
        newFrameL = tmpR;   // Since we are upsidedown, now left is right
        newFrameR = tmpL;   // Since we are upsidedown, now right is left
        //flip(newFrameL, newFrameL, 1);  // Flip horizontally
        //flip(newFrameR, newFrameR, 1);  // Flip horizontally
	
    }
   	cout<<"Why is it not working?";
	if(swapped)
	{	cout<<"Why";
		rightImage = newFrameL;
		leftImage = newFrameR;
	}
	else
	{	cout<<"Else\n";
		rightImage = newFrameR;
		leftImage = newFrameL;
		cout<<"After swap\n";
	}
		
    //rightImage = swapped? newFrameL : newFrameR;
    //leftImage = swapped? newFrameR : newFrameL;
	cout<<"Why is it not working?";
}
    
    /*if (rectify && canRectify) {
        remap(leftImage, leftImage,   rectification.rmap[0][0], rectification.rmap[0][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);
        remap(rightImage, rightImage, rectification.rmap[1][0], rectification.rmap[1][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);
    }*/





//————————————————————————————————————————————————————————————————————
//  displayImages
//————————————————————————————————————————————————————————————————————

void StereoPair::displayImages(bool drawLines) {
   
 // Create visualization windows
//    if (rectified) namedWindow("Rectified stereo images", CV_WINDOW_NORMAL);
//    else namedWindow("Uncalibrated stereo images", CV_WINDOW_NORMAL);
    // Reset frame counter
    int frameCount = 0;
    
    for (;;) {
        updateImages();
        Mat LR = glueTwoImagesHorizontal(leftImage, rightImage);
        
        if (drawLines) {
            cvtColor(LR, LR, COLOR_GRAY2BGR);   // Convert to BGR (RGB) color space for drawing coloured lines.
            for(int h=0; h<LR.rows; h+=25) {
                Point pt1(0, h);
                Point pt2(LR.cols, h);
                line(LR, pt1, pt2, CV_RGB(255, 123, 47), 1);
            }
        }
        
        // Draw available commands on the bottom-left corner of the window
        string message = "s: Save images    a: Autotune exposure     ESC: Close mode";
        putText(LR, message, Point(10, LR.rows - 10), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB(255, 123, 47), 1, CV_AA);
        rectify=0;
        if (rectify) {imshow("Rectified stereo images", LR);}
        else {imshow("Uncalibrated stereo images", LR);}
        int keyPressed = int(char(waitKey(10)));
        
        // Save the images if 's' or 'S' key has been pressed
        if( keyPressed==83 || keyPressed==115) {
            saveImage(leftImage, (rectify? "Rectified_L_" + to_string(frameCount) : "Uncalibrated_L_" + to_string(frameCount)), dataDirectory);
            saveImage(rightImage, (rectify? "Rectified_R_" + to_string(frameCount) : "Uncalibrated_R_" + to_string(frameCount)), dataDirectory);
            if (drawLines) saveImage(LR, "StereoPair" + to_string(frameCount), dataDirectory);
            frameCount++;
        }
        
#ifdef DUO3D
        // Run exposure autotune if 'a' or 'A' is pressed
        if(keyPressed==65 || keyPressed==97) {
            autoTuneExposure();
        }
#endif
        
        // Exit if 'esc' key is pressed
        if( keyPressed==27) {
            // Close the windows
            if (rectify) destroyWindow("Rectified stereo images");
            else destroyWindow("Uncalibrated stereo images");
            for(int i = 0; i < 10; i++) waitKey(1); // In some systems, if this is not included windows may becmome unresponsive.
            goto sesh;
        }
    }
sesh: return;
	}



//————————————————————————————————————————————————————————————————————
//  flipUpsideDown
//————————————————————————————————————————————————————————————————————
void StereoPair::flipUpsideDown() {
    flippedUpsideDown = !flippedUpsideDown;
}


//————————————————————————————————————————————————————————————————————
//  SaveImage
//————————————————————————————————————————————————————————————————————

bool StereoPair::saveImage(Mat image, string imageName, string outputDirectory) {
    
    if(!outputDirectory.empty()){
        char fileName[256];
        sprintf(fileName, "%s%s.png", outputDirectory.c_str(), imageName.c_str());
        try {
            imwrite(fileName, image);
            cout << "Saved image: " << fileName << endl;
            return 1;
        }
        catch (std::runtime_error& ex){
            fprintf(stderr, "Could not save image to store: %s \n", ex.what());
        }
    }
    else fprintf(stderr, "Output directory is not set. Image could not be saved.");
    return false;
}





//————————————————————————————————————————————————————————————————————
//  displayImage3D
//————————————————————————————————————————————————————————————————————

void StereoPair::displayImage3D(){
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getPointCloudVisualizer();
    
    while (!viewer->wasStopped())
    {
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        updateImages();
        updateDisparityImg();
        updateImage3D();
        updatePointCloudVisualizer(viewer);
        viewer->spinOnce(10, true);
        int keyPressed = int(char(waitKey(10)));
        if (keyPressed==27){
            viewer->close();
        }
    }
}

//————————————————————————————————————————————————————————————————————
//  updateImage3D
//————————————————————————————————————————————————————————————————————

void StereoPair::updateImage3D(){
    /*
    image3D = Mat(disparityMap.size(), CV_32FC3);
    //Get the interesting parameters from Q
    double Q03, Q13, Q23, Q32, Q33;
    Q03 = Q.at<double>(0,3);    //cx
    Q13 = Q.at<double>(1,3);    //cy
    Q23 = Q.at<double>(2,3);    //f 
    Q32 = Q.at<double>(3,2);    //
    Q33 = Q.at<double>(3,3);
    
    double px, py, pz;
    double minX = 10000000, maxX = 0;
    double minY = 10000000, maxY = 0;
    double minZ = 10000000, maxZ = 0;
    for (int i = 0; i < disparityMap.rows; i++)
    {
        uchar* disp_ptr = disparityMap.ptr<uchar>(i);

        for (int j = 0; j < disparityMap.cols; j++)
        {
            //Get 3D coordinates
            double d = static_cast<double>(disp_ptr[j]);
            if ( d == 0 ) continue; //Discard bad pixels
            double pw = 1.0 * d * Q32 + Q33;
            px = static_cast<double>(j) + Q03;
            py = static_cast<double>(i) + Q13;
            pz = Q23;
            
            px = px/pw;
            py = py/pw;
            pz = pz/pw;
            if (pz == 0) continue;
            if (pz > 0.3) continue;
            //if (px > 1.0 || px < 1.0) continue;
            //if (py > 1.0 || py < 1.0) continue;
            if(px < minX) minX = px;
            else if (px > maxX) maxX = px;
            if(py < minY) minY = py;
            else if (py > maxY) maxY = py;
            if(pz < minZ) minZ = pz;
            else if (pz > maxZ) maxZ = pz;
            image3D.at<Vec3f>(i, j).val[0] = px;
            image3D.at<Vec3f>(i, j).val[1] = -1*py;
            image3D.at<Vec3f>(i, j).val[2] = pz;
        }
    }
    //cout << "minX: " << minX << "   maxX: " << maxX << endl;
    //cout << "minY: " << minY << "   maxY: " << maxY << endl;
    //cout << "minZ: " << minZ << "   maxZ: " << maxZ << endl;
     */
    
    Mat tmpDisp;
    flip(disparityMap, tmpDisp, -1);  // Flip vertically because reprojectImageTo3D inverts image
    image3D = Mat(tmpDisp.size(), CV_32FC3);
    reprojectImageTo3D(tmpDisp, image3D, Q);
}



//————————————————————————————————————————————————————————————————————
//  updateDisparityImg
//————————————————————————————————————————————————————————————————————

void StereoPair::updateDisparityImg(float scaleFactor){
    const int numThreads = numberOfAvailableThreads();
    if(scaleFactor != 1.0){ 
        Mat scaledLeftImage, scaledRightImage;
        int interpolationMethod = INTER_AREA;

        resize(leftImage, scaledLeftImage, Size(), scaleFactor, scaleFactor, interpolationMethod);
        resize(rightImage, scaledRightImage, Size(), scaleFactor, scaleFactor, interpolationMethod);
        
        if (numThreads > 1){ cout<<"Disparity?\n";
            // Divide the disparity map computation into diferent threads by making each thread compute a strip of the disparity map
            const int extraMargin = 10; // This is for SGBM to use neighbour comparison and make the later strip fusion smooth
            Mat *dispMaps = new Mat[numThreads]; // Since numThreads is unknown, create a dynamically allocated array

            # pragma omp parallel for
            for (int i = 0; i < numThreads; i++) {
                int marginTop = i > 0 ? extraMargin : 0;
                int marginBottom = i < numThreads-1 ? extraMargin : 0;
                Rect roi(0, scaledLeftImage.rows*i/numThreads-marginTop, scaledLeftImage.cols, scaledLeftImage.rows/numThreads+marginBottom + marginTop);
                Mat imR_strip = scaledRightImage(roi);
                Mat imL_strip = scaledLeftImage(roi);
                semiGlobalBlobMatch(imL_strip, imR_strip, dispMaps[i]);
            }
            
            // Merge strips into a single disparity map
            disparityMap = Mat(scaledLeftImage.size(), 3);
            for (int i = 0; i < numThreads; i++) {
                int marginTop = i > 0 ? extraMargin : 0;
                int marginBottom = i < numThreads-1 ? extraMargin : 0;
                Mat roiDispMap = disparityMap(Rect(0, i*(disparityMap.rows/numThreads), disparityMap.cols, disparityMap.rows/numThreads));
                Mat roiStrip = dispMaps[i](Rect(0, marginTop, dispMaps[i].cols, dispMaps[i].rows - marginBottom - marginTop));
                roiStrip.copyTo(roiDispMap);
                
            }
            delete [] dispMaps;
        }
        else {
            semiGlobalBlobMatch(scaledLeftImage, scaledRightImage, disparityMap);
        }
        resize(disparityMap, disparityMap, Size(), 1/scaleFactor, 1/scaleFactor, interpolationMethod);
    }	
    
    else{
	
	semiGlobalBlobMatch(leftImage, rightImage, disparityMap);
	}
}



//————————————————————————————————————————————————————————————————————
//  getDisparityImgNormalised
//————————————————————————————————————————————————————————————————————

Mat StereoPair::getDisparityImageNormalised(){
	Mat dspn;
	normalize(disparityMap, dspn, 0, 255, CV_MINMAX, CV_8U);
	return dspn;
}


//————————————————————————————————————————————————————————————————————
//  displayDisparityMap
//————————————————————————————————————————————————————————————————————

void StereoPair::displayDisparityMap() {
    cvNamedWindow("Disparity");
    
    //  Create the Controls window
    namedWindow("Controls", CV_WINDOW_NORMAL);
    Mat controlsBackground(1, 450, leftImage.type(), Scalar(150));
    imshow("Controls", controlsBackground);
    createTrackbar("SADWindowSize", "Controls", &semiGlobalBlobMatch.SADWindowSize, 50);
    //  createTrackbar("numberOfDisparities", "Controls", &sgbm.numberOfDisparities, 1000);
    createTrackbar("preFilterCap", "Controls", &semiGlobalBlobMatch.preFilterCap, 100);
    createTrackbar("minDisparity", "Controls", &semiGlobalBlobMatch.minDisparity, 100);
    createTrackbar("uniquenessRatio", "Controls", &semiGlobalBlobMatch.uniquenessRatio, 100);
    createTrackbar("speckleWindowSize", "Controls", &semiGlobalBlobMatch.speckleWindowSize, 300);
    createTrackbar("speckleRange", "Controls", &semiGlobalBlobMatch.speckleRange, 100);
    createTrackbar("disp12MaxDiff", "Controls", &semiGlobalBlobMatch.disp12MaxDiff, 100);
    createTrackbar("P1", "Controls", &semiGlobalBlobMatch.P1, 3000);
    createTrackbar("P2", "Controls", &semiGlobalBlobMatch.P2, 10000);
    
    //int whiteThreshold = 255;
    int frameCount = 0;
    
    while(1){
        updateImages();
        updateDisparityImg(1);
	cout<<"disparityMapNormalised= ";
        Mat disparityMapNormalised = getDisparityImageNormalised();

	
        
        cvtColor(disparityMapNormalised, disparityMapNormalised, COLOR_GRAY2BGR);
        
        // Draw available commands on the bottom-left corner of the window
        string message = "s: Save images                  d: Launch point cloud viewer";
        putText(disparityMapNormalised, message, Point(10, disparityMapNormalised.rows - 30), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB(255, 123, 47), 1, CV_AA);
        message = "x: Save current parameters    ESC: Close mode";
        putText(disparityMapNormalised, message, Point(10, disparityMapNormalised.rows - 10), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB(255, 123, 47), 1, CV_AA);
        
        imshow("Disparity", disparityMapNormalised);
        
        // Wait for key press
        int keyPressed = int(char(waitKey(20)));
        
        // Run point cloud visualizer if 'd' or 'D' keys are pressed
        if(keyPressed== 68 || keyPressed==100) {
            updateImage3D();
            displayImage3D();
        }
        
        // Save the images if required (press 's' or 'S')
        else if(keyPressed== 83 || keyPressed==115) {
            saveImage(disparityMapNormalised, "DepthMap_" + to_string(frameCount), dataDirectory);
            frameCount++;
        }
        
        // Save disparity parameters (press 'x' or 'X')
        else if(keyPressed==120 || keyPressed==88){
            saveDisparityParameters();
        }
        
        // Exit when esc key is pressed
        if( keyPressed== 27) break;
    }
    destroyWindow("Disparity");
    destroyWindow("Controls");
    for(int i = 0; i < 10; i++) waitKey(1); // In some systems, if this is not included windows may becmome unresponsive.
}



//————————————————————————————————————————————————————————————————————
//  saveDisparityParameters
//————————————————————————————————————————————————————————————————————

void StereoPair::saveDisparityParameters() {
    string sgbmParametersFile = dataDirectory + sgbm_ParametersFileName;
    FileStorage fs(sgbmParametersFile.c_str(), FileStorage::WRITE);
    if (fs.isOpened()) {
        fs <<   "SADWindowSize" << semiGlobalBlobMatch.SADWindowSize <<
        "numberOfDisparities"   << semiGlobalBlobMatch.numberOfDisparities <<
        "preFilterCap"          << semiGlobalBlobMatch.preFilterCap <<
        "minDisparity"          << semiGlobalBlobMatch.minDisparity <<
        "uniquenessRatio"       << semiGlobalBlobMatch.uniquenessRatio <<
        "speckleWindowSize"     << semiGlobalBlobMatch.speckleWindowSize <<
        "speckleRange"          << semiGlobalBlobMatch.speckleRange <<
        "disp12MaxDiff"         << semiGlobalBlobMatch.disp12MaxDiff <<
        "fullDP"                << semiGlobalBlobMatch.fullDP <<
        "P1"                    << semiGlobalBlobMatch.P1 <<
        "P2"                    << semiGlobalBlobMatch.P2;
        
        cout << "\n***** semiGlobalBlobMatch parameters saved *****" << endl;
        cout << "  semiGlobalBlobMatch.SADWindowSize       = "<< semiGlobalBlobMatch.SADWindowSize        << ";" << endl;
        cout << "  semiGlobalBlobMatch.numberOfDisparities = "<< semiGlobalBlobMatch.numberOfDisparities  << ";" << endl;
        cout << "  semiGlobalBlobMatch.preFilterCap        = "<< semiGlobalBlobMatch.preFilterCap         << ";" << endl;
        cout << "  semiGlobalBlobMatch.minDisparity        = "<< semiGlobalBlobMatch.minDisparity         << ";" << endl;
        cout << "  semiGlobalBlobMatch.uniquenessRatio     = "<< semiGlobalBlobMatch.uniquenessRatio      << ";" << endl;
        cout << "  semiGlobalBlobMatch.speckleWindowSize   = "<< semiGlobalBlobMatch.speckleWindowSize    << ";" << endl;
        cout << "  semiGlobalBlobMatch.speckleRange        = "<< semiGlobalBlobMatch.speckleRange         << ";" << endl;
        cout << "  semiGlobalBlobMatch.disp12MaxDiff       = "<< semiGlobalBlobMatch.disp12MaxDiff        << ";" << endl;
        cout << "  semiGlobalBlobMatch.fullDP              = "<< (semiGlobalBlobMatch.fullDP==0?"false":"true") << ";" << endl;
        cout << "  semiGlobalBlobMatch.P1                  = "<< semiGlobalBlobMatch.P1                   << ";" << endl;
        cout << "  semiGlobalBlobMatch.P2                  = "<< semiGlobalBlobMatch.P2                   << ";" << endl << endl;
    }
    else cout << "SAVE ERROR: could not write to " << sgbmParametersFile << endl;
}


//————————————————————————————————————————————————————————————————————
//  getPointCloudVisualizer
//————————————————————————————————————————————————————————————————————

boost::shared_ptr<pcl::visualization::PCLVisualizer> StereoPair::getPointCloudVisualizer() {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (1, 1, 1);
    viewer->addCoordinateSystem ( 1.0 );
    viewer->initCameraParameters ();
    
    updatePointCloudVisualizer(viewer);
    
    return viewer;
}


//————————————————————————————————————————————————————————————————————
//  updatePointCloudVisualizer
//————————————————————————————————————————————————————————————————————

void StereoPair::updatePointCloudVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer) {
    bool usePixelColor = false;
    float pointCloudScale = 100.0;
    //Create point cloud and fill it
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    float minZ = 1000000, maxZ = 0;
    for(int i = 0; i < image3D.cols; i++){
        #pragma omp parallel for
        for(int j = 0; j < image3D.rows; j++){
            pcl::PointXYZRGB point;
            point.x = float(image3D.at<Vec3f>(j, i).val[0]*pointCloudScale);
            point.y = float(image3D.at<Vec3f>(j, i).val[1]*pointCloudScale);
            point.z = float(image3D.at<Vec3f>(j, i).val[2]*pointCloudScale);
            // Filter points that are at the background (noise)
            if (point.z > maximumDepth) continue;
            if(point.z < minZ) minZ = point.z;
            if (point.z > maxZ) maxZ = point.z;
            
            if (usePixelColor) {
                Scalar color = leftImage.at<uchar>(Point(i, j));
                uint8_t r(color.val[0]);
                uint8_t g(color.val[0]);
                uint8_t b(color.val[0]);
                if (float(image3D.at<Vec3f>(j, i).val[2]) > 0.015) {
                    r = uint8_t(0);
                    g = uint8_t(0);
                    b = uint8_t(255);
                }
                
                uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                                static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                point.rgb = *reinterpret_cast<float*>(&rgb);
            }
            
            /*  // Used for debugging the function populateScenario from ObstacleScenario class
            if(point.y > 0.0 && point.y < 0.1) {
                if(point.x > -3.0/2.0 && point.x < 3.0/2.0) {
                    if (point.z < 2.0) {
                        point_cloud_ptr->points.push_back(point);
                    }
                }
            }
            */
            point_cloud_ptr->points.push_back(point);
        }
    }
    
    if (!usePixelColor) {
        //  Apply color gradient to the point cloud
        #pragma omp parallel for
        for(unsigned int i = 0; i < point_cloud_ptr->size(); i++){
            float pz = point_cloud_ptr->at(i).z;
            int pz_mapped = int(mapValue(pz, minZ, maxZ, 0, 255));
            uint8_t r(255 - constrain(pz_mapped, 0, 255));
            uint8_t g(constrain(pz_mapped, 0, 255));
            uint8_t b(15);
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point_cloud_ptr->at(i).rgb = *reinterpret_cast<float*>(&rgb);
        }
    }
    
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    
    viewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb, "reconstruction");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reconstruction");
}






//————————————————————————————————————————————————————————————————————
//  autoTuneExposure
//————————————————————————————————————————————————————————————————————

void StereoPair::autoTuneExposure(){    // TODO: auto adjust infrared led intensity
    const int whiteThreshold = 254;
    const int maxNumberOfWhitePixels = 50000;
    const int minNumberOfWhitePixels = 40000;
    const int numberOfIterations = 60;
    
    updateImages();
    
    for(int i=0; i<numberOfIterations; i++){
        int whitePixelsCount = 0;
        for( int i = 0; i < leftImage.rows; ++i){
            for( int j = 0; j < leftImage.cols; ++j ){
                Scalar intensity = leftImage.at<uchar>(Point(j, i));
                if(intensity.val[0] > whiteThreshold){
                    whitePixelsCount++;
                }
            }
        }
        for( int i = 0; i < rightImage.rows; ++i){
            for( int j = 0; j < rightImage.cols; ++j ){
                Scalar intensity = rightImage.at<uchar>(Point(j, i));
                if(intensity.val[0] > whiteThreshold){
                    whitePixelsCount++;
                }
            }
        }
        
        if (whitePixelsCount > maxNumberOfWhitePixels)
            exposure = constrain(exposure-2, 0, 100);
        else if (whitePixelsCount < minNumberOfWhitePixels)
            exposure = constrain(exposure+2, 0, 100);
    
        SetExposure(exposure);
        updateImages();
        Mat LR = glueTwoImagesHorizontal(leftImage, rightImage);
        cvtColor(LR, LR, COLOR_GRAY2BGR);   // Convert to BGR (RGB) color space for drawing a coloured rectangle.
        rectangle(LR, Point(0, 0), Point(LR.cols*i/numberOfIterations, 10), CV_RGB(255, 123, 47), CV_FILLED, 8);
        putText(LR, "Exposure: " + to_string(exposure), Point(50,40), FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255, 123, 47), 2);
        putText(LR, "LED     : " + to_string(ledIntensity), Point(50,70), FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255, 123, 47), 2);
        imshow("autotune exposure", LR);
        waitKey(1);
    }
    destroyWindow("autotune exposure");
    for(int i = 0; i < 10; i++) waitKey(1);
}




//————————————————————————————————————————————————————————————————————
//  calibrate
//————————————————————————————————————————————————————————————————————

void StereoPair::calibrate(){

	///////////INITIAL PARAMETERS//////////////
	Size boardSize = Size(9, 6);	//Inner board corners
	float squareSize = 0.022;       //The actual square size, in any unit (meters prefered)
	int nimages = 16;				//Number of images to take for calibration
    int maxScale = 2;

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);
    vector<Mat> goodImages;			//Vector to store image pairs with calibration pattern correctly detected
    
    int itersSinceLastPattern = 1;

	// Create visualization windows
    float windowResize = 0.8;
    Mat calibDisplay;
	namedWindow("Live calibration view", CV_WINDOW_NORMAL);

	int i, j, k, frameID = 0;
	/*
	 * i: new image pair
	 * j: number of good image pairs
	 * k: image side. 0 = LEFT | 1 = RIGHT
	 */

    for( i = j = 0; j < nimages; i++ )
    {
    	while(1){
            this->updateImages();
    		Mat niml = leftImage, nimr = rightImage;
            Mat cimg1, cimg2;	//Corner images
            bool found = false;
            if( imageSize == Size() ) imageSize = niml.size();
            
            //////////DETECT CHESSBOARD CORNERS AND DISPLAY THEM/////////////
            for( k = 0; k < 2; k++ ) {
                Mat img = k==0 ? niml : nimr;
                if(img.empty()) break;
                
                vector<Point2f>& corners = imagePoints[k][j];
                
                for( int scale = 1; scale <= maxScale; scale++ ) {
                    Mat timg;
                    if( scale == 1 )
                        timg = img;
                    else
                        resize(img, timg, Size(), scale, scale);
                    
                    found = findChessboardCorners(timg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                    
                    if(found) {
                        if(scale > 1) {
                            Mat cornersMat(corners);
                            cornersMat *= 1./scale;
                        }
                        //////////////Update corner view images///////////////////////
                        Mat cimg;
                        cvtColor(img, cimg, COLOR_GRAY2BGR);
                        drawChessboardCorners(cimg, boardSize, corners, found);
                        double sf = 640./MAX(img.rows, img.cols);
                        Mat cornerImg;
                        resize(cimg, cornerImg, Size(), sf, sf);
                        if(k==0)     cimg1 = cornerImg;
                        else if(k==1)cimg2 = cornerImg;
                        
                        ///////////////IMPROVE CORNER ACCURACY////////////////
                        cornerSubPix(img, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
                        break;
                    }
                }
                
                if(!found) {
                    if (itersSinceLastPattern == 0) {
                        cout << "*** Alert! Calibration pattern must be detected in both frames!" << endl;
                    }
                    itersSinceLastPattern++;
                    break;
                }
            }
            
            if(k == 2 && found){
                calibDisplay = glueTwoImagesHorizontal(cimg1, cimg2);   // Update corner images
                resize(calibDisplay, calibDisplay, Size(), windowResize, windowResize);
                if(itersSinceLastPattern > 0){
                    cout << "*** Correctly detecting pattern!" << endl;
                    itersSinceLastPattern = 0;
                }
            } else {
                calibDisplay = glueTwoImagesHorizontal(leftImage, rightImage);
                resize(calibDisplay, calibDisplay, Size(), windowResize, windowResize);
            }
            imshow("Live calibration view", calibDisplay);
            
    		// Wait for key press
    		int keyPressed = int(char(waitKey(20)));

    		// Take images if 'n' or 'N' keys have been pressed
    		if( keyPressed==78 || keyPressed==110)
            {
                if(k == 2){
                    if (leftImage.channels() > 1) cvtColor(leftImage, leftImage, CV_RGB2GRAY);
                    if (rightImage.channels() > 1) cvtColor(rightImage, rightImage, CV_RGB2GRAY);
                    goodImages.push_back(leftImage);
                    goodImages.push_back(rightImage);
                    j++;
                    cout << "Took a new image pair, remaining images: " << nimages - j << endl;
                    break;
                }
                else cout << "Alert! Calibration pattern must be detected in both frames!" << endl;
    		}
    		// Save the images if required (pressing 's' or 'S')
    		if(keyPressed== 83 || keyPressed==115)
    		{
    			cout << "Saving image pairs..." << endl;

    			if(!dataDirectory.empty()){
    				// Create the file names for saving the images
    				char fileName[256];
    				sprintf(fileName, "%sCalib_%d.png", dataDirectory.c_str(), frameID);

    				// Write the images
    				try {
    					imwrite(fileName, calibDisplay);
    				}
    				catch (runtime_error& ex) {
    					fprintf(stderr, "Exception converting image to PNG format: %s \n", ex.what());
    				}
    			}
    			frameID++;
    		}
    		else if(keyPressed==27){
    			destroyWindow("Live calibration view");
                for(int i = 0; i < 10; i++) waitKey(1);
    			return;
    		}
    	}
    }
    
    destroyWindow("Live calibration view");
    for(int i = 0; i < 10; i++) waitKey(1);

    ////////FILL "objectPoints" WITH THE COORDINATES OF THE BOARD CORNERS////////
    #pragma omp parallel for
    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }

    /////////////RUN STEREO CALIBRATION//////////////
    cout << "******************************" << endl;
    cout << "Running stereo calibration ..." << endl;

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;
    
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    cout << "done with RMS error=" << rms << endl;
/*
 * /////////CALIBRATION QUALITY CHECK//////////////
 * because the output fundamental matrix implicitly
 * includes all the output information,
 * we can check the quality of calibration using the
 * epipolar geometry constraint: m2^t*F*m1=0
 */
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        #pragma omp parallel for
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    //////////Save calibration parameters//////////////////
    string calibration_parametersFile = dataDirectory + calibration_ParametersFileName;
    FileStorage fs(calibration_parametersFile.c_str(), CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
    	cout << "Saving parameters" << endl;
    	fs <<
        "Size"          << imageSize <<
        "K1"            << cameraMatrix[0] <<
        "distCoeffs1"   << distCoeffs[0] <<
    	"K2"            << cameraMatrix[1] <<
        "distCoeffs2"   << distCoeffs[1] <<
        "R"             << R <<
        "T"             << T <<
        "Q"             << Q;
    	fs.release();
    	cout << "Parameters saved" << endl;
    }
    else cout << "CALIBRATION ERROR: Cannot save calibration results to file" << endl;

   //////////Once parameters are saved, reinitialize rectification from them////////////////
    this->setupRectification();
    this->displayImages(true /*drawLines*/);
}




void launchAvoidanceSimulator(){
    float width = 3.0;       //In meters
    float depth = 2.0;       //In meters
    float squareSize = 0.1; //In meters
    int windowWidth = 800;
    Simulator simulator(width, depth, squareSize, windowWidth, DATA_DIRECTORY);
    simulator.runSimulation();
}











int main(int argc, char* argv[])
{	


	StereoPair stereoCam(WIDTH, HEIGHT, FPS, DATA_DIRECTORY);
	//StereoPair stereoCam=StereoPair();
 	//printf("hello=%d\n",stereoCam.rectify);
	printf("DUOLib Version:       v%s\n", GetDUOLibVersion());

	// Open DUO camera and start capturing
	if(!OpenDUOCamera(WIDTH, HEIGHT, FPS))
	{
		printf("Could not open DUO camera\n");
		return 0;
	}
	printf("\nHit <ESC> to exit.\n");



	launchAvoidanceSimulator();
	

// Create OpenCV windows
/*		cvNamedWindow("Left");
		cvNamedWindow("Right");

// Capture DUO frame
		//Create image headers for left & right frames
		IplImage *left, *right;
		// Set exposure and LED brightness
		SetGain(0);
		SetExposure(50);
		SetLed(25);



		PDUOFrame pFrameData = GetDUOFrame();
		//if(pFrameData == NULL) return;

		// Set the image data
	
		{	printf("Check number 2\n");
			//first = false;
			left = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
			right = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
		}
		left->imageData = (char*)pFrameData->leftData;
		right->imageData = (char*)pFrameData->rightData;

		// Process images here (optional)
		newFrameL = right;
   		newFrameR = left;
		// Display images
		
// Run capture loop until <Esc> key is pressed
		while((cvWaitKey(1) & 0xff) != 27)
		{
		cvShowImage("Left", left);
		cvShowImage("Right", right);
		}

		cvDestroyAllWindows();

*/


        //stereoCam.updateImages();

	//stereoCam.displayImages(true /*drawLines*/);

	
	
        // Create OpenCV windows
	//cvNamedWindow("Left");
	

	// Create image headers for left & right frames
	//IplImage *left, *right;

	// Set exposure and LED brightness
	SetGain(0);
	SetExposure(50);
	SetLed(25);

	bool first = true;
	// Run capture loop until <Esc> key is pressed
	//while((cvWaitKey(1) & 0xff) != 27)
	/*{
		// Capture DUO frame
		PDUOFrame pFrameData = GetDUOFrame();
		if(pFrameData == NULL) return 0;

		// Set the image data
		if(first)
		{
			first = false;
			left = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
			right = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
		}
		left->imageData = (char*)pFrameData->leftData;
		right->imageData = (char*)pFrameData->rightData;

		// Process images here (optional)

		// Display images
		cvNamedWindow("Left");		
		cvShowImage("Left", left);
		cvShowImage("Right", right);
		cvWaitKey(0);
		cvNamedWindow("Right");
		cvDestroyAllWindows();*/
		//stereoCam.updateImages();
		
		//stereoCam.displayImages(true /*drawLines*/);

		//stereoCam.displayDisparityMap();

	//}
	cvDestroyAllWindows();





	// Release image headers
	//cvReleaseImageHeader(&left);
	//cvReleaseImageHeader(&right);

	 //Close DUO camera
	CloseDUOCamera();
	return 0;
}
