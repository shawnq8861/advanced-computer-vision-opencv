/*

  Instructions for Ubuntu 16.04 with OpenCV 3.1 and opencv_contrib:

  clone the repository or download a compressed file and extract the files
  run make
  run ./program4 arg1 arg2 arg3

  arg1:  camera spacing in engineering units such as inches
  arg2:  camera focal length in pixels
  arg3:  horizontal distance from object to camera in engineering units
         such as inches

In this project, I calibrate a camera, measure 2D object features, detect rich
keypoint matchind features, match keypoints in successive images, determine
projective transformations, calculate depth from single camera images, and
calculate feature dimensions in engineering units.

1. Calibrate the camera by taking a sequence of images of a test target,
saving the images to jpeg files, and supplying the images to the OpenCV
function calibrateCamera().

2. Grab 3 reference images and save them to jpeg files named sideimage.jpg,
topeImage1.jpg, and topImage2.jpg.

3. Filter and segment the side view image.

4. Find edges and contours in the side image.

5. Calculate contour height and width from processed side image.

6. Detect SIFT keypoints in each top image, draw the keypoints, and
display the resulting overlayed image.

7. Calculate the essential matrix.

8. Determine the camera pose between 2 images.

9. Rectify the images.

10. Find the disparity map of the calibrated stereo image pair, display the
disparity map.

11. Perform uncalibrated depth measurement by warping with perspective
transform obtained from the fundamental matrix and uncalibrated
stereo rectification.

12. Find uncalibrated disparity map, display the disparity map.

13. Detect SIFT keypoints for rectified calibrated image pair and calculate
disparity map for the SIFT matches that lie on horizontal lines.

14. Find the SIFT correlated disparity map.

15. Convert disparity map to depth map in engineering units
for known camera spacing.

16. Calculate object depth.

17. Display point cloud representation of the depth map.

*/

#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <libgen.h>
#include <string>
#include <sstream>
#include <exception>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/persistence.hpp>
#include <ueye.h>

using namespace std;
using namespace cv;

//
// helper functions, display an image and save an image to jpeg file
//
void displayImage(const char *winName, Mat image, int windowSize, int xOffset, int yOffset);
void saveImageToFile(string filename, Mat image);

int main(int argc, char *argv[])
{
    //
    // parse the args to get camera spacing in inches and the apparent camera
    // focal length in pixels
    //
    float cameraSpacing = 0.0;
    stringstream cameraSpacingSS;
    cameraSpacingSS << argv[1];
    cameraSpacingSS >> cameraSpacing;
    cout << "camera spacing (in): " << cameraSpacing << endl;
    float focalLength = 0.0;
    stringstream focalLengthSS;
    focalLengthSS << argv[2];
    focalLengthSS >> focalLength;
    cout << "camera focal length (pixels): " << focalLength << endl;
    float cameraDistance = 0.0;
    stringstream cameraDistanceSS;
    cameraDistanceSS << argv[3];
    cameraDistanceSS >> cameraDistance;
    cout << "camera distance (inches): " << cameraDistance << endl;
    //
    // configure and initialize the camera
    //
    // camera used is an IDS UI-3280CP-C-HQ
    //
    // camera configuration variables
    //
    HIDS hCam = 1;
    cout << "configuring camera..." << endl;
    //
    // get the camera handle
    //
    if (is_InitCamera(&hCam, NULL) != IS_SUCCESS) {
        cout << "could not get camera handle..." << endl;
        return 1;
    }
    //
    // set the display mode to binary for direct image buffer access
    //
    INT displayMode = IS_SET_DM_DIB;    // binary display mode
    if (is_SetDisplayMode(hCam, displayMode) != IS_SUCCESS) {
        cout << "could not set display mode..." << endl;
        return 1;
    }
    const __useconds_t delay = 100000;      // .1 msec delay
    usleep(delay);
    //
    // get the camera sensor information
    //
    SENSORINFO camSensorInfo;
    camSensorInfo.nColorMode = -1;
    if (is_GetSensorInfo(hCam, &camSensorInfo) != IS_SUCCESS) {
        cout << "could not get sensor info..." << endl;
        return 1;
    }
    //
    // set value to enable auto gain
    //
    double enableDisable = 1.0;
    double unused = 0.0;
    INT nRet = is_SetAutoParameter (hCam,
                                IS_SET_ENABLE_AUTO_GAIN,
                                &enableDisable,
                                &unused);
    if (nRet == IS_SUCCESS) {
        cout << "auto gain set to enabled..." << endl;
    }
    else {
        cout << "could not set auto shutter to enabled..." << endl;
    }
    //
    // set value to enable auto exposure
    //
    nRet = is_SetAutoParameter (hCam,
                                IS_SET_ENABLE_AUTO_SHUTTER,
                                &enableDisable,
                                &unused);
    if (nRet == IS_SUCCESS) {
        cout << "auto shutter enabled..." << endl;
    }
    else {
        cout << "could not enable auto shutter..." << endl;
    }
    //
    // check if auto white balance is supported
    //
    UINT nSupportedTypes = 0;
    nRet = is_AutoParameter(hCam,
                            IS_AWB_CMD_GET_SUPPORTED_TYPES,
                            (void*)&nSupportedTypes,
                            sizeof(nSupportedTypes));
    if (nRet == IS_SUCCESS) {
        if ((nSupportedTypes & IS_AWB_COLOR_TEMPERATURE) != 0) {
            cout << "color temperature supported..." << endl;
            UINT nType = IS_AWB_COLOR_TEMPERATURE;
            nRet = is_AutoParameter(hCam,
                                    IS_AWB_CMD_SET_TYPE,
                                    (void*)&nType,
                                    sizeof(nType));
            if (nRet == IS_SUCCESS) {
                cout << "set awb type to color temperature..." << endl;
                if (nRet == IS_SUCCESS) {
                    UINT nEnable = IS_AUTOPARAMETER_ENABLE;
                    nRet = is_AutoParameter(hCam,
                                            IS_AWB_CMD_SET_ENABLE,
                                            (void*)&nEnable,
                                            sizeof(nEnable));
                    if (nRet == IS_SUCCESS) {
                        cout << "awb set to enable..." << endl;
                    }
                }
            }
        }
    }
    //
    // enable auto parameters
    //
    INT nEnable = IS_AUTOPARAMETER_ENABLE;
    nRet = is_AutoParameter(hCam,
                            IS_AES_CMD_SET_ENABLE,
                            &nEnable,
                            sizeof(nEnable));
    sleep(1);
    //
    // set the image properties
    //
    INT height = (INT)camSensorInfo.nMaxHeight;
    INT width = (INT)camSensorInfo.nMaxWidth;
    cout << "height = " << height
         << ", width = " << width << endl;
    char *ptrImgMem = NULL;     // pointer to start of active image memory
    int memId;                  // numerical identifier for active memory
    INT bitsPerPixel = 24;      // 8 bit per pixel/color channel
    //
    // intitialize camera memory
    //
    INT allocResult;
    allocResult = is_AllocImageMem(hCam, width, height,
                                   bitsPerPixel, &ptrImgMem, &memId);
    if (allocResult != IS_SUCCESS) {
        cout << "could not allocate image mem..." << endl;
        return 1;
    }
    usleep(delay);
    //
    // set allocated memory as active memory
    //
    INT setActiveResult = -1;
    setActiveResult = is_SetImageMem(hCam, ptrImgMem, memId);
    if (setActiveResult != IS_SUCCESS) {
        cout << "failed to set image mem active..." << endl;
        return 1;
    }
    //
    // create a matrix to hold the image data
    //
    Mat inputImage;          // the raw image data from camera
    uint imageFormat = CV_8UC3;   // 8/24 bit unsigned bayer image
    inputImage = Mat(Size(width, height), imageFormat);
    const int windowHeight = 500;
    cout << "calibrate camera? (y/n)" << endl;
    char calibrate = 'a';
    cin >> calibrate;
    if ('y' == calibrate) {
        cout << "use existing calibration images? (y/n)" << endl;
        char useExistingImages = 'a';
        cin >> useExistingImages;
        const int numCalImages = 10;
        if ('n' == useExistingImages) {
            //
            // grab the next available frame
            //
            INT frameGrabReturn = -1;
            cout << "creating calibration images..." << endl << endl;
            for (int index = 0; index < numCalImages; ++index) {
                cout << "press <space> to save image " << index + 1 << endl;
                while(1) {
                    frameGrabReturn = is_FreezeVideo(hCam, IS_WAIT);
                    if (frameGrabReturn == IS_SUCCESS) {
                        //
                        // link camera image buffer to OpenCV matrix
                        //
                        inputImage.data = (uchar *)ptrImgMem;
                        //
                        // show live image
                        //
                        displayImage("Input Image", inputImage, windowHeight, 0, 0);
                        //
                        // check if escape key pressed to terminate...
                        //
                        int waitReturn = -1;
                        waitReturn = waitKey(1);
                        if (waitReturn != -1) {
                            break;
                        }
                    }
                    else {
                        cout << "frame grab failed..." << endl;
                        return 1;
                    }
                }
                //
                // build up jpeg image data and write to file
                //
                cout << "saving file..." << endl;
                string fileName = "calImage";
                stringstream extra;
                extra << index;
                string number;
                extra >> number;
                fileName.append(number);
                saveImageToFile(fileName, inputImage);
            }
        }
        else {
            //
            // calibration code follows
            //
            // initialize the 3D object corner values
            //
            vector<Point3f> objectCorners;
            vector<Point2f> imageCorners;
            const int colCount = 9;
            const int rowCount = 6;
            const double squareSize = 31.75;  // size of chess board square
            double X = 0.0f;    // horizontal world coordinate
            double Y = 0.0f;    // vertical world coordinate
            for (int row = 0; row < rowCount; ++row) {
                for (int col = 0; col < colCount; ++col) {
                    objectCorners.push_back(Point3f(X, Y, 0.0f));
                    cout << "X = " << X << ", Y = " << Y << endl;
                    Y += squareSize;
                }
                Y = 0.0f;
                X += squareSize;
            }
            cout << "starting calibration loop" << endl;
            vector< vector<Point3f> > objectPoints;
            vector< vector<Point2f> > imagePoints;
            for (int index = 0; index < numCalImages; ++index) {
                //
                // build up jpeg image file name and read data from file
                //
                string fileName = "calImage";
                stringstream extra;
                extra << index;
                string number;
                extra >> number;
                fileName.append(number);
                fileName.append(".jpg");
                cout << "reading file: " << fileName << endl;
                Mat cornerImg = imread(fileName, IMREAD_COLOR);
                cvtColor(cornerImg, cornerImg, COLOR_BGR2GRAY);
                if (cornerImg.empty()) {
                    cout << "could not read image..." << endl;
                }
                else {
                    Size checkerSize(colCount, rowCount);
                    bool cornersFound = findChessboardCorners(cornerImg,
                                                              checkerSize,
                                                              imageCorners,
                                                              CALIB_CB_ADAPTIVE_THRESH
                                                              + CALIB_CB_NORMALIZE_IMAGE);
                    if (cornersFound) {
                        cornerSubPix(cornerImg, imageCorners, Size(11, 11), Size(-1, -1),
                                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                        cvtColor(cornerImg, cornerImg, COLOR_GRAY2BGR);
                        drawChessboardCorners(cornerImg,
                                              checkerSize,
                                              Mat(imageCorners),
                                              cornersFound);
                        string cornerWinName("Corner Image ");
                        cornerWinName.append(number);
                        displayImage(cornerWinName.c_str(), cornerImg, windowHeight, 0, 0);
                        //
                        // save the image and object corners
                        //
                        objectPoints.push_back(objectCorners);
                        imagePoints.push_back(imageCorners);
                    }
                    else {
                        cout << "could not find corners for image " << number << endl;
                    }
                }
            }
            //
            // calculate intrinsic matrix values and radial distortion coefficients
            //
            Mat cameraMatrix(3, 3, CV_64F);
            Mat distCoeffs(5, 1, CV_64F);
            vector<Mat> rVecs;
            vector<Mat> tVecs;
            double calRetVal = calibrateCamera (objectPoints,
                                                imagePoints,
                                                inputImage.size(),
                                                cameraMatrix,
                                                distCoeffs,
                                                rVecs,
                                                tVecs);
            cout << "calibration return value = " << calRetVal << endl;
            cout << "instrinsic matrix values:" << endl << endl;
            cout << "fx = " << cameraMatrix.at<double>(0, 0) << endl;
            cout << cameraMatrix.at<double>(0, 1) << endl;
            cout << "u0 = " << cameraMatrix.at<double>(0, 2) << endl;
            cout << cameraMatrix.at<double>(1, 0) << endl;
            cout << "fy = " << cameraMatrix.at<double>(1, 1) << endl;
            cout << "v0 = " << cameraMatrix.at<double>(1, 2) << endl;
            cout << cameraMatrix.at<double>(2, 0) << endl;
            cout << cameraMatrix.at<double>(2, 1) << endl;
            cout << cameraMatrix.at<double>(2, 2) << endl;
            cout << "distortion matrix values:" << endl << endl;
            cout << "k1 = " << distCoeffs.at<double>(0) << endl;
            cout << "k2 = "<< distCoeffs.at<double>(1) << endl;
            cout << "p1 = "<< distCoeffs.at<double>(2) << endl;
            cout << "p2 = "<< distCoeffs.at<double>(3) << endl;
            cout << "k3 = "<< distCoeffs.at<double>(4) << endl;
            //
            // save intrinsic parameters to yaml file
            //
            FileStorage fs("intrinsics.yaml", FileStorage::WRITE);
            if (fs.isOpened()) {
                fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
                fs.release();
            }
            else {
                cout << "Error: can not save the intrinsic parameters\n";
            }
            //
            // Wait for the user to press a key in any GUI window.
            //
            cvWaitKey(0);
            destroyAllWindows();
        }
    }
    //
    // assume camera is calibrated at this point
    //
    else {
        cout << "grab new images? (y/n): ";
        char grabNewImages = 'a';
        cin >> grabNewImages;
        string fileName;
        if ('y' == grabNewImages) {
            //
            // grab an image
            //
            INT frameGrabReturn = -1;
            cout << "grabbing raw images..." << endl << endl;
            const int numImages = 3;
            for (int index = 0; index < numImages; ++index) {
                if (index < 2) {
                    cout << "press <space> to save top image " << index + 1 << endl;
                    fileName = "topImage";
                    stringstream extra;
                    extra << index + 1;
                    string number;
                    extra >> number;
                    fileName.append(number);
                }
                else {
                    cout << "press <space> to save side image" << endl;
                    fileName = "sideImage";
                }
                while(1) {
                    frameGrabReturn = is_FreezeVideo(hCam, IS_WAIT);
                    if (frameGrabReturn == IS_SUCCESS) {
                        //
                        // link camera image buffer to OpenCV matrix
                        //
                        inputImage.data = (uchar *)ptrImgMem;
                        //
                        // show live image
                        //
                        displayImage("Input Image", inputImage, windowHeight, 0, 0);
                        //
                        // check if escape key pressed to terminate...
                        //
                        int waitReturn = -1;
                        waitReturn = waitKey(1);
                        if (waitReturn != -1) {
                            break;
                        }
                    }
                    else {
                        cout << "frame grab failed..." << endl;
                        return 1;
                    }
                }
                //
                // build up jpeg image data and write to file
                //
                cout << "saving file..." << endl;
                saveImageToFile(fileName, inputImage);
            }
        }
        else if ('n' != grabNewImages) {
            return 1;
        }
        else {
            cout << "using existing images..." << endl;
        }
        //
        // process side image
        //
        // convert to gray scale
        // look for contours
        // find max contour
        // measure height and width of max contour in pixels
        // apply scale to image
        // draw overlays
        // display and save results
        //
        // convert to grayscale
        //
        Mat sideImg = imread("sideImage.jpg");
        displayImage("Side Image", sideImg, windowHeight, 0, 0);
        //
        // Wait for the user to press a key.
        //
        cout << "hit any key to continue..." << endl;
        cvWaitKey(0);
        destroyAllWindows();
        //
        // convert to grayscale
        //
        uint imageFormat = CV_8U;   // 8 bit unsigned monochrome image
        Mat graySideImg = Mat(Size(width, height), imageFormat);
        cvtColor(sideImg, graySideImg, COLOR_BGR2GRAY);
        displayImage("Gray Side Image", graySideImg, windowHeight, 0, 0);
        fileName = "graySideImage";
        saveImageToFile(fileName, graySideImg);
        //
        // Wait for the user to press a key.
        //
        cout << "hit any key to continue..." << endl;
        cvWaitKey(0);
        destroyAllWindows();
        //
        // filter prior to edge detection, use small filter size to start
        // set the sigma values equal to each other for simplicity
        //
        Mat filterSideImg;
        int filterSize = 5;
        double sigmaValues = 25.0;
        bilateralFilter(graySideImg,
                        filterSideImg,
                        filterSize,
                        sigmaValues,
                        sigmaValues);
        displayImage("Filter Side Image", filterSideImg, windowHeight, 0, 0);
        fileName = "filterSideImage";
        saveImageToFile(fileName, filterSideImg);
        //
        // Wait for the user to press a key.
        //
        cout << "hit any key to continue..." << endl;
        cvWaitKey(0);
        destroyAllWindows();
        //
        // detect Canny edges
        //
        Mat cannySideImg;
        Canny(filterSideImg, cannySideImg, 50, 150);
        displayImage("Canny Side Image", cannySideImg, windowHeight, 0, 0);
        fileName = "cannySideImage";
        saveImageToFile(fileName, cannySideImg);
        //
        // Wait for the user to press a key.
        //
        cout << "hit any key to continue..." << endl;
        cvWaitKey(0);
        destroyAllWindows();
        //
        // dilate the image to eliminate smaller contours
        //
        Mat dilateSideImg;
        int iterations = 3;
        dilate(cannySideImg, dilateSideImg, Mat(), Point(-1,-1), iterations);
        displayImage("Dilate Side Image", dilateSideImg, windowHeight, 0, 0);
        fileName = "dilateSideImage";
        saveImageToFile(fileName, dilateSideImg);
        //
        // Wait for the user to press a key in any GUI window.
        //
        cout << "hit any key to continue..." << endl;
        cvWaitKey(0);
        destroyAllWindows();
        //
        // find the contours
        //
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours( dilateSideImg, contours, hierarchy, CV_RETR_TREE,
                      CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
        cout << "found " << contours.size() << " contours..." << endl;
        //
        // draw contours
        //
        Mat contourImg = Mat::zeros(graySideImg.rows,
                                    graySideImg.cols,
                                    graySideImg.type());
        //
        // find the largest contour
        //
        uint maxSize = 0;
        uint maxIndex = 0;
        for (uint index = 0; index < contours.size(); ++index) {
            cout << "contour " << index << " size = "
                 << contours[index].size() << endl;
            if (maxSize < contours[index].size()) {
                maxSize = contours[index].size();
                maxIndex = index;
            }
        }
        //
        // find the height and width of the largest contour
        //
        int xMin = 10000;
        int xMax = -1;
        int yMin = 10000;
        int yMax = -1;
        vector<Point>::iterator pointIt;
        for (pointIt = contours[maxIndex].begin();
             pointIt != contours[maxIndex].end();
             ++pointIt) {
            Point tempPoint = *pointIt;
            if (xMin > tempPoint.x) {
                xMin = tempPoint.x;
            }
            if (xMax < tempPoint.x) {
                xMax = tempPoint.x;
            }
            if (yMin > tempPoint.y) {
                yMin = tempPoint.y;
            }
            if (yMax < tempPoint.y) {
                yMax = tempPoint.y;
            }
        }
        cout << "y min = " << yMin
             << ", y max = " << yMax
             << ", x min = " << xMin
             << ", x max = " << xMax
             << endl;
        int contourHeightPix = yMax - yMin;
        int contourWidthPix = xMax - xMin;
        //
        // scale to engineering units
        //
        // triangulate:  focal length (pixels)/object width (pixels)
        //              = camera distance (units)/object width (units)
        //
        float contourHeight = (cameraDistance * (float)contourHeightPix)/(focalLength);
        cout << "contour height = " << contourHeight << endl;
        float contourWidth = (cameraDistance * (float)contourWidthPix)/(focalLength);
        cout << "contour width = " << contourWidth << endl;
        //
        // draw the largest contour on a black background
        //
        Scalar color(150, 150, 150);
        drawContours( contourImg, contours, maxIndex, color, FILLED, LINE_8, hierarchy );
        line(contourImg, Point(xMin, yMin), Point(xMin, yMax), color, 2, LINE_8);
        line(contourImg, Point(xMin, yMin), Point(xMax, yMin), color, 2, LINE_8);
        stringstream heightSS;
        heightSS << contourHeight;
        string heightStr;
        heightSS >> heightStr;
        putText(contourImg,
                heightStr,
                Point(xMin - 400, yMin + (yMax - yMin)/2),
                FONT_HERSHEY_SIMPLEX,
                2,
                color,
                2);
        stringstream widthSS;
        widthSS << contourWidth;
        string widthStr;
        widthSS >> widthStr;
        putText(contourImg,
                widthStr,
                Point(xMin + (xMax - xMin)/2, yMin - 100),
                FONT_HERSHEY_SIMPLEX,
                2,
                color,
                2);
        displayImage("Contour Side Image", contourImg, windowHeight, 0, 0);
        fileName = "contourSideImage";
        saveImageToFile(fileName, contourImg);
        //
        // Wait for the user to press a key.
        //
        cout << "hit any key to continue..." << endl;
        cvWaitKey(0);
        destroyAllWindows();
        //
        // detect and compute features
        //
        Mat perspImg1 = imread("topImage1.jpg");
        Mat grayImg1 = Mat(Size(width, height), imageFormat);
        //
        // SIFT and SURF require grayscale image inputs
        //
        cvtColor(perspImg1, grayImg1, COLOR_BGR2GRAY);
        Mat perspImg2 = imread("topImage2.jpg");
        Mat grayImg2 = Mat(Size(width, height), imageFormat);
        cvtColor(perspImg2, grayImg2, COLOR_BGR2GRAY);
        vector<KeyPoint> keypts1;
        vector<KeyPoint> keypts2;
        Mat desc1;
        Mat desc2;
        //
        // detect keypoints and extract SIFT Descriptors
        //
        cout << "detecting keypoints..." << endl;
        Ptr<Feature2D> sift = xfeatures2d::SIFT::create();
        sift->detectAndCompute(grayImg1, Mat(), keypts1, desc1);
        sift->detectAndCompute(grayImg2, Mat(), keypts2, desc2);
        //
        // display keypoints
        //
        cout << "drawing keypoints..." << endl;
        Mat siftKeyptImg1;
        Mat siftKeyptImg2;
        drawKeypoints(perspImg1, keypts1, siftKeyptImg1,
                      Scalar(255,255,255),
                      DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        drawKeypoints(perspImg2, keypts2, siftKeyptImg2,
                      Scalar(255,255,255),
                      DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //
        // match descriptors
        //
        BFMatcher matcher(NORM_L2, true);
        vector<DMatch> matches;
        cout << "matching feature descriptors..." << endl;
        matcher.match(desc1, desc2, matches, Mat());
        cout << "drawing matches..." << endl;
        Mat matchesImg;
        drawMatches(perspImg1, keypts1, perspImg2, keypts2, matches, matchesImg);
        //
        // display the keypoints and matches
        //
        displayImage("First Image Keypoints", siftKeyptImg1, windowHeight, 300, 300);
        displayImage("Second Image Keypoints", siftKeyptImg2, windowHeight, 200, 200);
        displayImage("Matched Image Keypoints", matchesImg, windowHeight, 100, 100);
        //
        // build up jpeg image data and write to files
        //
        cout << "saving files..." << endl;
        fileName = "firstKeypointImage";
        saveImageToFile(fileName, siftKeyptImg1);
        fileName = "secondKeypointImage";
        saveImageToFile(fileName, siftKeyptImg2);
        fileName = "keypointMatchImage";
        saveImageToFile(fileName, matchesImg);
        //
        // Wait for the user to press a key in any GUI window.
        //
        cout << "hit any key to continue..." << endl;
        cvWaitKey(0);
        destroyAllWindows();
        //
        // use the matching points as input to finding the essential matrix
        // and recovering the camera pose.
        //
        // first, convert keypoints into Point2f type prior to calling
        // findEssentialMat and recoverPose
        // iterate over the matches and get the x and y pixel coordinates
        //
        vector<Point2f> points1;
        vector<Point2f> points2;
        vector<DMatch>::const_iterator matchIter;
        cout << "converting keypoints to points..." << endl;
        for (matchIter = matches.begin();
             matchIter != matches.end(); ++matchIter) {
            //
            // get the position of first image keypoints
            //
            int index1 = matchIter->queryIdx;
            float x = keypts1[index1].pt.x;
            float y = keypts1[index1].pt.y;
            points1.push_back(Point2f(x, y));
            //
            // get the position of second image keypoints
            //
            int index2 = matchIter->trainIdx;
            x = keypts2[index2].pt.x;
            y = keypts2[index2].pt.y;
            points2.push_back(Point2f(x, y));
        }
        //
        // calculate the essential matrix for the 2 images and
        // corresponding camera poses.
        // the mask values are 0 if the point pair is an outlier,
        // and 1 if the point pair is an inlier
        //
        Mat cameraMatrix;
        Mat distCoeffs;
        //
        // reading intrinsic parameters
        //
        cout << "reading camera intrinsics..." << endl;
        FileStorage fs("intrinsics.yaml", FileStorage::READ);
        if(!fs.isOpened()) {
            printf("Failed to open file\n");
            return -1;
        }
        else {
            fs["cameraMatrix"] >> cameraMatrix;
            fs["distCoeffs"] >> distCoeffs;
            fs.release();
        }
        Mat inlierMask;
        cout << "calculating essential matrix..." << endl;
        Mat essentialMat = findEssentialMat(points1,
                                            points2,
                                            cameraMatrix,
                                            RANSAC,
                                            .99,
                                            1.0,
                                            inlierMask);
        //
        // redraw the matches with the inlier mask
        //
        Mat inlierMatchesImg;
        cout << " drawing inlier matches..." << endl;
        drawMatches(perspImg1,
                    keypts1,
                    perspImg2,
                    keypts2,
                    matches,
                    inlierMatchesImg,
                    Scalar(200,200,200),
                    Scalar(255,255,255),
                    inlierMask);
        displayImage("Inlier Matched Keypoints", inlierMatchesImg, windowHeight, 0, 0);
        fileName = "inlierMatchImage";
        saveImageToFile(fileName, matchesImg);
        //
        // determine the rotation matrix, R, and translation matrix, T
        // relating the 2 camera positions, we can use the mask we just
        // created in finding the essential matrix
        //
        Mat rotationMat;
        Mat translationMat;
        int numGoodInliers = recoverPose(essentialMat,
                                         points1,
                                         points2,
                                         cameraMatrix,
                                         rotationMat,
                                         translationMat,
                                         inlierMask);


        if (numGoodInliers < 1) {
            cout << "pose recovery failed..." << endl;
        }
        else {
            cout << "number of good inliers = " << numGoodInliers << endl;
            cout << "size of inlier mask: " << inlierMask.size() << endl;
            cout << "size of match points 1: " << points1.size() << endl;
            cout << "size of match points 2: " << points2.size() << endl;
            //
            // using R and T from recoverPose, calculate the recification
            // mapping matrices and projection mapping matrices using the
            // calibrated method of Bouget's algorithm, which will also return
            // the reprojection matrix needed to find the depth map
            //
            Mat rectMap1;
            Mat rectMap2;
            Mat projMap1;
            Mat projMap2;
            Mat reprojectQ;
            Size imageSize = grayImg1.size();
            stereoRectify(cameraMatrix,
                          distCoeffs,
                          cameraMatrix,
                          distCoeffs,
                          imageSize,
                          rotationMat,
                          translationMat,
                          rectMap1,
                          rectMap2,
                          projMap1,
                          projMap2,
                          reprojectQ,
                          CALIB_ZERO_DISPARITY,
                          -1,
                          imageSize);
            //
            // create the rectified image pair
            //
            // first, create rectification mappings for each image
            // by calling iniUndistortRectifyMap, using values calculated
            // by stereoRectify and the camera distortion coefficients
            //
            Mat image1ReMap1;
            Mat image1ReMap2;
            initUndistortRectifyMap(cameraMatrix,
                                    distCoeffs,
                                    rectMap1,
                                    projMap1,
                                    imageSize,
                                    CV_16SC2,
                                    image1ReMap1,
                                    image1ReMap2);
            Mat image2ReMap1;
            Mat image2ReMap2;
            initUndistortRectifyMap(cameraMatrix,
                                    distCoeffs,
                                    rectMap2,
                                    projMap2,
                                    imageSize,
                                    CV_16SC2,
                                    image2ReMap1,
                                    image2ReMap2);

            //
            // now, apply the remapping to each image to produced a pair
            // of rectified images by calling remap
            //
            Mat rectImg1;
            remap(grayImg1, rectImg1, image1ReMap1, image1ReMap2, INTER_LINEAR);
            Mat rectImg2;
            remap(grayImg2, rectImg2, image2ReMap1, image2ReMap2, INTER_LINEAR);
            displayImage("Rectified Image 1", rectImg1, windowHeight, 100, 100);
            fileName = "rectifiedImage1";
            saveImageToFile(fileName, rectImg1);
            displayImage("Rectified Image 2", rectImg2, windowHeight, 200, 200);
            fileName = "rectifiedImage2";
            saveImageToFile(fileName, rectImg2);
            //
            // compute the disparity map using the remapped, rectified images
            //
            Mat disparity;
            int minDisparity = 0;
            int numDisparities = 32;
            int blockSize = 7;
            int P1 = 8;
            int P2 = 32;
            int dsp12MaxDiff = 1;
            int preFilterCap = 63;
            int uniquenessRatio = 10;
            int speckleWindowSize = 100;
            int speckleRange = 32;
            Ptr<StereoMatcher> pStereo = StereoSGBM::create(minDisparity,
                                                            numDisparities,
                                                            blockSize,
                                                            P1,
                                                            P2,
                                                            dsp12MaxDiff,
                                                            preFilterCap,
                                                            uniquenessRatio,
                                                            speckleWindowSize,
                                                            speckleRange,
                                                            StereoSGBM::MODE_HH);
            pStereo->compute(rectImg1, rectImg2, disparity);
            //
            // normalize the disparity map to 8 bit color depth 0 to 255
            //
            Mat normDisparity;
            disparity.convertTo(normDisparity, CV_8U);
            //
            // show the disparity map
            //
            displayImage("Disparity Map", normDisparity, windowHeight, 250, 100);
            fileName = "calibratedDisparityImage";
            saveImageToFile(fileName, normDisparity);
            //
            // Wait for the user to press a key in any GUI window.
            //
            cout << "hit any key to continue..." << endl;
            cvWaitKey(0);
            destroyAllWindows();
            //
            // next, try an uncalibrated approach
            //
            // find fundamental matix, prune the points to only inliers,
            // call stereoRectifyUnclibrated to compute the homography for
            // each image, warp the images using the homographies, then
            // calculate the disparity using the rectifed. warped images.
            //
            // find the fundamental matrix
            //
            Mat inlierMaskUnCal;
            Mat F = findFundamentalMat(points1,
                                       points2,
                                       inlierMaskUnCal,
                                       FM_RANSAC,
                                       3.,
                                       .99);
            //
            // loop over inlier matrix, save points that are inliers
            //
            vector<Point2f> inlierPts1;
            vector<Point2f> inlierPts2;
            cout << "pruning points to inliers..." << endl;
            MatConstIterator_<uchar> matIter = inlierMaskUnCal.begin<uchar>();
            vector<Point2f>::iterator pt2Iter = points2.begin();
            for (vector<Point2f>::iterator pt1Iter = points1.begin();
                pt1Iter != points1.end(); ++pt1Iter, ++pt2Iter,++matIter) {
                //
                // check the value of the mask
                //
                if (*matIter > 0) {
                    cout << "point1 values x: " << (*pt1Iter).x
                         << ", y: " << (*pt1Iter).y << endl;
                    inlierPts1.push_back(*pt1Iter);
                    cout << "point2 values x: " << (*pt2Iter).x
                         << ", y: " << (*pt2Iter).y << endl;
                    inlierPts2.push_back(*pt2Iter);
                }
            }
            cout << "pushed back " << inlierPts1.size() << " inlier points"
                 << endl;
            //
            // perform uncalibrated stereo rectification
            //
            Mat h1;
            Mat h2;
            stereoRectifyUncalibrated(inlierPts1,
                                      inlierPts2,
                                      F,
                                      imageSize,
                                      h1,
                                      h2,
                                      5);
            //
            // warp to rectify the images
            //
            // Rectify the images through warping
            //
            Mat rectWarpImg1;
            warpPerspective(grayImg1, rectWarpImg1, h1, imageSize);
            Mat rectWarpImg2;
            warpPerspective(grayImg2, rectWarpImg2, h2, imageSize);
            displayImage("Warped Rectified Image 1", rectWarpImg1, windowHeight, 300, 300);
            fileName = "warpRectifiedImage1";
            saveImageToFile(fileName, rectWarpImg1);
            displayImage("Warped Rectified Image 2", rectWarpImg2, windowHeight, 150, 150);
            fileName = "warpRectifiedImage2";
            saveImageToFile(fileName, rectWarpImg2);
            //
            // find and display the disparity
            //
            Mat unCalDisparity;
            pStereo->compute(rectWarpImg1, rectWarpImg2, unCalDisparity);
            //
            // normalize the disparity map
            //
            Mat normUncalDisp;
            unCalDisparity.convertTo(normUncalDisp, CV_8U);
            //
            // show the disparity map
            //
            displayImage("Warped Disparity Map", normUncalDisp, windowHeight, 250, 100);
            fileName = "uncalibratedDisparityImage";
            saveImageToFile(fileName, normUncalDisp);
            //
            // Wait for the user to press a key in any GUI window.
            //
            cout << "hit any key to continue..." << endl;
            cvWaitKey(0);
            destroyAllWindows();
            //
            // reproject to 3D
            //
            // first, create a depth map from the disparity matrix
            // using the equation:
            //                      disparity = (T * f) / Z
            //                      T = spacing between cameras
            //                      f = focal length of camera
            //                      Z = depth, same units as T
            //                      Z = (T * f) / disparity
            //
            // this will allow explicitly specifying the value of T
            //
            // the alternative is to use the reprojection matrix Q, if the
            // value of Tx can be adjusted to scale
            //
            // might need to modify the translation matrix from obtained
            // from recoverPose prior to calling stereoRectify
            //
            Mat calXYZ;
            reprojectImageTo3D(disparity, calXYZ, reprojectQ, true);
            displayImage("3D Reprojection", calXYZ, windowHeight, 100, 200);
            fileName = "3DReprojectionImage";
            saveImageToFile(fileName, calXYZ);
            //
            // Wait for the user to press a key in any GUI window.
            //
            cout << "hit any key to continue..." << endl;
            cvWaitKey(0);
            destroyAllWindows();
            //
            // next, take a look at running SIFT again on the rectifed images
            //
            // iterate over the vector of points
            // find the x coordinate in each image for each point
            // calculate disparity as:
            //                          disparity = xl - xr
            //
            // first the calibrated, rectified images
            //
            vector<KeyPoint> rectKeypts1;
            vector<KeyPoint> rectKeypts2;
            Mat rectDesc1;
            Mat rectDesc2;
            cout << "detecting rectified keypoints..." << endl;
            sift->detectAndCompute(rectImg1, Mat(), rectKeypts1, rectDesc1);
            sift->detectAndCompute(rectImg2, Mat(), rectKeypts2, rectDesc2);
            //
            // display keypoints
            //
            cout << "drawing rectified keypoints..." << endl;
            Mat rectSiftKeyptImg1;
            Mat rectSiftKeyptImg2;
            drawKeypoints(rectImg1, rectKeypts1, rectSiftKeyptImg1,
                          Scalar(255,255,255),
                          DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            drawKeypoints(rectImg2, rectKeypts2, rectSiftKeyptImg2,
                          Scalar(255,255,255),
                          DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            //
            // match descriptors
            //
            BFMatcher rectMatcher(NORM_L2, true);
            vector<DMatch> rectMatches;
            cout << "matching rectified feature descriptors..." << endl;
            rectMatcher.match(rectDesc1, rectDesc2, rectMatches, Mat());
            cout << "drawing rectified matches..." << endl;
            Mat rectMatchesImg;
            drawMatches(rectImg1,
                        rectKeypts1,
                        rectImg2,
                        rectKeypts2,
                        rectMatches,
                        rectMatchesImg);
            //
            // display the keypoints and matches
            //
            displayImage("First Rectified Image Keypoints",
                         rectSiftKeyptImg1, windowHeight, 300, 300);
            displayImage("Second Rectfied Image Keypoints",
                         rectSiftKeyptImg2, windowHeight, 200, 200);
            displayImage("Matched Rectified Image Keypoints",
                         rectMatchesImg, windowHeight, 100, 100);
            //
            // filter matches according to whether they are horizontal or not
            // use a comparison threshold for the y coordinate, setting a
            // value in a matches mask
            //
            // while in this loop, store the corresponding disparity value
            // into the good disparity matrix
            //
            Mat goodDisparity = Mat::zeros(rectImg2.rows,
                                           rectImg2.cols,
                                           rectImg2.type());
            Mat depthMap = rectImg2.clone();
            float deltaY = 10.0;
            vector<DMatch>::iterator matchIter;
            vector<char> matchesMask;
            for (matchIter = rectMatches.begin();
                matchIter != rectMatches.end(); ++matchIter) {
                int index1 = matchIter->queryIdx;
                float x1 = rectKeypts1[index1].pt.x;
                float y1 = rectKeypts1[index1].pt.y;
                cout << "x1 = " << x1 << ", y1 = " << y1 << endl;
                int index2 = matchIter->trainIdx;
                float x2 = rectKeypts2[index2].pt.x;
                int row = (int)round(x2);
                float y2 = rectKeypts2[index2].pt.y;
                int col = (int)round(y2);
                cout << "x2 = " << x2 << ", y2 = " << y2 << endl << endl;
                //
                // if y values are essentiall equal, the line is hroizontal
                // and will be considered an epipolar line
                //
                if (abs(y1 - y2) < deltaY) {
                    matchesMask.push_back(1);
                    float disp = abs(x2 - x1);
                    float depth = (cameraSpacing * focalLength) / disp;
                    cout << "row = " << row << ", col = " << col
                         << ", disparity (pixels) = " << disp
                         << ", depth (in) = " << depth << endl;
                    goodDisparity.at<uchar>(row, col) = (int)disp;
                    Scalar white = Scalar(255, 255, 255);
                    circle(goodDisparity, Point( row, col ), 5, white, 3, LINE_8, 0);
                    circle(depthMap, Point( row, col ), 3, white, 2, 8, 0);
                    stringstream textSS;
                    textSS << depth;
                    string text;
                    textSS >> text;
                    putText(depthMap,
                            text,
                            Point(row, col),
                            FONT_HERSHEY_SIMPLEX,
                            1,
                            white,
                            1);
                }
                else {
                    matchesMask.push_back(0);
                }
            }
            //
            // redraw the good matches
            //
            Mat goodMatchesImg;
            cout << " drawing good rectified matches..." << endl;
            drawMatches(rectImg1,
                        rectKeypts1,
                        rectImg2,
                        rectKeypts2,
                        rectMatches,
                        goodMatchesImg,
                        Scalar(200,200,200),
                        Scalar(255,255,255),
                        matchesMask);
            displayImage("Good Rectified Matched Keypoints", goodMatchesImg, windowHeight, 0, 0);
            fileName = "goodRectifiedMatchImage";
            saveImageToFile(fileName, goodMatchesImg);
            displayImage("Good Disparity Map", goodDisparity, windowHeight, 200, 200);
            fileName = "goodDisparityMap";
            saveImageToFile(fileName, goodDisparity);
            displayImage("Good Depth Map", depthMap, windowHeight, 300, 300);
            fileName = "goodDepthMap";
            saveImageToFile(fileName, depthMap);
            //
            // build up jpeg image data and write to files
            //
            cout << "saving files..." << endl;
            string fileName = "firstRectifiedKeypointImage";
            saveImageToFile(fileName, rectSiftKeyptImg1);
            fileName = "secondRectifiedKeypointImage";
            saveImageToFile(fileName, rectSiftKeyptImg2);
            fileName = "rectifiedKeypointMatchImage";
            saveImageToFile(fileName, rectMatchesImg);
        }
        //
        // Wait for the user to press a key in any GUI window.
        //
        cout << "hit any key to exit..." << endl;
        cvWaitKey(0);
    }
    //
    // free up the memory allocated for the camera buffer
    //
    cout << "uninitializing..." << endl;
    is_FreeImageMem(hCam, ptrImgMem, memId);
    //
    // release the camera handle
    //
    is_ExitCamera(hCam);
    destroyAllWindows();

    return 0;
}

void displayImage(const char *winName, Mat image, int windowSize, int xOffset, int yOffset)
{
    int width = image.cols;
    int height = image.rows;
    namedWindow(winName, WINDOW_NORMAL | WINDOW_KEEPRATIO);
    resizeWindow(winName, (windowSize * width)/height, windowSize);
    moveWindow(winName, xOffset, yOffset);
    imshow(winName, image);
}

void saveImageToFile(string fileName, Mat image)
{
    fileName.append(".jpg");
    vector<int> compression_params;
    compression_params.push_back(IMWRITE_JPEG_QUALITY);
    compression_params.push_back(95);
    try {
        imwrite(fileName, image, compression_params);
    }
    catch (Exception& ex) {
        fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
    }
}
