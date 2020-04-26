/*

  Instructions for Ubuntu 16.04 with OpenCV 3.1:

  unzip the zip file
  run make
  run ./program1


3. Test your installation using one of the simple “Hello World” programs on
the web or in the book. There are some versions that use precompiled headers,
but I prefer simpler code. Here is an example:

// OpenCV_Helloworld.cpp
    #include <cv.h>
    #include <highgui.h>
    using namespace cv;

int main(int argc, char *argv[]) {
    Mat image = imread("photo.jpg");

    namedWindow("Input Image");
    imshow("Input Image", image);

    // Wait for the user to press a key in the GUI window.
    cvWaitKey(0);

    return 0;
}

4. Modify your program to use OpenCV methods to:
    - flip the image horizontally (mirror-image)
    - convert it to greyscale
    - blur it
    - detect edges.

The following methods will be useful: flip, cvtColor, GaussianBlur,
Canny (edge detector). When you are done display the image on the screen
and save a version of it to disk as “output.jpg”.

Documentation for these methods can be found at the OpenCV website:
http://opencv.itseez.com/

For cvtColor, use CV_RGB2GRAY as the color code to convert from RGB to grayscale.
For blurring, use Size(7, 7) and sigmaX = sigmaY = 2.0. (Note that Size is an OpenCV type.)
For edge detection (Canny method), use threshold1 = 20 and threshold2 = 60.

*/

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    //
    // Part 3
    //
    Mat image = imread("lena.jpg");
    namedWindow("Input Image");
    imshow("Input Image", image);
    //
    // Wait for the user to press a key in the GUI window.
    //
    waitKey(0);
    //
    // destroy the last window before displaying a new window
    //
    destroyWindow("Input Image");
    //
    // Part 4
    //
    // - flip the image horizontally (mirror-image)
    //
    int flipCodeHorizontal = 1;
    Mat flipImage;
    flip(image, flipImage, flipCodeHorizontal);
    namedWindow("Flipped Image");
    imshow("Flipped Image", flipImage);
    waitKey(0);
    destroyWindow("Flipped Image");
    //
    // - convert it to greyscale
    //
    Mat grayImage;
    cvtColor(flipImage, grayImage, COLOR_RGB2GRAY);
    namedWindow("Grayscale Image");
    imshow("Grayscale Image", grayImage);
    waitKey(0);
    destroyWindow("Grayscale Image");
    //
    // - blur it
    //
    Mat blurImage;
    int kSize = 7;          // kernel height and width
    double sigma = 2.0;     // kernel standard deviation
    GaussianBlur(grayImage, blurImage, Size(kSize, kSize), sigma, sigma);
    namedWindow("Blurred Image");
    imshow("Blurred Image", blurImage);
    waitKey(0);
    destroyWindow("Blurred Image");
    //
    // - detect edges.
    //
    Mat edgeImage;
    double threshold1 = 20.0;
    double threshold2 = 60.0;
    Canny(blurImage, edgeImage, threshold1, threshold2);
    namedWindow("Edge Image");
    imshow("Edge Image", edgeImage);
    waitKey(0);
    destroyWindow("Edge Image");

    return 0;
}
