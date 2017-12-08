/*

  Instructions for Ubuntu 16.04 with OpenCV 3.1:

  unzip the zip file
  run make
  run ./program1

In this assignment, you will be working directly with the image pixels to
modify an image similar to a “green screen” technique. Pixels of a selected
color will be replaced with pixels from a second image. For further
information on the general idea, see: http://en.wikipedia.org/wiki/Chroma_key

1. Read two images from the disk. (Use the default directory, which is the
same directory that the source code is in.) The images should be called
“foreground.jpg” and “background.jpg”.

2. Use a color histogram to find the most common color in the foreground
image. The histogram should be a three-dimensional matrix of integers.
(Note: don’t use the OpenCV methods for creating histograms.)
int dims[] = {size, size, size};		// size is a constant - the # of
                                        // buckets in each dimension
Mat hist(3, dims, CV_32S, Scalar::all(0));	// 3D histogram of integers
                                            // initialized to zero
To create the histogram, loop through the foreground image and assign each
pixel to a histogram bucket. That bucket should be incremented by one.
To decide which bucket to increment, you divide the color value by (256/size):
    int bucketSize = 256/size;
    int x = red / bucketSize;
    int y = green / bucketSize;
    int z= blue / bucketSize;
I have found that size = 4 works relatively well, but this may vary with
different images.

3. Find the histogram bin with the most “votes” by looping over all three
dimensions. If the bin with the most votes is [x, y, z], then the most common
color is approximately:
    int cRed = x * bucketSize + bucketSize/2;
    int cGreen = y * bucketSize + bucketSize/2;
    int cBlue = z * bucketSize + bucketSize/2;
This is similar to the color reduction in Chapter 2 of the OpenCV 2 book.

4. Replace every pixel in the foreground image that is close to the most
common color (within bucketSize/2 in all three bands) with the corresponding
pixel from the background image (same row and column, unless the background
image is too small). If the background image is too small, start over from the
start of the background image. (This can be accomplished by taking the
foreground row modulo the number of rows in the background image and similarly
for columns.)

5. Display the resulting image on the screen and save it to “”.

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
    // read 2 images from disk
    //
    Mat foregroundImage = imread("foreground.jpg");
    Mat backgroundImage = imread("background.jpg");
    namedWindow("Foreground Image");
    imshow("Foreground Image", foregroundImage);
    waitKey(0);
    cout << "number of channels = " << foregroundImage.channels() << endl;
    //
    // Use a color histogram to find the most common color in the foreground
    // image.
    //
    // create a 3D array to hold the histogram bins
    //
    // define the number of bins, i.e. the histogram matrix size, and
    // declare the histogram matrix, initialized to all 0
    //
    const int size = 4;
    int bucketSize = 256/size;
    int dims[] = {size, size, size};
    Mat histogram = Mat(3, dims, CV_32S, Scalar::all(0));
    cout << "histogram channels = " << histogram.channels() << endl;
    cout << "histogram size = " << histogram.size() << endl;
    //
    // loop over the foreground image, accessing each pixel in turn
    //
    int numRows= foregroundImage.rows;
    int numCols= foregroundImage.cols;
    for (int j = 0; j < numRows; j++) {
        for (int i = 0; i < numCols; i++) {
            //
            // for each color channel of each pixel, increment the histogram
            // bin that is closest to the value of the color channel
            //
            int red = foregroundImage.at<Vec3b>(j,i)[0];
            int green = foregroundImage.at<Vec3b>(j,i)[1];
            int blue = foregroundImage.at<Vec3b>(j,i)[2];
            int x = red / bucketSize;
            int y = green / bucketSize;
            int z= blue / bucketSize;
            histogram.at<int>(x, y, z) = histogram.at<int>(x, y, z) + 1;
        }
    }
    int maxVotes = 0;
    int xMax = 0;
    int yMax = 0;
    int zMax = 0;
    //
    // loop over every element in the histgram matix and locate the x, y, z
    // of the largest value, i.e. maximum number of votes
    //
    for (int k = 0; k < size; ++k) {
        for (int j = 0; j < size; ++j) {
            for (int i = 0; i < size; ++i) {
                int voteCount = histogram.at<int>(k, j, i);
                cout << "vote count = " << voteCount << endl;
                if ( histogram.at<int>(k, j, i) > maxVotes) {
                    maxVotes = histogram.at<int>(k, j, i);
                    xMax = k;
                    yMax = j;
                    zMax = i;
                }
            }
        }
    }
    //
    // assign most common color value
    //
    int cRed = xMax * bucketSize + bucketSize/2;
    int cGreen = yMax * bucketSize + bucketSize/2;
    int cBlue = zMax * bucketSize + bucketSize/2;
    cout << "cRed = " << cRed
         << ", xMax = " << xMax << endl
         << "cGreen = " << cGreen
         << ", yMax = " << yMax << endl
         << "cBlue = " << cBlue
         << ", zMax = " << zMax << endl;
    //
    // Replace every pixel in the foreground image that is close to the most
    // common color (within bucketSize/2 in all three bands) with the corresponding
    // pixel from the background image (same row and column, unless the background
    // image is too small). If the background image is too small, start over from the
    // start of the background image. (This can be accomplished by taking the
    // foreground row modulo the number of rows in the background image and similarly
    // for columns.)
    //
    // loop over the foreground image
    // compare each color value of each pixel to the most common color
    //
    cout << "foreground rows = " << foregroundImage.rows << endl
         << "background rows = " << backgroundImage.rows << endl
         << "foreground cols = " << foregroundImage.cols << endl
         << "background rows = " << backgroundImage.cols << endl;

    //
    // loop over the foreground image
    //
    // for each color channel of each pixel
    // compare the color channel value to the most common value
    // if the difference between the color channel value and the most common
    // value is less than bucketSize/2, replace the color channel value with
    // the corresponding pixel from the background image.
    //
    // check for reaching the end of each row and column of the background image
    // reset col index to first column if at a row end, reset row index to
    // first row if at bottom of column
    //
    numRows= foregroundImage.rows;
    int rowEnd = backgroundImage.rows;
    numCols= foregroundImage.cols;
    int colEnd = backgroundImage.cols;
    for (int j = 0, jj = 0; j < numRows; j++, jj++) {
        //
        // check for end of row
        //
        if(jj == rowEnd) {
            jj = 0;
        }
        for (int i = 0, ii = 0; i < numCols; i++, ii++) {
            //
            // check for bottom of column
            //
            if(ii == colEnd) {
                ii = 0;
            }
            //
            // for each color channel of each pixel, increment the histogram
            // bin that is closest to the value of the color channel
            //
            int red = foregroundImage.at<Vec3b>(j,i)[0];
            if ( abs(red - cRed) < bucketSize/2 ) {
                foregroundImage.at<Vec3b>(j,i)[0] = backgroundImage.at<Vec3b>(jj,ii)[0];
            }
            int green = foregroundImage.at<Vec3b>(j,i)[1];
            if ( abs(green - cGreen) < bucketSize/2 ) {
                foregroundImage.at<Vec3b>(j,i)[1] = backgroundImage.at<Vec3b>(jj,ii)[1];
            }
            int blue = foregroundImage.at<Vec3b>(j,i)[2];
            if ( abs(blue - cBlue) < bucketSize/2 ) {
                foregroundImage.at<Vec3b>(j,i)[2] = backgroundImage.at<Vec3b>(jj,ii)[2];
            }
            if(ii == colEnd) {
                ii = 0;
            }
        }
    }
    namedWindow("Output Image");
    imshow("Output Image", foregroundImage);
    waitKey(0);
    //
    // save image to file
    //
    vector<int> compression_params;
    compression_params.push_back(IMWRITE_JPEG_QUALITY);
    compression_params.push_back(95);
    try {
        imwrite("overlay.jpg", foregroundImage, compression_params);
    }
    catch (Exception& ex) {
        fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
        return 1;
    }

    return 0;
}
