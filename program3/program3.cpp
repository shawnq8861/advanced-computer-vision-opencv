/*

  Instructions for Ubuntu 16.04 with OpenCV 3.1:

  unzip the zip file
  run make
  run ./program3

In this assignment, you will perform a simple image segmentation incorporating
the OpenCV kmeans method.

1. Examine the documentation for the OpenCV kmeans method.

2. Read an image called input.jpg. (Use the default directory, which is the
same directory that the source code is in.)

3. Use the GaussianBlur method to smooth the image using sigmaX = sigmaY = 3.0.
Allow the method to choose the size of the kernel using Size(0, 0).

4. Create a matrix of pixel signatures. The number of rows should correspond to
the number of pixels in the input image.  For each pixel, the signature should
be composed of five values, the row, column, red, green, and blue values. Use
the colors from the blurred image. The colors should be weighted 16 times as
much as the positions.

5. Call the kmeans method with your pixel signatures. For the supplied test
image, I have had relatively good results with four clusters, ten iterations
of the algorithm, and one attempt using the KMEANS_PP_CENTERS flag.

6. Recolor the input image with each pixel having the average color of the
cluster it is assigned to (both the assignment labels and the cluster centers
can be returned from the algorithm). Make sure to scale the color to the
original scale.

7. Display the image and write it to output.jpg.

*/

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

//
// typedef for a vector of 5 integers
//
typedef Vec<float, 5> Vec5f;

int main(int argc, char *argv[])
{
    //
    // open the image file
    //
    Mat inputImage = imread("input.jpg");
    namedWindow("Input Image");
    imshow("Input Image", inputImage);
    //
    // Wait for the user to press a key in the GUI window.
    //
    cvWaitKey(0);
    //
    // destroy the last window before displaying a new window
    //
    destroyWindow("Input Image");
    //
    // blur it
    //
    Mat blurImage;
    int kSize = 0;          // kernel height and width
    double sigma = 3.0;     // kernel standard deviation
    GaussianBlur(inputImage, blurImage, Size(kSize, kSize), sigma, sigma);
    namedWindow("Blurred Image");
    imshow("Blurred Image", blurImage);
    cvWaitKey(0);
    destroyWindow("Blurred Image");
    //
    // create a 1D matrix of pixel signatures
    //
    int numRows = blurImage.rows;
    int numCols = blurImage.cols;
    int matSize = numRows * numCols;
    Mat pixelSig = Mat(1, matSize, CV_32FC(5), Scalar::all(0));
    //
    // loop over the pixels in the blur image
    //
    // for each pixel of the blur image, store the following values in the
    // row, col, 16*red, 16*green, 16*blue
    //
    // the conversion from 2D to 1D indexing is:
    //          index = row*numCols + col
    //
    int weight = 16;
    for (int row = 0; row < numRows; row++) {
        for (int col = 0; col < numCols; col++) {
            //
            // for each color channel of each pixel, increment the histogram
            // bin that is closest to the value of the color channel
            //
            int red = blurImage.at<Vec3b>(row, col)[0];
            int green = blurImage.at<Vec3b>(row, col)[1];
            int blue = blurImage.at<Vec3b>(row, col)[2];
            int index = row * numCols + col;
            //
            // row
            //
            pixelSig.at<Vec5f>(index)[0] = (float)row;
            //
            // col
            //
            pixelSig.at<Vec5f>(index)[1] = (float)col;
            //
            // weighted red, green and blue
            //
            pixelSig.at<Vec5f>(index)[2] = (float)(red * weight);
            pixelSig.at<Vec5f>(index)[3] = (float)(green * weight);
            pixelSig.at<Vec5f>(index)[4] = (float)(blue * weight);
        }
    }
    //
    // call kmeans to cluster the pixel signature matrix
    //
    int clusterCount = 4;
    Mat bestLabels;
    int numIterations = 10;
    double epsilon = 1.0;
    int numAttempts = 1;
    Mat centers;
    kmeans(pixelSig,
           clusterCount,
           bestLabels,
           TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, numIterations, epsilon),
           numAttempts,
           KMEANS_PP_CENTERS,
           centers);

    //
    // loop over the blur image and recolor it based on the results of kmeans
    //
    // bestLabels contains the cluster that each pixel in pixelSig is
    // assigned to
    //
    // centers is a matrix of 4 Vec5f, each element contains the row and col
    // of the cluster center and the average values of the 3 color channels
    // the color channel values are scaled by the weighting factor
    //
    // the first 2 elemenst of each Vec5f are the row and column location
    // of the cluster
    //
    // the last 3 elements of the Vec5f are the average red, green, and blue
    // channel values for that cluster, scaled by the weighting factor
    //
    for (int row = 0; row < numRows; row++) {
        for (int col = 0; col < numCols; col++) {
            int index = row * numCols + col;
            //
            // get the cluster
            //
            int clusterNum = bestLabels.at<int>(index);
            //
            // index the color channels and re-weight each one
            //
            Vec5f centerVal = centers.at<Vec5f>(clusterNum);
            int red = (int)(centerVal[2] / weight);
            int green = (int)(centerVal[3] / weight);
            int blue = (int)(centerVal[4] / weight);
            blurImage.at<Vec3b>(row, col)[0] = red;
            blurImage.at<Vec3b>(row, col)[1] = green;
            blurImage.at<Vec3b>(row, col)[2] = blue;
        }
    }
    namedWindow("Clustered Blurred Image");
    imshow("Clustered Blurred Image", blurImage);
    cvWaitKey(0);
    destroyWindow("Clustered Blurred Image");
    //
    // save image to file
    //
    vector<int> compression_params;
    compression_params.push_back(IMWRITE_JPEG_QUALITY);
    compression_params.push_back(95);
    try {
        imwrite("output.jpg", blurImage, compression_params);
    }
    catch (Exception& ex) {
        fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
        return 1;
    }

    return 0;
}
