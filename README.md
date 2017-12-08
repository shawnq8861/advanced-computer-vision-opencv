# advanced-computer-vision-opencv
Advanced computer vision application examples, some using stored image files and others using live images, covering image processing, image conversion, clustering, chroma-key/cartoonification, SIFT feature detection and extraction, stereo image processing, camera calibration, and single camera disparity and depth recovery from structure from motion.

Dependencies - OpenCV 3.1 with opencv_contrib

Program 1 - color conversion, edge detection, filtering, and flipping.

Program 2 - color histogram binning, merging of backjground and foreground images similar to "green screen" method.

Program 3 - k-means clustering of image colors, replacement of original pixel values with clustered values, Mat traversing and custom, 5 channel data type.

Program 4 - 3D stereo imaging, recovering depth from single camera images at known baseline distance (camera spacing), SIFT feature detection and extraction, comparison of 3 different structure from motion approaches:

    a. traditional calibrated camera.
    b. uncalibrated camera, warp rectification.
    c. calibrated camera, SIFT feature epipolar line identification, custom disparity calculation.
