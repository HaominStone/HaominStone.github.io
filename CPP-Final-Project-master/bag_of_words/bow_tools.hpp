/**
 * @file bow_tools.hpp
 * @brief This file contains some handy functions which will be used in bag of word algorithm.
 */
 #ifndef bow_tools_hpp
 #define bow_tools_hpp

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <eigen3/Eigen/Eigenvalues>
#include <opencv2/core/eigen.hpp>


#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
/**
 *build_dictionary() Be used to build up the visual dictionary

 * @param[out] dictionary The matrix we used to store the dictionary we get
 * @retval NULL void output
 */
void build_dictionary(Mat & dictionary);
#endif /* bow_tools_hpp */
