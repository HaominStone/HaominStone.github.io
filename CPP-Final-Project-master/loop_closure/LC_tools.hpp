/**
 * @file LC_tools.hpp
 * @brief This file is the handy functions for LC.
 */
#ifndef LC_tools_hpp
#define LC_tools_hpp
#include <stdlib.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <eigen3/Eigen/Eigenvalues>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
using namespace cv;
/**
 * dissimilar() Output the qualtified dissimilarity between to input matrix.
 * @param[in] first The first input matrix.
 * @param[in] second The second input matrix.
 * @return Return The number indicates the qualtified dissimilarity.
 */
float dissimilar(Mat & first, Mat & second);
/**
 * find_loop() Detecting which position in the index vector which indicates a loop has finished.
 * @param[in] index_features The feature matrix and its corresponding index.
 * @return Return The index indicates the end of a loop.
 */
int find_loop(std::map<int, Mat> & index_features);
/**
 * geometric_verification() Align the two frame given be their index.
 * @param[in] R_loop The rotation matrix between them.
 * @param[in] t_loop The translation matrix between them.
 * @param[in] K The intrinsic parameter matrix.
 * @param[in] index_points All the feature point with their indexs.
 * @param[in] index_descriptors All the feature descriptors with their index.
 * @param[in] index_first The index of the first frame.
 * @param[in] index_second The index of the second frame.
 * @return Return The index indicates the end of a loop.
 */
void geometric_verification(Mat& R_loop, Mat& t_loop, Eigen::VectorXd& K, std::map<int, std::vector<KeyPoint>>& index_points,std::map<int, Mat>& index_descriptors,int index_first,int index_second);
#endif /* LC_tools_hpp */
