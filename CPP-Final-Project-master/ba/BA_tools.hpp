/**
 * @file BA_tools.hpp
 * @brief Some tools that will be used in the bundle adjustment.
 */

#ifndef BA_tools_hpp
#define BA_tools_hpp


#include <stdlib.h>
#include <math.h>
#include <cfloat>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <thread>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
/**
 *get_world_point() Get the predicted position of the 3d point

 * @param[in] point1 The position of the first object point
 * @param[in] point2 The position of the second object point
 * @param[in] P1 The external parameter of the first camera
 * @param[in] P2 The external parameter of the second camera
 * @param[out] X The predicted position of the 3d point
 * @retval NULL void output
 */

void get_world_point(Eigen::Vector2f point1, Eigen::Vector2f point2, Eigen::MatrixXf P1, Eigen::MatrixXf P2, Eigen::VectorXf &X);
/**
 *find_corrospondence() Be used to find the corresponding inliner point
  in pre-found inlier and new ones.
 * @param[in] pre_all_inliers1 The matrix which contains pre-found inliers of first camera.
 * @param[in] pre_all_inliers2 The matrix which contains pre-found inliers of second camera.
 * @param[in] all_inliers1 The new inliers matrix of first camera.
 * @param[in] all_inliers2 The new inliers matrix of second camera.
 * @param[in] indexes_a The map which is used to record the mapping between two matrixs' element
 * @return NULL void output
 */
void find_corrospondence(std::vector<Eigen::Vector2f> & pre_all_inliers1,
                       std::vector<Eigen::Vector2f> & pre_all_inliers2,
                       std::vector<Eigen::Vector2f> & all_inliers1,
                       std::vector<Eigen::Vector2f> & all_inliers2,
                       std::map<int, int> & indexes_a);
/**
 * get_all_3d_landmarks() The function uses known rotation and transition matrix 
   also the 2d position of landmark to derive the 3d position of landmarks.
 * @param[in] all_2d_inliers The position of all the inlier point of landmarks
 * @param[in] all_indexes The map imply the mapping between corresponding index in preinlier and new ones
 * @param[in] all_3d_points The vector of all the 3D point we derive
 * @param[in] all_frames All the frames and its corresponding index
 * @param[in] all_R The rotation matrix between each two frame
 * @param[in] all_t The transition matrix of between each two frame
 * @return NULL void output
 */
void get_all_3d_landmarks(std::vector<std::vector<Eigen::Vector2f> >  & all_2d_inliers,
                          std::vector<std::map<int, int> >  & all_indexes,
                          std::vector<Eigen::Vector3f> & all_3d_points,
                          std::map<int, std::vector<Eigen::Vector2f> > & all_frames,
                          std::vector<Eigen::Matrix3f> & all_R,
                          std::vector<Eigen::Vector3f> & all_t);
/**
 *optimization() The main part of bundle adjustment which use ceres to realize.

 * @param[in] all_3d_points The vector of all the 3D point we derive
 * @param[in] all_frames All the frames and its corresponding index
 * @param[in] all_R The rotation matrix between each two frame
 * @param[in] all_t The transition matrix of between each two frame
 * @param[in] after_R The rotation matrix after optimization
 * @param[in] after_t The transition matrix after optimization
 * @return NULL void output
 */
void optimization(std::vector<Eigen::Vector3f> & all_3d_points,
                std::map<int, std::vector<Eigen::Vector2f> > & all_frames,
                std::vector<Eigen::Matrix3f> & all_R,
                std::vector<Eigen::Vector3f> & all_t,
                std::vector<Eigen::Matrix3f> & after_R,
                std::vector<Eigen::Vector3f> & after_t);
/**
 *@brief SnavelyReprojectionError The structure to record the parameters of error in the reprojection process.  
 */

struct SnavelyReprojectionError {
  /**
   * SnavelyReprojectionError() The constructor of the structure.
   * @brief Pass the parameters into the structure.
   * @param[in] observed_x  The x-axis position of the observation point 
   * @param[in] observed_y  The y-axis position of the observation point 
   * @return NULL void output
   */
    SnavelyReprojectionError(double observed_x, double observed_y)
    : observed_x(observed_x), observed_y(observed_y) {}
    template <typename T>
    /**
     * operator() Creating a new autodiffcostfunction for the observed position.
     * @param[in] camera  The position of the camera.
     * @param[in] point  The position of the predicted 3D point.
     * @param[in] K  The intrinsic parameter matrix.
     * @param[in] residuals  The error between real point and prediction.
     * @return true If the algorithm runs successfully.
     */    
    bool operator()(const T* const camera,
                    const T* const point,
                    const T* const K,
                    T* residuals) const {

        T p[3];
        ceres::QuaternionRotatePoint(camera, point, p);

        p[0] += camera[4];
        p[1] += camera[5];
        p[2] += camera[6];

        T xp = p[0] / p[2];
        T yp = p[1] / p[2];

        const T& fx = K[0];
        const T& fy = K[1];
        const T& cx = K[2];
        const T& cy = K[3];
        T predicted_x = fx * xp+cx;
        T predicted_y = fy * yp+cy;

        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }
    /**
     * Create() Creating a new autodiffcostfunction for the observed position.
     * @param[in] observed_x  The x-axis position of the observation point 
     * @param[in] observed_y  The y-axis position of the observation point 
     * @return Return the cost function of specific position .
     */
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError,2,7,3,4>(
                                                                                  new SnavelyReprojectionError(observed_x, observed_y)));
    }
    double observed_x;
    double observed_y;
};
#endif /* BA_tools_hpp */
