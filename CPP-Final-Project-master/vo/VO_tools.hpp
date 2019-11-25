/**
 * @file VO_tools.hpp
 * @brief This file is mainly to realize the visual odometer which will be used to judge the error between critical frames.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp> 
#include <opencv2/xfeatures2d.hpp>
#include <eigen3/Eigen/Eigenvalues>
#include <opencv2/core/eigen.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include "../ba/BA_tools.hpp"
using namespace std;
using namespace cv;
/**
 *@brief The main structure for VO algorithm, which contains the parameters
  we will use in the VO algorithm.
 */
class VO
{
private:
    /**
    *@brief The step size for take critical frames.
    */
    int step_size;

    /**
    *@brief The path to save the final result.
    */
    string store_path;

    /**
    *@brief The path we get data from.
    */
    string data_path;

    /**
    *@brief The internal parameter matrix.
    */
    Mat K;

    /**
    *@brief The eigen form of internal parameter matrix k.
    */
    Eigen::VectorXd K_;
    
    /**
    *@brief The parameter waiting to be used.
    */
    vector<string> vector_of_files;

    vector<Mat> Ps;
public:
    /**
    * VO() The constuctor of class VO.
    * VO() Pass store_path_, data_path_, step_size_ to the corresponding private parameter.
    * @param[in] store_path_ The value to be pass to store_path
    * @param[in] data_path_ The value to be pass to data_path
    * @param[in] step_size_ The value to be pass to step_size
    */
    VO(string data_path_ = "", string store_path_ = "", int step_size_ = 10);
    /**
    * ~VO() The destuctor of class VO.
    */
    ~VO();
    /**
     * SetK() This function which is used to pass the parameters in to the internal parameter matrix.
     * @param[in] focal_x The position of focal on x-axis.
     * @param[in] focal_y The position of focal on y-axis.
     * @param[in] principal_x The position of camera center on x-axis.
     * @param[in] principal_y The position of camera center on y-axis.
     * @retval NULL void output
     */
    void SetK(double focal_x,double focal_y, int principal_x, int principal_y);
    /**
     * read_file_names() This function is used to extract the names of files in a specific directory.
       the internal parameter matrix k will need and send it to k.
     * @return Return the number of file in this directory.
     */
    int read_file_names();
     /**
     * solve() The main structure of VO algorithm.
     * @param[in] store_trajectory Whether we need to store the trajectory parameters into a specific file
     * @param[in] need_plot Whether we need to plot the result
     * @param[in] all_R The vector which contains the rotation information between two step
     * @param[in] all_t The vector which contains the translation information between two step
     * @retval 1 Algorithm runs successfully
     * @retval 0 Fails
     */   
    void solve( std::vector<Eigen::Matrix3f>& all_R,
                std::vector<Eigen::Vector3f>& all_t,
                bool store_trajectory = false, 
                bool need_plot = false);
    /**
     * solve_once() Runs the vo algorithm once.
     * @param[in] all_R The vector which contains the rotation information between two step
     * @param[in] all_t The vector which contains the translation information between two step
     * @param[in] pre_all_inliers1 The vector which contains previous all inliers of the first frame.
     * @param[in] pre_all_inliers2 The vector which contains previous all inliers of the second frame.

     * @param[in] all_2d_inliers The inliers waiting to be stored.
     * @param[in] all_indexes The mapping between all the matched frames.
     * @param[in] index_points THe mapping between keypoint and its corresponing index.
     * @param[in] index_descriptors The mapping between the index and their corresponding feature descriptors.
     * @param[in] f2d The SIFT feature extractor.
     * @param[in] index The index of the frame that will be process in the solve percedure.
     * @param[in] files The index of file.
     * @return void
     */ 
    void solve_once(std::vector<Eigen::Matrix3f>& all_R,
                        std::vector<Eigen::Vector3f>& all_t,
                        vector<Eigen::Vector2f>& pre_all_inliers1,
                        vector<Eigen::Vector2f>& pre_all_inliers2,
                        vector<std::vector<Eigen::Vector2f>>&  all_2d_inliers,
                        vector<std::map<int, int> >&  all_indexes,
                        map<int, vector<KeyPoint>>& index_points,
                        map<int, Mat>& index_descriptors,
                        Ptr<Feature2D>& f2d,
                        int& index,
                        int files);
    /**
     * add_points() Be used to add new point into corresponding inlier vectors.
     * @param[in] POINT_1 The inlier which waiting to push into inlier vector as the first frame.
     * @param[in] POINT_2 The inlier which waiting to push into inlier vector as the second frame.
     * @param[in] mask The indicator to determine whether the point should be added.
     * @param[in] all_inliers1 The vector which contains all inliers of the first frame.
     * @param[in] all_inliers2 The vector which contains all inliers of the second frame.
     * @return void
     */     
    void add_points(vector<Point2d> & POINT_1,
                    vector<Point2d> & POINT_2,
                    Eigen::VectorXi & mask,
                    std::vector<Eigen::Vector2f> & all_inliers1,
                    std::vector<Eigen::Vector2f> & all_inliers2);
     /**
     * read_gt() This function reads the ground truth from specific file.
     * @return void
     */  
    void read_gt(std::string file_path);
      /**
     * get_K_() Get the K of the class.
     * @return The private parameter K
     */      
    Eigen::VectorXd get_K_();
};

