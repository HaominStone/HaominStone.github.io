
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <math.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <eigen3/Eigen/Eigenvalues>
#include <opencv2/core/eigen.hpp>
#include <sys/types.h>
#include <sys/stat.h>

#include "../bag_of_words/bow_tools.hpp"
#include "../loop_closure/LC_tools.hpp"
#include "../vo/VO_tools.hpp"
#include "../ba/BA_tools.hpp"

using namespace std;
using namespace cv;
using namespace gtsam;

Eigen::VectorXd K(9);

int main()
{

    Ptr<DescriptorMatcher> matcher_b(new FlannBasedMatcher);
    //create Sift feature point extracter
    Ptr<FeatureDetector> features(new cv::xfeatures2d::SiftFeatureDetector);
    //create Sift descriptor extractor
    Ptr<DescriptorExtractor> descriptors(new cv::xfeatures2d::SiftDescriptorExtractor);

    Ptr<BOWImgDescriptorExtractor> bowDE(new BOWImgDescriptorExtractor(descriptors, matcher_b));
    VO solver("trajectory.txt","../../data/rgbd_dataset_freiburg2_desk/rgb/");
    solver.SetK(525,525,319.5,239.5);

    Mat dictionary;
    build_dictionary(dictionary);
    bowDE->setVocabulary(dictionary);

    map<int, Mat> index_features;
    map<int, vector<KeyPoint>> index_points;
    map<int, Mat> index_descriptors;
    int last_loop = 0;

    NonlinearFactorGraph graph;
    noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.2));

    Values initialEstimate;

    vector<Eigen::Vector3f> all_pose;
    vector<Eigen::Vector3f> all_pose_pre;
    vector<Eigen::Matrix3f> all_R;
    vector<Eigen::Vector3f> all_t;
    vector<Eigen::Matrix3f> after_R;
    vector<Eigen::Vector3f> after_t;
    vector<Eigen::Vector2f> pre_all_inliers1;
    vector<Eigen::Vector2f> pre_all_inliers2;
    vector<vector<Eigen::Vector2f> >  all_2d_inliers;
    vector<map<int, int> >  all_indexes;
    vector<Eigen::Vector3f> all_3d_points;
    map<int, vector<Eigen::Vector2f> > all_frames;
    Eigen::Vector3f inital_pos;
    inital_pos << 0.0, 0.0, 0.0;

    int step_size = 10;
    K = solver.get_K_();
    int file_size = solver.read_file_names();
    Ptr<Feature2D> f2d = xfeatures2d::SIFT::create(0,3,0.04,10,1.6);

    int index = 0;

    for (int files = 0; files < file_size - step_size;
     files+=step_size)
    {
        solver.solve_once(all_R,
                        all_t,
                        pre_all_inliers1,
                        pre_all_inliers2,
                        all_2d_inliers,
                        all_indexes,
                        index_points,
                        index_descriptors,
                        f2d,
                        index,
                        files);

        if (all_R.size() == 9)
        {
            //relative to absolute
            for (int i = 1; i < 9; i++)
            {
                Eigen::Matrix3f R_tmp = all_R[i]*all_R[i-1];
                Eigen::Vector3f t_tmp = all_R[i]*all_t[i-1]+all_t[i];
                all_R[i] = R_tmp;
                all_t[i] = t_tmp;
            }
            get_all_3d_landmarks(all_2d_inliers,all_indexes,all_3d_points,all_frames,all_R,all_t);
            optimization(all_3d_points,all_frames,all_R,all_t,after_R,after_t);
            all_R.clear();
            all_t.clear();
            all_frames.clear();
            all_indexes.clear();
            all_2d_inliers.clear();
            all_3d_points.clear();
            all_pose_pre.push_back(inital_pos);
            for (int i = 0; i < 9; i++)
            {
                Eigen::Vector3f new_pos;
                new_pos = after_R[i]*inital_pos +after_t[i];
                all_pose.push_back(new_pos);
                all_pose_pre.push_back(new_pos);
            }

            inital_pos = all_pose[8];
            if (index == 9)
            {
                noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.3));
                graph.emplace_shared<PriorFactor<Point3> >(0, Point3(0, 0, 0), priorNoise);
                initialEstimate.insert(0, Point3(0.0, 0.0, 0.0));
                Mat bowDescriptor;
                vector<KeyPoint> keypoints;
                bowDE->compute(index_descriptors[0], bowDescriptor);
                index_features[0] = bowDescriptor;
            }
            for (int i = 0; i < all_pose.size(); i++)
            {
                Eigen::Vector3f trans;
                trans = all_pose[i]- all_pose_pre[i];
                graph.emplace_shared<BetweenFactor<Point3> >(index-9+i, index-8+i, Point3(trans[0], trans[1], trans[2]), model);
                initialEstimate.insert(index-8+i, Point3(all_pose[i][0], all_pose[i][1], all_pose[i][2]));
                Mat bowDescriptor;
                bowDE->compute(index_descriptors[index-8+i], bowDescriptor);
                index_features[index-8+i] = bowDescriptor;
                if ((index-8+i) - last_loop >= 20)
                {
                    int frame_looped = find_loop(index_features);
                    if (frame_looped != index_features.size())
                    {
                        Mat R_loop, t_loop;
                        geometric_verification(R_loop, t_loop, K, index_points, index_descriptors, frame_looped,index-8+i);
                        Mat pose_loop = (Mat_<double>(3,1) << all_pose[i][0], all_pose[i][1], all_pose[i][2]);
                        Mat pose_looped = R_loop*pose_loop+t_loop;
                        Mat trans_loop = pose_loop - pose_looped;
                        graph.emplace_shared<BetweenFactor<Point3> >(index-8+i, frame_looped, Point3(trans_loop.at<double>(0,0), trans_loop.at<double>(1,0), trans_loop.at<double>(2,0)), model);
                        GaussNewtonParams parameters;
                        parameters.relativeErrorTol = 1e-5;
                        parameters.maxIterations = 100;
                        GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
                        Values result = optimizer.optimize();
                        last_loop = index-8+i;
                    }
                }
            }
            after_R.clear();
            after_t.clear();
            all_pose.clear();
            all_pose_pre.clear();

        }

    }
    return 0;
}
