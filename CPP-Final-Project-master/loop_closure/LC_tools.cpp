/**
 * @file LC_tools.cpp
 * @brief This file is the application for the handy tools of LC.
 */

#include "LC_tools.hpp"

using namespace cv;
using namespace std;
float dissimilar(Mat & first, Mat & second)
{
    int num_dimenssion = first.cols;
    float sum = 0;
    for (int i = 0; i < num_dimenssion; i++)
    {
        sum += abs(first.at<float>(0,i) - second.at<float>(0,i));
    }
    return sum;
}

int find_loop(map<int, Mat> & index_features)
{
    int num_frames = index_features.size();
    Mat target = index_features[num_frames-1];
    int num_samples = 150;
    int start = 0;
    int end = num_frames-100;
    int step = (end-start)/num_samples;;
    float threshold = 0.31;
    float diff = 100;
    int similar_frame = num_frames;


    while (step > 0)
    {
        int similar_frame_tmp = num_frames;
        for (int i = start; i < end;)
        {
            Mat sample = index_features[i];
            float diff_tmp = dissimilar(target, sample);
            if (diff_tmp < diff)
            {
                diff = diff_tmp;
                similar_frame_tmp = i;
            }
            i += step;
        }
        if (diff < threshold)
        {
            similar_frame = similar_frame_tmp;
            break;
        }
        else
        {
            if (similar_frame_tmp-step >= 0)
            {
                start = similar_frame_tmp-step;
            }
            else
            {
                start = similar_frame_tmp;
            }
            if (similar_frame_tmp+step < num_frames-50)
            {
                end = similar_frame_tmp+step;
            }
            else
            {
                end = num_frames-50;
            }
            step = (end-start)/num_samples;
        }
    }
    if (similar_frame != num_frames)
    {
        cout << "target=" << num_frames-1 << "\n";
        cout << "looped=" << similar_frame << "\n";
        cout << "similar=" << diff << "\n";
    }
    return similar_frame;
}

void geometric_verification(Mat& R_loop, Mat& t_loop, Eigen::VectorXd& K, map<int, vector<KeyPoint>>& index_points, map<int, Mat>& index_descriptors, int index_first,int index_second)
{
    vector<KeyPoint> key_1 = index_points[index_first];
    vector<KeyPoint> key_2 = index_points[index_second];
    Mat des_1 = index_descriptors[index_first];
    Mat des_2 = index_descriptors[index_second];
    FlannBasedMatcher matcher_loop;
    vector<DMatch> matches_loop;
    vector<Point2d> POINT_1_loop;
    vector<Point2d> POINT_2_loop;
    matcher_loop.match(des_1, des_2, matches_loop);
    for( int i = 0; i < des_1.rows; i++ )
    {
        double diffx = abs(key_1[matches_loop[i].queryIdx].pt.x - key_2[matches_loop[i].trainIdx].pt.x);
        double diffy = abs(key_1[matches_loop[i].queryIdx].pt.y - key_2[matches_loop[i].trainIdx].pt.y);
        if(diffx + diffy < 100)
        {
            matches_loop.push_back( matches_loop[i]);
            Point2d tmp_point_1(key_1[matches_loop[i].queryIdx].pt.x,key_1[matches_loop[i].queryIdx].pt.y);
            Point2d tmp_point_2(key_2[matches_loop[i].trainIdx].pt.x,key_2[matches_loop[i].trainIdx].pt.y);
            POINT_2_loop.push_back(tmp_point_2);
            POINT_1_loop.push_back(tmp_point_1);
        }
    }
    Mat E_loop, mask_loop;
    E_loop = findEssentialMat(POINT_2_loop, POINT_1_loop, (K[0]+K[4])/2, Point2d(K[2], K[5]),RANSAC,0.9,8,mask_loop);

    recoverPose(E_loop, POINT_2_loop, POINT_1_loop, R_loop, t_loop, (K[0]+K[4])/2, Point2d(K[2], K[5]),mask_loop);
}