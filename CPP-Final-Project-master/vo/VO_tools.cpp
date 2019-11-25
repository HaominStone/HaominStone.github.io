/**
 * @file VO_tools.cpp
 * @brief This file is the main application of VO class.
 */

#include "VO_tools.hpp"

VO::VO(string data_path_, string store_path_, int step_size_):
        store_path(store_path_),data_path(data_path_),K_(9)
{
    step_size = step_size_;
}

void VO::SetK(double focal_x,double focal_y, int principal_x, int principal_y)
{
    K_ << focal_x,0,principal_x,0,focal_y,principal_y,0,0,1;
    K = (Mat_<double>(3,3) << K_(0), K_(1), K_(2), K_(3), K_(4), K_(5), K_(6), K_(7), K_(8));
}

int VO::read_file_names()
{
    if (data_path == "")
    {
        cout<<"No input path!"<<endl;
        return -1;
    }
    boost::filesystem::path my_path_b(data_path);
    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    vector<string> tm;
    for ( boost::filesystem::directory_iterator itr( my_path_b );
        itr != end_itr;
        ++itr )
    {
        cout<< itr->path().string()<<endl;
        tm.push_back(itr->path().string());
    }

    sort(tm.begin(), tm.end());
    for (auto i = tm.begin();i!=tm.end();i++)
    {
        cout << *i << endl;
        vector_of_files.push_back(*i);

    }
    return vector_of_files.size();
}

void VO::solve(std::vector<Eigen::Matrix3f>& all_R,std::vector<Eigen::Vector3f>& all_t,bool store_trajectory, bool need_plot)
{
    ofstream f1;
    if (store_trajectory)
    {
        f1 = ofstream("trajectory.txt");
    }
    Mat Initial_R = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Mat Inital_t = (Mat_<double>(3,1) << 0, 0, 0);
    Mat pose = (Mat_<double>(3,1) << 0, 0, 0);
    Ptr<Feature2D> f2d = xfeatures2d::SIFT::create(0,3,0.04,10,1.6);
    for (int files = 0; files < (int) vector_of_files.size() - step_size;//vector_of_files.size() - 1;
     files+=step_size)
    {
        Mat img_1 = imread(vector_of_files[files],IMREAD_GRAYSCALE);
        Mat img_2 = imread(vector_of_files[files+step_size],IMREAD_GRAYSCALE);
        Mat grayImg = cv::Mat::zeros(img_1.size(), CV_32FC1);
        cout << img_1.size()<<endl;




        vector<KeyPoint> keypoints_1, keypoints_2;
        f2d->detect(img_1, keypoints_1);
        f2d->detect(img_2, keypoints_2);

        //Calculate descriptors (feature vectors)
        Mat descriptors_1, descriptors_2;
        f2d->compute(img_1, keypoints_1, descriptors_1);
        f2d->compute(img_2, keypoints_2, descriptors_2);

        //Matching descriptor vector using BFMatcher
        FlannBasedMatcher matcher;
        vector<DMatch> matches_o;
        vector<Point2d> POINT_1;
        vector<Point2d> POINT_2;
        matcher.match(descriptors_1, descriptors_2, matches_o);

        //绘制匹配出的关键点
        std::vector< DMatch > matches;
        for( int i = 0; i < descriptors_1.rows; i++ )
        {
            double diffx = abs(keypoints_1[matches_o[i].queryIdx].pt.x - keypoints_2[matches_o[i].trainIdx].pt.x);
            double diffy = abs(keypoints_1[matches_o[i].queryIdx].pt.y - keypoints_2[matches_o[i].trainIdx].pt.y);
            if(diffx + diffy < 100)
            {
                matches.push_back( matches_o[i]);
                Point2d tmp_point_1(keypoints_1[matches_o[i].queryIdx].pt.x,keypoints_1[matches_o[i].queryIdx].pt.y);
                Point2d tmp_point_2(keypoints_2[matches_o[i].trainIdx].pt.x,keypoints_2[matches_o[i].trainIdx].pt.y);
                POINT_2.push_back(tmp_point_2);
                POINT_1.push_back(tmp_point_1);
            }
        }

    

        Mat E, R, t, mask;
        E = findEssentialMat(POINT_1, POINT_2, (K_[0]+K_[4])/2, Point2d(K_[2], K_[5]),RANSAC,0.9,8,mask);
        recoverPose(E, POINT_1, POINT_2, R, t, (K_[0]+K_[4])/2, Point2d(K_[2], K_[5]),mask);
        Mat P_1 = cv::Mat(3,4,Initial_R.type());
        Initial_R.copyTo(P_1.rowRange(0,3).colRange(0,3));
        Inital_t.copyTo(P_1.rowRange(0,3).col(3));
        P_1 = K*P_1;

        //Second camera
        Mat P_2 = cv::Mat(3,4,Initial_R.type());
        R.copyTo(P_2.rowRange(0,3).colRange(0,3));
        t.copyTo(P_2.rowRange(0,3).col(3));
        P_2 = K*P_2;
        Mat points3D = Mat_<double>(4, POINT_1.size());
        triangulatePoints(P_1, P_2, POINT_1, POINT_2, points3D);
        Eigen::VectorXd ttt(POINT_1.size());
        cv2eigen(mask,ttt);
        cout << ttt.sum()/POINT_1.size()<<endl;
//**********************************plotting!*****************************************
        if (need_plot)
        {
            for (int i = 0; i < (int)(POINT_1.size()); ++i)
            {
                if(ttt[i]==0)
                {
                    continue;
                }
                circle(img_1,Point(POINT_1[i]),3,Scalar(0),2,8,0);
                line(img_1,Point(POINT_1[i]),Point(POINT_2[i]),Scalar(0,255,255),1);
            }
            imshow("in",img_1);
            waitKey();
        }   

        if (store_trajectory)
        {
            pose = R*pose+t;
            Eigen::VectorXd pose_e(3);
            cv2eigen(pose,pose_e);
            f1 << pose_e.transpose()<<endl;
        }
        Eigen::Matrix3f R_e;
        Eigen::Vector3f t_e;
        cv2eigen(R,R_e);
        cv2eigen(t,t_e);
        all_R.push_back(R_e);
        all_t.push_back(t_e);

    }
}
void VO::add_points(vector<Point2d> & POINT_1,
                vector<Point2d> & POINT_2,
                Eigen::VectorXi & mask,
                std::vector<Eigen::Vector2f> & all_inliers1,
                std::vector<Eigen::Vector2f> & all_inliers2)
{
    int num_points = POINT_1.size();
    for(int i = 0; i < num_points; i++)
    {
        if (mask[i] == 1)
        {
            Eigen::Vector2f point_1;
            Eigen::Vector2f point_2;
            point_1 << POINT_1[i].x, POINT_1[i].y;
            point_2 << POINT_2[i].x, POINT_2[i].y;
            all_inliers1.push_back(point_1);
            all_inliers2.push_back(point_2);
        }
    }
}

void VO::solve_once(std::vector<Eigen::Matrix3f>& all_R,
                        std::vector<Eigen::Vector3f>& all_t,
                        vector<Eigen::Vector2f>& pre_all_inliers1,
                        vector<Eigen::Vector2f>& pre_all_inliers2,
                        vector<std::vector<Eigen::Vector2f>>&  all_2d_inliers,
                        vector<std::map<int, int>>&  all_indexes,
                        map<int, vector<KeyPoint>>& index_points,
                        map<int, Mat>& index_descriptors,
                        Ptr<Feature2D>& f2d,
                        int& index,
                        int files)
{
    vector<Eigen::Vector2f> all_inliers1;
    vector<Eigen::Vector2f> all_inliers2;
    Mat img_1 = imread(vector_of_files[files],IMREAD_GRAYSCALE);
    Mat img_2 = imread(vector_of_files[files+step_size],IMREAD_GRAYSCALE);


    //Detect the keypoints
    vector<KeyPoint> keypoints_1, keypoints_2;
    f2d->detect(img_1, keypoints_1);
    f2d->detect(img_2, keypoints_2);
    //Calculate descriptors (feature vectors)
    Mat descriptors_1, descriptors_2;
    f2d->compute(img_1, keypoints_1, descriptors_1);
    f2d->compute(img_2, keypoints_2, descriptors_2);
    //Matching descriptor vector using BFMatcher
    FlannBasedMatcher matcher;
    vector<DMatch> matches_o;
    vector<Point2d> POINT_1;
    vector<Point2d> POINT_2;
    matcher.match(descriptors_1, descriptors_2, matches_o);

    std::vector< DMatch > matches;
    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        double diffx = abs(keypoints_1[matches_o[i].queryIdx].pt.x - keypoints_2[matches_o[i].trainIdx].pt.x);
        double diffy = abs(keypoints_1[matches_o[i].queryIdx].pt.y - keypoints_2[matches_o[i].trainIdx].pt.y);
        if(diffx + diffy < 100)
        {
            matches.push_back( matches_o[i]);
            Point2d tmp_point_1(keypoints_1[matches_o[i].queryIdx].pt.x,keypoints_1[matches_o[i].queryIdx].pt.y);
            Point2d tmp_point_2(keypoints_2[matches_o[i].trainIdx].pt.x,keypoints_2[matches_o[i].trainIdx].pt.y);
            POINT_2.push_back(tmp_point_2);
            POINT_1.push_back(tmp_point_1);
        }
    }
    Mat E, R, t, mask;
    E = findEssentialMat(POINT_1, POINT_2, (K_[0]+K_[4])/2, Point2d(K_[2], K_[5]),RANSAC,0.9,8,mask);
    // cout<<mask<<endl;
    recoverPose(E, POINT_1, POINT_2, R, t, (K_[0]+K_[4])/2, Point2d(K_[2], K_[5]),mask);

    Eigen::VectorXi ttt(POINT_1.size());
    cv2eigen(mask,ttt);
    add_points(POINT_1, POINT_2, ttt, all_inliers1, all_inliers2);


    if(all_R.size() >= 1)
    {
        std::map<int,int> indexes;
        find_corrospondence(pre_all_inliers1, pre_all_inliers2, all_inliers1, all_inliers2, indexes);
        all_indexes.push_back(indexes);

    }
    all_2d_inliers.push_back(all_inliers1);
    all_2d_inliers.push_back(all_inliers2);
    pre_all_inliers1 = all_inliers1;
    pre_all_inliers2 = all_inliers2;
    Eigen::Matrix3f RR;
    Eigen::Vector3f tt;
    cv2eigen(R, RR);
    Eigen::Quaternionf Q2;//change into quaternion to output
    Q2 = RR;
    cv2eigen(t, tt);
    all_R.push_back(RR);
    all_t.push_back(tt);
    index++;
    std::cout << "index=" << index << "\n";

    if (index == 1)
    {
        index_points[0] = keypoints_1;
        index_descriptors[0] = descriptors_1;
    }
    index_points[index] = keypoints_2;
    index_descriptors[index] = descriptors_2;
}

Eigen::VectorXd VO::get_K_()
{
    return K_;
}

void VO::read_gt(std::string file_path)
{
    // std::string file_path = "../../data/rgbd_dataset_freiburg2_desk/rgb.txt";
    std::ifstream in(file_path);
    std::string time_stamp, file_name;
    if (!in.is_open())
    {
        std::cout << "Error opening file" << std::endl;
    }
    for (int i = 0; i < 3; i++ )
    {
        std::string tmp1, tmp2, tmp3;
        in >> tmp1 >> tmp2 >> tmp3;
    }


    std::string file_path_ground_truth = "../../data/rgbd_dataset_freiburg2_desk/groundtruth.txt";
    std::ifstream gt_in(file_path_ground_truth);
    std::string time,tx,ty,tz,qx,qy,qz,qw;

    if (!gt_in.is_open())
    {
        std::cout << "Error opening file" << std::endl;
    }
    for (int i = 0; i < 2; i++ )
    {
        gt_in >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    }

    float cur_time = 0;
    float cur_ground_time = 0;
    std::ofstream gt_postions("gt_positions.txt");
    int count = 0;

    while (!in.eof())
    {
        if (count % 20 != 0)
        {
            in >> time_stamp >> file_name;
            count ++;
            continue;
        }
        in >> time_stamp >> file_name;
        cur_time = atof(time_stamp.substr(6).c_str());


        while (!gt_in.eof())
        {
            gt_in >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            cur_ground_time = atof(time.substr(6).c_str());
            while (cur_ground_time - cur_time <= -0.01 )
            {
                gt_in >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
                cur_ground_time = atof(time.substr(6).c_str());
            }
            std::cout << "gt_time" << cur_ground_time << "\n";
            gt_postions << tx << " " << ty << " " << tz << "\n";
            break;
        }
        std::cout << "Count" << count << "\n";
        count ++;
    }
    in.close();
    gt_in.close();
    gt_postions.close();
}

VO::~VO()
{
}