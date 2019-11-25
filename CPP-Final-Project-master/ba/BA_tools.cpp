/**
 * @file BA_tools.cpp
 * @brief This file is the main application of some function that BA algorithm may use.
 */

#include "BA_tools.hpp"


void get_world_point(Eigen::Vector2f point1, Eigen::Vector2f point2, Eigen::MatrixXf P1, Eigen::MatrixXf P2, Eigen::VectorXf &X)
{
    Eigen::Matrix3f K;
    K << 525,0,319.5,0,525,239.5,0,0,1;

    P1 = K*P1;
    P2 = K*P2;
    Eigen::MatrixXf A(4,4);
    A(0,0) = point1[0]*P1(2,0)-P1(0,0);A(0,1) = point1[0]*P1(2,1)-P1(0,1);A(0,2) = point1[0]*P1(2,2)-P1(0,2);A(0,3) = point1[0]*P1(2,3)-P1(0,3);
    A(1,0) = point1[1]*P1(2,0)-P1(1,0);A(1,1) = point1[1]*P1(2,1)-P1(1,1);A(1,2) = point1[1]*P1(2,2)-P1(1,2);A(1,3) = point1[1]*P1(2,3)-P1(1,3);
    A(2,0) = point2[0]*P2(2,0)-P2(0,0);A(2,1) = point2[0]*P2(2,1)-P2(0,1);A(2,2) = point2[0]*P2(2,2)-P2(0,2);A(2,3) = point2[0]*P2(2,3)-P2(0,3);
    A(3,0) = point2[1]*P2(2,0)-P2(1,0);A(3,1) = point2[1]*P2(2,1)-P2(1,1);A(3,2) = point2[1]*P2(2,2)-P2(1,2);A(3,3) = point2[1]*P2(2,3)-P2(1,3);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXf tmp(4);
    tmp << 0,0,0,1;
    X = Eigen::VectorXf(4);
    X = svd.matrixV()*tmp;
}

void find_corrospondence(std::vector<Eigen::Vector2f> & pre_all_inliers1,
                       std::vector<Eigen::Vector2f> & pre_all_inliers2,
                       std::vector<Eigen::Vector2f> & all_inliers1,
                       std::vector<Eigen::Vector2f> & all_inliers2,
                       std::map<int, int> & indexes_a)
{
    for (int i = 0; i < pre_all_inliers2.size(); i++)
    {
        Eigen::Vector2i index;
        bool find_index = false;
        for(int j = 0; j < all_inliers1.size(); j++)
        {
            if (pre_all_inliers2[i][0]==all_inliers1[j][0] && pre_all_inliers2[i][1]==all_inliers1[j][1])
            {
                index << i, j;
                find_index = true;
                break;
            }
        }
        if (find_index)
        {
            indexes_a[index[0]] = index[1];
        }
    }
}


/*****************************************************
            Multiple view correspondences.
 *****************************************************/
void get_all_3d_landmarks(std::vector<std::vector<Eigen::Vector2f> >  & all_2d_inliers,
                          std::vector<std::map<int, int> >  & all_indexes,
                          std::vector<Eigen::Vector3f> & all_3d_points,
                          std::map<int, std::vector<Eigen::Vector2f> > & all_frames,
                          std::vector<Eigen::Matrix3f> & all_R,
                          std::vector<Eigen::Vector3f> & all_t)
{

    for (int i = 0; i< 8; i++)
    {
        for (std::map<int, int>::iterator iter = all_indexes[i].begin(); iter != all_indexes[i].end(); iter++)
        {
            if (iter->second == -1)
            {
                continue;
            }
            std::vector<Eigen::Vector2f> frames(10,Eigen::Vector2f::Zero());
            std::vector<float> common_x;
            std::vector<float> common_y;
            std::vector<float> common_z;
            Eigen::VectorXf X;
            Eigen::Vector2f inver_normal1;
            Eigen::Vector2f inver_normal2;
            Eigen::Matrix3f R1 = all_R[i];
            Eigen::Vector3f t1 = all_t[i];
            Eigen::MatrixXf P2(3,4);
            P2 << R1(0,0),R1(0,1),R1(0,2),t1[0],R1(1,0),R1(1,1),R1(1,2),t1[1],R1(2,0),R1(2,1),R1(2,2),t1[2];
            inver_normal1 = all_2d_inliers[2*i][iter->first];
            inver_normal2 = all_2d_inliers[2*i+1][iter->first];
            Eigen::MatrixXf P_ref(3,4);
            if (i == 0)
            {
                P_ref << 1,0,0,0,0,1,0,0,0,0,1,0;
            }
            else
            {
                Eigen::Matrix3f R0 = all_R[i-1];
                Eigen::Vector3f t0 = all_t[i-1];
                P_ref << R0(0,0),R0(0,1),R0(0,2),t0[0],R0(1,0),R0(1,1),R0(1,2),t0[1],R0(2,0),R0(2,1),R0(2,2),t0[2];
            }
            get_world_point(inver_normal1, inver_normal2, P_ref, P2, X);
            X = X/X[3];
            common_x.push_back(X[0]);
            common_y.push_back(X[1]);
            common_z.push_back(X[2]);
            frames[i] = all_2d_inliers[2*i][iter->first];
            frames[i+1] = all_2d_inliers[2*i+1][iter->first];
            P_ref = P2;
            R1 = all_R[i+1];
            t1 = all_t[i+1];
            P2 << R1(0,0),R1(0,1),R1(0,2),t1[0],R1(1,0),R1(1,1),R1(1,2),t1[1],R1(2,0),R1(2,1),R1(2,2),t1[2];
            inver_normal1 = all_2d_inliers[2*(i+1)][iter->second];
            inver_normal2 = all_2d_inliers[2*(i+1)+1][iter->second];
            get_world_point(inver_normal1, inver_normal2, P_ref, P2, X);
            X = X/X[3];
            common_x.push_back(X[0]);
            common_y.push_back(X[1]);
            common_z.push_back(X[2]);
            frames[i+2] = all_2d_inliers[2*(i+1)+1][iter->second];
            int now = i+1;
            int point_index = iter->second;
            all_indexes[i][iter->first] = -1;
            while (now < 8 && all_indexes[now].find(point_index)!=all_indexes[now].end() && all_indexes[now].find(point_index)->second != -1)
            {
                int ii = all_indexes[now].find(point_index)->second;
                P_ref = P2;
                R1 = all_R[now+1];
                t1 = all_t[now+1];
                P2 << R1(0,0),R1(0,1),R1(0,2),t1[0],R1(1,0),R1(1,1),R1(1,2),t1[1],R1(2,0),R1(2,1),R1(2,2),t1[2];
                inver_normal1 = all_2d_inliers[2*(now+1)][ii];
                inver_normal2 = all_2d_inliers[2*(now+1)+1][ii];
                get_world_point(inver_normal1, inver_normal2, P_ref, P2, X);
                X = X/X[3];
                common_x.push_back(X[0]);
                common_y.push_back(X[1]);
                common_z.push_back(X[2]);
                frames[now+2] = all_2d_inliers[2*(now+1)+1][ii];
                all_indexes[now][point_index] = -1;
                now++;
                point_index = ii;

            }
            sort(common_x.begin(),common_x.end());
            sort(common_y.begin(),common_y.end());
            sort(common_z.begin(),common_z.end());
            int num_x = common_x.size();
            float x,y,z;
            if (num_x % 2 == 0)
            {
                x = (common_x[num_x/2-1]+common_x[num_x/2])/2;
                y = (common_y[num_x/2-1]+common_y[num_x/2])/2;
                z = (common_z[num_x/2-1]+common_z[num_x/2])/2;
            }
            else
            {
                x = common_x[num_x/2];
                y = common_y[num_x/2];
                z = common_z[num_x/2];
            }
            Eigen::Vector3f avg_X;
            avg_X << x,y,z;
            all_3d_points.push_back(avg_X);
            all_frames[all_3d_points.size()-1] = frames;

        }
    }

    for (int i = 0; i < 8;i++)
    {
        for (int j = 0; j < all_2d_inliers[2*i].size(); j++)
        {
            if (all_indexes[i].find(j) == all_indexes[i].end())
            {
                std::vector<Eigen::Vector2f> frames(10,Eigen::Vector2f::Zero());
                Eigen::VectorXf X;
                Eigen::Vector2f inver_normal1;
                Eigen::Vector2f inver_normal2;
                Eigen::Matrix3f R1 = all_R[i];
                Eigen::Vector3f t1 = all_t[i];
                Eigen::MatrixXf P2(3,4);
                P2 << R1(0,0),R1(0,1),R1(0,2),t1[0],R1(1,0),R1(1,1),R1(1,2),t1[1],R1(2,0),R1(2,1),R1(2,2),t1[2];
                inver_normal1 = all_2d_inliers[2*i][j];
                inver_normal2 = all_2d_inliers[2*i+1][j];
                Eigen::MatrixXf P_ref(3,4);
                if (i == 0)
                {
                    P_ref << 1,0,0,0,0,1,0,0,0,0,1,0;
                }
                else
                {
                    Eigen::Matrix3f R0 = all_R[i-1];
                    Eigen::Vector3f t0 = all_t[i-1];
                    P_ref << R0(0,0),R0(0,1),R0(0,2),t0[0],R0(1,0),R0(1,1),R0(1,2),t0[1],R0(2,0),R0(2,1),R0(2,2),t0[2];
                }
                get_world_point(inver_normal1, inver_normal2, P_ref, P2, X);
                X = X/X[3];
                frames[i] = all_2d_inliers[2*i][j];
                frames[i+1] = all_2d_inliers[2*i+1][j];
                Eigen::Vector3f avg_X;
                avg_X << X[0], X[1], X[2];
                all_3d_points.push_back(avg_X);
                all_frames[all_3d_points.size()-1] = frames;

            }
        }
    }
}
/*****************************************************
           Bundle adjustment by ceres.
 *****************************************************/

void optimization(std::vector<Eigen::Vector3f> & all_3d_points,
                  std::map<int, std::vector<Eigen::Vector2f> > & all_frames,
                  std::vector<Eigen::Matrix3f> & all_R,
                  std::vector<Eigen::Vector3f> & all_t,
                  std::vector<Eigen::Matrix3f> & after_R,
                  std::vector<Eigen::Vector3f> & after_t)
{

    ceres::Problem problem;
    double * points = new double[3*all_3d_points.size()];
    for (int i = 0; i < all_3d_points.size(); i++)
    {
        double * point = points + 3*i;
        *(point) = all_3d_points[i][0];
        *(point+1) = all_3d_points[i][1];
        *(point+2) = all_3d_points[i][2];
    }
    double * intrinsic_camera = new double[4];
    * intrinsic_camera = 525;
    * (intrinsic_camera+1) = 525;
    * (intrinsic_camera+2) = 319.5;
    * (intrinsic_camera+3) = 239.5;
    double * extrinsic_camera = new double[7*9];
    for (int i = 0; i < 9; i++)
    {
        double * e_camera = extrinsic_camera + 7*i;
        Eigen::Quaternionf quaternion(all_R[i]);
        *(e_camera) = quaternion.w();
        *(e_camera+1) = quaternion.x();
        *(e_camera+2) = quaternion.y();
        *(e_camera+3) = quaternion.z();
        *(e_camera+4) = all_t[i][0];
        *(e_camera+5) = all_t[i][1];
        *(e_camera+6) = all_t[i][2];
    }

    Eigen::Matrix3f K;
    K << 525,0,319.5,0,525,239.5,0,0,1;

    for (int i = 0; i < 9; i++)
    {
        double * camera = extrinsic_camera + 7*i;
        problem.AddParameterBlock(camera, 7);
    }
    problem.SetParameterBlockConstant(extrinsic_camera);
    problem.AddParameterBlock(intrinsic_camera, 4);
    ceres::LossFunction* loss_function = new ceres::HuberLoss(4);
    for (int i = 0; i < all_3d_points.size(); i++)
    {
        double * point = points + 3*i;
        if (point[2] < 0)
        {
            continue;
        }
        for (int j = 0; j < 9; j++)
        {
            if (all_frames[i][j+1][0]==0 && all_frames[i][j+1][1]==0)
            {
                continue;
            }
            Eigen::Vector2f observation = all_frames[i][j+1];
            ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(observation[0],observation[1]);
            double * camera = extrinsic_camera + 7*j;
            problem.AddResidualBlock(cost_function, NULL, camera, point, intrinsic_camera);
        }
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    std::cout << summary.FullReport() << "\n";
    if (!summary.IsSolutionUsable())
    {
        std::cout << "Bundle Adjustment failed." << std::endl;
    }
    else
    {
        std::cout << std::endl
        << "Bundle Adjustment statistics (approximated RMSE):\n"
        << " #residuals: " << summary.num_residuals << "\n"
        << " Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
        << " Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
        << " Time (s): " << summary.total_time_in_seconds << "\n"
        << std::endl;
    }

    for (int i = 0; i < 9; i++)
    {
        Eigen::Quaternionf quaternion(extrinsic_camera[7*i+0],extrinsic_camera[7*i+1],extrinsic_camera[7*i+2],extrinsic_camera[7*i+3]);
        Eigen::Matrix3f RR;
        RR=quaternion.matrix();
        Eigen::Vector3f tt; tt<<extrinsic_camera[7*i+4],extrinsic_camera[7*i+5],extrinsic_camera[7*i+6];
        after_R.push_back(RR);
        after_t.push_back(tt);
    }
}
