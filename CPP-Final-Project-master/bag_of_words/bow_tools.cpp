
/**
 * @file bow_tools.cpp
 * @brief The application of the funtion decleared in bow_tools.hpp.
 */


#include "bow_tools.hpp"


using namespace cv;
/**
 *build_dictionary() Be used to build up the visual dictionary

 * @param[out] dictionary The matrix we used to store the dictionary we get
 * @retval NULL void output
 */
void build_dictionary(Mat & dictionary)
{
    std::string file_path = "../../data/rgbd_dataset_freiburg2_desk/rgb.txt";
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

    Mat img;
    Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
    std::vector<KeyPoint> key_points;
    Mat descriptor;


    TermCriteria tc(CV_TERMCRIT_ITER, 100, 0.001);
    int dictionarySize = 100;
    int retries = 1;
    int flags = KMEANS_PP_CENTERS;
    BOWKMeansTrainer bowTrainer(dictionarySize, tc, retries, flags);


    int count = 0;
    while(!in.eof())
    {
        if (count % 200 != 0)
        {
            in >> time_stamp >> file_name;
            count ++;
            continue;
        }
        in >> time_stamp >> file_name;
        std::string pre_path = "../../data/rgbd_dataset_freiburg2_desk/";
        file_name = pre_path+file_name;
        img = imread(file_name);
        f2d->detect(img, key_points);
        f2d->compute(img, key_points, descriptor);
        bowTrainer.add(descriptor);
        count ++;
    }
    in.close();
    dictionary = bowTrainer.cluster();
}
