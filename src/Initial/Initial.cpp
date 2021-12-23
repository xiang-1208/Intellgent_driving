#include "Initial.h"

Initial::Initial(VideoCapture src):car_capture(src)
{
    cout << "elude start" <<endl;
}

bool Initial::start()
{
    car_capture>>start_img;
    assert(start_img.data && "Can not load images!");
}

bool Initial::end(char* p_distance)
{
    car_capture>>end_img;
    assert(end_img.data && "Can not load images!");
    unsigned short distance = 0X0000;
    distance += (unsigned short)p_distance[0]<<8;
    distance += (unsigned short)p_distance[1];
    bool intial_run(distance);
    return true;
}

bool Initial::intial_run(unsigned short distance)
{
    std::vector<KeyPoint> keypoints_1,keypoints_2;
    Mat descriptors_1,descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // t1 = chrono::steady_clock::now();
    // detector->detect(start_img,keypoints_1);
    // detector->detect(end_img,keypoints_2);

    descriptor->compute(start_img,keypoints_1,descriptors_1);
    descriptor->compute(end_img,keypoints_2,descriptors_2);
    // t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    // cout << "ORB用时： " << time_used.count() << "秒" << endl;

    Mat outimg1;
    drawKeypoints (start_img,keypoints_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    // imshow ("ORB features",outimg1);

    vector<DMatch> matches;
    //t1 = chrono::steady_clock::now();
    matcher->match(descriptors_1,descriptors_2,matches);
    //t2 = chrono::steady_clock::now();
    //time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    //cout << "match用时： " << time_used.count() << "秒" << endl;

    //匹配点筛选
    auto min_max = minmax_element(matches.begin(),matches.end(),
        [](const DMatch &m1,const DMatch &m2){return m1.distance<m2.distance;});
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("--Max dist : %f\n",max_dist);
    printf("--Min dist : %f\n",min_dist);

    std::vector<DMatch> good_matches;
    for (int i=0;i<descriptors_1.rows;i++){
        if (matches[i].distance <= max(2*min_dist,15.0)){
            good_matches.push_back(matches[i]);
        }
    }

    Mat img_match;
    Mat img_goodmatch;
    drawMatches(start_img,keypoints_1,end_img,keypoints_2,matches,img_match);
    drawMatches(start_img,keypoints_1,end_img,keypoints_2,good_matches,img_goodmatch);
    imwrite ("../out/img_goodmatch.jpg",img_goodmatch);
    // imshow ("all matches",img_match);
    // imshow ("good matches",img_goodmatch);
    cout << "elude finish" << endl;

    cout << "Find " << good_matches.size() << " good matches" << endl;

    //估计两张图像间运动
    Mat R,t;
    pose_estimatiob_2d2d(keypoints_1, keypoints_2,matches,R,t);

    //-- 验证E=t^R*scale
    Mat t_x =
        (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
        t.at<double>(2, 0), 0, -t.at<double>(0, 0),
        -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    cout << "t^R=" << endl << t_x * R << endl;

    //-- 验证对极约束
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for (DMatch m: matches) {
        Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
        Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }

    return true;
    
    // waitKey(0);
}

void Initial::pose_estimatiob_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R, Mat &t) 
{
    //相机内参，TUM Freiburg2
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }   

    //计算基础矩阵F
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1,points2,CV_FM_8POINT);
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    //计算本质矩阵
    Point2d principal_point (325.1, 249.7);  //相机光心, TUM dataset标定值
    double focal_length = 521;
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1,points2,focal_length,principal_point);
    cout << "essential_matrix is " << endl << essential_matrix << endl;

    //-- 计算单应矩阵
    //-- 但是本例中场景不是平面，单应矩阵意义不大
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
    cout << "homography_matrix is " << endl << homography_matrix << endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    // 此函数仅在Opencv3中提供
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}

Point2d Initial::pixel2cam(const Point2d &p, const Mat &K) {
    return Point2d
        (
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
        );
}