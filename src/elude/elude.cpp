#include "elude.h"

// #define SERIAL

Eludeing::Eludeing(VideoCapture src):car_capture(src)
{
    cout << "elude start" <<endl;

    // int nFeatures = fSettings["ORBextractor.nFeatures"];
    // float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    // int nLevels = fSettings["ORBextractor.nLevels"];
    // int fastTh = fSettings["ORBextractor.fastTh"];    
    // int Score = fSettings["ORBextractor.nScoreType"];

    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fastTh = 20;    
    int Score = 1;

    mpORBextractor = new ORBextractor(fScaleFactor,nLevels,Score,fastTh);
    
    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Fast Threshold: " << fastTh << endl;
    if(Score==0)
        cout << "- Score: HARRIS" << endl;
    else
        cout << "- Score: FAST" << endl;

    // ORB extractor for initialization
    // Initialization uses only points from the finest scale level
    mpIniORBextractor = new ORBextractor(1.2,1,Score,fastTh);    
}

void Eludeing::run()
{
    //模板匹配,得到一个ROI区域
    /**
     * @brief 
     * 
     */

    //SLAM
    //初始化


    int nframe = car_capture.get(CV_CAP_PROP_FPS);
    //int milliseconds = car_capture.get(cv2.CAP_PROP_POS_MSEC);
    cout << "frame is " << nframe << endl;

    Mat img_1,img_2;

    // // Mat img_1=img_1(Rect(260,30,50,80));
    // // car_capture>>img_1;
    // for (int i=0;i<30;i++)
    // {
    //     car_capture>>img_1;
    // }
    img_1 = imread("../data/mask.jpg");
    if(img_1.channels()==3)
        cvtColor(img_1, img_1, CV_RGB2GRAY);
    // imwrite ("../data/start.jpg",img_1);
    if(mState==WORKING)
        mCurrentFrame = Frame(img_1,mpIniORBextractor,mK);
    Mat outimg1;
    drawKeypoints (img_1,mCurrentFrame.mvKeys,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    imwrite ("../out/outimg1.jpg",outimg1);

    for (int i=0;i<600;i++)
    {
        car_capture>>img_2;
    }
    if(img_2.channels()==3)
        cvtColor(img_2, img_2, CV_RGB2GRAY);
    // // Mat img_2=img_2(Rect(440,30,50,80));
    // // car_capture>>img_2;
    if(mState==WORKING)
        mLastFrame = Frame(img_2,mpORBextractor,mK);
    Mat outimg2;
    drawKeypoints (img_2,mLastFrame.mvKeys,outimg2,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    imwrite ("../out/outimg2.jpg",outimg2);
    car_capture.release();//释放资源
    assert(img_1.data && img_2.data && "Can not load images!");

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // for (auto it : keypoints_1_temp)
    // {
	//     if (it.pt.x>260 && it.pt.x<310 && it.pt.y>50 && it.pt.y<130)
    //         keypoints_1.push_back(it);
    //     else;
    // }
    // for (auto it : keypoints_2_temp)
    // {
	//     if (it.pt.x>440 && it.pt.x<490 && it.pt.y>30 && it.pt.y<110)
    //         keypoints_2.push_back(it);
    //     else
    //         keypoints_2.push_back(it);
    // }

    vector<DMatch> matches;
    // t1 = chrono::steady_clock::now();
    matcher->match(mCurrentFrame.mDescriptors,mLastFrame.mDescriptors,matches);
    // t2 = chrono::steady_clock::now();
    // time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    // cout << "match用时： " << time_used.count() << "秒" << endl;
    // cout << "match对： " << matches.size()<< endl;

    //匹配点筛选
    auto min_max = minmax_element(matches.begin(),matches.end(),
        [](const DMatch &m1,const DMatch &m2){return m1.distance<m2.distance;});
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("--Max dist : %f\n",max_dist);
    printf("--Min dist : %f\n",min_dist);

    std::vector<DMatch> good_matches;
    for (int i=0;i<matches.size();i++){
        if (matches[i].distance <= max(1.8*min_dist,10.0)){
            good_matches.push_back(matches[i]);
        }
    }

    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1,mCurrentFrame.mvKeys,img_2,mLastFrame.mvKeys,matches,img_match);
    drawMatches(img_1,mCurrentFrame.mvKeys,img_2,mLastFrame.mvKeys,good_matches,img_goodmatch);
    imwrite ("../out/img_goodmatch.jpg",img_goodmatch);
    // imshow ("all matches",img_match);
    // imshow ("good matches",img_goodmatch);

    cout << "Find " << good_matches.size() << " good matches" << endl;

    // 估计两张图像间运动
    Mat R,t;
    pose_estimatiob_2d2d(mCurrentFrame.mvKeys, mLastFrame.mvKeys,good_matches,R,t);

    //-- 验证E=t^R*scale
    // Mat t_x =
    //     (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
    //     t.at<double>(2, 0), 0, -t.at<double>(0, 0),
    //     -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    // cout << "t^R=" << endl << t_x * R << endl;

    //-- 验证对极约束
    // Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    // for (DMatch m: matches) {
    //     Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
    //     Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
    //     Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
    //     Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
    //     Mat d = y2.t() * t_x * R * y1;
    //     cout << "epipolar constraint = " << d << endl;
    // }
    
    // waitKey(0);
}

void Eludeing::pose_estimatiob_2d2d(std::vector<KeyPoint> keypoints_1,
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

Point2d Eludeing::pixel2cam(const Point2d &p, const Mat &K) {
    return Point2d
        (
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
        );
}