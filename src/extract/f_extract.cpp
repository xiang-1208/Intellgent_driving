#include "f_extract.h"

Extract::Extract(Mat image):Image(image)
{
    cameraMatrix = (cv::Mat_<double>(3, 3) << 
                    512.1114 , 0 , 307.9962,
                    0, 514.5664 , 253.5470,
                    0, 0, 1);
    distCoeffs = (cv::Mat_<double>(5, 1) << 
                    4.72e-02,3.18e-02,
                    0. ,0. ,-3.18e-01);
    namedWindow("src",WINDOW_AUTOSIZE );
    //imshow ("src",Image);
    //waitKey(0);
    //检测目标边框
    vector<vector<Point>> boxDetect;
    if (RectDetect(boxDetect))
    {
        cout << "有检测到框" << endl;
        drawContours(Image, boxDetect, -1, Scalar(0, 255, 255), 2, 8);       
        // imshow("approx", Image);
        // waitKey(0);

        //顺序化盒子
        vector<int> box_int;
        for (int i=0; i<boxDetect[0].size(); i++)
        {
            box_int.push_back(boxDetect[0][i].y);
        }
        std::vector<int>::iterator minner = std::min_element(std::begin(box_int), std::end(box_int));
        int index = std::distance(std::begin(box_int), minner);
        int next;
        if (index == 3)
            next = 0;
        else
            next = index+1;
        if (boxDetect[0][index].x >= boxDetect[0][next].x)
            index = index;
        else
            index = next;
        vector<Point2d> order;
        for (int i=0;i<boxDetect[0].size();i++)
        {   
            static int j = index;
            if (j == -1)
                j = 3;
            order.push_back(boxDetect[0][j]);
            j--;
        }

        vector<Point3d> targetCoor;
        targetCoor.push_back(Point3d(-425.0,0,425.0));
        targetCoor.push_back(Point3d(425.0,0,425.0));
        targetCoor.push_back(Point3d(425.0,0,-425.0));
        targetCoor.push_back(Point3d(-425.0,0,-425.0));
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

        //cout << "order: " << order << endl;
        //cout << "targetCoor: " << targetCoor << endl;
        solvePnP (targetCoor,order,cameraMatrix,distCoeffs,rvec,tvec, false,SOLVEPNP_EPNP);
        //cout << "rvec: " << rvec << endl;
        //cout << "tvecr: " << tvec << endl;   

        Mat rotM;
        Rodrigues(rvec,rotM);
        //couMat rotM;t<< "rotM:" << rotM <<endl;
        Mat rotM_invert;
        invert(rotM,rotM_invert);
        //cout<< "rotM:" << rotM <<endl;
        double angle = rotationMatrixToEulerAngles(rotM);

        Mat transVec_word = rotM_invert*tvec;
        //cout<< "transVec_word:" << transVec_word <<endl;
        //cout<< "angle:" << angle <<endl;
        raw = angle;
    }
    else
        cout << "没有检测到框" << endl;
}

bool Extract::RectDetect(vector<vector<Point>>& boxDetect)
{
    Mat img_HSV ;
    cvtColor(Image,img_HSV,COLOR_BGR2HSV);

    int HSVrange[6] = {24,37,59,240,53,255};

    inRange(img_HSV,Scalar(HSVrange[0],HSVrange[2],HSVrange[4]),
            Scalar(HSVrange[1],HSVrange[3],HSVrange[5]),img_HSV);

    //形态学操作：闭运算，闭合裂缝、孔洞等残缺
    Mat kernel = getStructuringElement(MORPH_RECT, cv::Size(5, 5));
    morphologyEx(img_HSV, img_HSV,MORPH_CLOSE, kernel);
   
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

    findContours(img_HSV,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	//Mat imageContours=Mat::zeros(img_HSV.size(),CV_8UC1);
    vector<Point> box;
    for (int i = 0; i < contours.size(); i++)
    {
        double epsilon = 0.01*arcLength(contours[i],true);
        approxPolyDP(contours[i],box,epsilon,true);
        if (box.size() == 4 && contourArea(box) > 100 && isContourConvex(box))
            boxDetect.push_back(box);
    }

	//imshow("Contours Image",boxDetect); //外轮廓
    //waitKey(0); 
    if (boxDetect[0].size() == 4)
        return true;
    else
        return false;
}



// Checks if a matrix is a valid rotation matrix.
bool Extract::isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
     
}
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
double Extract::rotationMatrixToEulerAngles(Mat R)
{
 
    assert(isRotationMatrix(R));
     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        cout << "!!!!!" << R.at<double>(2,1) << R.at<double>(2,2) << x <<endl;
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return y*180/pi;
}

Mat Extract::return_word()
{
    return word;
}

double Extract::return_raw()
{
    return raw;
}

