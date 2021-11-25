#include "f_extract.h"

Extract::Extract(Mat image):Image(image)
{
    namedWindow("src",WINDOW_AUTOSIZE );
    imshow ("src",Image);
    waitKey(0);
    //检测目标边框
    Mat imageContours;
    RectDetect(imageContours);
    if (RectDetect(imageContours))
    {
        cout << "有检测到框" << endl;
        drawContours(Image, imageContours, -1, Scalar(0, 255, 255), 2, 8);       
        imshow("approx", Image);
        waitKey(0);
    }
    else
        cout << "没有检测到框" << endl;
}

bool Extract::RectDetect(Mat& imageContours)
{
    Mat img_HSV ;
    namedWindow("Rect",WINDOW_AUTOSIZE );
    cvtColor(Image,img_HSV,COLOR_BGR2HSV);

    int HSVrange[6] = {24,37,59,240,53,255};

    inRange(img_HSV,Scalar(HSVrange[0],HSVrange[2],HSVrange[4]),
            Scalar(HSVrange[1],HSVrange[3],HSVrange[5]),img_HSV);

    //形态学操作：闭运算，闭合裂缝、孔洞等残缺
    Mat kernel = getStructuringElement(MORPH_RECT, cv::Size(5, 5));
    morphologyEx(img_HSV, img_HSV,MORPH_CLOSE, kernel);
   
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

    findContours(img_HSV,contours,hierarchy,CV_RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
	//Mat imageContours=Mat::zeros(img_HSV.size(),CV_8UC1);

    imageContours = Mat(contours[1]);
    Mat hull;
    convexHull(Mat(imageContours), hull, true);

	//绘制轮廓
	drawContours(imageContours,contours,1,Scalar(255),1,8,hierarchy);

    //double epsilon = 0.01*arcLength(contour,true);
    //approxPolyDP(Mat(contour),imageContours,15,true);
    
    //cout << Mat(contour) <<endl;
    cout << hull <<endl;
    return 1;

	//imshow("Contours Image",imageContours); //外轮廓
    //waitKey(0); 
    // if (imageContours.rows == 4)
    //     return true;
    // else
    //     return false;
}