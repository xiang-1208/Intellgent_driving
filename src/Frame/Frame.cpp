#include "Frame.h"

Frame::Frame()
{}

Frame::Frame(cv::Mat &im_, ORBextractor* extractor, cv::Mat &K)
    :mpORBextractor(extractor), im(im_), mK(K.clone())
{
    // 提取ORB特征(重载运算符（）)
    (*mpORBextractor)(im,mvKeys,mDescriptors);   

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    cout << "inital" <<endl;

}


