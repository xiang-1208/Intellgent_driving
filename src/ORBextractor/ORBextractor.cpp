#include "ORBextractor.h"

ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels, int _scoreType,
         int _fastTh):
    nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
    scoreType(_scoreType), fastTh(_fastTh)
{
    mvScaleFactor.resize(nlevels);
    mvScaleFactor[0]=1;
    for(int i=1; i<nlevels; i++)
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;

    float invScaleFactor = 1.0f/scaleFactor;
    mvInvScaleFactor.resize(nlevels);
    mvInvScaleFactor[0]=1;
    for(int i=1; i<nlevels; i++)
        mvInvScaleFactor[i]=mvInvScaleFactor[i-1]*invScaleFactor;

    mvImagePyramid.resize(nlevels);
}


void ORBextractor::ComputeKeyPoints(vector<vector<KeyPoint> >& allKeypoints)
{

    allKeypoints.resize(nlevels);

    float imageRatio = (float)mvImagePyramid[0].cols/mvImagePyramid[0].rows;
    cout << "imageRatio: " <<imageRatio<<endl;
    cout << "mvImagePyramid.cols: " <<mvImagePyramid[0].cols<<endl;
    cout << "mvImagePyramid.rows: " <<mvImagePyramid[0].rows<<endl;

    Ptr<FeatureDetector> detector = ORB::create();

    for (int level = 0; level < nlevels; ++level)
    {
        detector->detect(mvImagePyramid[level],allKeypoints[level]);
    }
}

void ORBextractor::operator()( InputArray _image, vector<KeyPoint>& _keypoints,
                      OutputArray _descriptors)
{ 
    if(_image.empty())
        return;

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1 );

    // Pre-compute the scale pyramids
    ComputePyramid(image);

    vector < vector<KeyPoint> > allKeypoints;
    ComputeKeyPoints(allKeypoints);

    Mat descriptors;

    int nkeypoints = 0;
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += (int)allKeypoints[level].size();
    if( nkeypoints == 0 )
        _descriptors.release();
    else
    {
        _descriptors.create(nkeypoints, 32, CV_8U);
        descriptors = _descriptors.getMat();
    }

    _keypoints.clear();
    _keypoints.reserve(nkeypoints);

    int offset = 0;
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    for (int level = 0; level < nlevels; ++level)
    {
        vector<KeyPoint>& keypoints = allKeypoints[level];
        int nkeypointsLevel = (int)keypoints.size();

        if(nkeypointsLevel==0)
            continue;

        // preprocess the resized image
        Mat& workingMat = mvImagePyramid[level];
        GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

        // Compute the descriptors
        Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        descriptor->compute(workingMat,keypoints,desc);

        offset += nkeypointsLevel;

        // Scale keypoint coordinates
        if (level != 0)
        {
            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                 keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
                // 关键点位置重投影到原图
                keypoint->pt *= scale;
        }
        // And add the keypoints to the output
        _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
    }
}

void ORBextractor::ComputePyramid(cv::Mat image)
{
    for (int level = 0; level < nlevels; ++level)
    {
        float scale = mvInvScaleFactor[level];
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        cout << sz << endl;

        if( level != 0 )
        {
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);
        }
        else
        {
            mvImagePyramid[level] = image.clone();
        }
    }
}

