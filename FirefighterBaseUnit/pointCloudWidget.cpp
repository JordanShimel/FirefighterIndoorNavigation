//PointCloudWidget code file
//PointCloudWidget handles PointCloud functions, including:
//  creating PointCloud from ROS image messages
//  displaying PointCloud in ui
//  allowing manipulation of PointCloud view

#include "pointCloudWidget.hpp"

//public: constructor
//sets internal data to passed data
pointCloudWidget::pointCloudWidget(ORB_SLAM2::System* pSLAM)
{
    mSLAM = pSLAM;
}

//public: destructor
//destroys class instance
pointCloudWidget::~pointCloudWidget()
{
    shutdown();
}

//public: shutdown
//stops the ORB_SLAM2 system from processing
void pointCloudWidget::shutdown()
{
    mSLAM->Shutdown();
}

//public: processFrames
//turns a pair of synchronized color and depth messages into a pair of cv images and passes them to ORB_SLAM2
void pointCloudWidget::processFrames(const sensor_msgs::ImageConstPtr &colorMessage, const sensor_msgs::ImageConstPtr &depthMessage)
{
        //turn the ros color message into a cv image
        cv_bridge::CvImageConstPtr cvColor;
        try
        {
            cvColor = cv_bridge::toCvShare(colorMessage);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //turn the ros depth message into a cv image
        cv_bridge::CvImageConstPtr cvDepth;
        try
        {
            cvDepth = cv_bridge::toCvShare(depthMessage);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //pass the images and timestamp to ORB_SLAM2 to have it do 'MaGiC PointCloud tHiNgS'
        mSLAM->TrackRGBD(cvColor->image, cvDepth->image, cvColor->header.stamp.toSec());
}
