//pointcloudWidget code file
//pointcloudWidget handles pointcloud functions, including:
//  creating pointcloud from ROS image messages
//  displaying pointcloud in ui
//  allowing manipulation of pointcloud view

#include "pointcloudWidget.hpp"

//public: constructor
//sets internal data to passed data
pointcloudWidget::pointcloudWidget(ORB_SLAM2::System* pSLAM)
{
    mSLAM = pSLAM;
}

//public: destructor
//destroys class instance
pointcloudWidget::~pointcloudWidget()
{
}

//public: processFrames
//turns a pair of synchronized color and depth messages into a pair of cv images and passes them to ORB_SLAM2
void pointcloudWidget::processFrames(const sensor_msgs::ImageConstPtr &colorMessage, const sensor_msgs::ImageConstPtr &depthMessage)
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

        //pass the images and timestamp to ORB_SLAM for 'MaGiC pOiNtClOuD tHiNgS'
        mSLAM->TrackRGBD(cvColor->image, cvDepth->image, cvColor->header.stamp.toSec());
}
