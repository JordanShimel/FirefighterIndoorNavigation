//pointcloudWidget code file
//pointcloudWidget handles pointcloud functions, including:
//  creating pointcloud from bag
//  displaying pointcloud in ui
//  allowing manipulation of pointcloud view
//  creating static images of pointcloud to be returned to remote unit(s)

#include "pointcloudWidget.hpp"


pointcloudWidget::pointcloudWidget()
{
}

pointcloudWidget::~pointcloudWidget()
{

}

void pointcloudWidget::setSLAM(ORB_SLAM2::System* pSLAM)
{
    mpSLAM = pSLAM;
}


void pointcloudWidget::grabRGBD(const sensor_msgs::ImageConstPtr &msgColor, const sensor_msgs::ImageConstPtr &msgDepth)
{
    // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(msgColor);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrD;
        try
        {
            cv_ptrD = cv_bridge::toCvShare(msgDepth);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}
