//PointCloudWidget header file
//PointCloudWidget handles PointCloud functions, including:
//  creating PointCloud from ROS image messages
//  displaying PointCloud in ui
//  allowing manipulation of PointCloud view

#ifndef POINTCLOUDWIDGET_HPP
#define POINTCLOUDWIDGET_HPP

//default ROS header
#include <ros/ros.h>

//cv_bridge is used to convert data between OpenCV format and ROS image message format
#include <cv_bridge/cv_bridge.h>

//message filters are used to synchronize the image messages
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//default OpenCV header
#include <opencv2/core/core.hpp>

//ORB_SLAM2 is used to create and render PointCloud
#include "ORB_SLAM2_firefighter/include/System.h"

//PointCloudWidget class, manages PointCloud functionality
class pointCloudWidget
{
    public:
        //constructor sets internal SLAM system to passed SLAM system
        pointCloudWidget(ORB_SLAM2::System* pSLAM);
        //destructor destroys PointCloudWidget object
        ~pointCloudWidget();

        //shutdown stops the ORB_SLAM2 system
        void shutdown();

        //processFrames builds a PointCloud from ROS image stream
        void processFrames(const sensor_msgs::ImageConstPtr& msgColor,const sensor_msgs::ImageConstPtr& msgDepth);

        //SLAM system for our PointCloud
        ORB_SLAM2::System* mSLAM;
};

#endif
