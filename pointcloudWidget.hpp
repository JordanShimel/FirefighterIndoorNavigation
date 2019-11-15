//pointcloudWidget header file
//pointcloudWidget handles pointcloud functions, including:
//  creating pointcloud from ROS image messages
//  displaying pointcloud in ui
//  allowing manipulation of pointcloud view

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

//ORB_SLAM2 is used to create and render pointcloud
#include "ORB_SLAM2/include/System.h"

//pointcloudWidget class, manages pointcloud functionality
class pointcloudWidget
{
    public:
        //constructor sets internal SLAM system to passed SLAM system
        pointcloudWidget(ORB_SLAM2::System* pSLAM);
        //destructor destroys pointcloudWidget object
        ~pointcloudWidget();

        //processFrames builds a pointcloud from ROS image stream
        void processFrames(const sensor_msgs::ImageConstPtr& msgColor,const sensor_msgs::ImageConstPtr& msgDepth);

        //SLAM system for our pointcloud
        ORB_SLAM2::System* mSLAM;
};

#endif
