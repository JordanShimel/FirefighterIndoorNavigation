//pointcloudWidget header file
//pointcloudWidget handles pointcloud functions, including:
//  creating pointcloud from bag
//  displaying pointcloud in ui
//  allowing manipulation of pointcloud view
//  creating static images of pointcloud to be returned to remote unit(s)

#ifndef POINTCLOUDWIDGET_HPP
#define POINTCLOUDWIDGET_HPP

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include "ORB_SLAM2/include/System.h"

//pointcloudWidget class, manages pointcloud functionality
class pointcloudWidget
{
    public:
        //constructor
        pointcloudWidget(ORB_SLAM2::System* pSLAM);
        //destructor
        ~pointcloudWidget();

        //build a pointcloud from ROS image stream
        void grabRGBD(const sensor_msgs::ImageConstPtr& msgColor,const sensor_msgs::ImageConstPtr& msgDepth);

        ORB_SLAM2::System* mpSLAM;
};

#endif
