//rosNodeWidget header file
//rosNodeWidget handles creating a ROS node and subscribing to data on it

#ifndef ROSNODEWIDGET_HPP
#define ROSNODEWIDGET_HPP

//pointcloud widget is used to handled received camera data
#include "pointcloudWidget.hpp"

//default ROS header
#include <ros/ros.h>

//ROS image transport header is used to subscribe to image message data
//TODO:maybe unneeded now? - Jordan
#include <image_transport/image_transport.h>

//ROS standard messages are used to subscribe to text based message data
#include <std_msgs/String.h>

//ROS built in IMU message handler
#include <sensor_msgs/Imu.h>

//cv_bridge is used to convert data between OpenCV format and ROS image message format
#include <cv_bridge/cv_bridge.h>

//QThread allows the publishing to be done on its own thread
#include <QThread>

//QSettings is used to handle saving and loading settings
#include <QSettings>

//ORB_SLAM2 is used to create and render pointcloud
#include "ORB_SLAM2_firefighter/include/System.h"

//rosNodeWidget class, manages ROS node and subscribing
class rosNodeWidget : public QThread
{
    //rosNodeWidget uses the Qt event system, and thus requires the Q_OBJECT macro so the precompiler can link things properly
    Q_OBJECT

    public:
        //constructor simply creates class object, no special logic
        rosNodeWidget();
        //destructor shuts down ROS node and then destroys class instance
        virtual ~rosNodeWidget();
        //init creates a ROS node if one doesn't already exist and then initiates the main subscribing loop in run
        bool init();
        //run is called by init and handles the main subscribing loop
        void run();
        //stop terminates the QThread subscribing to ROS messages
        bool stop();
        //loadSettings loads settings into internal variables
        void loadSettings();

    Q_SIGNALS:
        //rosShutdown is used to signal an unexpected stopping of the ROS node to the main application
        void rosShutdown();
        void launchSlam();
                /// create a qsignal here for the slamInit signal, use the signal for the win id

    private:
        //ROS image subsciber for color data
        std::string mColorTopicName;
        sensor_msgs::ImageConstPtr mSubscriberColor;
        //ROS color subscriber for depth data
        std::string mDepthTopicName;
        sensor_msgs::ImageConstPtr mSubscriberDepth;
        //ROS IMU subscriber for IMU data
        std::string mImuTopicName;
        sensor_msgs::Imu mSubscriberImu;

        //internal values to hold settings
        std::string settingRosMasterAddress;
        std::string settingRosLocalAddress;
        std::string settingColorTopicName;
        std::string settingDepthTopicName;
        std::string settingImuTopicName;
        float settingRefreshRate;
};

#endif
