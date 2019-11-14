//rosNodeWidget header file
//rosNodeWidget handles creating a ROS node and subscribing to and publishing data on it

#ifndef ROSNODEWIDGET_HPP
#define ROSNODEWIDGET_HPP

//pointcloud widget file
#include "pointcloudWidget.hpp"

//default ROS header
#include <ros/ros.h>

//ROS image transport header is used to subscribe to image message data
#include <image_transport/image_transport.h>

//ROS standard messages are used to subscribe to text based message data
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

//cv_bridge is used to convert data between OpenCV format and ROS image message format
#include <cv_bridge/cv_bridge.h>

//QThread allows the publishing to be done on its own thread
#include <QThread>

//allows the creation of simple message boxes
#include <QMessageBox>

#include "../../ORB_SLAM2/include/System.h"
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
        //init creates a ROS node if one doesn't already exist
        //it then initiates the main subscribing loop in run
        bool init(const std::string &rosMasterAddress, const std::string &rosLocalAddress);
        //run is called by init and handles the main subscribing loop
        void run();
        //stop terminates the QThread subscribing to ROS messages
        bool stop();

    Q_SIGNALS:
        //rosShutdown is used to signal an unexpected stopping of the ROS node to the main application
        void rosShutdown();

    private:
        //ROS image subsciber for depth data
        image_transport::Subscriber subscriberDepth;
        //ROS color subscriber for color data
        image_transport::Subscriber subscriberColor;
        //ROS IMU subscriber for IMU data
        ros::Subscriber subscriberIMU;

        sensor_msgs::ImageConstPtr depthMessageContainer;
        sensor_msgs::ImageConstPtr colorMessageContainer;
        sensor_msgs::Imu imuMessageContainer;

        //pointcloudWidget pcw;
        //callback for depth messages
        void callbackDepth(const sensor_msgs::ImageConstPtr &depthMessage);
        //callback for color messages
        void callbackColor(const sensor_msgs::ImageConstPtr &colorMessage);
        //callback for IMU messages
        void callbackIMU(const sensor_msgs::Imu &imuMessage);
};

#endif
