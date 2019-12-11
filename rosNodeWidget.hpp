//rosNodeWidget header file
//rosNodeWidget handles creating a ROS node and publishing data on it

#ifndef ROSNODEWIDGET_HPP
#define ROSNODEWIDGET_HPP

//Intel RealSense library provides camera functions
//be sure to include this before any Qt includes
//odd linker errors can result otherwise
#include <librealsense2/rs.hpp>

//default ROS header
#include <ros/ros.h>

//ROS IMU message handler
#include <sensor_msgs/Imu.h>

//OpenCV is used to convert data from Realsense format to OpenCV format
#include <opencv2/core.hpp>

//cv_bridge is used to convert data from OpenCV format to ROS image message format
#include <cv_bridge/cv_bridge.h>

//ROS image transport system is used to publish image message data
#include <image_transport/image_transport.h>

//QThread allows the publishing to be done on its own thread
#include <QThread>

//QMessageBox allows the creation of simple message boxes
#include <QMessageBox>

//QSettings is used to handle saving and loading settings
#include <QSettings>

//rosNodeWidget class, manages ROS node and publishing
class rosNodeWidget : public QThread
{
    //rosNodeWidget uses the Qt event system, and thus requires the Q_OBJECT macro so the precompiler can link things properly
    Q_OBJECT

    public:
        //constructor simply creates class object, no special logic
        rosNodeWidget();
        //destructor shuts down ROS node and then destroys class instance
        virtual ~rosNodeWidget();
        //init creates a ROS node if one doesn't already exist and then initiates the main publishing loop in run
        bool init();
        //run is called by init and handles the main publishing loop
        void run();
        //stop terminates the QThread publishing ROS messages
        bool stop();
        //loadSettings loads settings into internal variables
        void loadSettings();

    Q_SIGNALS:
        //rosShutdown is used to signal an unexpected stopping of the ROS node to the main application
        void rosShutdown();

    private:
        //ROS image publisher for color data
        image_transport::Publisher publisherColor;
        //ROS image publisher for depth data
        image_transport::Publisher publisherDepth;
        //ROS publisher for IMU data
        ros::Publisher publisherImu;

        //internal values to hold settings
        std::string settingRosMasterAddress;
        std::string settingRosLocalAddress;
        std::string settingColorTopicName;
        std::string settingDepthTopicName;
        std::string settingImuTopicName;
        float settingPublishRate;
        bool settingAutoExposure;
        float settingThresholdMin;
        float settingThresholdMax;
        float settingSpatialMagnitude;
        float settingSpatialAlpha;
        float settingSpatialDelta;
        float settingTemporalAlpha;
        float settingTemporalDelta;

        //showError displays an error window
        void showError(QString errorMessage);
};

#endif
