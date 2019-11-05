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

//ROS string handler
#include <std_msgs/String.h>

//OpenCV is used to convert data from Realsense format to OpenCV format
#include <opencv2/core.hpp>

//cv_bridge is used to convert data between OpenCV format and ROS image message format
#include <cv_bridge/cv_bridge.h>

//ROS image transport header is used to publish image message data
#include <image_transport/image_transport.h>

//standard string library
#include <string>

//QThread allows the publishing to be done on its own thread
#include <QThread>

//QStringListModel is used to create a model that links the logging pane in the mainWindow class to this class
#include <QStringListModel>

//allows the creation of simple message boxes
#include <QMessageBox>

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
        //init creates a ROS node if one doesn't already exist
        //it then initiates the main publishing loop in run
        bool init(const std::string &rosMasterAddress, const std::string &rosLocalAddress,
                  const std::string &depthTopicName, const std::string &colorTopicName);
        //run is called by init and handles the main publishing loop
        void run();
        //stop terminates the QThread publishing ROS messages
        bool stop();

    Q_SIGNALS:
        //rosShutdown is used to signal an unexpected stopping of the ROS node to the main application
        void rosShutdown();


    private:
        //ROS publisher for accelerometer data
        ros::Publisher publisherAccel;
        //ROS image publisher for color data
        image_transport::Publisher publisherColor;
        //ROS image publisher for depth data
        image_transport::Publisher publisherDepth;
        //ROS publisher for gyroscope data
        ros::Publisher publisherGyro;

        //publish is called by run and takes care of publishing a single set of messages
        void publish(const std::map<int, rs2::frame> rscFrames);
        //showError displays an error window
        void showError(QString errorMessage);
};

#endif
