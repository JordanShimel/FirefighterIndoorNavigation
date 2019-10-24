//rosNodeWidget header file
//rosNodeWidget handles creating a ROS node and subscribing to and publishing data on it

#ifndef ROSNODEWIDGET_HPP
#define ROSNODEWIDGET_HPP

//default ROS header
#include <ros/ros.h>

//ROS image transport header is used to subscribe to image message data
#include <image_transport/image_transport.h>

//ROS standard messages are used to subscribe to text based message data
#include <std_msgs/String.h>

//cv_bridge is used to convert data between OpenCV format and ROS image message format
#include <cv_bridge/cv_bridge.h>

//QThread allows the publishing to be done on its own thread
#include <QThread>

//allows the creation of simple message boxes
#include <QMessageBox>

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
        //getAccel will return acceleration data
        void getAccel();
        //getDepth returns depth data as a QImage
        QImage getDepth();
        //getGyro will return gyroscope data
        void getGyro();

    Q_SIGNALS:
        //rosShutdown is used to signal an unexpected stopping of the ROS node to the main application
        void rosShutdown();
        //receivedAccel is used to signal the receipt of a frame of acceleration data to the main application
        void receivedAccel();
        //receivedDepth is used to signal the receipt of a frame of depth data to the main application
        void receivedDepth();
        //receivedGyro is used to signal the receipt of a frame of gyroscope data to the main application
        void receivedGyro();

    private:
        //ROS subsciber for acceleration data
        ros::Subscriber subscriberAccel;
        //ROS image subsciber for depth data
        image_transport::Subscriber subscriberDepth;
        //ROS subsciber for gyroscope data
        ros::Subscriber subscriberGyro;

        //holder for acceleration data
        //holder for depth data
        QImage qimageDepth;
        //holder for gyroscope data

        //callback for acceleration messages
        void callbackAccel(const std_msgs::String::ConstPtr &accelMessage);
        //callback for depth messages
        void callbackDepth(const sensor_msgs::ImageConstPtr &depthMessage);
        //callback for gyroscope messages
        void callbackGyro(const std_msgs::String::ConstPtr &gyroMessage);
};

#endif
