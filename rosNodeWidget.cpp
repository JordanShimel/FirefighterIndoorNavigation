//rosNodeWidget code file
//rosNodeWidget handles creating a ROS node and publishing data on it

#include "rosNodeWidget.hpp"

//public: constructor
//simply creates a new class instance
rosNodeWidget::rosNodeWidget()
{
}

//public: destructor
//shuts down any ROS node
//destroy class instance
rosNodeWidget::~rosNodeWidget()
{
    //if a node has started
    if(ros::isStarted())
    {
        //close it
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

//public: init
//creates ROS node if one doesn't already exist
//set up topics to publish via that node
//creates thread to actually publish messages
bool rosNodeWidget::init(const std::string &rosMasterAddress, const std::string &rosLocalAddress,
                         const std::string &accelTopicName, const std::string &colorTopicName,
                         const std::string &depthTopicName, const std::string &gyroTopicName,
                         const float &publishRate)
{
    //create a map with the master and local addresses to pass to the ROS init function
    std::map<std::string, std::string> rosAddresses;
    rosAddresses["__master"] = rosMasterAddress;
    rosAddresses["__hostname"] = rosLocalAddress;

    //initialize a node with our addresses
    ros::init(rosAddresses, "remoteUnitNode");

    //check to see if our addresses are functional
    if(!ros::master::check())
    {
        //if not, return false now
        return false;
    }
    else
    {
        //if our addresses are functional, start the node
        //we need to start it explicitly so that it doesn't stop until we want it to
        //otherwise it would close if the node handles all closed
        ros::start();

        //create a handle to our node
        ros::NodeHandle remoteUnitNodeHandle;

        //assign publishers to this node
        //accelerometer publisher
        publisherAccel = remoteUnitNodeHandle.advertise<std_msgs::String>(accelTopicName, 1);

        //color publisher
        image_transport::ImageTransport imageTransportColor(remoteUnitNodeHandle);
        publisherColor = imageTransportColor.advertise(colorTopicName, 1);

        //depth publisher
        image_transport::ImageTransport imageTransportDepth(remoteUnitNodeHandle);
        publisherDepth = imageTransportDepth.advertise(depthTopicName, 1);

        //gyroscope publisher
        publisherGyro = remoteUnitNodeHandle.advertise<std_msgs::String>(gyroTopicName, 1);

        //set publish rate value
        publisherRate = publishRate;

        //starts Qt thread to run the publisher
        //does thread stuff and has thread call run()
        start();
        return true;
    }
}

//public: run
//extends QThread run
//is called by start
//handles main publishing loop
void rosNodeWidget::run()
{
    //initialize camera
    //logic here is similiar to logic in mainWindow on_pushButtonPreview_Clicked
    //see comments there for more detail
    rs2::context rscContext;
    rs2::device_list rscDevices;
    try
    {
        rscDevices = rscContext.query_devices();
    }
    catch(const rs2::error & e)
    {
        showError("Could not query RealSense devices.");
        return;
    }
    size_t rscDeviceCount = rscDevices.size();
    if(rscDeviceCount == 0)
    {
        //display error and return
        showError("No RealSense device detected.");
        return;
    }
    else if(rscDeviceCount == 1)
    {
        rs2::device rscDevice = rscDevices[0];
        rs2::pipeline rscPipe(rscContext);
        rs2::config rscConfig;
        rscConfig.enable_stream(RS2_STREAM_ACCEL);
        rscConfig.enable_stream(RS2_STREAM_COLOR);
        rscConfig.enable_stream(RS2_STREAM_DEPTH);
        rscConfig.enable_stream(RS2_STREAM_GYRO);
        rscPipe.start(rscConfig);
        rs2::colorizer rscColorizer;
        rs2::decimation_filter rscDecimator;
        std::map<int, rs2::frame> rscPublishFrames;

        //create variables used to build messages
        //height of camera output
        int height = 0;
        //width of camera output
        int width = 0;

        //create message variables
        //accelerometer message
        rs2::frame rscTempAccelFrame;
        std_msgs::String messageAccel;

        //color message
        rs2::frame rscPublishColorFrame;
        sensor_msgs::ImagePtr messageColor;

        //depth message
        rs2::frame rscPublishDepthFrame;
        sensor_msgs::ImagePtr messageDepth;

        //gyroscope message
        rs2::frame rscTempGyroFrame;
        std_msgs::String messageGyro;

        //rosLoopRate is how many times the while loop will attempt to run per second
        ros::Rate rosLoopRate(publisherRate);

        while(ros::ok())
        {
            std::vector<rs2::frame> rscNewFrames;
            rs2::frameset rscFrameSet;

            if(rscPipe.poll_for_frames(&rscFrameSet))
            {
                //accelerometer data publisher
                rscTempAccelFrame = rscFrameSet.first(RS2_STREAM_ACCEL);
                rs2::motion_frame rscPublishAccelFrame = rscTempAccelFrame.as<rs2::motion_frame>();
                std::stringstream stringStreamAccelMessage;
                stringStreamAccelMessage << "Accel X: " << rscPublishAccelFrame.get_motion_data().x << ", Accel Y: " << rscPublishAccelFrame.get_motion_data().y << ", Accel Z" << rscPublishAccelFrame.get_motion_data().z << std::endl;
                messageAccel.data = stringStreamAccelMessage.str();
                publisherAccel.publish(messageAccel);

                //color data publisher
                rscPublishColorFrame = rscFrameSet.first(RS2_STREAM_COLOR);
                width = rscPublishColorFrame.as<rs2::video_frame>().get_width();
                height = rscPublishColorFrame.as<rs2::video_frame>().get_height();
                cv::Mat imageColor(cv::Size(width, height), CV_8UC3, (void*)rscPublishColorFrame.get_data(), cv::Mat:: AUTO_STEP);
                messageColor = cv_bridge::CvImage(std_msgs::Header(), "rgb8", imageColor).toImageMsg();
                publisherColor.publish(messageColor);

                //depth data publisher
                rscPublishDepthFrame = rscFrameSet.first(RS2_STREAM_DEPTH);
                width = rscPublishDepthFrame.as<rs2::video_frame>().get_width();
                height = rscPublishDepthFrame.as<rs2::video_frame>().get_height();
                cv::Mat imageDepth(cv::Size(width, height), CV_16U, (void*)rscPublishDepthFrame.get_data(), cv::Mat:: AUTO_STEP);
                imageDepth.convertTo(imageDepth, CV_8UC1, 255.0/1000);
                messageDepth = cv_bridge::CvImage(std_msgs::Header(), "mono8", imageDepth).toImageMsg();
                publisherDepth.publish(messageDepth);

                //gyroscope data publisher
                rscTempGyroFrame = rscFrameSet.first(RS2_STREAM_GYRO);
                rs2::motion_frame rscPublishGyroFrame = rscTempGyroFrame.as<rs2::motion_frame>();
                std::stringstream stringStreamGyroMessage;
                stringStreamGyroMessage << "Gyro X: " << rscPublishGyroFrame.get_motion_data().x << ", Gyro Y: " << rscPublishGyroFrame.get_motion_data().y << ", Gyro Z" << rscPublishGyroFrame.get_motion_data().z << std::endl;
                messageGyro.data = stringStreamGyroMessage.str();
                publisherGyro.publish(messageGyro);
            }

            ros::spinOnce();

            rosLoopRate.sleep();
        }

        //stop RS2 pipeline
        rscPipe.stop();

        Q_EMIT rosShutdown();
    }
    else if(rscDeviceCount > 1)
    {
        //display error and return
        showError("More than 1 RealSense device detected.");
        return;
    }
    else
    {
        //display error and return
        showError("Unknown number of RealSense devices detected.");
        return;
    }
}

//public: stop
//terminates the QThread publishing messages
bool rosNodeWidget::stop()
{
    terminate();
    return true;
}

//private: showError
//outputs error message with passed text
void rosNodeWidget::showError(QString errorMessage)
{
    //display error and return
    QMessageBox errorMessageBox;
    errorMessageBox.setText(errorMessage);
    errorMessageBox.exec();
    return;
}
