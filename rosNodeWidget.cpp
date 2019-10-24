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
bool rosNodeWidget::init(const std::string &rosMasterAddress, const std::string &rosLocalAddress)
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
        //acceleration publisher
        //depth publisher
        image_transport::ImageTransport imageTransportDepth(remoteUnitNodeHandle);
        publisherDepth = imageTransportDepth.advertise("rscDepth", 1);
        //gyroscope publisher

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
        const char* rscSerial = rscDevice.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        rs2::pipeline rscPipe(rscContext);
        rs2::config rscConfig;
        rscConfig.enable_device(rscSerial);
        rscPipe.start(rscConfig);
        rs2::colorizer rscColorizer;
        std::map<int, rs2::frame> rscPublishFrames;

        //create variables used to build messages
        //height of camera output
        int height = 0;
        //width of camera output
        int width = 0;

        //create message variables
        //accelerometer message

        //depth message
        rs2::frame rscPublishDepthFrame;
        sensor_msgs::ImagePtr messageDepth;
        //gyroscope message

        //rosLoopRate is how many times the while loop will attempt to run per second
        ros::Rate rosLoopRate(10);
        while(ros::ok())
        {
            std::vector<rs2::frame> rscNewFrames;
            rs2::frameset rscFrameSet;

            if(rscPipe.poll_for_frames(&rscFrameSet))
            {
                //TODO: move into publish function
                rscPublishDepthFrame = rscFrameSet.get_depth_frame().apply_filter(rscColorizer);
                width = rscPublishDepthFrame.as<rs2::video_frame>().get_width();
                height = rscPublishDepthFrame.as<rs2::video_frame>().get_height();
                cv::Mat imageDepth(cv::Size(width, height), CV_8UC3, (void*)rscPublishDepthFrame.get_data(), cv::Mat:: AUTO_STEP);
                messageDepth = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageDepth).toImageMsg();
                publisherDepth.publish(messageDepth);
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

//private: publish
//TODO: move frame publishing logic here
void rosNodeWidget::publish(const std::map<int, rs2::frame> rscFrames)
{

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
