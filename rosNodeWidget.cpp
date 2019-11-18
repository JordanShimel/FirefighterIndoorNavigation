//rosNodeWidget code file
//rosNodeWidget handles creating a ROS node and publishing data on it

#include "rosNodeWidget.hpp"
#include <iostream>
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
                         const std::string &colorTopicName, const std::string &depthTopicName,
                         const std::string &imuTopicName, const float &publishRate)
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
        //set member variables from parameters so run can access them without need to have them passed
        mColorTopicName = colorTopicName;
        mDepthTopicName = depthTopicName;
        mImuTopicName = imuTopicName;
        mPublishRate = publishRate;

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
    //start the node
    //we need to start it explicitly so that it doesn't stop until we want it to
    //otherwise it would close if the node handles all closed
    ros::start();

    //create a handle to our node
    ros::NodeHandle remoteUnitNodeHandle;

    //assign publishers to this node
    //color publisher
    image_transport::ImageTransport imageTransportColor(remoteUnitNodeHandle);
    mPublisherColor = imageTransportColor.advertise(mColorTopicName, 1);
    //depth publisher
    image_transport::ImageTransport imageTransportDepth(remoteUnitNodeHandle);
    mPublisherDepth = imageTransportDepth.advertise(mDepthTopicName, 1);
    //imu publisher
    mPublisherImu = remoteUnitNodeHandle.advertise<sensor_msgs::Imu>(mImuTopicName, 1);

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
        showError("No RealSense device detected.");
        return;
    }
    else if(rscDeviceCount == 1)
    {
        rs2::device rscDevice = rscDevices[0];
        rs2::pipeline rscPipe(rscContext);
        rs2::config rscConfig;
        rscConfig.enable_stream(RS2_STREAM_ACCEL);
        rscConfig.enable_stream(RS2_STREAM_COLOR, 0, 640, 480, RS2_FORMAT_BGR8, 30);
        rscConfig.enable_stream(RS2_STREAM_DEPTH, 0, 640, 480, RS2_FORMAT_Z16, 30);
        rscConfig.enable_stream(RS2_STREAM_GYRO);
        rscPipe.start(rscConfig);

        std::vector<rs2::sensor> rscSensors = rscDevice.query_sensors();
        rs2::sensor rscDepthSensor = rscSensors[0];
        rscDepthSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);

        rs2::align align_to_color(RS2_STREAM_COLOR);

        //create variables used to build messages
        //height of camera output
        int height = 0;
        //width of camera output
        int width = 0;

        //create message variables
        //color message
        rs2::frame rscPublishColorFrame;
        sensor_msgs::ImagePtr messageColor;
        //depth message
        rs2::frame rscPublishDepthFrame;
        sensor_msgs::ImagePtr messageDepth;
        //imu message
        rs2::frame rscTempAccelFrame;
        rs2::frame rscTempGyroFrame;
        sensor_msgs::Imu messageImu;

        rs2::decimation_filter rscDecimationFilter;
        rscDecimationFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0);
        rs2::threshold_filter rscThresholdFilter;
        rscThresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, 0.1);
        rscThresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, 16.0);
        rs2::spatial_filter rscSpatialFilter;
        rscSpatialFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
        rscSpatialFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
        rscSpatialFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0);
        //rscSpatialFilter.set_option(RS2_OPTION_HOLES_FILL, 0);
        rs2::temporal_filter rscTemporalFilter;
        rscTemporalFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
        rscTemporalFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0);
        //rscTemporalFilter.set_option(RS2_OPTION_FILTER, 3);
        rs2::disparity_transform rscDepthToDisparity(true);
        rs2::disparity_transform rscDisparityToDepth(false);

        //rosLoopRate is how many times the while loop will attempt to run per second
        ros::Rate rosLoopRate(mPublishRate);

        //as long as ROS hasn't been shutdown
        while(ros::ok())
        {
            std::vector<rs2::frame> rscNewFrames;
            rs2::frameset rscFrameSet;

            if(rscPipe.poll_for_frames(&rscFrameSet))
            {
                rscFrameSet = align_to_color.process(rscFrameSet);

                //TODO:add better comments for these parts - Jordan
                //color data publisher
                rscPublishColorFrame = rscFrameSet.first(RS2_STREAM_COLOR);
                width = rscPublishColorFrame.as<rs2::video_frame>().get_width();
                height = rscPublishColorFrame.as<rs2::video_frame>().get_height();
                cv::Mat imageColor(cv::Size(width, height), CV_8UC3, (void*)rscPublishColorFrame.get_data(), cv::Mat::AUTO_STEP);
                messageColor = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageColor).toImageMsg();
                mPublisherColor.publish(messageColor);

                //depth data publisher
                rscPublishDepthFrame = rscFrameSet.first(RS2_STREAM_DEPTH);

                //rscPublishDepthFrame = rscDecimationFilter.process(rscPublishDepthFrame);
                rscPublishDepthFrame = rscThresholdFilter.process(rscPublishDepthFrame);
                rscPublishDepthFrame = rscDepthToDisparity.process(rscPublishDepthFrame);
                rscPublishDepthFrame = rscSpatialFilter.process(rscPublishDepthFrame);
                rscPublishDepthFrame = rscTemporalFilter.process(rscPublishDepthFrame);
                rscPublishDepthFrame = rscDisparityToDepth.process(rscPublishDepthFrame);

                width = rscPublishDepthFrame.as<rs2::video_frame>().get_width();
                height = rscPublishDepthFrame.as<rs2::video_frame>().get_height();
                cv::Mat imageDepth(cv::Size(width, height), CV_16UC1, (void*)rscPublishDepthFrame.get_data(), cv::Mat::AUTO_STEP);
                imageDepth.convertTo(imageDepth, CV_8UC1, 255.0/1000);
                messageDepth = cv_bridge::CvImage(std_msgs::Header(), "mono8", imageDepth).toImageMsg();
                mPublisherDepth.publish(messageDepth);

                //imu data publisher
                rscTempAccelFrame = rscFrameSet.first(RS2_STREAM_ACCEL);
                rscTempGyroFrame = rscFrameSet.first(RS2_STREAM_GYRO);
                messageImu.angular_velocity.x = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().x;
                messageImu.angular_velocity.y = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().y;
                messageImu.angular_velocity.z = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().z;
                messageImu.linear_acceleration.x = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().x;
                messageImu.linear_acceleration.y = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().y;
                messageImu.linear_acceleration.z = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().z;
                mPublisherImu.publish(messageImu);
            }

            //run each publisher once
            ros::spinOnce();

            //sleep until time to rerun(specified by rosLoopRate)
            rosLoopRate.sleep();
        }

        rscPipe.stop();

        Q_EMIT rosShutdown();
    }
    else if(rscDeviceCount > 1)
    {
        showError("More than 1 RealSense device detected.");
        return;
    }
    else
    {
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
