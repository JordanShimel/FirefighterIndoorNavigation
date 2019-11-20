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
bool rosNodeWidget::init()
{
    //create a map with the master and local addresses to pass to the ROS init function
    std::map<std::string, std::string> rosAddresses;
    rosAddresses["__master"] = settingRosMasterAddress;
    rosAddresses["__hostname"] = settingRosLocalAddress;

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
    publisherColor = imageTransportColor.advertise(settingColorTopicName, 1);
    //depth publisher
    image_transport::ImageTransport imageTransportDepth(remoteUnitNodeHandle);
    publisherDepth = imageTransportDepth.advertise(settingDepthTopicName, 1);
    //imu publisher
    publisherImu = remoteUnitNodeHandle.advertise<sensor_msgs::Imu>(settingImuTopicName, 1);

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
        rscDepthSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, settingAutoExposure);

        rs2::align rscAlign(RS2_STREAM_COLOR);

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

        rs2::threshold_filter rscThresholdFilter;
        rscThresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, settingThresholdMin);
        rscThresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, settingThresholdMax);
        rs2::disparity_transform rscDepthToDisparity(true);
        rs2::spatial_filter rscSpatialFilter;
        rscSpatialFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE, settingSpatialMagnitude);
        rscSpatialFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, settingSpatialAlpha);
        rscSpatialFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, settingSpatialDelta);
        rs2::temporal_filter rscTemporalFilter;
        rscTemporalFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, settingTemporalAlpha);
        rscTemporalFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, settingTemporalDelta);
        rs2::disparity_transform rscDisparityToDepth(false);

        //rosLoopRate is how many times the while loop will attempt to run per second
        ros::Rate rosLoopRate(settingPublishRate);

        //as long as ROS hasn't been shutdown
        while(ros::ok())
        {
            std::vector<rs2::frame> rscNewFrames;
            rs2::frameset rscFrameSet;

            if(rscPipe.poll_for_frames(&rscFrameSet))
            {
                rscFrameSet = rscAlign.process(rscFrameSet);

                //TODO:add better comments for these parts - Jordan
                //color data publisher
                rscPublishColorFrame = rscFrameSet.first(RS2_STREAM_COLOR);
                width = rscPublishColorFrame.as<rs2::video_frame>().get_width();
                height = rscPublishColorFrame.as<rs2::video_frame>().get_height();
                cv::Mat imageColor(cv::Size(width, height), CV_8UC3, (void*)rscPublishColorFrame.get_data(), cv::Mat::AUTO_STEP);
                messageColor = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageColor).toImageMsg();
                publisherColor.publish(messageColor);

                //depth data publisher
                rscPublishDepthFrame = rscFrameSet.first(RS2_STREAM_DEPTH);

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
                publisherDepth.publish(messageDepth);

                //imu data publisher
                rscTempAccelFrame = rscFrameSet.first(RS2_STREAM_ACCEL);
                rscTempGyroFrame = rscFrameSet.first(RS2_STREAM_GYRO);
                messageImu.angular_velocity.x = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().x;
                messageImu.angular_velocity.y = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().y;
                messageImu.angular_velocity.z = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().z;
                messageImu.linear_acceleration.x = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().x;
                messageImu.linear_acceleration.y = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().y;
                messageImu.linear_acceleration.z = rscTempGyroFrame.as<rs2::motion_frame>().get_motion_data().z;
                publisherImu.publish(messageImu);
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

//public: loadSettings
//loads settings from file
void rosNodeWidget::loadSettings()
{
    //settings file
    QSettings fileSettings("./config.ini", QSettings::NativeFormat);

    settingRosMasterAddress = fileSettings.value("ROS_MASTER_ADDRESS", "").toString().toStdString();
    settingRosLocalAddress = fileSettings.value("ROS_LOCAL_ADDRESS", "").toString().toStdString();
    settingColorTopicName = fileSettings.value("COLOR_TOPIC_NAME", "").toString().toStdString();
    settingDepthTopicName = fileSettings.value("DEPTH_TOPIC_NAME", "").toString().toStdString();
    settingImuTopicName = fileSettings.value("IMU_TOPIC_NAME", "").toString().toStdString();
    settingPublishRate = fileSettings.value("PUBLISH_RATE", "").toFloat();
    settingAutoExposure = fileSettings.value("AUTO_EXPOSURE", "").toBool();
    settingThresholdMin = fileSettings.value("THRESHOLD_MIN", "").toFloat();
    settingThresholdMax = fileSettings.value("THRESHOLD_MAX", "").toFloat();
    settingSpatialMagnitude = fileSettings.value("SPATIAL_MAGNITUDE", "").toFloat();
    settingSpatialAlpha = fileSettings.value("SPATIAL_ALPHA", "").toFloat();
    settingSpatialDelta = fileSettings.value("SPATIAL_DELTA", "").toFloat();
    settingTemporalAlpha = fileSettings.value("TEMPORAL_ALPHA", "").toFloat();
    settingTemporalDelta = fileSettings.value("TEMPORAL_DELTA", "").toFloat();
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
