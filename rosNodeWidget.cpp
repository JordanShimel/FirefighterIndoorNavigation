//rosNodeWidget code file
//rosNodeWidget handles creating a ROS node and subscribing to and publishing data on it

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
//set up topics to subscribe via that node
//creates thread to actually subscribe to messages
bool rosNodeWidget::init()
{
    //create a map with the master and local addresses to pass to the ROS init function
    std::map<std::string, std::string> rosAddresses;
    rosAddresses["__master"] = settingRosMasterAddress;
    rosAddresses["__hostname"] = settingRosLocalAddress;

    //initialize a node with our addresses
    ros::init(rosAddresses, "baseUnitNode");

    //check to see if our addresses are functional
    if(!ros::master::check())
    {
        //if not, return false now
        return false;
    }
    else
    {
        //starts Qt thread to run the subscriber
        //does thread stuff and has thread call run()
        start();
        return true;
    }
}

//public: run
//extends QThread run
//is called by start
//handles main subscriber loop
void rosNodeWidget::run()
{
    //if our addresses are functional, start the node
    //we need to start it explicitly so that it doesn't stop until we want it to
    //otherwise it would close if the node handles all closed
    ros::start();

    //create a handle to our node
    ros::NodeHandle nodeHandleBaseUnit;

    //code for ORB_SLAM2 stuff
    //create a new SLAM system, passing the prebuilt vocab and camera settings, set as type RGBD and use the built in renderer
    ORB_SLAM2::System rscSLAM("./ORBvoc.txt", "./camera.yaml", ORB_SLAM2::System::RGBD, true);
    //create a pointCloudWidget with our SLAM system
    pointCloudWidget rscPointCloud(&rscSLAM);
    //add the message filter subscribers for the incoming color and depth messages
    message_filters::Subscriber<sensor_msgs::Image> colorSubscriber(nodeHandleBaseUnit, settingColorTopicName, 1);
    message_filters::Subscriber<sensor_msgs::Image> depthSubscriber(nodeHandleBaseUnit, settingDepthTopicName, 1);
    //set up the system to synchronize the color and depth frames
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), colorSubscriber, depthSubscriber);
    //finally, call the pointCloudWidget's grabRGBD function with the RGBD data
    sync.registerCallback(boost::bind(&pointCloudWidget::processFrames, &rscPointCloud, _1, _2));

    //rosLoopRate is how many times the while loop will attempt to run per second
    ros::Rate rosLoopRate(settingRefreshRate);

    int spinCount = 0;
    //as long as ROS hasn't been shutdown
    while(ros::ok())
    {
        //check each subscribers once
        ros::spinOnce();

        //after 50 spins, attempt to capture ORB_SLAM2 windows into main ui
        //TODO: the number of spins is meant to ensure that all 4 child windows are created before
        //capture, the timing may need to be adjusted for hardware changes, or a more robust method of checking for them developed
        if(spinCount < 50)
        {
            spinCount++;
            if(spinCount == 50)
            {
                Q_EMIT grabSLAMWindows();
            }
        }

        //sleep until time to rerun(specified by rosLoopRate)
        rosLoopRate.sleep();
    }

    //shutdown the point cloud object if ROS fails
    rscPointCloud.shutdown();

    //let main application know ROS has shutdown
    Q_EMIT rosShutdown();
}

//public: stop
//terminates the QThread subscribing to messages
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
    settingRefreshRate = fileSettings.value("REFRESH_RATE", "").toFloat();
}
