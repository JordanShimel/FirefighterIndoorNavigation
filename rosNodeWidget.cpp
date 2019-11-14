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
bool rosNodeWidget::init(const std::string &rosMasterAddress, const std::string &rosLocalAddress,
          const std::string &colorTopicName, const std::string &depthTopicName,
          const std::string &imuTopicName, const float &refreshRate)
{
    //create a map with the master and local addresses to pass to the ROS init function
    std::map<std::string, std::string> rosAddresses;
    rosAddresses["__master"] = rosMasterAddress;
    rosAddresses["__hostname"] = rosLocalAddress;

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
        //set internal values to parameters to enable run loop to read them
        mColorTopicName = colorTopicName;
        mDepthTopicName = depthTopicName;
        mImuTopicName = imuTopicName;
        mRefreshRate = refreshRate;

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

    //assign subscribers to this node
    //depth subscriber
    //image_transport::ImageTransport imageTransportDepth(nodeHandleBaseUnit);
    //subscriberDepth = imageTransportDepth.subscribe("rscDepth", 1, &rosNodeWidget::callbackDepth, this);

    //Color subscriber
    //image_transport::ImageTransport imageTransportColor(nodeHandleBaseUnit);
    //subscriberColor = imageTransportColor.subscribe("rscColor", 1, &rosNodeWidget::callbackColor, this);

    //IMU subscriber
    //subscriberIMU = nodeHandleBaseUnit.subscribe("rscImu", 1, &rosNodeWidget::callbackIMU, this);

    //code for ORB_SLAM stuff
    ORB_SLAM2::System SLAM("./ORBvoc.txt", "./camera.yaml", ORB_SLAM2::System::RGBD, true);
    pointcloudWidget pcw(&SLAM);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nodeHandleBaseUnit, "rscColor", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nodeHandleBaseUnit, "rscDepth", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&pointcloudWidget::grabRGBD, &pcw, _1, _2));

    //rosLoopRate is how many times the while loop will attempt to run per second
    ros::Rate rosLoopRate(10);

    //as long as ROS hasn't been shutdown
    while(ros::ok())
    {
        //check all subscribers
        ros::spinOnce();

        //sleep until time to rerun(specified by rosLoopRate)
        rosLoopRate.sleep();
    }

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

void rosNodeWidget::callbackDepth(const sensor_msgs::ImageConstPtr &depthMessage)
{
    depthMessageContainer = depthMessage;
}

void rosNodeWidget::callbackColor(const sensor_msgs::ImageConstPtr &colorMessage)
{
    colorMessageContainer = colorMessage;
}

void rosNodeWidget::callbackIMU(const sensor_msgs::Imu &imuMessage)
{
    imuMessageContainer = imuMessage;
}
