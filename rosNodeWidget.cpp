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
bool rosNodeWidget::init(const std::string &rosMasterAddress, const std::string &rosLocalAddress)
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
        //if our addresses are functional, start the node
        //we need to start it explicitly so that it doesn't stop until we want it to
        //otherwise it would close if the node handles all closed
        ros::start();

        //create a handle to our node
        ros::NodeHandle nodeHandleBaseUnit;

        //assign subscribers to this node
        //acceleration subscriber
        //depth subscriber
        image_transport::ImageTransport imageTransportDepth(nodeHandleBaseUnit);
        subscriberDepth = imageTransportDepth.subscribe("rscDepth", 1, &rosNodeWidget::callbackDepth, this);
        //gyroscope subscriber

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
    //rosLoopRate is how many times the while loop will attempt to run per second
    ros::Rate rosLoopRate(1);

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

void rosNodeWidget::getAccel()
{

}

QImage rosNodeWidget::getDepth()
{
    return qimageDepth;
}

void rosNodeWidget::getGyro()
{

}

void rosNodeWidget::callbackAccel(const std_msgs::String::ConstPtr &accelMessage)
{

}

void rosNodeWidget::callbackDepth(const sensor_msgs::ImageConstPtr &depthMessage)
{
    //TODO:Add image message to Bag here

    //convert image message to CV image pointer
    cv_bridge::CvImagePtr testptr;
    testptr = cv_bridge::toCvCopy(depthMessage, sensor_msgs::image_encodings::BGR8);

    //TODO:Fix image format so color works properly

    //convert CV image pointer into a QImage and store in class member
    qimageDepth = QImage(testptr->image.data, testptr->image.rows, testptr->image.cols, testptr->image.step, QImage::Format_RGB32);

    //let main application know new depth data has been received
    Q_EMIT receivedDepth();
}

void rosNodeWidget::callbackGyro(const std_msgs::String::ConstPtr &gyroMessage)
{

}
