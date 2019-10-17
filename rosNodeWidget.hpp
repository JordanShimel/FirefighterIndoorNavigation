//rosNodeWidget header file
//rosNodeWidget handles creating a ROS node and subscribing to and publishing data on it

#ifndef ROSNODEWIDGET_HPP
#define ROSNODEWIDGET_HPP

//rosNodeWidget class, manages ROS node and publishing
//TODO: current functions are a guess at functionality required, update as needed
class rosNodeWidget
{
    public:
        //constructor
        rosNodeWidget();
        //destructor
        ~rosNodeWidget();

        //make a ROS node
        void init();
        //start subscribing on the ROS node
        void startSubscribing();
        //stop subscribing on the ROS node
        void stopSubscribing();
        //start publishing on the ROS node
        void startPublishing();
        //stop publishing on the ROS node
        void stopPublishing();
};

#endif
