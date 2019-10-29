//pointcloudWidget header file
//pointcloudWidget handles pointcloud functions, including:
//  creating pointcloud from bag
//  displaying pointcloud in ui
//  allowing manipulation of pointcloud view
//  creating static images of pointcloud to be returned to remote unit(s)

#ifndef POINTCLOUDWIDGET_HPP
#define POINTCLOUDWIDGET_HPP

//pointcloudWidget class, manages pointcloud functionality
//TODO: current functions are a guess at functionality required, update as needed
class pointcloudWidget
{
    public:
        //constructor
        pointcloudWidget();
        //destructor
        ~pointcloudWidget();

        //build a pointcloud from ROS bag
        void buildPointcloud();
        //display pointcloud in UI
        void displayPointcloud();
        //start manipulating pointcloud in UI
        void startManipulatingPointcloud();
        //stop manipulating pointcloud in UI
        void stopManipulatingPointcloud();
        //build image from pointcloud
        void capturePointcloud();
};

#endif
