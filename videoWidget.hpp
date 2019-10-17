//videoWidget header file
//videoWidget handles video functions, including:
//  creating ROS bag file from incoming video(depth) images
//  displaying live feed of video(depth) images in UI
//  allowing optional capture of live video(depth) feed to disk
//  displaying additional information about video(depth) feed in UI

#ifndef VIDEOWIDGET_HPP
#define VIDEOWIDGET_HPP

//videoWidget class, manages video functionality
//TODO: current functions are a guess at functionality required, update as needed
class videoWidget
{
    public:
        //constructor
        videoWidget();
        //destructor
        ~videoWidget();

        //build bag file from incoming images
        void buildBagFile();
        //show video feed in UI
        void showVideo();
        //capture video to disk
        void captureVideo();
        //show additional video information
        void detailsVideo();
};

#endif
