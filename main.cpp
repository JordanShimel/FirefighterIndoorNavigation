//FireFighterRemoteUnit main code file
//Serves as the code entry point for execution
//Currently runs a modified version of Intel's rs-multicamera sample program, edited to output all data from a single D435i camera
//TODO: Refactor to use Qt libraries instead of Intel example library for window and rendering
//TODO: Add capability to capture data from camera and convert it into a ROSBag
//TODO: Add capability to transfer ROSBag to base unit over Wi-Fi
//TODO?: Local/remote management system

//RealSense Cross Platform API
//Used to provide Intel RealSense Camera interface
#include <librealsense2/rs.hpp>

//This is Intel's 'helper' library for their examples
//TODO: Switch to Qt based system and use Qt libraries
#include "example.hpp"

//STL includes required for frame maps and vectors
#include <map>
#include <vector>

//Default Qt includes
#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    //Create a simple OpenGL window for rendering
    //TODO: Switch to Qt OpenGL Widget
    window app(1280, 960, "Camera Output Example");

    //Create librealsense context for managing devices
    rs2::context ctx;

    //Create device list
    rs2::device_list devices;
    //Get list of connected devices
    //TODO: Exception Handling here instead of device_count check below
    try{
        devices = ctx.query_devices();
    }
    catch(const rs2::error & e){
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch(const std::exception & e){
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    //Check number of connected devices
    size_t device_count = devices.size();

    //Exit if more or less than 1 device
    if(device_count != 1)
        return EXIT_FAILURE;

    //Create interface for first device
    rs2::device dev = devices[0];

    //Get serial number of device
    const char* device_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    //Create RS2 pipeline based on the current context
    rs2::pipeline pipe(ctx);

    //Create RS2 config
    //TODO: Determine optimum config settings
    rs2::config cfg;

    //Enable the device(via serial number)
    cfg.enable_device(device_serial);

    //Start RS2 pipeline for device
    pipe.start(cfg);

    //Initialize colorizer for device(changes depth data into RGB)
    rs2::colorizer *colorizer = new rs2::colorizer();

    //Keep track of the last frame of each stream available to make the presentation persistent
    std::map<int, rs2::frame> render_frames;

    //Main app loop, run while app window is open
    //TODO: Replace with something based on Qt
    while(app)
    {
        //Create a vector to hold new frames from the device
        std::vector<rs2::frame> new_frames;

        //Create RS2 frameset
        rs2::frameset fs;

        //Check to see if the frameset has new frames
        if(pipe.poll_for_frames(&fs))
        {
            //Iterate through any new frames until frameset is empty
            for(const rs2::frame& f : fs)
                //Place new frames into new_frames vector
                new_frames.emplace_back(f);
        }

        //Iterate through new_frames vector
        for(const auto& frame : new_frames)
        {
            //Apply the colorizer and store the colorized frame
            render_frames[frame.get_profile().unique_id()] = colorizer->process(frame);
        }

        //Present all the collected frames with openGl mosaic
        //TODO: Replace with Qt based renderer
        app.show(render_frames);
    }

    //Stop RS2 pipeline
    pipe.stop();

    return EXIT_SUCCESS;


    //Basic code to run a Qt based window
    //TODO: Move application code(above) into Qt window
    //QApplication application(argc, argv);
    //MainWindow FireFighterRemoteUnit_Window;
    //FireFighterRemoteUnit_Window.show();
    //return application.exec();
}
