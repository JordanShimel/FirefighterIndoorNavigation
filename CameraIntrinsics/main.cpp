//simple helper program to grab camera intrinsics from an Intel d435i camera
//the camera intrinsic values should be similar among all such cameras, but tiny variations can exist
//these values are used to create the camera.yaml file in the firefigherbaseunit project
#include <librealsense2/rs.hpp>
#include <QCoreApplication>
#include <iostream>
int main(int argc, char *argv[])
{
    //create librealsense context for managing devices
    rs2::context rscContext;

    //create device list
    rs2::device_list rscDevices;

    //get list of connected devices
    try
    {
        rscDevices = rscContext.query_devices();
    }
    //return if list cannot be obtained
    catch(const rs2::error & e)
    {
        //display error and return
        printf("Could not query RealSense devices.");
        return 0;
    }

    //get number of connected devices
    size_t rscDeviceCount = rscDevices.size();

    //number should be 1, as application is only meant to be used in situation with 1 camera
    if(rscDeviceCount == 0)
    {
        //display error and return
        printf("No RealSense device detected.");
        return 0;
    }
    else if(rscDeviceCount == 1)
    {
        //create interface for first(only) device
        rs2::device rscDevice = rscDevices[0];

        //get serial number of device
        const char* rscSerial = rscDevice.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        //create RS2 pipeline based on the current context
        rs2::pipeline rscPipe(rscContext);


        //create RS2 config
        rs2::config rscConfig;

        //enable the device(via serial number)
        rscConfig.enable_device(rscSerial);

        //start RS2 pipeline for device
        rscPipe.start(rscConfig);

        //output intrinsics to console
        std::cout << "COLOR:" << std::endl;
        std::cout << "fx: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().fx << std::endl;
        std::cout << "fy: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().fy << std::endl;
        std::cout << "ppx: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().ppx << std::endl;
        std::cout << "ppy: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().ppy << std::endl;
        std::cout << "model: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().model << std::endl;
        std::cout << "width: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().width << std::endl;
        std::cout << "coeffs: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().coeffs << std::endl;
        std::cout << "height: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().height << std::endl;
        std::cout << "DEPTH:" << std::endl;
        std::cout << "fx: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics().fx << std::endl;
        std::cout << "fy: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics().fy << std::endl;
        std::cout << "ppx: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics().ppx << std::endl;
        std::cout << "ppy: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics().ppy << std::endl;
        std::cout << "model: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics().model << std::endl;
        std::cout << "width: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics().width << std::endl;
        std::cout << "coeffs: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics().coeffs << std::endl;
        std::cout << "height: " << rscPipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics().height << std::endl;


        //stop RS2 pipeline
        rscPipe.stop();
    }
    else if(rscDeviceCount > 1)
    {
        //display error and return
        printf("More than 1 RealSense device detected.");
        return 0;
    }
    else
    {
        //display error and return
        printf("Unknown number of RealSense devices detected.");
        return 0;
    }
}
