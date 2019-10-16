//FireFighterRemoteUnit mainwindow code file
//TODO?: Implement local configuration system

//RealSense Cross Platform API
//Used to provide Intel RealSense Camera interface
#include <librealsense2/rs.hpp>

#include "mainwindow.h"
#include "ui_mainwindow.h"

//STL includes required for frame maps and vectors
#include <map>
#include <vector>

//viewer class handles the preview window
#include "viewer.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    //Initialize UI
    ui->setupUi(this);
    //Initialize status message and buttons
    ui->pushButtonPreview->setEnabled(true);
    ui->pushButtonConnect->setEnabled(true);
    ui->labelStatus->setText("Initialization complete.");
    ui->labelStatus->setEnabled(true);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButtonPreview_clicked()
{
    //Create librealsense context for managing devices
    rs2::context rscContext;
    ui->labelStatus->setText("rs2::context rscContext;");

    //Create device list
    rs2::device_list rscDevices;

    //Get list of connected devices
    try
    {
        rscDevices = rscContext.query_devices();
    }
    catch(const rs2::error & e)
    {
        ui->labelStatus->setText("rscDevices = rscContext.query_devices(); failed.");
        return;
    }

    //Get number of connected devices
    size_t rscDeviceCount = rscDevices.size();

    //Verify number of devices
    if(rscDeviceCount == 0)
    {
        ui->labelStatus->setText("No camera detected, ensure the R435 is connected and retry.");
        return;
    }
    else if(rscDeviceCount == 1)
    {
        ui->labelStatus->setText("Camera detected");
        ui->labelStatus->update();

        //Create interface for first device
        rs2::device rscDevice = rscDevices[0];

        //Get serial number of device
        const char* rscSerial = rscDevice.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        //Create RS2 pipeline based on the current context
        rs2::pipeline rscPipe(rscContext);

        //Create RS2 config
        //TODO: Determine optimum config settings
        rs2::config rscConfig;

        //Enable the device(via serial number)
        rscConfig.enable_device(rscSerial);

        //Start RS2 pipeline for device
        rscPipe.start(rscConfig);

        //Initialize colorizer for device(changes depth data into RGB)
        rs2::colorizer *rscColorizer = new rs2::colorizer();

        //Keep track of the last frame of each stream available to make the presentation persistent
        std::map<int, rs2::frame> rscRenderFrames;

        //Create a simple OpenGL window for rendering
        //TODO: Switch to Qt OpenGL Widget
        class viewer rscViewer;

        //Main app loop, run while app window is open
        //TODO: Replace with something based on Qt
        while(rscViewer)
        {
            //Create a vector to hold new frames from the device
            std::vector<rs2::frame> rscNewFrames;

            //Create RS2 frameset
            rs2::frameset rscFrameset;

            //Check to see if the frameset has new frames
            if(rscPipe.poll_for_frames(&rscFrameset))
            {
                //Iterate through any new frames until frameset is empty
                for(const rs2::frame& rscFrame : rscFrameset)
                    //Place new frames into new_frames vector
                    rscNewFrames.emplace_back(rscFrame);
            }

            //Iterate through new_frames vector
            for(const auto& rscFrame : rscNewFrames)
            {
                //Apply the colorizer and store the colorized frame
                rscRenderFrames[rscFrame.get_profile().unique_id()] = rscColorizer->process(rscFrame);
            }

            //Present all the collected frames with openGl mosaic
            //TODO: Replace with Qt based renderer
            rscViewer.show(rscRenderFrames);
        }

        //Stop RS2 pipeline
        rscPipe.stop();
    }
    else if(rscDeviceCount > 1)
    {
        ui->labelStatus->setText("Multiple cameras detected, remove all but one R435 and retry.");
        return;
    }
    else
    {
        ui->labelStatus->setText("Negative or non number of cameras detected, shouldn't be possible, fix please");
        return;
    }

    ui->labelStatus->setText("Preview ended.");
    ui->labelStatus->update();
}

void MainWindow::on_pushButtonConnect_clicked()
{

}
