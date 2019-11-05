//mainWindow code file
//mainWindow handles the GUI for the remote unit application

#include "mainWindow.hpp"

#include <iostream>//DEBUG:TODO:REMOVE
//public: constructor
//creates ui instance
//connects Qt Signals used by the application
mainWindow::mainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::mainWindow)
{
    //initialize UI
    ui->setupUi(this);

    //set initial visibility
    ui->labelRosMasterIP->setVisible(false);
    ui->textEditRosMasterIP->setVisible(false);
    ui->textEditRosMasterIP->setText("http://jordan-VirtualBox:11311/");

    ui->labelRosLocalIP->setVisible(false);
    ui->textEditRosLocalIP->setVisible(false);
    ui->textEditRosLocalIP->setText("jordan-VirtualBox");

    ui->labelDepthTopicName->setVisible(false);
    ui->textEditDepthTopicName->setVisible(false);
    ui->textEditDepthTopicName->setText("rscDepth");

    ui->labelColorTopicName->setVisible(false);
    ui->textEditColorTopicName->setVisible(false);
    ui->textEditColorTopicName->setText("rscVideo");

    //connection to allow application to close rather than hang if ROS has to shutdown unexpectedly
    QObject::connect(&rosNode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

//public: destructor
//deletes ui and closes application
mainWindow::~mainWindow()
{
    delete ui;
}

//private slot: on_pushButtonPreview_clicked
//creates a preview window for live data from camera
void mainWindow::on_pushButtonTest_clicked()
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
        showError("Could not query RealSense devices.");
        return;
    }

    //get number of connected devices
    size_t rscDeviceCount = rscDevices.size();

    //number should be 1, as application is only meant to be used in situation with 1 camera
    if(rscDeviceCount == 0)
    {
        //display error and return
        showError("No RealSense device detected.");
        return;
    }
    else if(rscDeviceCount == 1)
    {
        //disable preview button
        //TODO: find out why this doesn't work(thread?)
        ui->pushButtonTest->setEnabled(false);

        //create interface for first(only) device
        rs2::device rscDevice = rscDevices[0];

        //get serial number of device
        const char* rscSerial = rscDevice.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        //create RS2 pipeline based on the current context
        rs2::pipeline rscPipe(rscContext);

        //create RS2 config
        //TODO: Determine optimum config settings
        rs2::config rscConfig;

        //enable the device(via serial number)
        rscConfig.enable_device(rscSerial);

        //start RS2 pipeline for device
        rscPipe.start(rscConfig);

        //create colorizer for device(changes depth data into RGB)
        rs2::colorizer *rscColorizer = new rs2::colorizer();

        //keep track of the last frame of each stream available to make the output persistent
        std::map<int, rs2::frame> rscRenderFrames;

        //create a viewerWidget for rendering
        class viewerWidget rscViewer;

        //main app loop, run while viewerWidget is open
        while(rscViewer)
        {
            //create a vector to hold new frames from the device
            std::vector<rs2::frame> rscNewFrames;

            //create RS2 frameset
            rs2::frameset rscFrameset;

            //check to see if the frameset has new frames
            if(rscPipe.poll_for_frames(&rscFrameset))
            {
                //iterate through any new frames until frameset is empty
                for(const rs2::frame& rscFrame : rscFrameset)
                    //place new frames into new_frames vector
                    rscNewFrames.emplace_back(rscFrame);
            }

            //iterate through new_frames vector
            for(const auto& rscFrame : rscNewFrames)
            {
                //apply the colorizer and add to map
                rscRenderFrames[rscFrame.get_profile().unique_id()] = rscColorizer->process(rscFrame);
            }

            //output the new frames
            rscViewer.show(rscRenderFrames);
        }

        //stop RS2 pipeline
        rscPipe.stop();

        //reenable preview button
        ui->pushButtonTest->setEnabled(true);
    }
    else if(rscDeviceCount > 1)
    {
        //display error and return
        showError("More than 1 RealSense device detected.");
        return;
    }
    else
    {
        //display error and return
        showError("Unknown number of RealSense devices detected.");
        return;
    }

}

//private slot: on_pushButtonConnect_clicked
//creates ROS node if one doesn't already exist
//starts publishing selected data on ROS node
//subsequent clicks will toggle publishing QThread
void mainWindow::on_pushButtonSend_clicked()
{
    if(isPublishing == false)
    {
        //attempt to start rosNodeWidget with inputted values
        if(!rosNode.init(ui->textEditRosMasterIP->toPlainText().toStdString(), ui->textEditRosLocalIP->toPlainText().toStdString(),
                         ui->textEditDepthTopicName->toPlainText().toStdString(), ui->textEditColorTopicName->toPlainText().toStdString()))
        {
            //if we fail, it should be because the addresses for ROS master and/or ROS local are incorrect
            showError("Could not connect to ROS master.");
        }
        else
        {
            //set publishing flag to true and update buttons
            isPublishing = true;
            ui->pushButtonSend->setText("STOP");
            ui->pushButtonTest->setText("TEST"
                                        "(DISABLED))");
            ui->pushButtonTest->setEnabled(false);
        }
    }
    else
    {
        //attempt to terminate publishing QThread
        if(rosNode.stop())
        {
            //set publishing flag to false and update button
            isPublishing = false;
            ui->pushButtonSend->setText("SEND");
            ui->pushButtonTest->setText("TEST");
            ui->pushButtonTest->setEnabled(true);
        }
        else
        {
            //show error
            showError("Could not stop publisher.");
        }
    }
}

//private slot: on_pushButtonConfig_clicked
//on first click
// hides main ui elements
// makes config elements visible
// turns config button into done button
//on second click
// reverts to pre first click state
void mainWindow::on_pushButtonConfig_clicked()
{
    if(isConfig == false)
    {
        ui->labelRosMasterIP->setVisible(true);
        ui->textEditRosMasterIP->setVisible(true);

        ui->labelRosLocalIP->setVisible(true);
        ui->textEditRosLocalIP->setVisible(true);

        ui->labelDepthTopicName->setVisible(true);
        ui->textEditDepthTopicName->setVisible(true);

        ui->labelColorTopicName->setVisible(true);
        ui->textEditColorTopicName->setVisible(true);

        ui->pushButtonTest->setVisible(false);
        ui->pushButtonSend->setVisible(false);
        ui->pushButtonView->setVisible(false);
        ui->pushButtonConfig->setText("DONE");

        isConfig = true;
    }
    else if(isConfig == true)
    {
        ui->labelRosMasterIP->setVisible(false);
        ui->textEditRosMasterIP->setVisible(false);

        ui->labelRosLocalIP->setVisible(false);
        ui->textEditRosLocalIP->setVisible(false);

        ui->labelDepthTopicName->setVisible(false);
        ui->textEditDepthTopicName->setVisible(false);

        ui->labelColorTopicName->setVisible(false);
        ui->textEditColorTopicName->setVisible(false);

        ui->pushButtonTest->setVisible(true);
        ui->pushButtonSend->setVisible(true);
        ui->pushButtonView->setVisible(true);
        ui->pushButtonConfig->setText("CONFIG");

        isConfig = false;
    }
}

//private: showError
//outputs error message with passed text
void mainWindow::showError(QString errorMessage)
{
    //display error and return
    QMessageBox errorMessageBox;
    errorMessageBox.setText(errorMessage);
    errorMessageBox.exec();
    return;
}
