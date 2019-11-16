//mainWindow code file
//mainWindow handles the GUI for the remote unit application

#include "mainWindow.hpp"

//public: constructor
//creates ui instance
//initializes ui elements
//connects Qt Signals used by the application
mainWindow::mainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::mainWindow)
{
    //initialize UI
    ui->setupUi(this);

    //set initial visibility
    initUI();

    //connection to allow application to close rather than hang if ROS has to shutdown unexpectedly
    QObject::connect(&rosNode, SIGNAL(rosShutdown()), this, SLOT(close()));

    //load settings from file
    loadSettings();

}

//public: destructor
//saves settings, deletes ui, and closes application
mainWindow::~mainWindow()
{
    saveSettings();
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
        //TODO: find out why this doesn't work(thread?) - Jordan
        ui->pushButtonTest->setEnabled(false);

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
                         ui->textEditColorTopicName->toPlainText().toStdString(), ui->textEditDepthTopicName->toPlainText().toStdString(),
                         ui->textEditImuTopicName->toPlainText().toStdString(), ui->textEditPublishRate->toPlainText().toFloat()))
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

        ui->labelColorTopicName->setVisible(true);
        ui->textEditColorTopicName->setVisible(true);

        ui->labelDepthTopicName->setVisible(true);
        ui->textEditDepthTopicName->setVisible(true);

        ui->labelImuTopicName->setVisible(true);
        ui->textEditImuTopicName->setVisible(true);

        ui->labelPublishRate->setVisible(true);
        ui->textEditPublishRate->setVisible(true);

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

        ui->labelColorTopicName->setVisible(false);
        ui->textEditColorTopicName->setVisible(false);

        ui->labelDepthTopicName->setVisible(false);
        ui->textEditDepthTopicName->setVisible(false);

        ui->labelImuTopicName->setVisible(false);
        ui->textEditImuTopicName->setVisible(false);

        ui->labelPublishRate->setVisible(false);
        ui->textEditPublishRate->setVisible(false);

        ui->pushButtonTest->setVisible(true);
        ui->pushButtonSend->setVisible(true);
        ui->pushButtonView->setVisible(true);
        ui->pushButtonConfig->setText("CONFIG");

        isConfig = false;

        saveSettings();
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

//private: initUI
//sets initial visibility for some ui elements
void mainWindow::initUI()
{
    ui->labelRosMasterIP->setVisible(false);
    ui->textEditRosMasterIP->setVisible(false);

    ui->labelRosLocalIP->setVisible(false);
    ui->textEditRosLocalIP->setVisible(false);

    ui->labelColorTopicName->setVisible(false);
    ui->textEditColorTopicName->setVisible(false);

    ui->labelDepthTopicName->setVisible(false);
    ui->textEditDepthTopicName->setVisible(false);

    ui->labelImuTopicName->setVisible(false);
    ui->textEditImuTopicName->setVisible(false);

    ui->labelPublishRate->setVisible(false);
    ui->textEditPublishRate->setVisible(false);
}

//private: loadSettings
//read and load saved settings
void mainWindow::loadSettings()
{
    //settings file
    QSettings fileSettings("./config.ini", QSettings::NativeFormat);

    //ros master IP
    if(fileSettings.value("ROS_MASTER_IP", "").toString().toStdString() == "")
    {
        ui->textEditRosMasterIP->setText("localhost:11311");
    }
    else
    {
        ui->textEditRosMasterIP->setText(fileSettings.value("ROS_MASTER_IP", "").toString());
    }
    //ros local IP
    if(fileSettings.value("ROS_LOCAL_IP", "").toString().toStdString() == "")
    {
        ui->textEditRosLocalIP->setText("localhost");
    }
    else
    {
        ui->textEditRosLocalIP->setText(fileSettings.value("ROS_LOCAL_IP", "").toString());
    }
    //color topic
    if(fileSettings.value("COLOR_TOPIC_NAME", "").toString().toStdString() == "")
    {
        ui->textEditColorTopicName->setText("rscColor");
    }
    else
    {
        ui->textEditColorTopicName->setText(fileSettings.value("COLOR_TOPIC_NAME", "").toString());
    }
    //depth topic
    if(fileSettings.value("DEPTH_TOPIC_NAME", "").toString().toStdString() == "")
    {
        ui->textEditDepthTopicName->setText("rscDepth");
    }
    else
    {
        ui->textEditDepthTopicName->setText(fileSettings.value("DEPTH_TOPIC_NAME", "").toString());
    }
    //imu topic
    if(fileSettings.value("IMU_TOPIC_NAME", "").toString().toStdString() == "")
    {
        ui->textEditImuTopicName->setText("rscImu");
    }
    else
    {
        ui->textEditImuTopicName->setText(fileSettings.value("IMU_TOPIC_NAME", "").toString());
    }
    //publish rate
    if(fileSettings.value("PUBLISH_RATE", "").toString().toStdString() == "")
    {
        ui->textEditPublishRate->setText("30");
    }
    else
    {
        ui->textEditPublishRate->setText(fileSettings.value("PUBLISH_RATE", "").toString());
    }
}

//private: saveSetings
//save settings to disk
void mainWindow::saveSettings()
{
    //settings file
    QSettings fileSettings("./config.ini", QSettings::NativeFormat);

    fileSettings.setValue("ROS_MASTER_IP", ui->textEditRosMasterIP->toPlainText());
    fileSettings.setValue("ROS_LOCAL_IP", ui->textEditRosLocalIP->toPlainText());
    fileSettings.setValue("COLOR_TOPIC_NAME", ui->textEditColorTopicName->toPlainText());
    fileSettings.setValue("DEPTH_TOPIC_NAME", ui->textEditDepthTopicName->toPlainText());
    fileSettings.setValue("IMU_TOPIC_NAME", ui->textEditImuTopicName->toPlainText());
    fileSettings.setValue("PUBLISH_RATE", ui->textEditPublishRate->toPlainText());
}
