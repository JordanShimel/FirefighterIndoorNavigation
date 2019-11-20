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

    //load settings from file
    loadSettings();

    //load settings in rosNode
    rosNode.loadSettings();

    //connection to allow application to close rather than hang if ROS has to shutdown unexpectedly
    QObject::connect(&rosNode, SIGNAL(rosShutdown()), this, SLOT(close()));

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

        //set viewer size
        rscViewer.setSize(640, 480);

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
        if(!rosNode.init())
        {
            //if we fail, it should be because the addresses for ROS master and/or ROS local are incorrect
            showError("Could not connect to ROS master.");
        }
        else
        {
            //set publishing flag to true and update buttons
            isPublishing = true;
            ui->pushButtonSend->setText("STOP");
            ui->pushButtonConfig->setVisible(false);
            ui->pushButtonTest->setVisible(false);
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
            ui->pushButtonConfig->setVisible(true);
            ui->pushButtonTest->setVisible(true);
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
        ui->labelSettingRosMasterAddress->setVisible(true);
        ui->textEditSettingRosMasterAddress->setVisible(true);

        ui->labelSettingRosLocalAddress->setVisible(true);
        ui->textEditSettingRosLocalAddress->setVisible(true);

        ui->labelSettingColorTopicName->setVisible(true);
        ui->textEditSettingColorTopicName->setVisible(true);

        ui->labelSettingDepthTopicName->setVisible(true);
        ui->textEditSettingDepthTopicName->setVisible(true);

        ui->labelSettingImuTopicName->setVisible(true);
        ui->textEditSettingImuTopicName->setVisible(true);

        ui->labelSettingPublishRate->setVisible(true);
        ui->textEditSettingPublishRate->setVisible(true);

        ui->labelSettingAutoExposure->setVisible(true);
        ui->checkBoxSettingAutoExposure->setVisible(true);

        ui->labelSettingThresholdMin->setVisible(true);
        ui->textEditSettingThresholdMin->setVisible(true);

        ui->labelSettingThresholdMax->setVisible(true);
        ui->textEditSettingThresholdMax->setVisible(true);

        ui->labelSettingSpatialMagnitude->setVisible(true);
        ui->textEditSettingSpatialMagnitude->setVisible(true);

        ui->labelSettingSpatialAlpha->setVisible(true);
        ui->textEditSettingSpatialAlpha->setVisible(true);

        ui->labelSettingSpatialDelta->setVisible(true);
        ui->textEditSettingSpatialDelta->setVisible(true);

        ui->labelSettingTemporalAlpha->setVisible(true);
        ui->textEditSettingTemporalAlpha->setVisible(true);

        ui->labelSettingTemporalDelta->setVisible(true);
        ui->textEditSettingTemporalDelta->setVisible(true);

        ui->pushButtonTest->setVisible(false);
        ui->pushButtonSend->setVisible(false);
        ui->pushButtonConfig->setText("DONE");

        isConfig = true;
    }
    else if(isConfig == true)
    {
        ui->labelSettingRosMasterAddress->setVisible(false);
        ui->textEditSettingRosMasterAddress->setVisible(false);

        ui->labelSettingRosLocalAddress->setVisible(false);
        ui->textEditSettingRosLocalAddress->setVisible(false);

        ui->labelSettingColorTopicName->setVisible(false);
        ui->textEditSettingColorTopicName->setVisible(false);

        ui->labelSettingDepthTopicName->setVisible(false);
        ui->textEditSettingDepthTopicName->setVisible(false);

        ui->labelSettingImuTopicName->setVisible(false);
        ui->textEditSettingImuTopicName->setVisible(false);

        ui->labelSettingPublishRate->setVisible(false);
        ui->textEditSettingPublishRate->setVisible(false);

        ui->labelSettingAutoExposure->setVisible(false);
        ui->checkBoxSettingAutoExposure->setVisible(false);

        ui->labelSettingThresholdMin->setVisible(false);
        ui->textEditSettingThresholdMin->setVisible(false);

        ui->labelSettingThresholdMax->setVisible(false);
        ui->textEditSettingThresholdMax->setVisible(false);

        ui->labelSettingSpatialMagnitude->setVisible(false);
        ui->textEditSettingSpatialMagnitude->setVisible(false);

        ui->labelSettingSpatialAlpha->setVisible(false);
        ui->textEditSettingSpatialAlpha->setVisible(false);

        ui->labelSettingSpatialDelta->setVisible(false);
        ui->textEditSettingSpatialDelta->setVisible(false);

        ui->labelSettingTemporalAlpha->setVisible(false);
        ui->textEditSettingTemporalAlpha->setVisible(false);

        ui->labelSettingTemporalDelta->setVisible(false);
        ui->textEditSettingTemporalDelta->setVisible(false);

        ui->pushButtonTest->setVisible(true);
        ui->pushButtonSend->setVisible(true);
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
    ui->labelSettingRosMasterAddress->setVisible(false);
    ui->textEditSettingRosMasterAddress->setVisible(false);

    ui->labelSettingRosLocalAddress->setVisible(false);
    ui->textEditSettingRosLocalAddress->setVisible(false);

    ui->labelSettingColorTopicName->setVisible(false);
    ui->textEditSettingColorTopicName->setVisible(false);

    ui->labelSettingDepthTopicName->setVisible(false);
    ui->textEditSettingDepthTopicName->setVisible(false);

    ui->labelSettingImuTopicName->setVisible(false);
    ui->textEditSettingImuTopicName->setVisible(false);

    ui->labelSettingPublishRate->setVisible(false);
    ui->textEditSettingPublishRate->setVisible(false);

    ui->labelSettingAutoExposure->setVisible(false);
    ui->checkBoxSettingAutoExposure->setVisible(false);

    ui->labelSettingThresholdMin->setVisible(false);
    ui->textEditSettingThresholdMin->setVisible(false);

    ui->labelSettingThresholdMax->setVisible(false);
    ui->textEditSettingThresholdMax->setVisible(false);

    ui->labelSettingSpatialMagnitude->setVisible(false);
    ui->textEditSettingSpatialMagnitude->setVisible(false);

    ui->labelSettingSpatialAlpha->setVisible(false);
    ui->textEditSettingSpatialAlpha->setVisible(false);

    ui->labelSettingSpatialDelta->setVisible(false);
    ui->textEditSettingSpatialDelta->setVisible(false);

    ui->labelSettingTemporalAlpha->setVisible(false);
    ui->textEditSettingTemporalAlpha->setVisible(false);

    ui->labelSettingTemporalDelta->setVisible(false);
    ui->textEditSettingTemporalDelta->setVisible(false);
}

//private: loadSettings
//read and load saved settings
void mainWindow::loadSettings()
{
    //settings file
    QSettings fileSettings("./config.ini", QSettings::NativeFormat);

    //ros master address
    if(fileSettings.value("ROS_MASTER_ADDRESS", "").toString().toStdString() == "")
    {
        ui->textEditSettingRosMasterAddress->setText("localhost:11311");
    }
    else
    {
        ui->textEditSettingRosMasterAddress->setText(fileSettings.value("ROS_MASTER_ADDRESS", "").toString());
    }
    //ros local address
    if(fileSettings.value("ROS_LOCAL_ADDRESS", "").toString().toStdString() == "")
    {
        ui->textEditSettingRosLocalAddress->setText("localhost");
    }
    else
    {
        ui->textEditSettingRosLocalAddress->setText(fileSettings.value("ROS_LOCAL_ADDRESS", "").toString());
    }
    //color topic
    if(fileSettings.value("COLOR_TOPIC_NAME", "").toString().toStdString() == "")
    {
        ui->textEditSettingColorTopicName->setText("rscColor");
    }
    else
    {
        ui->textEditSettingColorTopicName->setText(fileSettings.value("COLOR_TOPIC_NAME", "").toString());
    }
    //depth topic
    if(fileSettings.value("DEPTH_TOPIC_NAME", "").toString().toStdString() == "")
    {
        ui->textEditSettingDepthTopicName->setText("rscDepth");
    }
    else
    {
        ui->textEditSettingDepthTopicName->setText(fileSettings.value("DEPTH_TOPIC_NAME", "").toString());
    }
    //imu topic
    if(fileSettings.value("IMU_TOPIC_NAME", "").toString().toStdString() == "")
    {
        ui->textEditSettingImuTopicName->setText("rscImu");
    }
    else
    {
        ui->textEditSettingImuTopicName->setText(fileSettings.value("IMU_TOPIC_NAME", "").toString());
    }
    //publish rate
    if(fileSettings.value("PUBLISH_RATE", "").toString().toStdString() == "")
    {
        ui->textEditSettingPublishRate->setText("30");
    }
    else
    {
        ui->textEditSettingPublishRate->setText(fileSettings.value("PUBLISH_RATE", "").toString());
    }
    //auto exposure
    if(fileSettings.value("AUTO_EXPOSURE", "").toString().toStdString() == "")
    {
        ui->checkBoxSettingAutoExposure->setCheckState(Qt::CheckState::Checked);
    }
    else
    {
        if(fileSettings.value("AUTO_EXPOSURE", "").toString() == "1")
        {
            ui->checkBoxSettingAutoExposure->setCheckState(Qt::CheckState::Checked);
        }
        else
        {
            ui->checkBoxSettingAutoExposure->setCheckState(Qt::CheckState::Unchecked);
        }
    }
    //threshold min
    if(fileSettings.value("THRESHOLD_MIN", "").toString().toStdString() == "")
    {
        ui->textEditSettingThresholdMin->setText("0.1");
    }
    else
    {
        ui->textEditSettingThresholdMin->setText(fileSettings.value("THRESHOLD_MIN", "").toString());
    }
    //threshold max
    if(fileSettings.value("THRESHOLD_MAX", "").toString().toStdString() == "")
    {
        ui->textEditSettingThresholdMax->setText("16");
    }
    else
    {
        ui->textEditSettingThresholdMax->setText(fileSettings.value("THRESHOLD_MAX", "").toString());
    }
    //spatial magnitude
    if(fileSettings.value("SPATIAL_MAGNITUDE", "").toString().toStdString() == "")
    {
        ui->textEditSettingSpatialMagnitude->setText("2");
    }
    else
    {
        ui->textEditSettingSpatialMagnitude->setText(fileSettings.value("SPATIAL_MAGNITUDE", "").toString());
    }
    //spatial alpha
    if(fileSettings.value("SPATIAL_ALPHA", "").toString().toStdString() == "")
    {
        ui->textEditSettingSpatialAlpha->setText("0.5");
    }
    else
    {
        ui->textEditSettingSpatialAlpha->setText(fileSettings.value("SPATIAL_ALPHA", "").toString());
    }
    //spatial delta
    if(fileSettings.value("SPATIAL_DELTA", "").toString().toStdString() == "")
    {
        ui->textEditSettingSpatialDelta->setText("20.0");
    }
    else
    {
        ui->textEditSettingSpatialDelta->setText(fileSettings.value("SPATIAL_DELTA", "").toString());
    }
    //temporal alpha
    if(fileSettings.value("TEMPORAL_ALPHA", "").toString().toStdString() == "")
    {
        ui->textEditSettingTemporalAlpha->setText("0.4");
    }
    else
    {
        ui->textEditSettingTemporalAlpha->setText(fileSettings.value("TEMPORAL_ALPHA", "").toString());
    }
    //temporal delta
    if(fileSettings.value("TEMPORAL_DELTA", "").toString().toStdString() == "")
    {
        ui->textEditSettingTemporalDelta->setText("20.0");
    }
    else
    {
        ui->textEditSettingTemporalDelta->setText(fileSettings.value("TEMPORAL_DELTA", "").toString());
    }
}

//private: saveSetings
//save settings to disk
void mainWindow::saveSettings()
{
    //settings file
    QSettings fileSettings("./config.ini", QSettings::NativeFormat);

    fileSettings.setValue("ROS_MASTER_ADDRESS", ui->textEditSettingRosMasterAddress->toPlainText());
    fileSettings.setValue("ROS_LOCAL_ADDRESS", ui->textEditSettingRosLocalAddress->toPlainText());
    fileSettings.setValue("COLOR_TOPIC_NAME", ui->textEditSettingColorTopicName->toPlainText());
    fileSettings.setValue("DEPTH_TOPIC_NAME", ui->textEditSettingDepthTopicName->toPlainText());
    fileSettings.setValue("IMU_TOPIC_NAME", ui->textEditSettingImuTopicName->toPlainText());
    fileSettings.setValue("PUBLISH_RATE", ui->textEditSettingPublishRate->toPlainText());
    if(ui->checkBoxSettingAutoExposure->checkState() == Qt::CheckState::Checked)
    {
        fileSettings.setValue("AUTO_EXPOSURE", "1");
    }
    else
    {
        fileSettings.setValue("AUTO_EXPOSURE", "0");
    }
    fileSettings.setValue("THRESHOLD_MIN", ui->textEditSettingThresholdMin->toPlainText());
    fileSettings.setValue("THRESHOLD_MAX", ui->textEditSettingThresholdMax->toPlainText());
    fileSettings.setValue("SPATIAL_MAGNITUDE", ui->textEditSettingSpatialMagnitude->toPlainText());
    fileSettings.setValue("SPATIAL_ALPHA", ui->textEditSettingSpatialAlpha->toPlainText());
    fileSettings.setValue("SPATIAL_DELTA", ui->textEditSettingSpatialDelta->toPlainText());
    fileSettings.setValue("TEMPORAL_ALPHA", ui->textEditSettingTemporalAlpha->toPlainText());
    fileSettings.setValue("TEMPORAL_DELTA", ui->textEditSettingTemporalDelta->toPlainText());
}
