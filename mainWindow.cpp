//mainWindow code file
//mainWindow handles the GUI for the base unit application

#include "mainWindow.hpp"
#include <stdio.h>

//public: constructor
//creates ui instance
//initializes ui elements
mainWindow::mainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::mainWindow)
{
    //initialize UI
    ui->setupUi(this);
    //load settings from file
    loadSettings();

    //create ROS node to recieve messages over
    rosNode.init(ui->textEditRosMasterIP->toPlainText().toStdString(), ui->textEditRosLocalIP->toPlainText().toStdString(),
                 ui->textEditColorTopicName->toPlainText().toStdString(), ui->textEditDepthTopicName->toPlainText().toStdString(),
                 ui->textEditImuTopicName->toPlainText().toStdString(), ui->textEditRefreshRate->toPlainText().toFloat());
                 
    /// connect to the signal sent from run after spinonce so so can do something when it sees it (single with the win id), do this only
    /// on the first time and ignore the rest, link to a function in mainwindow class to move map viewer and frame viewer into our app
    QObject::connect(&rosNode, SIGNAL(launchSlam()), this, SLOT(importSlam()));
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

//private slot: on_PushButtonSaveConfig_clicked
//will eventually reinitialize pointcloud system with new values, currently placeholder
void mainWindow::on_PushButtonSaveConfig_clicked()
{
    //TODO:make this restart thing with new config values, rather than just save them - Jordan
    saveSettings();
}

void mainWindow::importSlam()
{
    //when cmdCnt is == to 1 that is when the slam applications will have  launched and can be imported
    if (cmdCnt == 1) {

// --------------------------- Grab POINT CLOUD application
        // Get the output of the command that displays information on the running orb-slam2 pointcloud map application
        std::shared_ptr<FILE> pipe(popen("xwininfo -name \"ORB-SLAM2: Map Viewer\"", "r"), pclose);

        //move the output of the previous command into string mvResult
        if (!pipe) throw std::runtime_error("popen() failed!");
             while (!feof(pipe.get())) {
                 if (fgets(mvBuffer.data(), 128, pipe.get()) != nullptr)
                     mvResult += mvBuffer.data();
            }

        // parse the result for the programs window id, a 9 digit hexademical number
        mvResult = mvResult.substr(22, 9);
        system("wmctrl -r \"ORB-SLAM2: Map Viewer\" -e 0,0,0,640,546");  // FIXME: might be dead code

        //convert the 9 digit hexadecimal number into an acceptable type for fromWindId();
        unsigned long mvWinID = std::strtoul(mvResult.c_str(), 0, 16);

        //import the map viewer application
        QWindow *window = QWindow::fromWinId(mvWinID);
        window->setFlags(Qt::FramelessWindowHint);

        QWidget *widget = QWidget::createWindowContainer(window);
        widget->setFixedHeight(546);
     //   widget->setStyleSheet("border: 50px solid red");

        ui->boxLayoutPointcloud->addWidget(widget);

// --------------------------- Grab VIDEO STREAM application
        std::shared_ptr<FILE> pipe2(popen("xwininfo -name \"ORB-SLAM2: Current Frame\"", "r"), pclose);

        if (!pipe2) throw std::runtime_error("popen() failed!");
             while (!feof(pipe2.get())) {
                 if (fgets(cfBuffer.data(), 128, pipe2.get()) != nullptr)
                     cfResult += cfBuffer.data();
             }
         cfResult = cfResult.substr(22, 9);
         system("wmctrl -r \"ORB-SLAM2: Current Frame\" -e 0,0,0,640,480");   // FIXME: might be dead code
         unsigned long cfwinID2 = std::strtoul(cfResult.c_str(), 0, 16);

         QWindow *window2 = QWindow::fromWinId(cfwinID2);
         window2->setFlags(Qt::FramelessWindowHint);

         QWidget *widget2 = QWidget::createWindowContainer(window2);
         ui->boxLayoutVideoStream->addWidget(widget2);
    }
    cmdCnt ++;
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
    //refresh rate
    if(fileSettings.value("REFRESH_RATE", "").toString().toStdString() == "")
    {
        ui->textEditRefreshRate->setText("10");
    }
    else
    {
        ui->textEditRefreshRate->setText(fileSettings.value("REFRESH_RATE", "").toString());
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
    fileSettings.setValue("REFRESH_RATE", ui->textEditRefreshRate->toPlainText());
}
