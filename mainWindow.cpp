//mainWindow code file
//mainWindow handles the GUI for the base unit application

#include "mainWindow.hpp"

//public: constructor
//creates ui instance
//initializes ui elements
mainWindow::mainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::mainWindow)
{
    //initialize UI
    ui->setupUi(this);

    //load settings from file
    loadSettings();

    //save settings file, this will create one if this was first load
    saveSettings();

    //load settings in rosNode
    rosNode.loadSettings();

    //create ROS node to recieve messages over
    rosNode.init();
                 
    //connection to signal to the main window that it is time to integrate the ORB_SLAM2 windows
    QObject::connect(&rosNode, SIGNAL(grabSLAMWindows()), this, SLOT(mergeWindows()));
    //connection to allow application to close rather than hang if ROS has to shutdown unexpectedly
    QObject::connect(&rosNode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

//public: destructor
//saves settings, deletes ui, and closes application
mainWindow::~mainWindow()
{
    rosNode.stop();
    saveSettings();
    delete ui;
}

//private slot: mergeWindows
//merges the windows created by ORB_SLAM2 into the Qt window for the main application
void mainWindow::mergeWindows()
{
    //sparse point cloud viewer
    //values to store results of capturing window ID
    string sWindowIDSparsePointCloud;
    array<char, 128> cWindowIDSparsePointCloud;

    //use xwin to open a pipe to grab the window data by name
    std::shared_ptr<FILE> pipeSparsePointCloud(popen("xwininfo -name \"FIN_SparsePointCloudViewer\"", "r"), pclose);

    //transfer data from the pipe to the string using the char array
    if(!pipeSparsePointCloud)
    {
        throw std::runtime_error("popen() failed!");
    }
    while(!feof(pipeSparsePointCloud.get()))
    {
        if(fgets(cWindowIDSparsePointCloud.data(), 128, pipeSparsePointCloud.get()) != nullptr)
        {
            sWindowIDSparsePointCloud += cWindowIDSparsePointCloud.data();
        }
    }

    //pull the window id out and convert it for use in Qt
    //parse the result for the programs window id, a 9 digit hexademical number
    sWindowIDSparsePointCloud = sWindowIDSparsePointCloud.substr(22, 9);
    //convert the 9 digit hexadecimal number into an acceptable type for fromWindId();
    unsigned long lWinIDSparsePointCloud = std::strtoul(sWindowIDSparsePointCloud.c_str(), nullptr, 16);

    //import the map viewer application
    QWindow *windowSparsePointCloud = QWindow::fromWinId(lWinIDSparsePointCloud);
    windowSparsePointCloud->setFlags(Qt::FramelessWindowHint);
    QWidget *widgetSparsePointCloud = QWidget::createWindowContainer(windowSparsePointCloud);
    ui->boxLayoutSparsePointCloud->addWidget(widgetSparsePointCloud);


    //dense point cloud viewer
    string sWindowIDDensePointCloud;
    array<char, 128> cWindowIDDensePointCloud;

    std::shared_ptr<FILE> pipeDensePointCloud(popen("xwininfo -name \"FIN_DensePointCloudViewer\"", "r"), pclose);

    if(!pipeDensePointCloud)
    {
        throw std::runtime_error("popen() failed!");
    }
    while(!feof(pipeDensePointCloud.get()))
    {
        if(fgets(cWindowIDDensePointCloud.data(), 128, pipeDensePointCloud.get()) != nullptr)
        {
            sWindowIDDensePointCloud += cWindowIDDensePointCloud.data();
        }
    }

    sWindowIDDensePointCloud = sWindowIDDensePointCloud.substr(22, 9);
    unsigned long lWinIDDensePointCloud = std::strtoul(sWindowIDDensePointCloud.c_str(), nullptr, 16);

    QWindow *windowDensePointCloud = QWindow::fromWinId(lWinIDDensePointCloud);
    windowDensePointCloud->setFlags(Qt::FramelessWindowHint);
    QWidget *widgetDensePointCloud = QWidget::createWindowContainer(windowDensePointCloud);
    ui->boxLayoutDensePointCloud->addWidget(widgetDensePointCloud);


    //top down viewer
    string sWindowIDTopDown;
    array<char, 128> cWindowIDTopDown;

    std::shared_ptr<FILE> pipeTopDown(popen("xwininfo -name \"FIN_TopDownViewer\"", "r"), pclose);

    if(!pipeTopDown)
    {
        throw std::runtime_error("popen() failed!");
    }
    while(!feof(pipeTopDown.get()))
    {
        if(fgets(cWindowIDTopDown.data(), 128, pipeTopDown.get()) != nullptr)
        {
            sWindowIDTopDown += cWindowIDTopDown.data();
        }
    }

    sWindowIDTopDown = sWindowIDTopDown.substr(22, 9);
    unsigned long lWinIDTopDown = std::strtoul(sWindowIDTopDown.c_str(), nullptr, 16);

    QWindow *windowTopDown = QWindow::fromWinId(lWinIDTopDown);
    windowTopDown->setFlags(Qt::FramelessWindowHint);
    QWidget *widgetTopDown = QWidget::createWindowContainer(windowTopDown);
    ui->boxLayoutOverheadView->addWidget(widgetTopDown);


    //video frame viewer
    string sWindowIDVideoFrame;
    array<char, 128> cWindowIDVideoFrame;

    std::shared_ptr<FILE> pipeVideoFrame(popen("xwininfo -name \"FIN_CurrentFrameViewer\"", "r"), pclose);

    if(!pipeVideoFrame)
    {
        throw std::runtime_error("popen() failed!");
    }
    while(!feof(pipeVideoFrame.get()))
    {
        if(fgets(cWindowIDVideoFrame.data(), 128, pipeVideoFrame.get()) != nullptr)
        {
            sWindowIDVideoFrame += cWindowIDVideoFrame.data();
        }
    }

    sWindowIDVideoFrame = sWindowIDVideoFrame.substr(22, 9);
    unsigned long lWinIDVideoFrame = std::strtoul(sWindowIDVideoFrame.c_str(), nullptr, 16);

    QWindow *windowVideoFrame = QWindow::fromWinId(lWinIDVideoFrame);
    windowVideoFrame->setFlags(Qt::FramelessWindowHint);
    QWidget *widgetVideoFrame = QWidget::createWindowContainer(windowVideoFrame);
    ui->boxLayoutVideoStream->addWidget(widgetVideoFrame);
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
