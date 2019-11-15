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

    //add a QWebEngineWidget page to ui
    //TODO:tweak settings in this function for aesthetics - Jordan
    initColorView();

    //add a QOpenGLWidget to ui
    //TODO:implement this, currently we use standard ORB_SLAM2 viewer - Jordan
    initPointcloudView();

    //create ROS node to recieve messages over
    rosNode.init(ui->textEditRosMasterIP->toPlainText().toStdString(), ui->textEditRosLocalIP->toPlainText().toStdString(),
                 ui->textEditColorTopicName->toPlainText().toStdString(), ui->textEditDepthTopicName->toPlainText().toStdString(),
                 ui->textEditImuTopicName->toPlainText().toStdString(), ui->textEditRefreshRate->toPlainText().toFloat());
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

//private: initColorView
//connects a QWebEngineWidget to the main ui to show color stream
void mainWindow::initColorView()
{
    //create a webengineview
    QWebEngineView* viewColorData = new QWebEngineView(this);

    //point it to the stream viewer topic(handled externally by web_video_server)
    viewColorData->load(QUrl("http://localhost:8080/stream_viewer?topic=/" + ui->textEditColorTopicName->toPlainText()));

    //configure settings to make it fit nicely in ui
    viewColorData->setZoomFactor(0.68);
    viewColorData->page()->setBackgroundColor((Qt::transparent));

    //show the stream in widget
    viewColorData->show();

    //add widget to main ui
    ui->boxLayoutVideoStream->addWidget(viewColorData);
}

//private: initPointcloudView
//will eventually add ORB_SLAM2 style pointcloud viewer to main window, currently incomplete
//TODO:implement this fully - Jordan
void mainWindow::initPointcloudView()
{
    //create an OpenGL widget
    QOpenGLWidget* viewPointcloud = new QOpenGLWidget(this);

    //create variable for OpenGL context
    QOpenGLContext* openGLContext;

    //get context from widget
    openGLContext = viewPointcloud->context();

    //put widget in UI
    //possible way
    //glGetString: return a string describing the current GL connection
    const GLubyte* glGetString(GLenum name);

    ui->boxLayoutPointcloud->addWidget(viewPointcloud);

    //void glBind*(GLenum target​, GLuint object​); saving this for reference
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
