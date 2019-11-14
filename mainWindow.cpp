//mainWindow code file
//mainWindow handles the GUI for the base unit application

#include "mainWindow.hpp"

//public: constructor
//creates ui instance
//initializes ui elements
//connects Qt Signals used by the application
mainWindow::mainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::mainWindow)
{
    //initialize UI
    ui->setupUi(this);

    initVidView();

    initPCView();

    rosNode.init("http://localhost:11311/", "localhost");
}

//public: destructor
//deletes ui and closes application
mainWindow::~mainWindow()
{
    delete ui;
}

void mainWindow::initVidView()
{
    QWebEngineView* viewDepthData;
    viewDepthData = new QWebEngineView(this);
    viewDepthData->load(QUrl("http://localhost:8080/stream_viewer?topic=/rscColor"));
    viewDepthData->setZoomFactor(0.68);
    viewDepthData->page()->setBackgroundColor((Qt::transparent));
    viewDepthData->show();
    ui->tabCamera1VideoStream->addWidget(viewDepthData);
}

void mainWindow::initPCView()
{
    // create widget
    QOpenGLWidget* viewPC;
    viewPC = new QOpenGLWidget(this);
    // create variable for context
    QOpenGLContext* openGLContext;
    // get context from widget
    openGLContext = viewPC->context();

    // put widget in UI

    // possible way
    // glGetString: return a string describing the current GL connection
    const GLubyte* glGetString(GLenum name);

    ui->tabCamera1PC->addWidget(viewPC);

    //void glBind*(GLenum target​, GLuint object​); saving this for reference
}

//private: loadSettings
void mainWindow::loadSettings()
{
    /*QSettings fileSettings("./config.ini", QSettings::NativeFormat);

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
    //accel topic
    if(fileSettings.value("ACCEL_TOPIC_NAME", "").toString().toStdString() == "")
    {
        ui->textEditAccelTopicName->setText("rscAccel");
    }
    else
    {
        ui->textEditAccelTopicName->setText(fileSettings.value("ACCEL_TOPIC_NAME", "").toString());
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
    //gyro topic
    if(fileSettings.value("GYRO_TOPIC_NAME", "").toString().toStdString() == "")
    {
        ui->textEditGyroTopicName->setText("rscGyro");
    }
    else
    {
        ui->textEditGyroTopicName->setText(fileSettings.value("GYRO_TOPIC_NAME", "").toString());
    }
    //publish rate
    if(fileSettings.value("PUBLISH_RATE", "").toString().toStdString() == "")
    {
        ui->textEditPublishRate->setText("10");
    }
    else
    {
        ui->textEditPublishRate->setText(fileSettings.value("PUBLISH_RATE", "").toString());
    }*/
}

//private: saveSetings
void mainWindow::saveSettings()
{
    /*QSettings fileSettings("./config.ini", QSettings::NativeFormat);
    fileSettings.setValue("ROS_MASTER_IP", ui->textEditRosMasterIP->toPlainText());
    fileSettings.setValue("ROS_LOCAL_IP", ui->textEditRosLocalIP->toPlainText());
    fileSettings.setValue("ACCEL_TOPIC_NAME", ui->textEditAccelTopicName->toPlainText());
    fileSettings.setValue("COLOR_TOPIC_NAME", ui->textEditColorTopicName->toPlainText());
    fileSettings.setValue("DEPTH_TOPIC_NAME", ui->textEditDepthTopicName->toPlainText());
    fileSettings.setValue("GYRO_TOPIC_NAME", ui->textEditGyroTopicName->toPlainText());
    fileSettings.setValue("PUBLISH_RATE", ui->textEditPublishRate->toPlainText());*/
}
