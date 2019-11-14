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

    //set initial visibility
    initUI();

    initVidView();

    initPCView();

    //TODO:pass settings here
    rosNode.init(ui->textEditRosMasterIP->toPlainText().toStdString(), ui->textEditRosLocalIP->toPlainText().toStdString(),
                 ui->textEditColorTopicName->toPlainText().toStdString(), ui->textEditDepthTopicName->toPlainText().toStdString(),
                 ui->textEditImuTopicName->toPlainText().toStdString(), ui->textEditRefreshRate->toPlainText().toFloat());
}

//public: destructor
//deletes ui and closes application
mainWindow::~mainWindow()
{
    saveSettings();
    delete ui;
}

void mainWindow::onPushButtonSaveConfig_clicked()
{
    //TODO: Make this restart thing with new config values, rather than just save them - Jordan
    saveSettings();
}

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

    ui->labelRefreshRate->setVisible(false);
    ui->textEditRefreshRate->setVisible(false);
}

void mainWindow::initVidView()
{
    QWebEngineView* viewColorData;
    viewColorData = new QWebEngineView(this);
    //TODO:pass color topic name
    viewColorData->load(QUrl("http://localhost:8080/stream_viewer?topic=/rscColor"));
    viewColorData->setZoomFactor(0.68);
    viewColorData->page()->setBackgroundColor((Qt::transparent));
    viewColorData->show();
    ui->boxLayoutVideoStream->addWidget(viewColorData);
}

void mainWindow::initPCView()
{
    // create widget
    QOpenGLWidget* viewPointcloud;
    viewPointcloud = new QOpenGLWidget(this);
    // create variable for context
    QOpenGLContext* openGLContext;
    // get context from widget
    openGLContext = viewPointcloud->context();

    // put widget in UI

    // possible way
    // glGetString: return a string describing the current GL connection
    const GLubyte* glGetString(GLenum name);

    ui->boxLayoutPointcloud->addWidget(viewPointcloud);

    //void glBind*(GLenum target​, GLuint object​); saving this for reference
}

//private: loadSettings
void mainWindow::loadSettings()
{
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
void mainWindow::saveSettings()
{
    QSettings fileSettings("./config.ini", QSettings::NativeFormat);
    fileSettings.setValue("ROS_MASTER_IP", ui->textEditRosMasterIP->toPlainText());
    fileSettings.setValue("ROS_LOCAL_IP", ui->textEditRosLocalIP->toPlainText());
    fileSettings.setValue("COLOR_TOPIC_NAME", ui->textEditColorTopicName->toPlainText());
    fileSettings.setValue("DEPTH_TOPIC_NAME", ui->textEditDepthTopicName->toPlainText());
    fileSettings.setValue("IMU_TOPIC_NAME", ui->textEditImuTopicName->toPlainText());
    fileSettings.setValue("REFRESH_RATE", ui->textEditRefreshRate->toPlainText());
}
