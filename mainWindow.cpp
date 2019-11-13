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

    initRosNodes();

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



void mainWindow::initRosNodes()
{
    //connection to allow application to close rather than hang if ROS has to shutdown unexpectedly

    QObject::connect(&rosNode, SIGNAL(rosShutdown()), this, SLOT(close()));

    //connection to detect receipt of new acceleration data
    //TODO:implement
    //connection to detect receipt of new depth data, prompting output to update
    QObject::connect(&rosNode, SIGNAL(receivedDepth()), this, SLOT(updateDepth()));
    //connection to detect receipt of new gyroscope data
    //TODO:implement


    //temporary location for logic to start ROS node
    rosNode.init("http://localhost:11311/", "localhost");
}


void mainWindow::initVidView()
{
    QWebEngineView* viewDepthData;
    viewDepthData = new QWebEngineView(this);
    viewDepthData->load(QUrl("http://localhost:8080/stream_viewer?topic=/rscDepth"));
    viewDepthData->setZoomFactor(0.68);
    viewDepthData->page()->setBackgroundColor((Qt::transparent));
    viewDepthData->show();
    ui->tabCamera1VideoStream->addWidget(viewDepthData);
}

//public: destructor
//deletes ui and closes application
mainWindow::~mainWindow()
{
    delete ui;
}

void mainWindow::updatePointCloud()
{

}

void mainWindow::updateDepth()
{
    QPixmap buffer = QPixmap::fromImage(rosNode.getDepth());
}
