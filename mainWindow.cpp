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

    QWebEngineView* viewDepthData;
    viewDepthData = new QWebEngineView(this);
    viewDepthData->load(QUrl("http://localhost:8080/stream_viewer?topic=/rscDepth"));
    viewDepthData->setZoomFactor(0.68);
    viewDepthData->page()->setBackgroundColor((Qt::transparent));
    viewDepthData->show();
    ui->tabCamera1VideoStream->addWidget(viewDepthData);

    QPixmap pix("/home/jordan/Projects/firefighterbaseunit/PC.jpg");
    ui->labelpc->setPixmap(pix);

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
