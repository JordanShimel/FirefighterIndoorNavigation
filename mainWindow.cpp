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

    QWebEngineView* _view;
    //QVBoxLayout* _layout = new QVBoxLayout(this);
    _view = new QWebEngineView(this);
    _view->load(QUrl("http://localhost:8080/stream_viewer?topic=/rscVideo"));
    _view->setZoomFactor(0.5);
    _view->show();
    ui->tabCamera1VideoStream->addWidget(_view);

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

void mainWindow::updateDepth()
{
    QPixmap buffer = QPixmap::fromImage(rosNode.getDepth());
    //ui->tabCamera1labelDepthImage->setPixmap(buffer);

}
