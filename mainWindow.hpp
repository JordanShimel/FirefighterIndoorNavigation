//mainWindow header file
//mainWindow handles the GUI for the base unit application

#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

//providess PointCloud functionality
#include <pointCloudWidget.hpp>

//handles ROS node connection and subscribing to data
#include <rosNodeWidget.hpp>

//the auto-generated ui_mainwindow contains declarations for all the Qt elements built in the designer
#include "ui_mainWindow.h"

//QMainWindow has the basic Qt includes for a gui application
#include <QMainWindow>

//QMainWindow isused to display video feed in main window
#include <QtWebEngineWidgets/QWebEngineView>

//QOpenGLWidget is used to show point cloud in main window
#include <QOpenGLWidget>

//QSettings is used to handle saving and loading settings
#include <QSettings>

//QWindow is used to capture ui elements
#include <QWindow>

//xdo is used to capture ui elements
#include <xdo.h>

using namespace std;

//namespace required for access to ui elements
namespace Ui
{
    class mainWindow;
}

//mainWindow class, manages main GUI
class mainWindow : public QMainWindow
{
    //mainWindow uses the Qt event system, and thus requires the Q_OBJECT macro so the precompiler can link things properly
    Q_OBJECT

    public:
        //constructor creates UI and initializes UI element values
        explicit mainWindow(QWidget *parent = nullptr);
        //destructor saves settings, closes UI window, and terminates application
        ~mainWindow();

    private slots:
        void mergeWindows();

private:
        //ui is used to access all the GUI Qt elements
        Ui::mainWindow *ui;
        //rosNode is used to assign a persistent ROS node to this window
        rosNodeWidget rosNode;

        //loadSettings reads settings from the config file
        void loadSettings();
        //saveSettings writes settings to the config file
        void saveSettings();
};

#endif
