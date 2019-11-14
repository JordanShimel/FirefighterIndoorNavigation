//mainWindow header file
//mainWindow handles the GUI for the base unit application

#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

//pointcloudWidget provides pointcloud functionality
#include <pointcloudWidget.hpp>

//rosNodeWidget provides ROS functionality
#include <rosNodeWidget.hpp>

//videoWidget provides video functionality
#include <videoWidget.hpp>

//the auto-generated ui_mainwindow contains declarations for all the Qt elements built in the designer
#include "ui_mainWindow.h"

//basic Qt includes
#include <QMainWindow>

//Qt webengine
#include <QtWebEngineWidgets/QWebEngineView>

//Qt openGL widgets
#include <QOpenGLWidget>

//allows using Qt's setting system
#include <QSettings>

namespace Ui
{
    class mainWindow;
}

class mainWindow : public QMainWindow
{
    //mainWindow uses the Qt event system, and thus requires the Q_OBJECT macro so the precompiler can link things properly
    Q_OBJECT

    public:
        //constructor creates UI and initializes UI element values
        explicit mainWindow(QWidget *parent = nullptr);
        //destructor closes UI window and terminates application
        ~mainWindow();

    private slots:
        void onPushButtonSaveConfig_clicked();

    private:
        //ui is used to access all the GUI Qt elements
        Ui::mainWindow *ui;

        //rosNode is used to assign a persistent ROS node to this window
        rosNodeWidget rosNode;

        void initUI();
        void initVidView();
        void initPCView();

        //loadSettings reads settings from the config file
        void loadSettings();
        //saveSettings writes settings to the config file
        void saveSettings();
};

#endif
