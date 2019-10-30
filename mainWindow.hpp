//mainWindow header file
//mainWindow handles the GUI for the remote unit application

#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

//STL includes required for frame maps and vectors
#include <map>
#include <vector>

//Intel RealSense library provides camera functions
//be sure to include this before any Qt includes
//odd linker errors can result otherwise
#include <librealsense2/rs.hpp>

//rosNodeWidget handles ROS node connection
//and publishing of data
#include "rosNodeWidget.hpp"

//viewerWidget class handles the preview window
#include "viewerWidget.hpp"

//the auto-generated ui_mainwindow contains declarations for all the Qt elements built in the designer
#include "ui_mainWindow.h"

//basic Qt includes
#include <QMainWindow>

//allows the creation of simple message boxes
#include <QMessageBox>

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
        mainWindow(QWidget *parent = nullptr);
        //destructor closes UI window and terminates application
        ~mainWindow();

    private slots:
        //on_pushButtonTest_clicked uses viewerWidget class to generate a preview of live camera data
        void on_pushButtonTest_clicked();
        //on_pushButtonSend_clicked uses rosNodeWidget to initialize a ROS node if one does not exist
        //it then creates a QThread to begin publishing on that node
        //subsequent click will toggle the state of the publishing QThread
        void on_pushButtonSend_clicked();
        //on_pushButtonConfig_clicked makes the config fields visible and hides the main ui elements
        void on_pushButtonConfig_clicked();

private:
        //ui is used to access all the GUI Qt elements
        Ui::mainWindow *ui;
        //rosNode is used to assign a persistent ROS node to this window
        rosNodeWidget rosNode;
        //flag to determine behavior of publish button
        bool isPublishing = false;
        //flag to determine config mode
        bool isConfig = false;

        //showError displays an error window
        void showError(QString errorMessage);
};

#endif
