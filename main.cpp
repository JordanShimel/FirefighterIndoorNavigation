//FireFighterRemoteUnit main code file
//Serves as the code entry point for execution
//Currently runs a modified version of Intel's rs-multicamera sample program, edited to output all data from a single D435i camera
//TODO: Add capability to transfer ROSBag to base unit over Wi-Fi
//TODO?: Local/remote management system

//Default Qt includes
#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    //Basic code to run a Qt based window
    QApplication application(argc, argv);
    MainWindow FireFighterRemoteUnit_Window;
    FireFighterRemoteUnit_Window.show();
    return application.exec();
}
