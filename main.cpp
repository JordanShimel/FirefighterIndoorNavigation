//FireFighterBaseUnit main code file
//serves as the code entry point for execution

//mainWindow class handles GUI
#include "mainWindow.hpp"

//default Qt include
#include <QApplication>

int main(int argc, char *argv[])
{
    //basic code to run a Qt based window
    QApplication application(argc, argv);
    mainWindow FireFighterBaseUnitWindow;
    FireFighterBaseUnitWindow.show();
    return application.exec();
}
