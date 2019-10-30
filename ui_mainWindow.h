/********************************************************************************
** Form generated from reading UI file 'mainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_mainWindow
{
public:
    QAction *actiontest;
    QWidget *centralwidget;
    QPushButton *pushButtonPreview;
    QPushButton *pushButtonPublish;
    QPushButton *pushButtonView;
    QPushButton *pushButtonConfig;
    QStatusBar *statusbar;
    QMenuBar *menuBar;
    QMenu *menu;

    void setupUi(QMainWindow *mainWindow)
    {
        if (mainWindow->objectName().isEmpty())
            mainWindow->setObjectName(QStringLiteral("mainWindow"));
        mainWindow->resize(612, 438);
        mainWindow->setStyleSheet(QLatin1String("background-color: rgb(235, 242, 250)\n"
""));
        actiontest = new QAction(mainWindow);
        actiontest->setObjectName(QStringLiteral("actiontest"));
        centralwidget = new QWidget(mainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        pushButtonPreview = new QPushButton(centralwidget);
        pushButtonPreview->setObjectName(QStringLiteral("pushButtonPreview"));
        pushButtonPreview->setGeometry(QRect(10, 10, 291, 181));
        QFont font;
        font.setPointSize(36);
        font.setBold(true);
        font.setWeight(75);
        pushButtonPreview->setFont(font);
        pushButtonPreview->setStyleSheet(QLatin1String("background-color: rgb(50, 199, 86);\n"
"color:rgb(235, 242, 250);\n"
"border-radius: 10px;"));
        pushButtonPublish = new QPushButton(centralwidget);
        pushButtonPublish->setObjectName(QStringLiteral("pushButtonPublish"));
        pushButtonPublish->setGeometry(QRect(310, 10, 291, 181));
        pushButtonPublish->setFont(font);
        pushButtonPublish->setStyleSheet(QLatin1String("background-color: rgb(50, 199, 86);\n"
"color:rgb(235, 242, 250);\n"
"border-radius: 10px;"));
        pushButtonView = new QPushButton(centralwidget);
        pushButtonView->setObjectName(QStringLiteral("pushButtonView"));
        pushButtonView->setGeometry(QRect(10, 200, 291, 181));
        pushButtonView->setFont(font);
        pushButtonView->setStyleSheet(QLatin1String("background-color: rgb(50, 199, 86);\n"
"color:rgb(235, 242, 250);\n"
"border-radius: 10px;"));
        pushButtonConfig = new QPushButton(centralwidget);
        pushButtonConfig->setObjectName(QStringLiteral("pushButtonConfig"));
        pushButtonConfig->setGeometry(QRect(310, 200, 291, 181));
        pushButtonConfig->setFont(font);
        pushButtonConfig->setStyleSheet(QLatin1String("background-color: rgb(50, 199, 86);\n"
"color:rgb(235, 242, 250);\n"
"border-radius: 10px;"));
        mainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(mainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        mainWindow->setStatusBar(statusbar);
        menuBar = new QMenuBar(mainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 612, 25));
        menu = new QMenu(menuBar);
        menu->setObjectName(QStringLiteral("menu"));
        mainWindow->setMenuBar(menuBar);

        menuBar->addAction(menu->menuAction());

        retranslateUi(mainWindow);

        QMetaObject::connectSlotsByName(mainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *mainWindow)
    {
        mainWindow->setWindowTitle(QApplication::translate("mainWindow", "FireFighterRemoteUnit", 0));
        actiontest->setText(QApplication::translate("mainWindow", "test", 0));
        pushButtonPreview->setText(QApplication::translate("mainWindow", "PREVIEW", 0));
        pushButtonPublish->setText(QApplication::translate("mainWindow", "SEND", 0));
        pushButtonView->setText(QApplication::translate("mainWindow", "VIEW", 0));
        pushButtonConfig->setText(QApplication::translate("mainWindow", "CONFIG", 0));
        menu->setTitle(QApplication::translate("mainWindow", "File", 0));
    } // retranslateUi

};

namespace Ui {
    class mainWindow: public Ui_mainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
