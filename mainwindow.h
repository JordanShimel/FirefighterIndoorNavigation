//FireFighterRemoteUnit mainwindow header file
//Contains definitions for mainwindow elements
//TODO?: Add config options

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButtonPreview_clicked();

    void on_pushButtonConnect_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
