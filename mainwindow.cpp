#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "swarmsimulator.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // TODO: Make pretty beutiful part of running SwarmSimulator.
    SwarmSimulator::SwarmSphereFormation(3, QVector3D(10, 10, 10), 0.8);

}

MainWindow::~MainWindow()
{
    delete ui;
}
