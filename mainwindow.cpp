//-------------------------------------------------------
// Filename: mainwindow.cpp
//
// Description:  The cpp file for the qt5 bullet bouncy ball example.
//
// Creator:  Professor Corey McBride for MEEN 570 - Brigham Young University
//
// Creation Date: 11/7/18
//
// Owner: Corey McBride
//-------------------------------------------------------
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "btBulletDynamicsCommon.h"
#include <QMessageBox>
#include "OSGWidget.h"


MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->mOSGMainWidget->setup_single_ball();
}

MainWindow::~MainWindow()
{
  delete ui;
}


void MainWindow::on_bGenerate_clicked()
{
    numberOfObstacles = ui->numObstacles->value();
    sizeOfObstacles = ui->sizeObstacles->value();
    ui->mOSGMainWidget->create_obstacles(numberOfObstacles,sizeOfObstacles);

}

void MainWindow::on_bReset_clicked()
{
    ui->mOSGMainWidget->reset_world();
}

void MainWindow::on_bRun_clicked()
{
    ui->mOSGMainWidget->make_balls();
}
