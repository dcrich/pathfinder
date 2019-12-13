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
}

MainWindow::~MainWindow()
{
  delete ui;
}


void MainWindow::on_bGenerate_clicked()
{
    numberOfObstacles = ui->numObstacles->value();
    ui->mOSGMainWidget->create_obstacles(numberOfObstacles);
}

void MainWindow::on_bReset_clicked()
{
    ui->mOSGMainWidget->reset_world();
}

void MainWindow::on_bRun_clicked()
{
    ui->mOSGMainWidget->make_balls();
}

void MainWindow::on_bAutoRun_clicked()
{
    ui->mOSGMainWidget->run_auto_path();
}
