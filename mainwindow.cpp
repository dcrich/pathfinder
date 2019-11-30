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
    ui->mOSGMainWidget->setup_environment();
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
