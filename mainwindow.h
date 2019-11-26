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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

public slots:


private slots:


    void on_bGenerate_clicked();

    void on_bReset_clicked();

    void on_bRun_clicked();

private:
  Ui::MainWindow *ui;
  int numberOfObstacles;
  int sizeOfObstacles;

};

#endif // MAINWINDOW_H
