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

    void on_bAutoRun_clicked();

private:
  Ui::MainWindow *ui;
  int numberOfObstacles;
  int sizeOfObstacles;

};

#endif // MAINWINDOW_H
