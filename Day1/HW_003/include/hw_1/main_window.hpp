#ifndef hw_1_MAIN_WINDOW_H
#define hw_1_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QIcon>
#include <QTimer>
#include <QImage>
#include <opencv2/opencv.hpp>
#include "qnode.hpp"
#include "ui_mainwindow.h"

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

private:
  Ui::MainWindowDesign* ui;
  QNode* qnode;
  QTimer* timer;

  void closeEvent(QCloseEvent* event);

private Q_SLOTS:
  void updateRobotArm();
};

#endif  // hw_1_MAIN_WINDOW_H

