/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2022, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#ifndef ROBOTCARPLAYER_H
#define ROBOTCARPLAYER_H
// QT
#include <QButtonGroup>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDebug>
#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <QMutex>
#include <QSettings>
#include <QThread>
// mine
#include "radar_robotcar_dataset_ros2/rclthread.hpp"

QT_BEGIN_NAMESPACE
namespace Ui {
class RobotCarPlayer;
}
QT_END_NAMESPACE

class RobotCarPlayer : public QMainWindow {
  Q_OBJECT

 public:
  RobotCarPlayer(QWidget *parent = nullptr);
  ~RobotCarPlayer();

  void RCLInital(rclcpp::Node::SharedPtr n) { rcl_thread->inital(n); }

 private slots:
  // closed
  void closeEvent(QCloseEvent *event);
  // buttons
  void on_buttonSDK_clicked();
  void on_buttonDataset_clicked();
  void on_buttonPlay_clicked();
  void on_buttonPause_clicked();
  void on_buttonAbort_clicked();
  void on_buttonSave_clicked();
  // checkBoxs
  void on_checkPicture_stateChanged();
  void on_checkMutex_stateChanged();
  void on_checkUndistort_stateChanged();
  void on_checkStereoMode_stateChanged();
  void on_checkStereo_stateChanged();
  void on_checkRadar_stateChanged();
  void on_checkMono_stateChanged();
  void on_check3DLiDAR_stateChanged();
  void on_checkGPS_stateChanged();
  void on_check2DLiDAR_stateChanged();
  // spinBox
  void on_SpinSpeed_valueChanged(double arg);
  // slider
  void on_SliderPlayer_valueChanged(int arg);
  void on_SliderPlayer_sliderPressed();
  void on_SliderPlayer_sliderReleased();
  void on_SliderPlayer_valueReset(int arg);
  void on_SliderPlayer_setMaximum(int arg);

 signals:
  void sensor_choiced(sensor_type type);

 private:
  // ui
  Ui::RobotCarPlayer *ui;
  // thread
  RCLThread *rcl_thread;
  // mutex
  QMutex Qmutex;
  // paths
  QString Qdir_sdk, Qdir_data, Qdir_save;
  // checkboxs
  QButtonGroup *pcheckGroup;
};
#endif  // ROBOTCARPLAYER_H
