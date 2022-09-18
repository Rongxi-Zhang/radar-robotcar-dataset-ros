/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2022, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

// mine
#include "radar_robotcar_dataset_ros2/robotcarplayer.h"

#include "./ui_robotcarplayer.h"

RobotCarPlayer::RobotCarPlayer(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::RobotCarPlayer) {
  ui->setupUi(this);
  // rcl thread created
  rcl_thread = new RCLThread(this, &Qmutex);
  // fixed window size
  this->setMaximumSize(845, 360);
  this->setMinimumSize(845, 360);
  // spinBox
  // ui->SpinSpeed->setEnabled(false);
  ui->SpinSpeed->setRange(0.5, 10.0);
  ui->SpinSpeed->setValue(1.0);
  ui->SpinSpeed->setSingleStep(0.1);
  // slider
  ui->SliderPlayer->setMinimum(0);
  ui->SliderPlayer->setValue(0);
  ui->SliderPlayer->setSingleStep(1);
  ui->SliderPlayer->setMaximum(1e6);

  // settings read
  ui->textStatus->setText(
      "Welcome to the RobotCarPlayer (^o^)\nThis player is based on ROS2 and "
      "Qt5 and will be maintained continuously.\n");
  QString curr_path = QCoreApplication::applicationDirPath();
  QSettings setting(curr_path + "/config/RobotCarPlayer.ini",
                    QSettings::IniFormat);
  if (setting.contains(tr("RobotCarPlayer/SDK")) &&
      setting.contains(tr("RobotCarPlayer/Dataset"))) {
    Qdir_sdk = setting.value("RobotCarPlayer/SDK").toString();
    Qdir_data = setting.value("RobotCarPlayer/Dataset").toString();
    ui->lineEditSDK->setText(Qdir_sdk);
    ui->lineEditDataset->setText(Qdir_data);
    ui->textStatus->append("RobotCarPlayer has read the last saved paths.");
  }

  // connect
  connect(rcl_thread, SIGNAL(index_signal(int)), this,
          SLOT(on_SliderPlayer_valueReset(int)));
  connect(rcl_thread, SIGNAL(range_signal(int)), this,
          SLOT(on_SliderPlayer_setMaximum(int)));
  connect(this, SIGNAL(sensor_choiced(sensor_type)), rcl_thread,
          SLOT(map_slot(sensor_type)));
  connect(this, SIGNAL(sensor_cancel(sensor_type)), rcl_thread,
          SLOT(remap_slot(sensor_type)));
}

RobotCarPlayer::~RobotCarPlayer() {
  rcl_thread->quit();
  if (!rcl_thread->wait(10)) {
    rcl_thread->terminate();
    rcl_thread->wait();
  }
  delete rcl_thread;
  delete ui;
}

/***************************SLOT******************************/

void RobotCarPlayer::closeEvent(QCloseEvent *event) {
  QMessageBox::StandardButton resBtn = QMessageBox::question(
      this, "RobotCarPlayer",
      tr("Are you sure to close the player (T_T)?\nIf you click the yes, the "
         "paths you set will be saved (^_^)."),
      QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes);
  if (resBtn == QMessageBox::Cancel) {
    event->ignore();
  } else if (resBtn == QMessageBox::Yes) {
    QString curr_path = QCoreApplication::applicationDirPath();
    QSettings *setting_ptr = new QSettings(
        curr_path + "/config/RobotCarPlayer.ini", QSettings::IniFormat);
    setting_ptr->sync();
    setting_ptr->beginGroup(tr("RobotCarPlayer"));
    setting_ptr->setValue("SDK", Qdir_sdk);
    setting_ptr->setValue("Dataset", Qdir_data);
    setting_ptr->endGroup();
    delete setting_ptr;
    event->accept();
  } else
    event->accept();
}

//-------------------------buttons---------------------------//
void RobotCarPlayer::on_buttonSDK_clicked() {
  Qdir_sdk = QFileDialog::getExistingDirectory(
      this, tr("Please open the directory named robotcar-dataset-sdk."),
      Qdir_sdk);
  if (!Qdir_sdk.isEmpty()) {
    QFileInfo QFi_e(Qdir_sdk + "/extrinsics");
    QFileInfo QFi_m(Qdir_sdk + "/models");
    if (QFi_e.isDir() && QFi_m.isDir()) {
      ui->lineEditSDK->setText(Qdir_sdk);
      ui->textStatus->setText(
          "The directory named robotcar-dataset-sdk has been found.\n");

    } else {
      QPalette Qpal = ui->textStatus->palette();
      QColor Qcol = Qpal.color(QPalette::Text);
      ui->textStatus->setTextColor(Qt::red);
      ui->textStatus->setText(
          "No directory named robotcar-dataset-sdk. Please check the file "
          "path "
          "you selected. \n");
      ui->textStatus->setTextColor(Qcol);
    }
  } else {
    QPalette Qpal = ui->textStatus->palette();
    QColor Qcol = Qpal.color(QPalette::Text);
    ui->textStatus->setTextColor(Qt::red);
    ui->textStatus->setText("Please select a non empty path.\n");
    ui->textStatus->setTextColor(Qcol);
  }
}

void RobotCarPlayer::on_buttonDataset_clicked() {
  Qdir_data = QFileDialog::getExistingDirectory(
      this, tr("Please open the directory containing the dataset."), Qdir_data);
  if (!Qdir_data.isEmpty()) {
    ui->lineEditDataset->setText(Qdir_data);
    ui->textStatus->setText(
        "Whether the data corresponding to each sensor you selected exists "
        "will be checked later.\n");

    // must
    rcl_thread->press_flag = false;
    rcl_thread->run_flag = false;
    rcl_thread->timestamp_index_pre = -1;
    rcl_thread->timestamp_index = 0;
    rcl_thread->timestamps.clear();
    rcl_thread->timestamps_v.clear();
    rcl_thread->stereo_added = false;
    rcl_thread->radar_added = false;
    rcl_thread->mono_added = false;
    rcl_thread->lidar3d_added = false;
    rcl_thread->gps_added = false;
    rcl_thread->lidar2d_added = false;

    ui->checkStereo->setCheckState(Qt::Unchecked);
    ui->check2DLiDAR->setCheckState(Qt::Unchecked);
    ui->check3DLiDAR->setCheckState(Qt::Unchecked);
    ui->checkMono->setCheckState(Qt::Unchecked);
    ui->checkRadar->setCheckState(Qt::Unchecked);
    ui->checkGPS->setCheckState(Qt::Unchecked);

  } else {
    QPalette Qpal = ui->textStatus->palette();
    QColor Qcol = Qpal.color(QPalette::Text);
    ui->textStatus->setTextColor(Qt::red);
    ui->textStatus->setText("Please select a non empty path.\n");
    ui->textStatus->setTextColor(Qcol);
  }
}

void RobotCarPlayer::on_buttonPlay_clicked() {
  ui->checkPicture->setEnabled(false);
  ui->buttonSDK->setEnabled(false);
  ui->buttonDataset->setEnabled(false);
  // rcl thread start
  rcl_thread->start();
  rcl_thread->run_flag = true;
  ui->textStatus->setText("Start playing...");
  if (rcl_thread->isRunning()) ui->buttonPlay->setEnabled(false);
}

void RobotCarPlayer::on_buttonPause_clicked() {
  ui->buttonPlay->setEnabled(true);
  rcl_thread->run_flag = false;
  ui->textStatus->setText("Pause playback");
}

void RobotCarPlayer::on_buttonAbort_clicked() {
  ui->buttonPlay->setEnabled(true);
  ui->checkPicture->setEnabled(true);
  ui->buttonSDK->setEnabled(true);
  ui->buttonDataset->setEnabled(true);
  // rcl thread quit
  rcl_thread->run_flag = false;
  rcl_thread->quit();
  ui->textStatus->setText("Aborting playback.");
  ui->SliderPlayer->setValue(0);
}

void RobotCarPlayer::on_buttonSave_clicked() {
  Qdir_save = QFileDialog::getSaveFileName(this, tr("Save to rosbag2"));
  ui->textStatus->setText("Your rosbag2 will be saved in\n" + Qdir_save);
  rcl_thread->bag_name = Qdir_save.toStdString();
}

//----------------------------checkBoxs------------------------------//

void RobotCarPlayer::on_checkPicture_stateChanged() {
  if (ui->checkPicture->isChecked()) {
    ui->textStatus->setText(
        "The Oxford RobotCar Dataset\n3D LiDAR: LD-MRS\nNo Radar");
    ui->Image->setPixmap(QPixmap(":/robotcar0.png"));
    ui->Image->resize(343, 300);
    ui->Image->move(497, 36);
    ui->checkRadar->setEnabled(false);
    QPalette Qpal = ui->check3DLiDAR->palette();
    Qpal.setColor(Qpal.WindowText, QColor("#800080"));
    ui->check3DLiDAR->setPalette(Qpal);
    rcl_thread->is_radar_dataset = false;

  } else {  // default
    ui->textStatus->setText(
        "The Oxford Radar RobotCar Dataset\n3D LiDAR: HDL-32E\nRadar: "
        "CTS350-X");
    ui->Image->setPixmap(QPixmap(":/robotcar.png"));
    ui->Image->resize(341, 351);
    ui->Image->move(500, 0);
    ui->checkRadar->setEnabled(true);
    QPalette Qpal = ui->check3DLiDAR->palette();
    Qpal.setColor(Qpal.WindowText, QColor("#729FCF"));
    ui->check3DLiDAR->setPalette(Qpal);
    rcl_thread->is_radar_dataset = true;
  }
}

void RobotCarPlayer::on_checkMutex_stateChanged() {
  if (ui->checkMutex->isChecked()) {
    ui->textStatus->setText("Single sensor mode with acceleration function.");
    pcheckGroup = new QButtonGroup(this);
    pcheckGroup->addButton(ui->checkStereo, 1);
    pcheckGroup->addButton(ui->check2DLiDAR, 2);
    pcheckGroup->addButton(ui->check3DLiDAR, 3);
    pcheckGroup->addButton(ui->checkMono, 4);
    pcheckGroup->addButton(ui->checkRadar, 5);
    pcheckGroup->addButton(ui->checkGPS, 6);
    ui->SpinSpeed->setEnabled(true);
    rcl_thread->is_single = true;
  } else {
    ui->textStatus->setText("Multi sensor mode, no acceleration function.");
    ui->textStatus->append("The sensors will operate synchronously.");
    // ui->SpinSpeed->setEnabled(false);
    rcl_thread->is_single = false;
    delete pcheckGroup;
  }
  rcl_thread->run_flag = false;
}

void RobotCarPlayer::on_checkUndistort_stateChanged() {
  if (ui->checkUndistort->isChecked()) {
    ui->textStatus->setText("Remove images distortion.");
    rcl_thread->is_undistort = true;
  } else {
    ui->textStatus->setText("Do not remove images distortion.");
    rcl_thread->is_undistort = false;
  }
}

void RobotCarPlayer::on_checkStereoMode_stateChanged() {
  if (ui->checkStereoMode->isChecked()) {
    ui->textStatus->setText("Stereo narrow mode. left and centre");
    rcl_thread->width = true;
  } else {
    ui->textStatus->setText("Stereo wide mode. left and right");
    rcl_thread->width = false;
  }
}

void RobotCarPlayer::on_checkStereo_stateChanged() {
  if (ui->checkStereo->isChecked()) {
    QFileInfo QFi_sensor(Qdir_data + "/stereo");
    QFileInfo QFi_stamps(Qdir_data + "/stereo.timestamps");
    if (QFi_sensor.isDir() && QFi_stamps.exists()) {
      string sdk_path = Qdir_sdk.toStdString();
      string dataset_path = Qdir_data.toStdString();
      // stereo camera inital
      rcl_thread->stereo_camera.inital(sdk_path, dataset_path, sensor_stereo);
      // show something
      {
        ui->textStatus->setText("stereo camera data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->stereo_camera.t.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->stereo_camera.t.end_time()));
        ui->textStatus->append(
            "Duration: " +
            QString("%1").arg(rcl_thread->stereo_camera.t.duration()) + "s");
        ui->textStatus->append(
            "Number of files: " +
            QString("%1").arg(rcl_thread->stereo_camera.t.size()));
      }
      rcl_thread->stereo_checked = true;
      emit sensor_choiced(sensor_stereo);

    } else {
      QPalette Qpal = ui->textStatus->palette();
      QColor Qcol = Qpal.color(QPalette::Text);
      ui->textStatus->setTextColor(Qt::red);
      ui->textStatus->setText("Stereo camera data not found.\n");
      ui->textStatus->setTextColor(Qcol);
    }
  } else {
    ui->textStatus->setText("Stereo camera unchecked\n");
    rcl_thread->stereo_checked = false;
  }
}

void RobotCarPlayer::on_checkRadar_stateChanged() {
  if (ui->checkRadar->isChecked()) {
    QFileInfo QFi_sensor(Qdir_data + "/radar");
    QFileInfo QFi_stamps(Qdir_data + "/radar.timestamps");
    if (QFi_sensor.isDir() && QFi_stamps.exists()) {
      string sdk_path = Qdir_sdk.toStdString();
      string dataset_path = Qdir_data.toStdString();
      // radar inital
      rcl_thread->spin_radar.inital(sdk_path, dataset_path);
      // show something
      {
        ui->textStatus->setText("Radar data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->spin_radar.t.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->spin_radar.t.end_time()));
        ui->textStatus->append(
            "Duration: " +
            QString("%1").arg(rcl_thread->spin_radar.t.duration()) + "s");
        ui->textStatus->append(
            "Number of files: " +
            QString("%1").arg(rcl_thread->spin_radar.t.size()));
      }
      rcl_thread->radar_checked = true;
      emit sensor_choiced(sensor_radar);

    } else {
      QPalette Qpal = ui->textStatus->palette();
      QColor Qcol = Qpal.color(QPalette::Text);
      ui->textStatus->setTextColor(Qt::red);
      ui->textStatus->setText("Radar data not found.\n");
      ui->textStatus->setTextColor(Qcol);
    }
  } else {
    ui->textStatus->setText("Radar unchecked\n");
    rcl_thread->radar_checked = false;
  }
}

void RobotCarPlayer::on_checkMono_stateChanged() {
  if (ui->checkMono->isChecked()) {
    QFileInfo QFi_sensor_l(Qdir_data + "/mono_left");
    QFileInfo QFi_stamps_l(Qdir_data + "/mono_left.timestamps");
    QFileInfo QFi_sensor_re(Qdir_data + "/mono_rear");
    QFileInfo QFi_stamps_re(Qdir_data + "/mono_rear.timestamps");
    QFileInfo QFi_sensor_ri(Qdir_data + "/mono_right");
    QFileInfo QFi_stamps_ri(Qdir_data + "/mono_right.timestamps");

    if (QFi_sensor_l.isDir() && QFi_stamps_l.exists() &&
        QFi_sensor_re.isDir() && QFi_stamps_re.exists() &&
        QFi_sensor_ri.isDir() && QFi_stamps_ri.exists()) {
      string sdk_path = Qdir_sdk.toStdString();
      string dataset_path = Qdir_data.toStdString();
      // mono cameras inital
      rcl_thread->mono_cameras.inital(sdk_path, dataset_path);
      // show something
      {
        ui->textStatus->setText("Left mono camera data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->mono_cameras.t_l.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->mono_cameras.t_l.end_time()));
        ui->textStatus->append(
            "Duration: " +
            QString("%1").arg(rcl_thread->mono_cameras.t_l.duration()) + "s");
        ui->textStatus->append(
            "Number of files: " +
            QString("%1").arg(rcl_thread->mono_cameras.t_l.size()));

        ui->textStatus->append("\nRear mono camera data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->mono_cameras.t_re.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->mono_cameras.t_re.end_time()));
        ui->textStatus->append(
            "Duration: " +
            QString("%1").arg(rcl_thread->mono_cameras.t_re.duration()) + "s");
        ui->textStatus->append(
            "Number of files: " +
            QString("%1").arg(rcl_thread->mono_cameras.t_re.size()));

        ui->textStatus->append("\nRight mono camera data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->mono_cameras.t_ri.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->mono_cameras.t_ri.end_time()));
        ui->textStatus->append(
            "Duration: " +
            QString("%1").arg(rcl_thread->mono_cameras.t_ri.duration()) + "s");
        ui->textStatus->append(
            "Number of files: " +
            QString("%1").arg(rcl_thread->mono_cameras.t_ri.size()));
      }
      rcl_thread->mono_checked = true;
      emit sensor_choiced(sensor_mono_left);
      emit sensor_choiced(sensor_mono_rear);
      emit sensor_choiced(sensor_mono_right);

    } else {
      QPalette Qpal = ui->textStatus->palette();
      QColor Qcol = Qpal.color(QPalette::Text);
      ui->textStatus->setTextColor(Qt::red);
      ui->textStatus->setText(
          "Mono cameras data not found or data is incomplete.\n");
      ui->textStatus->setTextColor(Qcol);
    }
  } else {
    ui->textStatus->setText("Mono cameras unchecked\n");
    rcl_thread->mono_checked = false;
  }
}

void RobotCarPlayer::on_check3DLiDAR_stateChanged() {
  if (ui->check3DLiDAR->isChecked() && !ui->checkPicture->isChecked()) {
    QFileInfo QFi_sensor_l(Qdir_data + "/velodyne_left");
    QFileInfo QFi_stamps_l(Qdir_data + "/velodyne_left.timestamps");
    QFileInfo QFi_sensor_r(Qdir_data + "/velodyne_right");
    QFileInfo QFi_stamps_r(Qdir_data + "/velodyne_right.timestamps");

    if (QFi_sensor_l.isDir() && QFi_stamps_l.exists() && QFi_sensor_r.isDir() &&
        QFi_stamps_r.exists()) {
      string sdk_path = Qdir_sdk.toStdString();
      string dataset_path = Qdir_data.toStdString();
      // 3D LiDARs inital
      rcl_thread->lidar_3ds.inital(sdk_path, dataset_path);
      // show something
      {
        ui->textStatus->setText("Left 3D LiDAR data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->lidar_3ds.t_l.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->lidar_3ds.t_l.end_time()));
        ui->textStatus->append(
            "Duration: " +
            QString("%1").arg(rcl_thread->lidar_3ds.t_l.duration()) + "s");
        ui->textStatus->append(
            "Number of files: " +
            QString("%1").arg(rcl_thread->lidar_3ds.t_l.size()));

        ui->textStatus->append("\nRight 3D LiDAR data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->lidar_3ds.t_r.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->lidar_3ds.t_r.end_time()));
        ui->textStatus->append(
            "Duration: " +
            QString("%1").arg(rcl_thread->lidar_3ds.t_r.duration()) + "s");
        ui->textStatus->append(
            "Number of files: " +
            QString("%1").arg(rcl_thread->lidar_3ds.t_r.size()));
      }
      rcl_thread->lidar3d_checked = true;
      emit sensor_choiced(sensor_lidar_left);
      // emit sensor_choiced(sensor_lidar_right);

    } else {
      QPalette Qpal = ui->textStatus->palette();
      QColor Qcol = Qpal.color(QPalette::Text);
      ui->textStatus->setTextColor(Qt::red);
      ui->textStatus->setText(
          "3D LiDARs data not found or data is incomplete.\n");
      ui->textStatus->setTextColor(Qcol);
    }
  } else if (ui->check3DLiDAR->isChecked() && ui->checkPicture->isChecked()) {
    QFileInfo QFi_sensor(Qdir_data + "/ldmrs");
    QFileInfo QFi_stamps(Qdir_data + "/ldmrs.timestamps");

    if (QFi_sensor.isDir() && QFi_stamps.exists()) {
      string sdk_path = Qdir_sdk.toStdString();
      string dataset_path = Qdir_data.toStdString();
      // Front 3D LiDAR initial
      rcl_thread->lidar_old.inital(sdk_path, dataset_path);
      // show something
      {
        ui->textStatus->setText("Front 3D LiDAR data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->lidar_old.t.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->lidar_old.t.end_time()));
        ui->textStatus->append(
            "Duration: " +
            QString("%1").arg(rcl_thread->lidar_old.t.duration()) + "s");
        ui->textStatus->append(
            "Number of files: " +
            QString("%1").arg(rcl_thread->lidar_old.t.size()));
      }

      rcl_thread->lidarold_checked = true;
      emit sensor_choiced(sensor_lidar_old);

    } else {
      QPalette Qpal = ui->textStatus->palette();
      QColor Qcol = Qpal.color(QPalette::Text);
      ui->textStatus->setTextColor(Qt::red);
      ui->textStatus->setText(
          "Front 3D LiDAR data not found or data is incomplete.\n");
      ui->textStatus->setTextColor(Qcol);
    }

  } else {
    ui->textStatus->setText("3D LiDARs unchecked\n");
    rcl_thread->lidar3d_checked = false;
    rcl_thread->lidarold_checked = false;
  }
}

void RobotCarPlayer::on_checkGPS_stateChanged() {
  if (ui->checkGPS->isChecked()) {
    QFileInfo QFi_csv(Qdir_data + "/gps/ins.csv");
    if (QFi_csv.exists()) {
      string sdk_path = Qdir_sdk.toStdString();
      string dataset_path = Qdir_data.toStdString();
      // gps/ins initial
      rcl_thread->gps_ins.inital(sdk_path, dataset_path);
      // show something
      {
        ui->textStatus->setText("GPS/INS data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->gps_ins.t.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->gps_ins.t.end_time()));
        ui->textStatus->append(
            "Duration: " + QString("%1").arg(rcl_thread->gps_ins.t.duration()) +
            "s");
        ui->textStatus->append("Number of files: " +
                               QString("%1").arg(rcl_thread->gps_ins.t.size()));
      }
      rcl_thread->gps_checked = true;
      emit sensor_choiced(sensor_gps_ins);
    } else {
      QPalette Qpal = ui->textStatus->palette();
      QColor Qcol = Qpal.color(QPalette::Text);
      ui->textStatus->setTextColor(Qt::red);
      ui->textStatus->setText("GPS/INS data not found.\n");
      ui->textStatus->setTextColor(Qcol);
    }
  } else {
    ui->textStatus->setText("GPS/INS unchecked\n");
    rcl_thread->gps_checked = false;
  }
}

// That's all for now.
void RobotCarPlayer::on_check2DLiDAR_stateChanged() {
  if (ui->check2DLiDAR->isChecked()) {
    QFileInfo QFi_sensor_f(Qdir_data + "/lms_front");
    QFileInfo QFi_stamps_f(Qdir_data + "/lms_front.timestamps");
    QFileInfo QFi_sensor_r(Qdir_data + "/lms_rear");
    QFileInfo QFi_stamps_r(Qdir_data + "/lms_rear.timestamps");

    if (QFi_sensor_f.isDir() && QFi_stamps_f.exists() && QFi_sensor_r.isDir() &&
        QFi_stamps_r.exists()) {
      string sdk_path = Qdir_sdk.toStdString();
      string dataset_path = Qdir_data.toStdString();
      // 2D LiDARs inital
      rcl_thread->lidar_2ds.inital(sdk_path, dataset_path);
      // show something
      {
        ui->textStatus->setText("Front 2D LiDAR data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->lidar_2ds.t_f.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->lidar_2ds.t_f.end_time()));
        ui->textStatus->append(
            "Duration: " +
            QString("%1").arg(rcl_thread->lidar_2ds.t_f.duration()) + "s");
        ui->textStatus->append(
            "Number of files: " +
            QString("%1").arg(rcl_thread->lidar_2ds.t_f.size()));

        ui->textStatus->append("\nRear 2D LiDAR data found.");
        ui->textStatus->append(
            "Start time: " +
            QString::fromStdString(rcl_thread->lidar_2ds.t_r.start_time()));
        ui->textStatus->append(
            "End  time: " +
            QString::fromStdString(rcl_thread->lidar_2ds.t_r.end_time()));
        ui->textStatus->append(
            "Duration: " +
            QString("%1").arg(rcl_thread->lidar_2ds.t_r.duration()) + "s");
        ui->textStatus->append(
            "Number of files: " +
            QString("%1").arg(rcl_thread->lidar_2ds.t_r.size()));
      }
      rcl_thread->lidar2d_checked = true;
      emit sensor_choiced(sensor_lidar_front);
      // emit sensor_choiced(sensor_lidar_right);

    } else {
      QPalette Qpal = ui->textStatus->palette();
      QColor Qcol = Qpal.color(QPalette::Text);
      ui->textStatus->setTextColor(Qt::red);
      ui->textStatus->setText(
          "2D LiDARs data not found or data is incomplete.\n");
      ui->textStatus->setTextColor(Qcol);
    }
  } else {
    ui->textStatus->setText("2D LiDARs unchecked\n");
    rcl_thread->lidar2d_checked = false;
  }
}

//----------------------------spinBox-----------------------------//
void RobotCarPlayer::on_SpinSpeed_valueChanged(double arg) {
  rcl_thread->play_speed = arg;
}

//----------------------------slider-----------------------------//
void RobotCarPlayer::on_SliderPlayer_valueChanged(int arg) {
  rcl_thread->timestamp_index = arg;
  ui->labelProgess->setText(QString::number(static_cast<int>(
                                arg * 100.00 / ui->SliderPlayer->maximum())) +
                            "%");
  ui->textStatus->setText("Current index \n" +
                          QString::number(static_cast<int>(arg)) + " / " +
                          QString::number(ui->SliderPlayer->maximum()));
}

void RobotCarPlayer::on_SliderPlayer_sliderPressed() {
  rcl_thread->press_flag = true;
  rcl_thread->run_flag = false;
}

void RobotCarPlayer::on_SliderPlayer_sliderReleased() {
  rcl_thread->press_flag = false;
  if (!rcl_thread->run_flag) ui->buttonPlay->setEnabled(true);
  ui->textStatus->setText("Pause playback");
}

void RobotCarPlayer::on_SliderPlayer_valueReset(int arg) {
  ui->SliderPlayer->setValue(arg);
}

void RobotCarPlayer::on_SliderPlayer_setMaximum(int arg) {
  ui->SliderPlayer->setMaximum(arg);
  qDebug() << "Set Maximum of the slider =" << arg;
}