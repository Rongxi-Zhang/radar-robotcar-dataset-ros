#include <qdesktopwidget.h>

#include <QApplication>

#include "radar_robotcar_dataset_ros2/robotcarplayer.h"

int main(int argc, char *argv[]) {
  // rclcpp
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("RobotCarPlayer");

  // QT
  QApplication a(argc, argv);
  RobotCarPlayer w;
  w.RCLInital(node);

  QDesktopWidget *desktop = QApplication::desktop();
  w.move((desktop->width() - w.width()) / 2,
         (desktop->height() - w.height()) / 2);
  w.show();

  return a.exec();
}
