#ifndef TEST_TRAJ_PANEL__TEST_TRAJ_PANEL_HPP_
#define TEST_TRAJ_PANEL__TEST_TRAJ_PANEL_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <QWidget>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

class QDoubleSpinBox;
class QLineEdit;
class QPushButton;

namespace test_traj_panel
{

class TestTrajPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TestTrajPanel(QWidget * parent = nullptr);
  ~TestTrajPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onStartStopClicked();
  void onPublishTick();

private:
  void setupUi();
  static double oscillation(double magnitude, double frequency_hz, double t, double phase_rad);

  std::string resolveParentFrame() const;
  void ensurePublishers();
  void startPublishTimer();
  void stopPublishTimer();
  geometry_msgs::msg::PoseStamped makePose(double t) const;
  void publishTrajectorySample();

  rclcpp::Node::WeakPtr node_weak_;
  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::string pose_topic_cached_;
  std::string path_topic_cached_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  QLineEdit * topic_edit_{nullptr};
  QLineEdit * path_topic_edit_{nullptr};
  QLineEdit * parent_frame_edit_{nullptr};
  QLineEdit * target_frame_edit_{nullptr};

  QDoubleSpinBox * zero_x_{nullptr};
  QDoubleSpinBox * zero_y_{nullptr};
  QDoubleSpinBox * zero_z_{nullptr};
  QDoubleSpinBox * zero_roll_{nullptr};
  QDoubleSpinBox * zero_pitch_{nullptr};
  QDoubleSpinBox * zero_yaw_{nullptr};

  QDoubleSpinBox * mag_x_{nullptr};
  QDoubleSpinBox * mag_y_{nullptr};
  QDoubleSpinBox * mag_z_{nullptr};
  QDoubleSpinBox * freq_x_{nullptr};
  QDoubleSpinBox * freq_y_{nullptr};
  QDoubleSpinBox * freq_z_{nullptr};
  QDoubleSpinBox * publish_rate_hz_{nullptr};

  QPushButton * start_stop_btn_{nullptr};

  bool running_{false};
  uint64_t trajectory_step_{0};
  nav_msgs::msg::Path path_msg_;

  static constexpr size_t kMaxPathPoints = 5000;
};

}  // namespace test_traj_panel

#endif  // TEST_TRAJ_PANEL__TEST_TRAJ_PANEL_HPP_
