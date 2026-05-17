#include "test_traj_panel/test_traj_panel.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace test_traj_panel
{
namespace
{

void set_orientation_rpy(
  double roll, double pitch, double yaw,
  geometry_msgs::msg::Quaternion & q)
{
  const double half_roll = roll * 0.5;
  const double half_pitch = pitch * 0.5;
  const double half_yaw = yaw * 0.5;
  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);

  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  q.w = cr * cp * cy + sr * sp * sy;
}

}  // namespace

TestTrajPanel::TestTrajPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setupUi();
}

TestTrajPanel::~TestTrajPanel() = default;

void TestTrajPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();

  auto * context = getDisplayContext();
  if (!context) {
    return;
  }

  clock_ = context->getClock();

  auto ros_node_abs = context->getRosNodeAbstraction().lock();
  if (!ros_node_abs) {
    return;
  }

  auto node = ros_node_abs->get_raw_node();
  node_weak_ = node;
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
}

void TestTrajPanel::setupUi()
{
  setObjectName("TestTrajPanel");

  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  auto * inner = new QWidget();
  scroll->setWidget(inner);

  auto * root = new QVBoxLayout(this);
  root->addWidget(scroll);

  auto * main_layout = new QVBoxLayout(inner);

  auto add_spin = [](QDoubleSpinBox * box, double min_v, double max_v, double step,
      int decimals, double default_value) {
      box->setRange(min_v, max_v);
      box->setSingleStep(step);
      box->setDecimals(decimals);
      box->setValue(default_value);
    };

  {
    auto * box = new QGroupBox("Topics & frames");
    auto * form = new QFormLayout(box);

    topic_edit_ = new QLineEdit("/curent_target");
    form->addRow("Pose topic", topic_edit_);

    path_topic_edit_ = new QLineEdit("/test_traj_panel/path");
    form->addRow("Path topic", path_topic_edit_);

    parent_frame_edit_ = new QLineEdit("");
    parent_frame_edit_->setPlaceholderText("(use RViz Fixed Frame)");
    form->addRow("Parent frame", parent_frame_edit_);

    target_frame_edit_ = new QLineEdit("test_traj_target");
    form->addRow("Target TF frame", target_frame_edit_);

    publish_rate_hz_ = new QDoubleSpinBox();
    add_spin(publish_rate_hz_, 0.1, 500.0, 1.0, 3, 30.0);
    form->addRow("Publish rate (Hz)", publish_rate_hz_);

    main_layout->addWidget(box);
  }

  {
    auto * box = new QGroupBox("Zero pose (base offset)");
    auto * form = new QFormLayout(box);

    zero_x_ = new QDoubleSpinBox();
    zero_y_ = new QDoubleSpinBox();
    zero_z_ = new QDoubleSpinBox();
    add_spin(zero_x_, -1e6, 1e6, 0.01, 6, 0.0);
    add_spin(zero_y_, -1e6, 1e6, 0.01, 6, 0.0);
    add_spin(zero_z_, -1e6, 1e6, 0.01, 6, 0.0);
    form->addRow("X (m)", zero_x_);
    form->addRow("Y (m)", zero_y_);
    form->addRow("Z (m)", zero_z_);

    zero_roll_ = new QDoubleSpinBox();
    zero_pitch_ = new QDoubleSpinBox();
    zero_yaw_ = new QDoubleSpinBox();
    add_spin(zero_roll_, -M_PI, M_PI, 0.01, 6, 0.0);
    add_spin(zero_pitch_, -M_PI, M_PI, 0.01, 6, 0.0);
    add_spin(zero_yaw_, -M_PI, M_PI, 0.01, 6, 0.0);
    form->addRow("Roll (rad)", zero_roll_);
    form->addRow("Pitch (rad)", zero_pitch_);
    form->addRow("Yaw (rad)", zero_yaw_);

    main_layout->addWidget(box);
  }

  {
    auto * box = new QGroupBox("Oscillation (per axis)");
    auto * form = new QFormLayout(box);

    mag_x_ = new QDoubleSpinBox();
    mag_y_ = new QDoubleSpinBox();
    mag_z_ = new QDoubleSpinBox();
    add_spin(mag_x_, -1e6, 1e6, 0.01, 6, 0.0);
    add_spin(mag_y_, -1e6, 1e6, 0.01, 6, 0.0);
    add_spin(mag_z_, -1e6, 1e6, 0.01, 6, 0.0);
    form->addRow("Magnitude X (m)", mag_x_);
    form->addRow("Magnitude Y (m)", mag_y_);
    form->addRow("Magnitude Z (m)", mag_z_);

    freq_x_ = new QDoubleSpinBox();
    freq_y_ = new QDoubleSpinBox();
    freq_z_ = new QDoubleSpinBox();
    add_spin(freq_x_, 0.0, 100.0, 0.01, 6, 0.0);
    add_spin(freq_y_, 0.0, 100.0, 0.01, 6, 0.0);
    add_spin(freq_z_, 0.0, 100.0, 0.01, 6, 0.0);
    form->addRow("Frequency X (Hz)", freq_x_);
    form->addRow("Frequency Y (Hz)", freq_y_);
    form->addRow("Frequency Z (Hz)", freq_z_);

    auto * hint = new QLabel(
      "Frequency 0 disables motion on that axis. Y uses a π/2 phase shift versus X so equal X/Y "
      "frequency and magnitude trace a circle; mismatched frequencies give Lissajous figures. "
      "Z uses the same phase reference as X.");
    hint->setWordWrap(true);
    main_layout->addWidget(box);
    main_layout->addWidget(hint);
  }

  auto * btn_row = new QHBoxLayout();
  start_stop_btn_ = new QPushButton("Start");
  btn_row->addWidget(start_stop_btn_);
  main_layout->addLayout(btn_row);

  connect(start_stop_btn_, &QPushButton::clicked, this, &TestTrajPanel::onStartStopClicked);

  main_layout->addStretch(1);

  setMinimumWidth(320);
}

double TestTrajPanel::oscillation(double magnitude, double frequency_hz, double t, double phase_rad)
{
  if (frequency_hz <= 0.0 || std::abs(magnitude) < 1e-15) {
    return 0.0;
  }
  return magnitude * std::sin(2.0 * M_PI * frequency_hz * t + phase_rad);
}

std::string TestTrajPanel::resolveParentFrame() const
{
  const QString override_frame = parent_frame_edit_->text().trimmed();
  if (!override_frame.isEmpty()) {
    return override_frame.toStdString();
  }
  auto * context = getDisplayContext();
  if (!context) {
    return "map";
  }
  return context->getFixedFrame().toStdString();
}

void TestTrajPanel::ensurePublishers()
{
  auto node = node_weak_.lock();
  if (!node) {
    return;
  }

  std::string pose_topic = topic_edit_->text().trimmed().toStdString();
  if (pose_topic.empty()) {
    pose_topic = "/curent_target";
  }

  std::string path_topic = path_topic_edit_->text().trimmed().toStdString();
  if (path_topic.empty()) {
    path_topic = "/test_traj_panel/path";
  }

  if (!pose_pub_ || pose_topic != pose_topic_cached_) {
    pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_topic, rclcpp::QoS(10));
    pose_topic_cached_ = pose_topic;
  }

  if (!path_pub_ || path_topic != path_topic_cached_) {
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(10));
    path_topic_cached_ = path_topic;
  }
}

geometry_msgs::msg::PoseStamped TestTrajPanel::makePose(double t) const
{
  geometry_msgs::msg::PoseStamped out;
  const double ox = oscillation(mag_x_->value(), freq_x_->value(), t, 0.0);
  const double oy = oscillation(mag_y_->value(), freq_y_->value(), t, M_PI / 2.0);
  const double oz = oscillation(mag_z_->value(), freq_z_->value(), t, 0.0);

  out.pose.position.x = zero_x_->value() + ox;
  out.pose.position.y = zero_y_->value() + oy;
  out.pose.position.z = zero_z_->value() + oz;

  set_orientation_rpy(
    zero_roll_->value(), zero_pitch_->value(), zero_yaw_->value(),
    out.pose.orientation);
  return out;
}

void TestTrajPanel::startPublishTimer()
{
  stopPublishTimer();

  auto node = node_weak_.lock();
  if (!node || !clock_) {
    return;
  }

  double rate_hz = publish_rate_hz_->value();
  if (rate_hz <= 0.0) {
    rate_hz = 10.0;
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_hz));

  publish_timer_ = node->create_timer(
    period,
    [this]() { onPublishTick(); });
}

void TestTrajPanel::stopPublishTimer()
{
  if (publish_timer_) {
    publish_timer_->cancel();
    publish_timer_.reset();
  }
}

void TestTrajPanel::onStartStopClicked()
{
  if (!running_) {
    ensurePublishers();

    auto node = node_weak_.lock();
    if (!node || !pose_pub_ || !path_pub_ || !clock_) {
      return;
    }

    path_msg_.poses.clear();
    trajectory_step_ = 0;

    startPublishTimer();
    running_ = true;
    start_stop_btn_->setText("Stop");

    // Publish immediately so pose/path/TF start aligned at t = 0.
    publishTrajectorySample();
  } else {
    stopPublishTimer();
    running_ = false;
    start_stop_btn_->setText("Start");
  }
}

void TestTrajPanel::onPublishTick()
{
  if (!running_) {
    return;
  }
  publishTrajectorySample();
}

void TestTrajPanel::publishTrajectorySample()
{
  auto node = node_weak_.lock();
  if (!node || !clock_ || !tf_broadcaster_ || !pose_pub_ || !path_pub_) {
    return;
  }

  double rate_hz = publish_rate_hz_->value();
  if (rate_hz <= 0.0) {
    rate_hz = 10.0;
  }

  const double trajectory_time = static_cast<double>(trajectory_step_) / rate_hz;
  ++trajectory_step_;

  const rclcpp::Time now = clock_->now();
  const std::string frame = resolveParentFrame();

  geometry_msgs::msg::PoseStamped pose = makePose(trajectory_time);
  pose.header.stamp = now;
  pose.header.frame_id = frame;

  std::string child = target_frame_edit_->text().trimmed().toStdString();
  if (child.empty()) {
    child = "test_traj_target";
  }

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now;
  tf.header.frame_id = frame;
  tf.child_frame_id = child;
  tf.transform.translation.x = pose.pose.position.x;
  tf.transform.translation.y = pose.pose.position.y;
  tf.transform.translation.z = pose.pose.position.z;
  tf.transform.rotation = pose.pose.orientation;
  tf_broadcaster_->sendTransform(tf);

  pose_pub_->publish(pose);

  path_msg_.header.stamp = now;
  path_msg_.header.frame_id = frame;
  path_msg_.poses.push_back(pose);
  if (path_msg_.poses.size() > kMaxPathPoints) {
    const size_t excess = path_msg_.poses.size() - kMaxPathPoints;
    path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + excess);
  }
  path_pub_->publish(path_msg_);
}

}  // namespace test_traj_panel

PLUGINLIB_EXPORT_CLASS(test_traj_panel::TestTrajPanel, rviz_common::Panel)
