#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

namespace {

struct PoseSample {
  double timestamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

class CompetitionTrajectoryWriter {
 public:
  CompetitionTrajectoryWriter() : nh_(), pnh_("~") {
    pnh_.param<std::string>("scene_name", scene_name_, "scene_0001");
    pnh_.param<std::string>("output_dir", output_dir_, "");
    pnh_.param<std::string>("lidar_topic", lidar_topic_, "/rslidar_front_points");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<double>("idle_flush_sec", idle_flush_sec_, 1.0);

    if (output_dir_.empty()) {
      ROS_FATAL("competition trajectory writer: output_dir is empty");
      ros::shutdown();
      return;
    }

    const std::filesystem::path trajectory_dir =
        std::filesystem::path(output_dir_) / "trajectories";
    std::error_code ec;
    std::filesystem::create_directories(trajectory_dir, ec);
    if (ec) {
      ROS_FATAL_STREAM("cannot create trajectory directory " << trajectory_dir
                       << ": " << ec.message());
      ros::shutdown();
      return;
    }

    output_path_ = trajectory_dir / (scene_name_ + ".txt");
    ofs_.open(output_path_.string(), std::ios::out | std::ios::trunc);
    if (!ofs_) {
      ROS_FATAL_STREAM("cannot open trajectory file " << output_path_);
      ros::shutdown();
      return;
    }
    ofs_ << std::fixed << std::setprecision(15);

    lidar_sub_ = nh_.subscribe(lidar_topic_, 10000,
                              &CompetitionTrajectoryWriter::lidarCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10000,
                             &CompetitionTrajectoryWriter::odomCallback, this);
    idle_timer_ = nh_.createWallTimer(
        ros::WallDuration(0.25),
        &CompetitionTrajectoryWriter::idleTimerCallback, this);

    last_activity_wall_ = ros::WallTime::now();

    ROS_INFO_STREAM("competition trajectory writer: scene=" << scene_name_
                    << ", output=" << output_path_);
  }

  ~CompetitionTrajectoryWriter() {
    flushSuffix();
    if (ofs_.is_open()) {
      ofs_.flush();
      ofs_.close();
    }
  }

 private:
  void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    const double t = msg->header.stamp.toSec();
    if (!std::isfinite(t))
      return;

    if (last_lidar_time_ >= 0.0 && t <= last_lidar_time_) {
      ROS_WARN_THROTTLE(1.0,
                        "competition writer: ignore duplicate/non-increasing LiDAR timestamp");
      return;
    }

    pending_lidar_times_.push_back(t);
    last_lidar_time_ = t;
    last_activity_wall_ = ros::WallTime::now();
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    PoseSample current;
    current.timestamp = msg->header.stamp.toSec();
    current.position <<
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z;
    current.orientation = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    if (!std::isfinite(current.timestamp) ||
        !current.position.allFinite() ||
        !current.orientation.coeffs().allFinite() ||
        current.orientation.norm() < 1e-12) {
      ROS_WARN_THROTTLE(1.0, "competition writer: ignore invalid odometry sample");
      return;
    }
    current.orientation.normalize();

    if (have_current_odom_ && current.timestamp <= current_odom_.timestamp) {
      ROS_WARN_THROTTLE(1.0,
                        "competition writer: ignore duplicate/non-increasing odometry timestamp");
      return;
    }

    if (have_current_odom_) {
      previous_odom_ = current_odom_;
      have_previous_odom_ = true;
    }
    current_odom_ = current;
    have_current_odom_ = true;

    flushBracketed();
    last_activity_wall_ = ros::WallTime::now();
  }

  static PoseSample interpolate(const PoseSample &a,
                                const PoseSample &b,
                                double timestamp) {
    PoseSample out;
    out.timestamp = timestamp;

    const double dt = b.timestamp - a.timestamp;
    double alpha = dt > 0.0 ? (timestamp - a.timestamp) / dt : 0.0;
    alpha = std::max(0.0, std::min(1.0, alpha));

    out.position = (1.0 - alpha) * a.position + alpha * b.position;
    out.orientation = a.orientation.slerp(alpha, b.orientation);
    out.orientation.normalize();
    return out;
  }

  void flushBracketed() {
    if (!have_current_odom_)
      return;

    while (!pending_lidar_times_.empty() &&
           pending_lidar_times_.front() <= current_odom_.timestamp) {
      const double t = pending_lidar_times_.front();
      pending_lidar_times_.pop_front();

      PoseSample out;
      if (!have_previous_odom_ || t <= previous_odom_.timestamp) {
        // Prefix before the first usable FAST-LIO pose. Competition data start
        // stationary, so use the earliest available IMU-origin pose.
        out = have_previous_odom_ ? previous_odom_ : current_odom_;
        out.timestamp = t;
        ++prefix_clamped_;
      } else {
        out = interpolate(previous_odom_, current_odom_, t);
      }
      write(out);
    }
  }

  void idleTimerCallback(const ros::WallTimerEvent &) {
    if (pending_lidar_times_.empty() || !have_current_odom_)
      return;

    const double idle =
        (ros::WallTime::now() - last_activity_wall_).toSec();
    if (idle >= idle_flush_sec_) {
      flushSuffix();
    }
  }

  void flushSuffix() {
    if (!have_current_odom_)
      return;

    while (!pending_lidar_times_.empty()) {
      PoseSample out = current_odom_;
      out.timestamp = pending_lidar_times_.front();
      pending_lidar_times_.pop_front();
      ++suffix_clamped_;
      write(out);
    }

    if (prefix_clamped_ > 0 || suffix_clamped_ > 0) {
      ROS_INFO_STREAM_THROTTLE(
          2.0,
          "competition writer clamp counts: prefix=" << prefix_clamped_
          << ", suffix=" << suffix_clamped_);
    }
  }

  void write(const PoseSample &pose) {
    if (!ofs_.is_open())
      return;
    if (last_written_time_ >= 0.0 && pose.timestamp <= last_written_time_)
      return;

    const auto &q = pose.orientation;
    ofs_ << pose.timestamp << " "
         << pose.position.x() << " "
         << pose.position.y() << " "
         << pose.position.z() << " "
         << q.x() << " "
         << q.y() << " "
         << q.z() << " "
         << q.w() << "\n";
    ofs_.flush();
    last_written_time_ = pose.timestamp;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber odom_sub_;
  ros::WallTimer idle_timer_;

  std::string scene_name_;
  std::string output_dir_;
  std::string lidar_topic_;
  std::string odom_topic_;
  std::filesystem::path output_path_;
  std::ofstream ofs_;

  std::deque<double> pending_lidar_times_;
  PoseSample previous_odom_;
  PoseSample current_odom_;
  bool have_previous_odom_ = false;
  bool have_current_odom_ = false;

  double last_lidar_time_ = -1.0;
  double last_written_time_ = -1.0;
  double idle_flush_sec_ = 1.0;
  std::size_t prefix_clamped_ = 0;
  std::size_t suffix_clamped_ = 0;
  ros::WallTime last_activity_wall_;
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "competition_trajectory_writer");
  CompetitionTrajectoryWriter writer;
  ros::spin();
  return 0;
}
