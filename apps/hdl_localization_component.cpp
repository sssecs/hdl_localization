#include <mutex>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include <hdl_localization/msg/scan_matching_status.hpp>
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>

namespace hdl_localization {

class HdlLocalizationComponent : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  explicit HdlLocalizationComponent(const rclcpp::NodeOptions& options) : Node("hdl_localization", options) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    initialize_params();
    initialize_ros();
  }

  void initialize_ros() {
    if (use_imu_) {
      RCLCPP_INFO(this->get_logger(), "enable imu-based prediction");
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/gpsimu_driver/imu_data", 200, std::bind(&HdlLocalizationComponent::imu_callback, this, std::placeholders::_1));
    }
    points_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 5, std::bind(&HdlLocalizationComponent::points_callback, this, std::placeholders::_1));
    globalmap_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>("/globalmap", 1, std::bind(&HdlLocalizationComponent::globalmap_callback, this, std::placeholders::_1));
    initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",
      10,
      std::bind(&HdlLocalizationComponent::initialpose_callback, this, std::placeholders::_1));

    pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", 10);
    status_pub_ = this->create_publisher<hdl_localization::msg::ScanMatchingStatus>("/status", 10);

    // global localization
    if (use_global_localization_) {
      RCLCPP_INFO(this->get_logger(), "wait for global localization services");
      set_global_map_client_ = this->create_client<hdl_global_localization::srv::SetGlobalMap>("/hdl_global_localization/set_global_map");
      query_global_localization_client_ = this->create_client<hdl_global_localization::srv::QueryGlobalLocalization>("/hdl_global_localization/query");

      while (rclcpp::ok() &&  !set_global_map_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for /hdl_global_localization/set_global_map service...");
      }
      while (rclcpp::ok() &&  !query_global_localization_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for /hdl_global_localization/query service...");
      }

      relocalize_service_ =
        this->create_service<std_srvs::srv::Empty>("/relocalize", std::bind(&HdlLocalizationComponent::relocalize, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

private:
  pcl::Registration<PointT, PointT>::Ptr create_registration() const {
    if (reg_method_ == "NDT_OMP") {
      RCLCPP_INFO(this->get_logger(), "NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution_);
      if (ndt_neighbor_search_method_ == "DIRECT1") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method_ == "DIRECT7") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method_ == "KDTREE") {
          RCLCPP_INFO(this->get_logger(), "search_method KDTREE is selected");
        } else {
          RCLCPP_WARN(this->get_logger(), "invalid search method was given");
          RCLCPP_WARN(this->get_logger(), "default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    }
    //  else if (reg_method_.find("NDT_CUDA") != std::string::npos) {
    //   RCLCPP_INFO(this->get_logger(), "NDT_CUDA is selected");
    //   auto ndt = std::make_shared<fast_gicp::NDTCuda<PointT, PointT>>();
    //   ndt->setResolution(ndt_resolution_);

    //   if (reg_method_.find("D2D") != std::string::npos) {
    //     ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
    //   } else if (reg_method_.find("P2D") != std::string::npos) {
    //     ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
    //   }

    //   if (ndt_neighbor_search_method_ == "DIRECT1") {
    //     RCLCPP_INFO(this->get_logger(), "search_method DIRECT1 is selected");
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
    //   } else if (ndt_neighbor_search_method_ == "DIRECT7") {
    //     RCLCPP_INFO(this->get_logger(), "search_method DIRECT7 is selected");
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
    //   } else if (ndt_neighbor_search_method_ == "DIRECT_RADIUS") {
    //     RCLCPP_INFO(this->get_logger(), "search_method DIRECT_RADIUS is selected : %f", ndt_neighbor_search_radius_);
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, ndt_neighbor_search_radius_);
    //   } else {
    //     RCLCPP_WARN(this->get_logger(), "invalid search method was given");
    //   }
    //   return ndt;
    // }

    RCLCPP_ERROR(this->get_logger(), "unknown registration method: %s", reg_method_.c_str());

    return nullptr;
  }

  void initialize_params() {
    robot_odom_frame_id_ = this->declare_parameter("robot_odom_frame_id", "robot_odom");
    odom_child_frame_id_ = this->declare_parameter("odom_child_frame_id", "base_link");
    use_imu_ = this->declare_parameter("use_imu", true);
    imu_linear_acc_unit_g_ = this->declare_parameter("imu_linear_acc_unit_g", true);
    invert_acc_ = this->declare_parameter("invert_acc", false);
    invert_gyro_ = this->declare_parameter("invert_gyro", false);

    use_global_localization_ = this->declare_parameter("use_global_localization", true);

    downsample_resolution_ = this->declare_parameter<double>("downsample_resolution", 0.1);
    specify_init_pose_ = this->declare_parameter<bool>("specify_init_pose", true);

    init_pos_x_ = this->declare_parameter<double>("init_pos_x", 0.0);
    init_pos_y_ = this->declare_parameter<double>("init_pos_y", 0.0);
    init_pos_z_ = this->declare_parameter<double>("init_pos_z", 0.0);
    init_ori_w_ = this->declare_parameter<double>("init_ori_w", 1.0);
    init_ori_x_ = this->declare_parameter<double>("init_ori_x", 0.0);
    init_ori_y_ = this->declare_parameter<double>("init_ori_y", 0.0);
    init_ori_z_ = this->declare_parameter<double>("init_ori_z", 0.0);
    cool_time_duration_ = this->declare_parameter<double>("cool_time_duration", 0.5);

    reg_method_ = this->declare_parameter<std::string>("reg_method", "NDT_OMP");
    ndt_neighbor_search_method_ = this->declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT7");
    ndt_neighbor_search_radius_ = this->declare_parameter<double>("ndt_neighbor_search_radius", 2.0);
    ndt_resolution_ = this->declare_parameter<double>("ndt_resolution", 1.0);

    max_correspondence_dist_ = this->declare_parameter<double>("status_max_correspondence_dist", 0.5);
    max_valid_point_dist_ = this->declare_parameter<double>("status_max_valid_point_dist", 25.0);

    enable_odom_prediction_ = this->declare_parameter<bool>("enable_robot_odometry_prediction", false);

    // intialize scan matching method
    auto voxelgrid = std::make_shared<pcl::VoxelGrid<PointT>>();
    voxelgrid->setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);
    downsample_filter_ = voxelgrid;

    RCLCPP_INFO(this->get_logger(), "create registration method for localization");
    registration_ = create_registration();

    // global localization
    RCLCPP_INFO(this->get_logger(), "create registration method for fallback during relocalization");
    relocalizing_ = false;
    delta_estimater_.reset(new DeltaEstimater(create_registration()));

    // initialize pose estimator
    if (specify_init_pose_) {
      RCLCPP_INFO(this->get_logger(), "initialize pose estimator with specified parameters!!");

      pose_estimator_.reset(new hdl_localization::PoseEstimator(
        registration_,
        Eigen::Vector3f(init_pos_x_, init_pos_y_, init_pos_z_),
        Eigen::Quaternionf(init_ori_w_, init_ori_x_, init_ori_y_, init_ori_z_),
        cool_time_duration_));
    }
  }

private:
  /**
   * @brief callback for imu data
   * @param imu_msg
   */
void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
  // Convert g → m/s^2 if needed
  if (imu_linear_acc_unit_g_) {     
    constexpr double g = 9.80665;
    msg->linear_acceleration.x *= g;
    msg->linear_acceleration.y *= g;
    msg->linear_acceleration.z *= g;
  }

  // Transform IMU data into odom_child_frame_id_
  geometry_msgs::msg::TransformStamped tf_imu;
  try {
    tf_imu = tf_buffer_->lookupTransform(
      odom_child_frame_id_,        // target frame
      msg->header.frame_id,        // IMU frame
      msg->header.stamp,           // use IMU timestamp
      rclcpp::Duration::from_seconds(0.05)
    );
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "IMU TF failed: %s", ex.what());
    return;
  }

  // Extract rotation
  Eigen::Matrix3d R = tf2::transformToEigen(tf_imu).rotation();

  // Original vectors
  Eigen::Vector3f acc(msg->linear_acceleration.x,
                      msg->linear_acceleration.y,
                      msg->linear_acceleration.z);

  Eigen::Vector3f gyro(msg->angular_velocity.x,
                       msg->angular_velocity.y,
                       msg->angular_velocity.z);

  // Rotate into odom_child_frame_id_
  Eigen::Vector3f acc_rot = R.cast<float>() * acc;
  Eigen::Vector3f gyro_rot = R.cast<float>() * gyro;

  // Write back into message (overwrite with transformed data)
  msg->linear_acceleration.x = acc_rot.x();
  msg->linear_acceleration.y = acc_rot.y();
  msg->linear_acceleration.z = acc_rot.z();

  msg->angular_velocity.x = gyro_rot.x();
  msg->angular_velocity.y = gyro_rot.y();
  msg->angular_velocity.z = gyro_rot.z();

  // IMPORTANT: also update frame_id to reflect new frame
  msg->header.frame_id = odom_child_frame_id_;

  // Store transformed IMU
  std::lock_guard<std::mutex> lock(imu_data_mutex_);
  imu_data_.push_back(msg);
}

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
    if (!globalmap_) {
      RCLCPP_ERROR(this->get_logger(), "No global map");
      return;
    }

    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    if (pcl_cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "cloud is empty!!");
      return;
    }

    // transform pointcloud into odom_child_frame_id
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    try {
      geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(odom_child_frame_id_, points_msg->header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));

      sensor_msgs::msg::PointCloud2 transformed_msg;
      tf2::doTransform(*points_msg, transformed_msg, tf);
      pcl::fromROSMsg(transformed_msg, *cloud);

    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }

    auto filtered = downsample(cloud);
    last_scan_ = filtered;

    if (relocalizing_) {
      delta_estimater_->add_frame(filtered);
    }

    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex_);
    if (!pose_estimator_) {
      RCLCPP_ERROR(this->get_logger(), "waiting for initial pose input!!");
      return;
    }
    Eigen::Matrix4f before = pose_estimator_->matrix();

    // predict
    if (!use_imu_) {
      pose_estimator_->predict(stamp);
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex_);
      auto imu_iter = imu_data_.begin();
      for (imu_iter; imu_iter != imu_data_.end(); imu_iter++) {
        if (rclcpp::Time(stamp) < rclcpp::Time((*imu_iter)->header.stamp)) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double acc_sign = invert_acc_ ? -1.0 : 1.0;
        double gyro_sign = invert_gyro_ ? -1.0 : 1.0;
        pose_estimator_->predict((*imu_iter)->header.stamp, acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data_.erase(imu_data_.begin(), imu_iter);
    }

    // odometry-based prediction
    rclcpp::Time last_correction_time = pose_estimator_->last_correction_time();

    if (enable_odom_prediction_ && last_correction_time.nanoseconds() != 0) {
      geometry_msgs::msg::TransformStamped odom_delta;
      try {
        odom_delta =
          tf_buffer_->lookupTransform(odom_child_frame_id_, last_correction_time, odom_child_frame_id_, stamp, robot_odom_frame_id_, rclcpp::Duration::from_seconds(0.1));
      } catch (tf2::TransformException&) {
        try {
          odom_delta = tf_buffer_->lookupTransform(odom_child_frame_id_, last_correction_time, odom_child_frame_id_, rclcpp::Time(0), robot_odom_frame_id_);
        } catch (tf2::TransformException& ex) {
          RCLCPP_WARN(this->get_logger(), "failed to look up transform: %s", ex.what());
        }
      }
      if (odom_delta.header.stamp.sec == 0) {
        RCLCPP_WARN(this->get_logger(), "failed to look up transform between %s and %s", cloud->header.frame_id.c_str(), robot_odom_frame_id_.c_str());
      } else {
        Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
        pose_estimator_->predict_odom(delta.cast<float>().matrix());
      }
    }

    // correct
    auto aligned = pose_estimator_->correct(stamp, filtered);

    if (aligned_pub_->get_subscription_count()) {
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);

      aligned_msg.header.frame_id = "map";
      aligned_msg.header.stamp = points_msg->header.stamp;
      aligned_pub_->publish(aligned_msg);
    }

    if (status_pub_->get_subscription_count()) {
      publish_scan_matching_status(points_msg->header, aligned);
    }

    publish_odometry(points_msg->header.stamp, pose_estimator_->matrix());
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
    RCLCPP_INFO(this->get_logger(), "globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap_ = cloud;

    registration_->setInputTarget(globalmap_);

    if (use_global_localization_) {
      RCLCPP_INFO(this->get_logger(), "set globalmap for global localization!");
      auto request = std::make_shared<hdl_global_localization::srv::SetGlobalMap::Request>();
      pcl::toROSMsg(*globalmap_, request->global_map);

      set_global_map_client_->async_send_request(
        request,
        [this](rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedFuture future) {
          try {
            future.get();  // no fields to inspect
            RCLCPP_INFO(this->get_logger(), "done");
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "service call failed: %s", e.what());
          }
        }
      );
    }
  }

  /**
   * @brief perform global localization to relocalize the sensor position
   * @param
   */
  bool relocalize(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    if (last_scan_ == nullptr) {
      RCLCPP_INFO(this->get_logger(), "no scan has been received");
      return false;
    }

    relocalizing_ = true;
    delta_estimater_->reset();
    pcl::PointCloud<PointT>::ConstPtr scan = last_scan_;

    auto request = std::make_shared<hdl_global_localization::srv::QueryGlobalLocalization::Request>();
    pcl::toROSMsg(*scan, request->cloud);
    request->max_num_candidates = 1;

    auto future = query_global_localization_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
      relocalizing_ = false;
      RCLCPP_INFO(this->get_logger(), "global localization failed (service call)");
      return false;
    }

    auto response = future.get();
    if (response->poses.empty()) {
      relocalizing_ = false;
      RCLCPP_INFO(this->get_logger(), "global localization failed (no poses)");
      return false;
    }
    const auto& result = response->poses[0];

    RCLCPP_INFO(this->get_logger(), "--- Global localization result ---");
    RCLCPP_INFO(this->get_logger(), "Trans : %f %f %f", result.position.x, result.position.y, result.position.z);
    RCLCPP_INFO(this->get_logger(), "Quat  : %f %f %f %f", result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w);
    RCLCPP_INFO(this->get_logger(), "Error : %f", response->errors[0]);
    RCLCPP_INFO(this->get_logger(), "Inlier: %f", response->inlier_fractions[0]);

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
    pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
    pose = pose * delta_estimater_->estimated_delta();

    std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
    pose_estimator_.reset(new hdl_localization::PoseEstimator(registration_, pose.translation(), Eigen::Quaternionf(pose.linear()), cool_time_duration_));

    relocalizing_ = false;

    return true;
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
    RCLCPP_INFO(this->get_logger(), "initial pose received!!");

    std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator_.reset(new hdl_localization::PoseEstimator(registration_, Eigen::Vector3f(p.x, p.y, p.z), Eigen::Quaternionf(q.w, q.x, q.y, q.z), cool_time_duration_));
  }

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if (!downsample_filter_) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter_->setInputCloud(cloud);
    downsample_filter_->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    if (tf_buffer_->_frameExists(robot_odom_frame_id_) &&
        tf_buffer_->_frameExists(odom_child_frame_id_) &&
        tf_buffer_->canTransform(robot_odom_frame_id_, odom_child_frame_id_, tf2::TimePointZero)) {
      geometry_msgs::msg::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
      map_wrt_frame.header.stamp = stamp;
      map_wrt_frame.header.frame_id = odom_child_frame_id_;
      map_wrt_frame.child_frame_id = "map";

      geometry_msgs::msg::TransformStamped frame_wrt_odom = tf_buffer_->lookupTransform(robot_odom_frame_id_, odom_child_frame_id_, tf2::TimePointZero, tf2::durationFromSec(0.1));
      Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

      geometry_msgs::msg::TransformStamped map_wrt_odom;
      tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

      tf2::Transform odom_wrt_map;
      tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
      odom_wrt_map = odom_wrt_map.inverse();

      geometry_msgs::msg::TransformStamped odom_trans;
      odom_trans.transform = tf2::toMsg(odom_wrt_map);
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = robot_odom_frame_id_;

      tf_broadcaster_->sendTransform(odom_trans);
    } else {
      geometry_msgs::msg::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = odom_child_frame_id_;
      tf_broadcaster_->sendTransform(odom_trans);
    }

    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    odom.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
    odom.child_frame_id = odom_child_frame_id_;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub_->publish(odom);
  }

  /**
   * @brief publish scan matching status information
   */
  void publish_scan_matching_status(const std_msgs::msg::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    hdl_localization::msg::ScanMatchingStatus status;
    status.header = header;

    status.has_converged = registration_->hasConverged();
    status.matching_error = 0.0;

    int num_inliers = 0;
    int num_valid_points = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for (int i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      if (pt.getVector3fMap().norm() > max_valid_point_dist_) {
        continue;
      }
      num_valid_points++;

      registration_->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if (k_sq_dists[0] < max_correspondence_dist_ * max_correspondence_dist_) {
        status.matching_error += k_sq_dists[0];
        num_inliers++;
      }
    }

    status.matching_error /= num_inliers;
    status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
    status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration_->getFinalTransformation().cast<double>())).transform;

    status.prediction_labels.reserve(2);
    status.prediction_errors.reserve(2);

    std::vector<double> errors(6, 0.0);

    if (pose_estimator_->wo_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "without_pred";

      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator_->wo_prediction_error().get().cast<double>())).transform);
    }

    if (pose_estimator_->imu_prediction_error()) {
      std_msgs::msg::String label_msg;
      label_msg.data = use_imu_ ? "imu" : "motion_model";
      status.prediction_labels.push_back(label_msg);

      geometry_msgs::msg::Transform transform_msg = tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator_->imu_prediction_error().get().cast<double>())).transform;
      status.prediction_errors.push_back(transform_msg);
    }

    if (pose_estimator_->odom_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "odom";

      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator_->odom_prediction_error().get().cast<double>())).transform);
    }

    status_pub_->publish(status);
  }

private:
  // parameters
  std::string robot_odom_frame_id_;
  std::string odom_child_frame_id_;

  double downsample_resolution_;
  bool specify_init_pose_;

  bool use_imu_;
  bool imu_linear_acc_unit_g_;
  bool invert_acc_;
  bool invert_gyro_;
  double max_correspondence_dist_, max_valid_point_dist_;
  double init_pos_x_, init_pos_y_, init_pos_z_;
  double init_ori_w_, init_ori_x_, init_ori_y_, init_ori_z_;
  double cool_time_duration_;

  std::string reg_method_;
  std::string ndt_neighbor_search_method_;
  double ndt_neighbor_search_radius_;
  double ndt_resolution_;

  bool enable_odom_prediction_;
  // parameters end

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<hdl_localization::msg::ScanMatchingStatus>::SharedPtr status_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // imu input buffer
  std::mutex imu_data_mutex_;
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_data_;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap_;
  pcl::Filter<PointT>::Ptr downsample_filter_;
  pcl::Registration<PointT, PointT>::Ptr registration_;

  // pose estimator
  std::mutex pose_estimator_mutex_;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator_;

  // global localization
  bool use_global_localization_;
  std::atomic_bool relocalizing_;
  std::unique_ptr<DeltaEstimater> delta_estimater_;

  pcl::PointCloud<PointT>::ConstPtr last_scan_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_service_;
  rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedPtr set_global_map_client_;
  rclcpp::Client<hdl_global_localization::srv::QueryGlobalLocalization>::SharedPtr query_global_localization_client_;
};
}  // namespace hdl_localization

RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::HdlLocalizationComponent)
