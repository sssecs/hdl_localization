#include <mutex>
#include <memory>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace hdl_localization {

class GlobalmapServerComponent : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  explicit GlobalmapServerComponent(const rclcpp::NodeOptions& options)
  : Node("globalmap_server", options)
  {
    initialize_params();

    rclcpp::QoS qos(5);
    qos.transient_local();

    // publish globalmap with "latched" publisher
    globalmap_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/globalmap", qos);

    map_update_sub = this->create_subscription<std_msgs::msg::String>(
      "/map_request/pcd",
      10,
      std::bind(&GlobalmapServerComponent::map_update_callback, this, std::placeholders::_1)
    );
    globalmap_pub_timer = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GlobalmapServerComponent::pub_once_cb, this)
    );
  }

private:
  void initialize_params() {
    // read globalmap from a pcd file
    std::string globalmap_pcd = this->declare_parameter<std::string>("globalmap_pcd", "");
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    std::ifstream utm_file(globalmap_pcd + ".utm");
    if (utm_file.is_open() && this->declare_parameter<bool>("convert_utm_to_local", true)) {
      double utm_easting;
      double utm_northing;
      double altitude;
      utm_file >> utm_easting >> utm_northing >> altitude;
      for(auto& pt : globalmap->points) {
        pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
      }
      RCLCPP_INFO(this->get_logger(),
        "Global map offset (x=%f, y=%f, z=%f)",
        utm_easting, utm_northing, altitude);
    }

    // downsample globalmap
    double downsample_resolution = this->declare_parameter<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
  }

  void pub_once_cb() {
    publish_map();

    // emulate one-shot timer
    globalmap_pub_timer->cancel();
  }

  void publish_map() {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*globalmap, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();

    globalmap_pub->publish(msg);
  }

  void map_update_callback(
    const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received map request, map path: %s", msg->data.c_str());
    std::string globalmap_pcd = msg->data;
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    // downsample globalmap
    double downsample_resolution = this->get_parameter("downsample_resolution").as_double();
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
    publish_map();
  }

private:
  // ROS
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_update_sub;
  rclcpp::TimerBase::SharedPtr globalmap_pub_timer;

  pcl::PointCloud<PointT>::Ptr globalmap;
};

}


RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::GlobalmapServerComponent)
