#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger() : Node("pointcloud_merger")
    {
        sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/MID360_lidar", 10,
            std::bind(&PointCloudMerger::callback1, this, std::placeholders::_1));

        sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/Horizon_lidar", 10,
            std::bind(&PointCloudMerger::callback2, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_pointcloud", 10);
    }

private:
    void callback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        cloud1_ = msg;
        mergeClouds();
    }

    void callback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        cloud2_ = msg;
        mergeClouds();
    }

    void mergeClouds()
    {
        if (!cloud1_ || !cloud2_) return;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1_pcl(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2_pcl(new pcl::PointCloud<pcl::PointXYZI>());

pcl::fromROSMsg(*cloud1_, *cloud1_pcl);
pcl::fromROSMsg(*cloud2_, *cloud2_pcl);

pcl::PointCloud<pcl::PointXYZI> merged_pcl = *cloud1_pcl + *cloud2_pcl;

sensor_msgs::msg::PointCloud2 merged_msg;
pcl::toROSMsg(merged_pcl, merged_msg);
merged_msg.header.frame_id = cloud1_->header.frame_id;
pub_->publish(merged_msg);
        RCLCPP_INFO(this->get_logger(), "Merged pointcloud published. Total points: %u", merged_pcl.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    sensor_msgs::msg::PointCloud2::SharedPtr cloud1_;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud2_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
