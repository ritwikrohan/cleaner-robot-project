#ifndef LASER_SEGMENTATION__LASER_SEGMENTATION_HPP_
#define LASER_SEGMENTATION__LASER_SEGMENTATION_HPP_

// C++
#include <string>

// ROS
#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

// SIMPLE LASER GEOMETRY
#include "slg_msgs/segment2D.hpp"
#include "slg_msgs/msg/segment_array.hpp"
#include "slg_msgs/msg/centroids.hpp"

// LASER SEGMENTATION
#include "laser_segmentation/parula.hpp"
#include "laser_segmentation/segmentation/segmentation.hpp"
#include "laser_segmentation/segmentation/segmentationJumpDistance.hpp"
#include "laser_segmentation/segmentation/segmentationJumpDistanceMerge.hpp"

class laserSegmentation: public rclcpp::Node{
	public:
		laserSegmentation();
		~laserSegmentation();
	private:
		rclcpp::Publisher<slg_msgs::msg::SegmentArray>::SharedPtr segment_pub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr segment_viz_points_pub_;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<slg_msgs::msg::Centroids>::SharedPtr centroid_pub_;

		OnSetParametersCallbackHandle::SharedPtr callback_handle_;
		//std::vector<rclcpp::Parameter> lastConfig_, defaultConfig_;

		std::string scan_topic_, seg_topic_, segmentation_type_, method_thres_;
		int min_points_, max_points_;
		double min_avg_distance_from_sensor_, max_avg_distance_from_sensor_, min_segment_width_, max_segment_width_, distance_thres_, noise_reduction_;
		bool setup_, restore_;
		Segmentation::SharedPtr segmentation_;
        std_msgs::msg::Header odom_header;
        rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
        rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
        bool initialized = false;
        slg::Point2D prev_centroid;
        slg_msgs::msg::SegmentArray segment_array_msg;
        slg_msgs::msg::Centroids centroids_array;
        geometry_msgs::msg::Point single_centroid;

		rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
		void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
        void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
		void show_visualization(std_msgs::msg::Header header, std::vector<slg::Segment2D> segmentList);
        double calculateAngle(const slg::Segment2D& seg1, const slg::Segment2D& seg2, const slg::Segment2D& seg3);
		std_msgs::msg::ColorRGBA get_parula_color(unsigned int index, unsigned int max);
		std_msgs::msg::ColorRGBA get_palette_color(unsigned int index);
};

#endif  // LASER_SEGMENTATION__LASER_SEGMENTATION_HPP_
