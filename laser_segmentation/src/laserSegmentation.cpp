#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include "laser_segmentation/laserSegmentation.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "std_msgs/msg/detail/header__struct.hpp"

/* Initialize the subscribers and publishers */
laserSegmentation::laserSegmentation()
    : Node("laser_segmentation"), setup_(false) {
  // Initialize ROS parameters

  // INTEGER PARAMS
  // ..........................................................................
  nav2_util::declare_parameter_if_not_declared(
      this, "min_points_segment", rclcpp::ParameterValue(3),
      rcl_interfaces::msg::ParameterDescriptor()
          .set__description("Minimium number of points per segment")
          .set__integer_range({rcl_interfaces::msg::IntegerRange()
                                   .set__from_value(0)
                                   .set__to_value(100)
                                   .set__step(1)}));
  this->get_parameter("min_points_segment", min_points_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter min_points_segment is set to: [%d]", min_points_);

  nav2_util::declare_parameter_if_not_declared(
      this, "max_points_segment", rclcpp::ParameterValue(200),
      rcl_interfaces::msg::ParameterDescriptor()
          .set__description("Maximum number of points per segment")
          .set__integer_range({rcl_interfaces::msg::IntegerRange()
                                   .set__from_value(5)
                                   .set__to_value(500)
                                   .set__step(1)}));
  this->get_parameter("max_points_segment", max_points_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter max_points_segment is set to: [%d]", max_points_);

  // FLOAT PARAMS
  // ..........................................................................
  nav2_util::declare_parameter_if_not_declared(
      this, "min_avg_distance_from_sensor", rclcpp::ParameterValue(0.0),
      rcl_interfaces::msg::ParameterDescriptor()
          .set__description("Minimium average distance from sensor")
          .set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
                                          .set__from_value(0.0)
                                          .set__to_value(1.0)
                                          .set__step(0.01)}));
  this->get_parameter("min_avg_distance_from_sensor",
                      min_avg_distance_from_sensor_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter min_avg_distance_from_sensor is set to: [%f]",
              min_avg_distance_from_sensor_);

  nav2_util::declare_parameter_if_not_declared(
      this, "max_avg_distance_from_sensor", rclcpp::ParameterValue(20.0),
      rcl_interfaces::msg::ParameterDescriptor()
          .set__description("Maximum average distance from sensor")
          .set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
                                          .set__from_value(0.0)
                                          .set__to_value(50.0)
                                          .set__step(0.01)}));
  this->get_parameter("max_avg_distance_from_sensor",
                      max_avg_distance_from_sensor_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter max_avg_distance_from_sensor is set to: [%f]",
              max_avg_distance_from_sensor_);

  nav2_util::declare_parameter_if_not_declared(
      this, "min_segment_width", rclcpp::ParameterValue(0.20),
      rcl_interfaces::msg::ParameterDescriptor()
          .set__description("Minimium width of the segment.")
          .set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
                                          .set__from_value(0.0)
                                          .set__to_value(5.0)
                                          .set__step(0.01)}));
  this->get_parameter("min_segment_width", min_segment_width_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter min_segment_width is set to: [%f]",
              min_segment_width_);

  nav2_util::declare_parameter_if_not_declared(
      this, "max_segment_width", rclcpp::ParameterValue(10.0),
      rcl_interfaces::msg::ParameterDescriptor()
          .set__description("Maximum width of the segment.")
          .set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
                                          .set__from_value(0.0)
                                          .set__to_value(100.0)
                                          .set__step(0.1)}));
  this->get_parameter("max_segment_width", max_segment_width_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter max_segment_width is set to: [%f]",
              max_segment_width_);

  nav2_util::declare_parameter_if_not_declared(
      this, "distance_threshold", rclcpp::ParameterValue(0.30),
      rcl_interfaces::msg::ParameterDescriptor()
          .set__description("The jump distance above which a new segment is "
                            "created or noise reduction if method is selected.")
          .set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
                                          .set__from_value(0.0)
                                          .set__to_value(2.0)
                                          .set__step(0.01)}));
  this->get_parameter("distance_threshold", distance_thres_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter distance_threshold is set to: [%f]",
              distance_thres_);

  nav2_util::declare_parameter_if_not_declared(
      this, "noise_reduction", rclcpp::ParameterValue(0.30),
      rcl_interfaces::msg::ParameterDescriptor()
          .set__description("Parameter for noise reduction in 'Santos' and "
                            "'Dietmayer' algorithms.")
          .set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
                                          .set__from_value(0.0)
                                          .set__to_value(2.0)
                                          .set__step(0.1)}));
  this->get_parameter("noise_reduction", noise_reduction_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter noise_reduction is set to: [%f]",
              noise_reduction_);

  // BOOLEAN PARAMS
  // ..........................................................................
  nav2_util::declare_parameter_if_not_declared(
      this, "restore_defaults", rclcpp::ParameterValue(false),
      rcl_interfaces::msg::ParameterDescriptor().set__description(
          "Restore to the original configuration"));
  this->get_parameter("restore_defaults", restore_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter restore_defaults is set to: [%s]",
              restore_ ? "true" : "false");

  // STRING PARAMS
  // ..........................................................................
  nav2_util::declare_parameter_if_not_declared(
      this, "method_threshold", rclcpp::ParameterValue(""),
      rcl_interfaces::msg::ParameterDescriptor().set__description(
          "Method to calculate a dynamic jump distance threshold in "
          "jump_distance algorithm"));
  this->get_parameter("method_threshold", method_thres_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter method_threshold is set to: [%s]",
              method_thres_.c_str());

  nav2_util::declare_parameter_if_not_declared(
      this, "scan_topic", rclcpp::ParameterValue("scan"),
      rcl_interfaces::msg::ParameterDescriptor().set__description(
          "Laser topic to read]"));
  this->get_parameter("scan_topic", scan_topic_);
  RCLCPP_INFO(this->get_logger(), "The parameter scan_topic is set to: [%s]",
              scan_topic_.c_str());

  nav2_util::declare_parameter_if_not_declared(
      this, "segments_topic", rclcpp::ParameterValue("segments"),
      rcl_interfaces::msg::ParameterDescriptor().set__description(
          "published Segements topic"));
  this->get_parameter("segments_topic", seg_topic_);
  RCLCPP_INFO(this->get_logger(), "The parameter segment_topic is set to: [%s]",
              seg_topic_.c_str());

  nav2_util::declare_parameter_if_not_declared(
      this, "segmentation_type", rclcpp::ParameterValue("jump_distance"),
      rcl_interfaces::msg::ParameterDescriptor().set__description(
          "Type of segmentation"));
  this->get_parameter("segmentation_type", segmentation_type_);
  RCLCPP_INFO(this->get_logger(),
              "The parameter segmentation_type is set to: [%s]",
              segmentation_type_.c_str());

  // Callback for monitor changes in parameters
  callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &laserSegmentation::parameters_callback, this, std::placeholders::_1));

  // Setting for segmentation algorithm
  if (segmentation_type_ == "jump_distance") {
    segmentation_.reset(new JumpDistanceSegmentation);
  } else if (segmentation_type_ == "jump_distance_merge") {
    segmentation_.reset(new JumpDistanceSegmentationMerge);
  } else {
    RCLCPP_FATAL(this->get_logger(), "Segmentation algorithm is invalid: %s]",
                 segmentation_type_.c_str());
    return;
  }

  // Publishers
  segment_pub_ =
      this->create_publisher<slg_msgs::msg::SegmentArray>(seg_topic_, 10);
  segment_viz_points_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          seg_topic_ + "/visualization", 10);
	centroid_pub_ = this->create_publisher<slg_msgs::msg::Centroids>(seg_topic_ + "/centroids", 10);

  // Subscribers
  odom_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options_odom;
  options_odom.callback_group = odom_callback_group_;
  scan_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options_scan;
  options_scan.callback_group = scan_callback_group_;

  auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, default_qos,
      std::bind(&laserSegmentation::scan_callback, this, std::placeholders::_1),
      options_scan);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/turtlebot_5/odom", default_qos,
      std::bind(&laserSegmentation::odom_callback, this, std::placeholders::_1),
      options_odom);
}

laserSegmentation::~laserSegmentation() {}

rcl_interfaces::msg::SetParametersResult laserSegmentation::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param : parameters) {
    // INTEGER PARAMS
    // ..........................................................................
    if (param.get_name() == "min_points_segment" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      min_points_ = param.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "The parameter min_points_segment is set to: [%d]",
                  min_points_);
    }

    if (param.get_name() == "max_points_segment" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      max_points_ = param.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "The parameter max_points_segment is set to: [%d]",
                  max_points_);
    }
    // FLOAT PARAMS
    // ..........................................................................
    if (param.get_name() == "min_avg_distance_from_sensor" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      min_avg_distance_from_sensor_ = param.as_double();
      RCLCPP_INFO(
          this->get_logger(),
          "The parameter min_avg_distance_from_sensor is set to: [%3.3f]",
          min_avg_distance_from_sensor_);
    }

    if (param.get_name() == "max_avg_distance_from_sensor" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      max_avg_distance_from_sensor_ = param.as_double();
      RCLCPP_INFO(
          this->get_logger(),
          "The parameter max_avg_distance_from_sensor is set to: [%3.3f]",
          max_avg_distance_from_sensor_);
    }

    if (param.get_name() == "min_segment_width" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      min_segment_width_ = param.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "The parameter min_segment_width is set to: [%3.3f]",
                  min_segment_width_);
    }

    if (param.get_name() == "max_segment_width" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      max_segment_width_ = param.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "The parameter max_segment_width is set to: [%3.3f]",
                  max_segment_width_);
    }

    if (param.get_name() == "distance_threshold" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      distance_thres_ = param.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "The parameter distance_threshold is set to: [%3.3f]",
                  distance_thres_);
    }

    if (param.get_name() == "noise_reduction" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      distance_thres_ = param.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "The parameter noise_reduction is set to: [%3.3f]",
                  noise_reduction_);
    }

    // STRING PARAMS
    // ..........................................................................
    if (param.get_name() == "method_threshold" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      method_thres_ = param.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "The parameter method_threshold is set to: [%s]",
                  method_thres_.c_str());
    }
  }

  return result;
}

/* Callback function */
// This function is required so that there is no time discrepancy in rviz visualisation. setting odom header in visualisation makes it consistent
void laserSegmentation::odom_callback(
    const std::shared_ptr<nav_msgs::msg::Odometry> msg) {

  this->odom_header = msg->header;
}

// Function to calculate the angle between three segments
double laserSegmentation::calculateAngle(const slg::Segment2D& seg1, const slg::Segment2D& seg2, const slg::Segment2D& seg3) {
    slg::Vector2D vec1 = seg2.centroid() - seg1.centroid();
    slg::Vector2D vec2 = seg3.centroid() - seg2.centroid();

    double dotProduct = vec1.dot(vec2);
    double magnitudeProduct = vec1.length() * vec2.length();

    if (magnitudeProduct == 0) {
        return 0; // Avoid division by zero
    }

    double cosTheta = dotProduct / magnitudeProduct;
    return std::acos(cosTheta) * (180.0 / M_PI); // Convert radians to degrees
}

void laserSegmentation::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  // Note: Only perform laserscan segmentation if there's any subscriber
  if (segment_pub_->get_subscription_count() == 0 &&
      segment_viz_points_pub_->get_subscription_count() == 0) {
      RCLCPP_INFO_ONCE(this->get_logger(), "NOT Subscribed to laser scan topic: [%s]",
                   scan_topic_.c_str());
    return;
  }
  RCLCPP_INFO_ONCE(this->get_logger(), "Subscribed to laser scan topic: [%s]",
                   scan_topic_.c_str());

    double front_angle_min = -M_PI / 4.0;  // Adjust as needed
    double front_angle_max = M_PI / 4.0;   // Adjust as needed

    // Read the laser scan for the specified range of angles
    std::vector<slg::Point2D> point_list;
    double phi = scan_msg->angle_min;
    double angle_resolution = scan_msg->angle_increment;
    for (const auto r : scan_msg->ranges) {
        // Check if the angle is within the specified front range
        // if (phi >= front_angle_min && phi <= front_angle_max) {
            if (r >= scan_msg->range_min && r <= scan_msg->range_max) {
                point_list.push_back(slg::Point2D::from_polar_coords(r, phi));
            } else {
                point_list.push_back(slg::Point2D::NaN());
            }
        // }
        phi += angle_resolution;
    }
  // Read the laser scan
//   std::vector<slg::Point2D> point_list;
//   double phi = scan_msg->angle_min;
//   double angle_resolution = scan_msg->angle_increment;
//   for (const auto r : scan_msg->ranges) {
//     if (r >= scan_msg->range_min && r <= scan_msg->range_max) {
//       point_list.push_back(slg::Point2D::from_polar_coords(r, phi));
//     } else {
//       point_list.push_back(slg::Point2D::NaN());
//     }
//     phi += angle_resolution;
//   }

 // Segment the points
  std::vector<slg::Segment2D> segment_list;
  segmentation_->initialize_segmentation(distance_thres_, angle_resolution,
                                         noise_reduction_, method_thres_);
  segmentation_->perform_segmentation(point_list, segment_list);

  // Filter segments
  double squared_min_segment_width = min_segment_width_ * min_segment_width_;
  double squared_max_segment_width = max_segment_width_ * max_segment_width_;
//   std::vector<slg::Segment2D> segment_pre_filtered_list;
  std::vector<slg::Segment2D> segment_filtered_list;
  segment_filtered_list.reserve(segment_list.size());
  double max_centroid_distance = 1.3; // Set your desired maximum distance here

  for (const auto &segment : segment_list) {
    // By number of points
    if (segment.size() < min_points_ || segment.size() > max_points_)
      continue;

    // By distance to sensor
    if (segment.centroid().length() < min_avg_distance_from_sensor_ ||
        segment.centroid().length() > max_avg_distance_from_sensor_)
      continue;

    // By width
    if (segment.width_squared() < squared_min_segment_width ||
        segment.width_squared() > squared_max_segment_width)
      continue;

    // //To make the code more robust, I tried adding an additional check to check if scan is L shaped. But its a bit inconsistent
    // if (segment.IsLShaped())
    //     continue;

    
    segment_filtered_list.push_back(segment);
  }
  
const double min_pair_distance = 0.48;  // 0.48 - 0.02
const double max_pair_distance = 0.499;  // 0.48 + 0.02

// Create a set to keep track of already considered pairs
std::set<std::pair<size_t, size_t>> considered_pairs;

// Create a vector to store the final filtered segments
std::vector<slg::Segment2D> pre_final_filtered_segments;


std::set<slg::Point2D> unique_centroids;

for (size_t i = 0; i < segment_filtered_list.size(); ++i) {
    for (size_t j = i + 1; j < segment_filtered_list.size(); ++j) {
        double distance_between_segments = (segment_filtered_list[i].centroid() - segment_filtered_list[j].centroid()).length();

        // Check if the pair is within the desired range and not already considered
        if (distance_between_segments >= min_pair_distance && distance_between_segments <= max_pair_distance) {
            std::pair<size_t, size_t> current_pair(i, j);
            std::pair<size_t, size_t> reversed_pair(j, i);

            if (considered_pairs.find(current_pair) == considered_pairs.end() && considered_pairs.find(reversed_pair) == considered_pairs.end()) {
                // Both segments in the pair pass the distance filter
                // Check if centroids are unique before adding to the final list
                if (unique_centroids.insert(segment_filtered_list[i].centroid()).second) {
                    pre_final_filtered_segments.push_back(segment_filtered_list[i]);
                }
                if (unique_centroids.insert(segment_filtered_list[j].centroid()).second) {
                    pre_final_filtered_segments.push_back(segment_filtered_list[j]);
                }

                // Mark the pair as considered
                considered_pairs.insert(current_pair);
            }
        }
    }
}



// Create a set to keep track of already considered triplets
std::set<std::tuple<size_t, size_t, size_t>> considered_triplets;

// Create a vector to store the final filtered segments
std::vector<slg::Segment2D> final_filtered_segments;

std::set<slg::Point2D> unique_final_centroids;

for (size_t i = 0; i < pre_final_filtered_segments.size(); ++i) {
    for (size_t j = i + 1; j < pre_final_filtered_segments.size(); ++j) {
        for (size_t k = j + 1; k < pre_final_filtered_segments.size(); ++k) {
            double angle = calculateAngle(pre_final_filtered_segments[i], pre_final_filtered_segments[j], pre_final_filtered_segments[k]);

            // Adjust the threshold as needed for your specific scenario
            if (std::abs(angle - 90.0) < 5.0) {
                // If the angle is close to 90 degrees, keep the segments
                final_filtered_segments.push_back(pre_final_filtered_segments[i]);
                final_filtered_segments.push_back(pre_final_filtered_segments[j]);
                final_filtered_segments.push_back(pre_final_filtered_segments[k]);

                // Check if centroids are unique before adding to the final list
                if (unique_final_centroids.insert(pre_final_filtered_segments[i].centroid()).second) {
                    final_filtered_segments.push_back(pre_final_filtered_segments[i]);
                }
                if (unique_final_centroids.insert(pre_final_filtered_segments[j].centroid()).second) {
                    final_filtered_segments.push_back(pre_final_filtered_segments[j]);
                }
                if (unique_final_centroids.insert(pre_final_filtered_segments[k].centroid()).second) {
                    final_filtered_segments.push_back(pre_final_filtered_segments[k]);
                }

                // Mark the triplet as considered
                considered_triplets.insert(std::make_tuple(i, j, k));
            }
        }
    }
}

// Step 3: Sort the filtered segments based on centroid length
std::vector<slg::Segment2D> segment_history_;
std::sort(final_filtered_segments.begin(), final_filtered_segments.end(),
          [](const slg::Segment2D& a, const slg::Segment2D& b) {
              return a.centroid().length() < b.centroid().length();
          });

// Print the distance between segments in segment_filtered_list
for (size_t i = 0; i < final_filtered_segments.size(); ++i) {
    for (size_t j = i + 1; j < final_filtered_segments.size(); ++j) {
        double distance_between_segments = (final_filtered_segments[i].centroid() - final_filtered_segments[j].centroid()).length();
        RCLCPP_INFO(this->get_logger(), "Distance between segment %zu and segment %zu: %f", i, j, distance_between_segments);
    }
}

    // Create a vector to store the current detected legs
    std::vector<slg::Segment2D> current_detected_legs;

    // Assign unique IDs based on centroid proximity
    for (size_t s = 0; s < final_filtered_segments.size(); s++) {
    slg::Point2D current_centroid = final_filtered_segments[s].centroid();

    bool matched = false;

    // Check if the current centroid is close to any centroid in history
    for (size_t i = 0; i < segment_history_.size(); i++) {
        double centroid_distance = (current_centroid - segment_history_[i].centroid()).length();

        // Adjust the threshold as needed for your specific scenario
        if (centroid_distance < 0.5) {
        // If the centroid is close to any centroid in history, assign the same ID
        final_filtered_segments[s].set_id(segment_history_[i].id);
        matched = true;
        current_detected_legs.push_back(final_filtered_segments[s]);
        break;
        }
    }

    if (!matched) {
        // If the centroid is not close to any centroid in history, assign a new ID
        final_filtered_segments[s].set_id(s + segment_history_.size());
        current_detected_legs.push_back(final_filtered_segments[s]);
    }
    }

    // Update the history with the current detected legs
    segment_history_ = current_detected_legs;

  // Identification of segments and set angular distance
  segment_array_msg.header = scan_msg->header; // this->odom_header;
  for (const auto &segment : final_filtered_segments) {
		if(segment.id !=2)
    {
			segment_array_msg.segments.push_back(segment);
			single_centroid.x = segment.centroid().x;
			single_centroid.y = segment.centroid().y;
            // single_centroid.x = segment.minimumPoint().x;
			// single_centroid.y = segment.minimumPoint().y;
			single_centroid.z = segment.id;
			centroids_array.centroids.push_back(single_centroid); 
		}
	}

  centroid_pub_->publish(centroids_array);
  segment_pub_->publish(segment_array_msg);

  // Shows the segments
  show_visualization(scan_msg->header, final_filtered_segments);
}

/* Show the segments in rviz */
void laserSegmentation::show_visualization(
    std_msgs::msg::Header header, std::vector<slg::Segment2D> segment_list) {
  // Note: Only perform visualization if there's any subscriber
  if (segment_viz_points_pub_->get_subscription_count() == 0) {
    return;
  }

  // Create the visualization message
  visualization_msgs::msg::MarkerArray viz_array;

  // Create a marker point
  visualization_msgs::msg::Marker viz_point;
  viz_point.header = header;
//   viz_point.header.stamp = this->odom_header.stamp;
//   viz_point.header.frame_id = header.frame_id;
  viz_point.lifetime = rclcpp::Duration(0, 10);
  viz_point.ns = "segments";
  viz_point.type = visualization_msgs::msg::Marker::POINTS;
  viz_point.action = visualization_msgs::msg::Marker::ADD;
  viz_point.scale.x = 0.02;
  viz_point.scale.y = 0.02;

  // Create a marker centroid
  visualization_msgs::msg::Marker viz_centroids;
  viz_centroids.header = header;
//   viz_centroids.header.stamp = this->odom_header.stamp;
//   viz_centroids.header.frame_id = header.frame_id;
  viz_centroids.lifetime = rclcpp::Duration(0, 10);
  viz_centroids.ns = "centroids";
  viz_centroids.type = visualization_msgs::msg::Marker::CUBE;
  viz_centroids.action = visualization_msgs::msg::Marker::ADD;
  viz_centroids.scale.x = 0.2;
  viz_centroids.scale.y = 0.2;
  viz_centroids.scale.z = 0.2;
  viz_centroids.pose.orientation.x = 0.0;
  viz_centroids.pose.orientation.y = 0.0;
  viz_centroids.pose.orientation.z = 0.0;
  viz_centroids.pose.orientation.w = 1.0;
  viz_centroids.frame_locked = false;

  // Create a marker id text
  visualization_msgs::msg::Marker viz_text;
  viz_text.header = header;
//   viz_text.header.stamp = this->odom_header.stamp;
//   viz_text.header.frame_id = header.frame_id;
  viz_text.lifetime = rclcpp::Duration(0, 10);
  viz_text.ns = "id";
  viz_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  viz_text.action = visualization_msgs::msg::Marker::ADD;
  viz_text.pose.orientation.x = 0.0;
  viz_text.pose.orientation.y = 0.0;
  viz_text.pose.orientation.z = 0.0;
  viz_text.pose.orientation.w = 1.0;
  viz_text.scale.z = 0.25;
  viz_text.color.r = 1.0;
  viz_text.color.g = 1.0;
  viz_text.color.b = 1.0;
  viz_text.color.a = 1.0;

  // Create a deletion marker
  visualization_msgs::msg::Marker deletion_marker;
  deletion_marker.header = header;
  deletion_marker.action = visualization_msgs::msg::Marker::DELETEALL;

  // Push the deletion marker
  viz_array.markers.push_back(deletion_marker);

  /* Show the segments and the id */
  for (std::vector<slg::Segment2D>::size_type i = 0; i < segment_list.size(); i++) 
	// for (int i = 0; i<4; i++)
	{
    slg::Segment2D current_segment = segment_list[i];
    viz_point.id = i;
    viz_text.id = i;
    viz_centroids.id = i;

    // Change the color of the segment
    viz_point.color = get_palette_color(i);
    viz_centroids.color = get_palette_color(i);

    // Iterate over the points of the segment
    for (slg::Point2D point : current_segment.get_points()) {
      viz_point.points.push_back(point);
    }

    // Get position of the text
    viz_text.text = std::to_string(current_segment.get_id());
    viz_text.pose.position = current_segment.centroid();
    viz_text.pose.position.z = 0.10;

    // Place centroid under text
    viz_centroids.pose.position = current_segment.centroid();
    // RCLCPP_INFO(this->get_logger()," centroid: [%f], [%f]", current_segment.centroid().x, current_segment.centroid().y);
    viz_centroids.pose.position.z = 0.0;

    // Push to arrays
    viz_array.markers.push_back(viz_point);
    viz_array.markers.push_back(viz_centroids);
    viz_array.markers.push_back(viz_text);

    // Clear markers
    viz_point.points.clear();
  }
  // Publish visualization
  segment_viz_points_pub_->publish(viz_array);
}

/* Get Parula color of the class */
std_msgs::msg::ColorRGBA laserSegmentation::get_parula_color(unsigned int index,
                                                             unsigned int max) {
  std_msgs::msg::ColorRGBA color;
  int div = round(256 / max);
  color.r = parula[index * div][0];
  color.g = parula[index * div][1];
  color.b = parula[index * div][2];
  color.a = 1.0;
  return color;
}

/* Get palette color of the class */
std_msgs::msg::ColorRGBA
laserSegmentation::get_palette_color(unsigned int index) {
  std_msgs::msg::ColorRGBA color;
  switch (index % 8) {
  case 0:
    color.r = 255;
    color.g = 051;
    color.b = 051;
    break;
  case 2:
    color.r = 255;
    color.g = 153;
    color.b = 051;
    break;
  case 4:
    color.r = 255;
    color.g = 255;
    color.b = 051;
    break;
  case 6:
    color.r = 153;
    color.g = 051;
    color.b = 051;
    break;
  case 1:
    color.r = 051;
    color.g = 255;
    color.b = 051;
    break;
  case 3:
    color.r = 051;
    color.g = 255;
    color.b = 153;
    break;
  case 5:
    color.r = 051;
    color.g = 153;
    color.b = 255;
    break;
  case 7:
    color.r = 255;
    color.g = 051;
    color.b = 255;
    break;
  }

  color.r /= 255.0;
  color.g /= 255.0;
  color.b /= 255.0;
  color.a = 1.0;
  return color;
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<laserSegmentation>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
