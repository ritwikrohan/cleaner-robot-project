#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "attach_table/srv/go_to_loading.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <thread>
#include "slg_msgs/msg/centroids.hpp"
#include "slg_msgs/msg/segment_array.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace std::chrono_literals;

class FinalApproachService : public rclcpp::Node
{
    public:
        FinalApproachService() : Node("approach_service_server_node")
        {
            this->approach_server_ = this->create_service<attach_table::srv::GoToLoading>("/approach_table", std::bind(&FinalApproachService::approach_callback, this, std::placeholders::_1, std::placeholders::_2));
            centroid_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options_centroid;
            options_centroid.callback_group = centroid_callback_group_;
            centroid_sub_ = this->create_subscription<slg_msgs::msg::Centroids>("/segments/centroids", 10, std::bind(&FinalApproachService::centroidCallback, this, std::placeholders::_1), options_centroid);
            elevator_pub_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 1);
            // robot_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 1);
            robot_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }




    private:
        rclcpp::Service<attach_table::srv::GoToLoading>::SharedPtr approach_server_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;
        rclcpp::Subscription<slg_msgs::msg::Centroids>::SharedPtr centroid_sub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        rclcpp::CallbackGroup::SharedPtr centroid_callback_group_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_cmd_vel_publisher;
        int firstSetLastValue;
        int secondSetFirstValue;
        double kp_yaw = 0.8;//0.7;
        double kp_orient = 2.0;//0.7;
        double kp_distance = 0.5;
        double x;
        double y;
        std::vector<float> range;
        std::vector<float> intensity;
        double centroid0_x = 0.0;
        double centroid0_y = 0.0;
        double centroid1_x = 0.0;
        double centroid1_y = 0.0;
        double id;
        double map_x = 0.0;
        double map_y = 0.0;
        int table_number = 0;

        
        // Using centroid of the legs to calculate a middle point of two legs to create a static tf
        void centroidCallback(const std::shared_ptr<slg_msgs::msg::Centroids> msg)
        {   
            for (size_t i = 0; i < msg->centroids.size(); ++i)
            {
                const auto& current_centroid = msg->centroids[i];
                if (current_centroid.z == 0.0)
                {
                    this->centroid0_x = current_centroid.x;
                    this->centroid0_y = current_centroid.y;
                }
                else if (current_centroid.z == 1.0) 
                {
                    this->centroid1_x = current_centroid.x;
                    this->centroid1_y = current_centroid.y;
                }
            }
            this->x = (this->centroid0_x + this->centroid1_x)/2.0;
            this->y = (this->centroid0_y + this->centroid1_y)/2.0;
        }

        // function to create a static tf based on the arguement given which will be the mid point of centroids
        void publishStaticTransform(double x, double y, int table_number) {
            // Create a transform message
            geometry_msgs::msg::TransformStamped broadcast_transformStamped;
            broadcast_transformStamped.header.stamp = this->now();
            broadcast_transformStamped.header.frame_id = "robot_front_laser_base_link";  // Set the parent frame ID
            broadcast_transformStamped.child_frame_id = "table_front_frame";  // Set the child frame ID


            
            // Set the translation (x, y, z)
            RCLCPP_DEBUG(this->get_logger(),"base centroid x: %f",x);
            RCLCPP_DEBUG(this->get_logger(),"base centroid y: %f",y);
            broadcast_transformStamped.transform.translation.x = x;
            broadcast_transformStamped.transform.translation.y = y;
            broadcast_transformStamped.transform.translation.z = 0.0;  // Z is usually 0 for 2D transforms

            // Set the rotation (no rotation in a static transform)
            broadcast_transformStamped.transform.rotation.x = 0.0;
            broadcast_transformStamped.transform.rotation.y = 0.0;
            broadcast_transformStamped.transform.rotation.z = 0.0;
            broadcast_transformStamped.transform.rotation.w = 1.0;

            // Publish the static transform
            tf_static_broadcaster_->sendTransform(broadcast_transformStamped);
            RCLCPP_DEBUG(this->get_logger(), "PUBLISHED");


             // Wait for TF2 buffer to be ready
            tf2::TimePoint time_point = tf2::TimePoint(std::chrono::seconds(0));  // Use zero time for the latest available transform

            while (rclcpp::ok()) {
                if (tf_buffer_->canTransform("map", "table_front_frame", time_point)) {
                    // Transform the coordinates from "table_front_frame" to "map"
                    geometry_msgs::msg::TransformStamped map_transform;
                    try {
                        map_transform = tf_buffer_->lookupTransform("map", "table_front_frame", time_point);
                    } catch (tf2::TransformException& ex) {
                        RCLCPP_INFO(this->get_logger(), "Failed to transform coordinates: %s", ex.what());
                        continue;  // Retry if the transformation fails
                    }

                    this->map_x = map_transform.transform.translation.x;
                    this->map_y = map_transform.transform.translation.y;
                    
                    // Exit the loop once map_x and map_y have been found
                    break;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Transform not available. Waiting for transform...");
                }

                // Sleep for a short duration to avoid busy waiting
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            RCLCPP_DEBUG(this->get_logger(),"map_x: %f", this->map_x);
            RCLCPP_DEBUG(this->get_logger(),"map_y: %f", this->map_y);
            // publishNewFrame(1.000980, 0.785102);
            publishNewFrame(this->map_x, this->map_y, table_number);
        }

        void publishNewFrame(double map_x, double map_y, int table_number) {
            // Create a transform message for the new frame
            geometry_msgs::msg::TransformStamped new_frame_transformStamped;
            new_frame_transformStamped.header.stamp = this->now();
            new_frame_transformStamped.header.frame_id = "map";  // Set the parent frame ID
            new_frame_transformStamped.child_frame_id = "new_frame";  // Set the child frame ID

            // Set the translation (x, y, z)
            new_frame_transformStamped.transform.translation.x = map_x;
            new_frame_transformStamped.transform.translation.y = map_y;
            new_frame_transformStamped.transform.translation.z = 0.0;  // Z is usually 0 for 2D transforms

            // Set the rotation based on table_number
            double angle = M_PI / 2.0;  // 90 degrees in radians
            double sin_half_angle = sin(angle / 2.0);
            double cos_half_angle = cos(angle / 2.0);

            if (table_number == 1) {
                // Set the rotation for table_number 1
                new_frame_transformStamped.transform.rotation.x = 0.0;
                new_frame_transformStamped.transform.rotation.y = 0.0;
                new_frame_transformStamped.transform.rotation.z = 0.0;
                new_frame_transformStamped.transform.rotation.w = 1.0;
            } else if (table_number == 2) {
                // Set the rotation for table_number 2
                new_frame_transformStamped.transform.rotation.x = 0.0;
                new_frame_transformStamped.transform.rotation.y = 0.0;
                new_frame_transformStamped.transform.rotation.z = sin_half_angle;
                new_frame_transformStamped.transform.rotation.w = cos_half_angle;
            } else {
                // Handle invalid table_number or add more cases if needed
                RCLCPP_ERROR(this->get_logger(), "Invalid table_number: %d", table_number);
                return;
            }

            // Publish the new frame transform
            tf_static_broadcaster_->sendTransform(new_frame_transformStamped);
            RCLCPP_DEBUG(this->get_logger(), "Published new frame");
        }



        // void publishStaticmapTransform(double x_robot_base, double y_robot_base) {
        //     // Create a transform message
        //     geometry_msgs::msg::TransformStamped broadcast_transformStamped;
        //     broadcast_transformStamped.header.stamp = this->now();
        //     broadcast_transformStamped.header.frame_id = "map";  // Set the parent frame ID to map
        //     broadcast_transformStamped.child_frame_id = "table_front_frame";  // Set the child frame ID

        //     // Transform from robot_base_link to map
        //     geometry_msgs::msg::TransformStamped transform_robot_base_to_map;
        //     try {
        //         transform_robot_base_to_map = tf_buffer_->lookupTransform("map", "robot_front_laser_base_link", tf2::TimePoint(), tf2::durationFromSec(1.0));
        //     } catch (tf2::TransformException &ex) {
        //         RCLCPP_ERROR(this->get_logger(), "TF Exception: %s", ex.what());
        //         return;
        //     }

        //     // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
        //     tf2::Quaternion tf_quaternion(
        //         transform_robot_base_to_map.transform.rotation.x,
        //         transform_robot_base_to_map.transform.rotation.y,
        //         transform_robot_base_to_map.transform.rotation.z,
        //         transform_robot_base_to_map.transform.rotation.w
        //     );

        //     // Convert geometry_msgs::msg::Vector3 to tf2::Vector3
        //     tf2::Vector3 tf_translation(
        //         transform_robot_base_to_map.transform.translation.x,
        //         transform_robot_base_to_map.transform.translation.y,
        //         transform_robot_base_to_map.transform.translation.z
        //     );

        //     // Apply the transformation to get x and y in the map frame
        //     tf2::Transform tf_transform;
        //     tf_transform.setOrigin(tf_translation);  // Set the translation
        //     tf_transform.setRotation(tf_quaternion);

        //     // Transform a point from robot base to map frame
        //     // tf2::Vector3 point_in_map_frame = tf_transform * tf2::Vector3(x_robot_base, y_robot_base, 0.0);
        //     tf2::Vector3 point_in_map_frame = tf_transform * tf2::Vector3(0.738296, -0.007031, 0.0);

        //     // Set the translation (x, y, z)
        //     RCLCPP_INFO(this->get_logger(), "map centroid x: %f", point_in_map_frame.getX());
        //     RCLCPP_INFO(this->get_logger(), "map centroid y: %f", point_in_map_frame.getY());
        //     broadcast_transformStamped.transform.translation.x = 1.04;//point_in_map_frame.getX();
        //     broadcast_transformStamped.transform.translation.y = 0.808;//point_in_map_frame.getY();
        //     broadcast_transformStamped.transform.translation.z = 0.0;//point_in_map_frame.getZ();

        //     // Set the rotation (no rotation in a static transform)
        //     broadcast_transformStamped.transform.rotation.x = 0.0;
        //     broadcast_transformStamped.transform.rotation.y = 0.0;
        //     broadcast_transformStamped.transform.rotation.z = 0.0;
        //     broadcast_transformStamped.transform.rotation.w = 1.0;

        //     // Publish the static transform
        //     tf_static_broadcaster_->sendTransform(broadcast_transformStamped);
        //     RCLCPP_DEBUG(this->get_logger(), "PUBLISHED");
        // }





    

        // Simple P controller to follow the newly created tf between the legs
        void controlLoop() 
        {   
            bool final_goal = false;
            bool should_set_final_goal = false;
            // bool should_set_yaw_goal = false;
            // bool yaw_after_stopping =false;
            double maxAllowedDifference = 1.0;  // Adjust this threshold based on your requirement
            double lastPublishedX = this->x;
            double lastPublishedY = this->y;
            rclcpp::Rate loop_rate(100);
            while(!final_goal)
            {   
                // Check if the difference is not significant compared to the last published values
                if (std::abs(this->x - lastPublishedX) <= maxAllowedDifference && std::abs(this->y - lastPublishedY) <= maxAllowedDifference) {
                    publishStaticTransform(this->x, this->y, this->table_number);
                    // Update last published positions immediately after publishing
                    lastPublishedX = this->x;
                    lastPublishedY = this->y;
                }
                // publishStaticTransform(this->x, this->y);
                // final_goal = true;
                if ((tf_buffer_->canTransform("robot_base_link", "new_frame", tf2::TimePoint(), tf2::durationFromSec(0.5)))&& (!std::isinf(this->x)|| !std::isinf(this->y)) && !should_set_final_goal)
                {   
                    RCLCPP_DEBUG(this->get_logger(), "I have Entered if ");
                    try {
                        RCLCPP_DEBUG(this->get_logger(), "I have Entered try ");
                        geometry_msgs::msg::TransformStamped transform;
                        transform = tf_buffer_->lookupTransform("robot_base_link", "new_frame", tf2::TimePoint(), tf2::durationFromSec(1.0));

                        double error_distance = calculateDistanceError(transform);
                        double error_yaw = calculateYawError(transform);
                        double error_orientation = calculateOrientationError(transform);
                        RCLCPP_DEBUG(this->get_logger(),"error_d : %f", error_distance);
                        RCLCPP_DEBUG(this->get_logger(),"error_y : %f", error_yaw);
                        RCLCPP_DEBUG(this->get_logger(),"error_orient : %f", error_orientation);
                        geometry_msgs::msg::Twist twist;
                        if (error_distance < 0.5 && std::abs(error_yaw < 0.05)){
                        //  if (error_yaw < 0.05){
                            RCLCPP_DEBUG(this->get_logger(),"Early stage");
                            if (std::abs(error_orientation)>0.01)
                            {   
                                // RCLCPP_INFO(this->get_logger(),"error_orient : %f", error_orientation);
                                RCLCPP_DEBUG(this->get_logger(),"error_orient : %f", error_orientation);
                                twist.linear.x = 0.0; 
                                twist.angular.z = kp_orient * error_orientation;
                                robot_cmd_vel_publisher->publish(twist);
                            }
                            else{
                                should_set_final_goal = true;
                            }
                        }
                        // else if (should_set_yaw_goal)
                        // {
                        //     if (error_yaw>0.05) {
                        //         RCLCPP_INFO(this->get_logger(),"Correcting stage");
                        //         twist.linear.x = 0.0; //kp_distance * error_distance; 
                        //         twist.angular.z = kp_yaw * error_yaw;
                        //         robot_cmd_vel_publisher->publish(twist);
                        //     }
                        //     else {
                        //         should_set_final_goal = true;
                        //     }
                        // }
                        else{
                            RCLCPP_DEBUG(this->get_logger(), "I have Entered pub ");
                            twist.linear.x = 0.10; //kp_distance * error_distance; 
                            twist.angular.z = kp_yaw * error_yaw;
                            robot_cmd_vel_publisher->publish(twist);
                        }


                    } catch (tf2::TransformException &ex) {
                        RCLCPP_INFO(this->get_logger(), "TF Exception: %s", ex.what());
                    }
                }
                else if (should_set_final_goal)
                {   
                    // final_goal=true;
                    
                    rclcpp::Time start_time = this->now();
                    std::chrono::seconds duration(13); // Move for 4 seconds
                    rclcpp::Rate small(100);
                    while (rclcpp::ok() && (this->now() - start_time) < duration) {
                        RCLCPP_DEBUG(this->get_logger(),"TIME: %f", (this->now() - start_time).seconds());
                        // Your control logic here
                        geometry_msgs::msg::Twist default_twist;
                        default_twist.linear.x = 0.10;  
                        default_twist.angular.z = 0.0;  
                        robot_cmd_vel_publisher->publish(default_twist);
                        small.sleep();
                    }

                    // Stop the robot after 3 seconds
                    geometry_msgs::msg::Twist stop_twist;
                    stop_twist.linear.x = 0.0;
                    stop_twist.angular.z = 0.0;
                    robot_cmd_vel_publisher->publish(stop_twist);
                    final_goal = true;
                }   
                loop_rate.sleep();
            }
        }

        double calculateDistanceError(const geometry_msgs::msg::TransformStamped &transform) {
            return sqrt(((transform.transform.translation.x)) * ((transform.transform.translation.x)) +
                        transform.transform.translation.y * transform.transform.translation.y);
        }

        double calculateYawError(const geometry_msgs::msg::TransformStamped &transform) {
            return atan2(transform.transform.translation.y, transform.transform.translation.x);
        }

        double calculateOrientationError(const geometry_msgs::msg::TransformStamped &transform) {
            // Extract yaw (Z-axis rotation) from the quaternion
            double target_yaw = tf2::getYaw(transform.transform.rotation);

            // Compute yaw error
            return target_yaw;
        }


        void approach_callback(const std::shared_ptr<attach_table::srv::GoToLoading::Request> req, const std::shared_ptr<attach_table::srv::GoToLoading::Response> res) 
        {
            bool attach_action = req->attach_to_table;
            this->table_number = req->table_number;
            if (attach_action)
            {
                // publishStaticTransform(this->x, this->y);
                // std::this_thread::sleep_for(std::chrono::seconds(5));
                // RCLCPP_INFO(this->get_logger(),"map_x: %d", map_x);
                // RCLCPP_INFO(this->get_logger(),"map_y: %d", map_y);
                // publishNewFrame(this->map_x, this->map_y);
                if (this->x!=0.0 && this->y!=0.0)
                {
                    RCLCPP_INFO(this->get_logger(),"Approaching table and picking it up.");
                    controlLoop();
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    auto message = std_msgs::msg::String();
                    message.data = "";
                    elevator_pub_->publish(message);
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    res->complete = true;
                }
                else 
                {
                    RCLCPP_INFO(this->get_logger(),"Only one or no leg detected.");
                    res->complete=false;
                }
            }
            else 
            {
                publishStaticTransform(x, y,table_number);
                res->complete = true;
            }
        }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FinalApproachService>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);

    rclcpp::shutdown();
}