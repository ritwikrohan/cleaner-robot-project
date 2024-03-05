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
// #include <vector>
#include <limits>
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
            centroid_sub_ = this->create_subscription<slg_msgs::msg::Centroids>("/segments/min", 10, std::bind(&FinalApproachService::centroidCallback, this, std::placeholders::_1), options_centroid);
            elevator_pub_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 1);
            // robot_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 1);
            robot_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/turtlebot_5/cmd_vel", 1);
            // robot_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cleaner_2/cmd_vel", 1);
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            centroid_history = std::vector<std::vector<double>>(3, std::vector<double>(ROLLING_WINDOW_SIZE, std::numeric_limits<double>::max()));
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
        double kp_yaw = 1.8;//0.7;
        double kp_orient = 1.0;//0.7;
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
        geometry_msgs::msg::Quaternion orientation_quaternion;
        // const int ROLLING_WINDOW_SIZE = 50;
        // std::vector<std::vector<double>> centroid_history(3, std::vector<double>(ROLLING_WINDOW_SIZE, std::numeric_limits<double>::max()));
        static const int ROLLING_WINDOW_SIZE = 50;
        // std::vector<std::vector<double>> centroid_history{3, std::vector<double>(ROLLING_WINDOW_SIZE, std::numeric_limits<double>::max())};
        std::vector<std::vector<double>> centroid_history;
        double min_sum_squares[3] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
        double selected_x[3] = {0.0, 0.0, 0.0};
        double selected_y[3] = {0.0, 0.0, 0.0};

        
        // Using centroid of the legs to calculate a middle point of two legs to create a static tf
        void centroidCallback(const std::shared_ptr<slg_msgs::msg::Centroids> msg)
        {   
            

            // for (size_t i = 0; i < msg->centroids.size(); ++i)
            // {
            //     const auto& current_centroid = msg->centroids[i];
            //     if (current_centroid.z == 0.0)
            //     {
            //         this->centroid0_x = current_centroid.x;
            //         this->centroid0_y = current_centroid.y;
            //     }
            //     else if (current_centroid.z == 1.0) 
            //     {
            //         this->centroid1_x = current_centroid.x;
            //         this->centroid1_y = current_centroid.y;
            //     }
            // }
            for (size_t i = 0; i < msg->centroids.size(); ++i) {
            const auto& current_centroid = msg->centroids[i];
            size_t id = static_cast<size_t>(current_centroid.z);

            // Update the centroid history for the corresponding ID
            double sum_squares = current_centroid.x * current_centroid.x + current_centroid.y * current_centroid.y;
            
            centroid_history[id].push_back(sum_squares);
            
            // Keep the history size constant
            if (centroid_history[id].size() > ROLLING_WINDOW_SIZE) {
                centroid_history[id].erase(centroid_history[id].begin());
            }

            // Find the minimum sum of squared values for each ID
            min_sum_squares[id] = *std::min_element(centroid_history[id].begin(), centroid_history[id].end());

            // Select the centroid with the minimum sum of squared values
            if (sum_squares == min_sum_squares[id]) {
                selected_x[id] = current_centroid.x;
                selected_y[id] = current_centroid.y;
            }
        }
            // Use the selected centroids for further processing
            this->centroid0_x = selected_x[0];
            this->centroid0_y = selected_y[0];
            // Do further processing with the selected centroid for ID 0
            // ...
            this->centroid1_x = selected_x[1];
            this->centroid1_y = selected_y[1];

            this->x = (this->centroid0_x + this->centroid1_x)/2.0;
            this->y = (this->centroid0_y + this->centroid1_y)/2.0;
        }

        // function to create a static tf based on the arguement given which will be the mid point of centroids
        void publishStaticTransform(double x, double y) {
            // Create a transform message
            geometry_msgs::msg::TransformStamped broadcast_transformStamped;
            broadcast_transformStamped.header.stamp = this->now();
            broadcast_transformStamped.header.frame_id = "turtlebot_5_laser_link";  // Set the parent frame ID
            broadcast_transformStamped.child_frame_id = "table_front_frame";  // Set the child frame ID


            double direction_x = this->centroid1_x - this->centroid0_x;
            double direction_y = this->centroid1_y - this->centroid0_y;
            int sign_x = (direction_x >= 0) ? 1 : -1;
            int sign_y = (direction_y >= 0) ? 1 : -1;
            int sign_rotation = (sign_x * sign_y >= 0) ? 1 : -1;
            RCLCPP_DEBUG(this->get_logger(),"Sign X: %d", sign_x);
            RCLCPP_DEBUG(this->get_logger(),"Sign Y: %d", sign_y);
            RCLCPP_DEBUG(this->get_logger(),"Sign X * Y: %d", sign_rotation);

            // Calculate the angle of rotation
            double rotation_angle = atan2(direction_y, direction_x);

            // Create a Quaternion representing the desired rotation
            tf2::Quaternion rotation_quaternion;
            rotation_quaternion.setRPY(0.0, 0.0, rotation_angle);

            // Set the translation (x, y, z)
            broadcast_transformStamped.transform.translation.x = x;
            broadcast_transformStamped.transform.translation.y = y;
            broadcast_transformStamped.transform.translation.z = 0.0;  // Z is usually 0 for 2D transforms

            // We keep the z-axis upwards
            tf2::Quaternion rotation_correction;
            rotation_correction.setRPY(0.0, 0.0, M_PI / 2.0);  // Rotate 90 degrees around Z to align y-axis

            // Combine the rotation_correction with the rotation_quaternion
            tf2::Quaternion final_rotation = rotation_correction * rotation_quaternion;

            // Set the rotation using the final_rotation quaternion
            broadcast_transformStamped.transform.rotation.x = final_rotation.x();
            broadcast_transformStamped.transform.rotation.y = final_rotation.y();
            broadcast_transformStamped.transform.rotation.z = final_rotation.z();
            broadcast_transformStamped.transform.rotation.w = final_rotation.w();





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
                    // Directly use the rotation from map_transform for map_pre_loading_frame
                    this->orientation_quaternion = map_transform.transform.rotation;
                    // Exit the loop once map_x and map_y have been found
                    break;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Transform not available. Waiting for transform...");
                }

                // Sleep for a short duration to avoid busy waiting
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            RCLCPP_DEBUG(this->get_logger(),"map_x: %f", this->map_x);
            RCLCPP_DEBUG(this->get_logger(),"map_y: %f", this->map_y);
            // publishNewFrame(1.000980, 0.785102);
            publishNewFrame(this->map_x, this->map_y, this->orientation_quaternion);
        }

        void publishNewFrame(double map_x, double map_y, const geometry_msgs::msg::Quaternion& orientation) {
            // Create a transform message for the new frame
            geometry_msgs::msg::TransformStamped new_frame_transformStamped;
            new_frame_transformStamped.header.stamp = this->now();
            new_frame_transformStamped.header.frame_id = "map";  // Set the parent frame ID
            new_frame_transformStamped.child_frame_id = "new_frame";  // Set the child frame ID

            // Set the translation (x, y, z)
            new_frame_transformStamped.transform.translation.x = map_x;
            new_frame_transformStamped.transform.translation.y = map_y;
            new_frame_transformStamped.transform.translation.z = 0.0;  // Z is usually 0 for 2D transforms

            // Set the rotation for map_pre_loading_frame
            new_frame_transformStamped.transform.rotation = orientation;

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
            bool diststop = false;
            bool reached_table_front = false;
            bool reached_table = false;
            // bool should_set_yaw_goal = false;
            // bool yaw_after_stopping =false;
            double maxAllowedDifference = 0.3;  // Adjust this threshold based on your requirement
            double lastPublishedX = this->x;
            double lastPublishedY = this->y;
            rclcpp::Rate loop_rate(100);
            while(!final_goal)
            {   
                // Check if the difference is not significant compared to the last published values
                if (std::abs(this->x - lastPublishedX) <= maxAllowedDifference && std::abs(this->y - lastPublishedY) <= maxAllowedDifference && !diststop) {
                    publishStaticTransform(this->x, this->y);
                    // Update last published positions immediately after publishing
                    lastPublishedX = this->x;
                    lastPublishedY = this->y;
                }
                // publishStaticTransform(this->x, this->y);
                // final_goal = true;
                if ((tf_buffer_->canTransform("turtlebot_5_base_link", "new_frame", tf2::TimePoint(), tf2::durationFromSec(0.5)))&& (!std::isinf(this->x)|| !std::isinf(this->y)) && !should_set_final_goal)
                {   
                    RCLCPP_DEBUG(this->get_logger(), "I have Entered if ");
                    try {
                        RCLCPP_DEBUG(this->get_logger(), "I have Entered try ");
                        geometry_msgs::msg::TransformStamped transform;
                        transform = tf_buffer_->lookupTransform("turtlebot_5_base_link", "new_frame", tf2::TimePoint(), tf2::durationFromSec(1.0));

                        double error_distance = calculateDistanceError(transform);
                        double error_yaw = calculateYawError(transform);
                        double error_orientation = calculateOrientationError(transform);
                        RCLCPP_DEBUG(this->get_logger(),"error_d : %f", error_distance);
                        RCLCPP_DEBUG(this->get_logger(),"error_y : %f", error_yaw);
                        RCLCPP_DEBUG(this->get_logger(),"error_orient : %f", error_orientation);
                        geometry_msgs::msg::Twist twist;
                        if (error_distance < 0.5 && std::abs(error_yaw) < 0.05 && !diststop){
                        //  if (error_yaw < 0.05){
                        //     RCLCPP_DEBUG(this->get_logger(),"Early stage");
                        //     if (std::abs(error_orientation)>0.01)
                        //     {   
                        //         // RCLCPP_INFO(this->get_logger(),"error_orient : %f", error_orientation);
                        //         RCLCPP_DEBUG(this->get_logger(),"error_orient : %f", error_orientation);
                        //         twist.linear.x = 0.0; 
                        //         twist.angular.z = kp_orient * error_orientation;
                        //         robot_cmd_vel_publisher->publish(twist);
                        //     }
                        //     else{
                                diststop = true;
                                // should_set_final_goal = true;
                        //     }
                        // }
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
                        }
                        else if (error_distance < 0.2 && std::abs(error_yaw) < 0.1){
                            reached_table =true;
                            while (std::abs(error_orientation)>0.05)
                            {   

                                // Recalculate the transform inside the loop
                                try {
                                    geometry_msgs::msg::TransformStamped transform;
                                    transform = tf_buffer_->lookupTransform("turtlebot_5_base_link", "new_frame", tf2::TimePoint(), tf2::durationFromSec(1.0));

                                    // Update error_orientation based on the new transform
                                    error_orientation = calculateOrientationError(transform);

                                    // Your existing control logic here
                                    // RCLCPP_INFO(this->get_logger(),"error_orient : %f", error_orientation);
                                    RCLCPP_INFO(this->get_logger(), "CORRECTING ORIENTATION");
                                    RCLCPP_INFO(this->get_logger(),"error_orient : %f", error_orientation);
                                    twist.linear.x = 0.0; 
                                    // twist.angular.z = kp_orient * error_orientation;
                                    double desired_angular_velocity = kp_orient * error_orientation;

                                    // Limit the angular velocity to the minimum between desired value and 0.5
                                    // twist.angular.z = std::min(desired_angular_velocity, 0.5);
                                    twist.angular.z = std::min(desired_angular_velocity, 0.5 * (desired_angular_velocity / std::abs(desired_angular_velocity)));
                                    robot_cmd_vel_publisher->publish(twist);

                                } catch (tf2::TransformException &ex) {
                                    RCLCPP_INFO(this->get_logger(), "TF Exception: %s", ex.what());
                                    // Handle the exception if necessary
                                }
                                // Add a small delay to avoid busy waiting
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                                
                            }
                            // else {
                            twist.linear.x = 0.0;
                            twist.angular.z = 0.0;
                            robot_cmd_vel_publisher->publish(twist);
                            std::this_thread::sleep_for(std::chrono::seconds(5));
                            should_set_final_goal = true;
                            // }
                        }
                        else if (!reached_table){
                            RCLCPP_INFO(this->get_logger(), "GOING TO TABLE");
                            RCLCPP_INFO(this->get_logger(),"error_d : %f", error_distance);
                            RCLCPP_INFO(this->get_logger(),"error_y : %f", error_yaw);
                            twist.linear.x = 0.10; //kp_distance * error_distance; 
                            twist.angular.z = kp_yaw * error_yaw;
                            robot_cmd_vel_publisher->publish(twist);
                        }
                        else {
                            twist.linear.x = 0.0;
                            twist.angular.z = 0.0;
                            robot_cmd_vel_publisher->publish(twist);
                            should_set_final_goal = true;
                        }


                    } catch (tf2::TransformException &ex) {
                        RCLCPP_INFO(this->get_logger(), "TF Exception: %s", ex.what());
                    }
                }
                else if (should_set_final_goal)
                {   
                    // final_goal=true;
                    RCLCPP_INFO(this->get_logger(), "GOING FORWARD AFTER CORRECTION");
                    
                    rclcpp::Time start_time = this->now();
                    std::chrono::seconds duration(6); // Move for 4 seconds
                    rclcpp::Rate small(100);
                    while (rclcpp::ok() && (this->now() - start_time) < duration) {
                        RCLCPP_DEBUG(this->get_logger(),"TIME: %f", (this->now() - start_time).seconds());
                        // Your control logic here
                        geometry_msgs::msg::Twist default_twist;
                        default_twist.linear.x = 0.1;  
                        default_twist.angular.z = 0.0;  
                        robot_cmd_vel_publisher->publish(default_twist);
                        small.sleep();
                    }

                    // Stop the robot after 3 seconds
                    geometry_msgs::msg::Twist stop_twist;
                    stop_twist.linear.x = 0.0;
                    stop_twist.angular.z = 0.0;
                    robot_cmd_vel_publisher->publish(stop_twist);
                    // RCLCPP_INFO(this->get_logger(),"error_orient : %f", error_orientation);
                    RCLCPP_INFO(this->get_logger(), "Reached Underneath the Table");
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

        // double calculateOrientationError(const geometry_msgs::msg::TransformStamped &transform) {
        //     // Extract yaw (Z-axis rotation) from the quaternion
        //     double target_yaw = tf2::getYaw(transform.transform.rotation);

        //     // Compute yaw error
        //     return target_yaw;
        // }

        double calculateOrientationError(const geometry_msgs::msg::TransformStamped &transform) {
            // Extract yaw (Z-axis rotation) from the quaternion
            double target_yaw = tf2::getYaw(transform.transform.rotation);

            // Ensure the yaw error represents the shortest angular distance
            if (target_yaw > M_PI/2) {
                return (target_yaw - M_PI);
            } 
            else if (target_yaw < -M_PI/2)
            {
                return (target_yaw + M_PI);
            }
            else {
                return target_yaw;
            }

            // Return the corrected yaw error
            // return target_yaw;
        }



        void approach_callback(const std::shared_ptr<attach_table::srv::GoToLoading::Request> req, const std::shared_ptr<attach_table::srv::GoToLoading::Response> res) 
        {
            bool attach_action = req->attach_to_table;
            // this->table_number = req->table_number;
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
                    RCLCPP_INFO(this->get_logger(),"Elevator going up.");
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    // auto message = std_msgs::msg::String();
                    // message.data = "";
                    // elevator_pub_->publish(message);
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
                publishStaticTransform(x, y);
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