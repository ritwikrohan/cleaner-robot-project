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
#include "find_table/srv/find_table.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <thread>
#include "slg_msgs/msg/centroids.hpp"
#include "slg_msgs/msg/segment_array.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>


using namespace std::chrono_literals;

class FindTableService : public rclcpp::Node
{
    public:
        FindTableService() : Node("find_table_server_node")
        {
            this->find_server_ = this->create_service<find_table::srv::FindTable>("/find_table", std::bind(&FindTableService::find_callback, this, std::placeholders::_1, std::placeholders::_2));
            centroid_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options_centroid;
            options_centroid.callback_group = centroid_callback_group_;
            centroid_sub_ = this->create_subscription<slg_msgs::msg::Centroids>("/segments/centroids", 10, std::bind(&FindTableService::centroidCallback, this, std::placeholders::_1), options_centroid);
            // elevator_pub_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 1);
            // robot_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 1);
            robot_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }




    private:
        rclcpp::Service<find_table::srv::FindTable>::SharedPtr find_server_;
        // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;
        rclcpp::Subscription<slg_msgs::msg::Centroids>::SharedPtr centroid_sub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        rclcpp::CallbackGroup::SharedPtr centroid_callback_group_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_cmd_vel_publisher;
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
        double distance_between_clusters = 0.0;

        
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
            this->distance_between_clusters =  sqrt(((this->centroid0_x - this->centroid1_x) * (this->centroid0_x - this->centroid1_x)) + ((this->centroid0_y - this->centroid1_y) * (this->centroid0_y - this->centroid1_y)));
        }


        double computeDistanceToRobotBase(const geometry_msgs::msg::Point& point) {
            // Replace this with your logic to compute the distance to the robot base
            // For example, you might use the Euclidean distance.
            return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        }

        void publishTransform(const double& Ax, const double& Ay, const double& Bx, const double& By) {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.frame_id = "turtlebot_5_laser_link"; // Replace with your frame_id
            transformStamped.child_frame_id = "pre_loading_frame"; // Replace with your child_frame_id
            transformStamped.header.stamp = this->now();

            // Compute the center point C
            geometry_msgs::msg::Point pointC;
            pointC.x = (Ax + Bx) / 2.0;
            pointC.y = (Ay + By) / 2.0;
            pointC.z = 0.0;

            // Compute the vector AB
            geometry_msgs::msg::Vector3 vectorAB;
            vectorAB.x = Bx - Ax;
            vectorAB.y = By - Ay;
            vectorAB.z = 0.0;

            // Normalize the vector AB
            double lengthAB = sqrt(pow(vectorAB.x, 2) + pow(vectorAB.y, 2) + pow(vectorAB.z, 2));
            vectorAB.x /= lengthAB;
            vectorAB.y /= lengthAB;
            vectorAB.z /= lengthAB;

            // Compute the cross product of AB and the up vector (0, 0, 1) to get a perpendicular vector
            geometry_msgs::msg::Vector3 vectorPerpendicular;
            vectorPerpendicular.x = -vectorAB.y;  // Cross product with (0, 0, 1)
            vectorPerpendicular.y = vectorAB.x;
            vectorPerpendicular.z = 0.0;

            // Normalize the perpendicular vector
            double lengthPerpendicular = sqrt(pow(vectorPerpendicular.x, 2) + pow(vectorPerpendicular.y, 2));
            vectorPerpendicular.x /= lengthPerpendicular;
            vectorPerpendicular.y /= lengthPerpendicular;
            vectorPerpendicular.z /= lengthPerpendicular;

            // Check the direction with the dot product
            double dotProduct = vectorPerpendicular.x * vectorAB.x + vectorPerpendicular.y * vectorAB.y;
            if (dotProduct < 0) {
                // Reverse the direction of the perpendicular vector
                vectorPerpendicular.x = -vectorPerpendicular.x;
                vectorPerpendicular.y = -vectorPerpendicular.y;
            }

            // Compute the two possible points D1 and D2
            geometry_msgs::msg::Point pointD1, pointD2;
            pointD1.x = pointC.x + vectorPerpendicular.x;
            pointD1.y = pointC.y + vectorPerpendicular.y;
            pointD1.z = pointC.z + vectorPerpendicular.z;

            pointD2.x = pointC.x - vectorPerpendicular.x;
            pointD2.y = pointC.y - vectorPerpendicular.y;
            pointD2.z = pointC.z - vectorPerpendicular.z;

            // Compute the distances from the robot base link to D1 and D2
            double distanceD1 = computeDistanceToRobotBase(pointD1);
            double distanceD2 = computeDistanceToRobotBase(pointD2);

            // Choose the point with the minimum distance
            geometry_msgs::msg::Point chosenPoint;
            if (distanceD1 < distanceD2) {
                chosenPoint = pointD1;
            } else {
                chosenPoint = pointD2;
            }

            // Compute the cross product of vectorPerpendicular and vectorAB to get a vector pointing along the z-axis
            geometry_msgs::msg::Vector3 vectorZ;
            vectorZ.x = vectorPerpendicular.y * vectorAB.z - vectorPerpendicular.z * vectorAB.y;
            vectorZ.y = vectorPerpendicular.z * vectorAB.x - vectorPerpendicular.x * vectorAB.z;
            vectorZ.z = vectorPerpendicular.x * vectorAB.y - vectorPerpendicular.y * vectorAB.x;

            // Normalize the z-axis vector
            double lengthZ = sqrt(pow(vectorZ.x, 2) + pow(vectorZ.y, 2) + pow(vectorZ.z, 2));
            vectorZ.x /= lengthZ;
            vectorZ.y /= lengthZ;
            vectorZ.z /= lengthZ;

            // Compute the transformation matrix using vectors AC, AD, and the cross product vectorZ
            tf2::Matrix3x3 rotationMatrix(vectorPerpendicular.x, vectorAB.x, vectorZ.x,
                                        vectorPerpendicular.y, vectorAB.y, vectorZ.y,
                                        vectorPerpendicular.z, vectorAB.z, vectorZ.z);

            // Set the translation and rotation components of the transformation
            transformStamped.transform.translation.x = chosenPoint.x;
            transformStamped.transform.translation.y = chosenPoint.y;
            transformStamped.transform.translation.z = chosenPoint.z;

            tf2::Quaternion rotationQuaternion;
            rotationMatrix.getRotation(rotationQuaternion);
            transformStamped.transform.rotation = tf2::toMsg(rotationQuaternion);

            // Publish the transformation
            tf_static_broadcaster_->sendTransform(transformStamped);
        }

        // // function to create a static tf based on the arguement given which will be the mid point of centroids
        // void publishStaticTransform(double x, double y, int table_number) {
        //     // Create a transform message
        //     geometry_msgs::msg::TransformStamped broadcast_transformStamped;
        //     broadcast_transformStamped.header.stamp = this->now();
        //     broadcast_transformStamped.header.frame_id = "robot_front_laser_base_link";  // Set the parent frame ID
        //     broadcast_transformStamped.child_frame_id = "table_front_frame";  // Set the child frame ID


            
        //     // Set the translation (x, y, z)
        //     RCLCPP_DEBUG(this->get_logger(),"base centroid x: %f",x);
        //     RCLCPP_DEBUG(this->get_logger(),"base centroid y: %f",y);
        //     broadcast_transformStamped.transform.translation.x = x;
        //     broadcast_transformStamped.transform.translation.y = y;
        //     broadcast_transformStamped.transform.translation.z = 0.0;  // Z is usually 0 for 2D transforms

        //     // Set the rotation (no rotation in a static transform)
        //     broadcast_transformStamped.transform.rotation.x = 0.0;
        //     broadcast_transformStamped.transform.rotation.y = 0.0;
        //     broadcast_transformStamped.transform.rotation.z = 0.0;
        //     broadcast_transformStamped.transform.rotation.w = 1.0;

        //     // Publish the static transform
        //     tf_static_broadcaster_->sendTransform(broadcast_transformStamped);
        //     RCLCPP_DEBUG(this->get_logger(), "PUBLISHED");


        //      // Wait for TF2 buffer to be ready
        //     tf2::TimePoint time_point = tf2::TimePoint(std::chrono::seconds(0));  // Use zero time for the latest available transform

        //     while (rclcpp::ok()) {
        //         if (tf_buffer_->canTransform("map", "table_front_frame", time_point)) {
        //             // Transform the coordinates from "table_front_frame" to "map"
        //             geometry_msgs::msg::TransformStamped map_transform;
        //             try {
        //                 map_transform = tf_buffer_->lookupTransform("map", "table_front_frame", time_point);
        //             } catch (tf2::TransformException& ex) {
        //                 RCLCPP_INFO(this->get_logger(), "Failed to transform coordinates: %s", ex.what());
        //                 continue;  // Retry if the transformation fails
        //             }

        //             this->map_x = map_transform.transform.translation.x;
        //             this->map_y = map_transform.transform.translation.y;
                    
        //             // Exit the loop once map_x and map_y have been found
        //             break;
        //         } else {
        //             RCLCPP_INFO(this->get_logger(), "Transform not available. Waiting for transform...");
        //         }

        //         // Sleep for a short duration to avoid busy waiting
        //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     }
        //     RCLCPP_DEBUG(this->get_logger(),"map_x: %f", this->map_x);
        //     RCLCPP_DEBUG(this->get_logger(),"map_y: %f", this->map_y);
        //     // publishNewFrame(1.000980, 0.785102);
        //     publishNewFrame(this->map_x, this->map_y, table_number);
        // }

        // void publishNewFrame(double map_x, double map_y, int table_number) {
        //     // Create a transform message for the new frame
        //     geometry_msgs::msg::TransformStamped new_frame_transformStamped;
        //     new_frame_transformStamped.header.stamp = this->now();
        //     new_frame_transformStamped.header.frame_id = "map";  // Set the parent frame ID
        //     new_frame_transformStamped.child_frame_id = "new_frame";  // Set the child frame ID

        //     // Set the translation (x, y, z)
        //     new_frame_transformStamped.transform.translation.x = map_x;
        //     new_frame_transformStamped.transform.translation.y = map_y;
        //     new_frame_transformStamped.transform.translation.z = 0.0;  // Z is usually 0 for 2D transforms

        //     // Set the rotation based on table_number
        //     double angle = M_PI / 2.0;  // 90 degrees in radians
        //     double sin_half_angle = sin(angle / 2.0);
        //     double cos_half_angle = cos(angle / 2.0);

        //     if (table_number == 1) {
        //         // Set the rotation for table_number 1
        //         new_frame_transformStamped.transform.rotation.x = 0.0;
        //         new_frame_transformStamped.transform.rotation.y = 0.0;
        //         new_frame_transformStamped.transform.rotation.z = 0.0;
        //         new_frame_transformStamped.transform.rotation.w = 1.0;
        //     } else if (table_number == 2) {
        //         // Set the rotation for table_number 2
        //         new_frame_transformStamped.transform.rotation.x = 0.0;
        //         new_frame_transformStamped.transform.rotation.y = 0.0;
        //         new_frame_transformStamped.transform.rotation.z = sin_half_angle;
        //         new_frame_transformStamped.transform.rotation.w = cos_half_angle;
        //     } else {
        //         // Handle invalid table_number or add more cases if needed
        //         RCLCPP_ERROR(this->get_logger(), "Invalid table_number: %d", table_number);
        //         return;
        //     }

        //     // Publish the new frame transform
        //     tf_static_broadcaster_->sendTransform(new_frame_transformStamped);
        //     RCLCPP_DEBUG(this->get_logger(), "Published new frame");
        // }


        void find_callback(const std::shared_ptr<find_table::srv::FindTable::Request> req, const std::shared_ptr<find_table::srv::FindTable::Response> res) 
        {
            bool find_action = req->look_for_table;
            // this->table_number = req->table_number;
            if (find_action)
            {
                // publishStaticTransform(this->x, this->y);
                // std::this_thread::sleep_for(std::chrono::seconds(5));
                // RCLCPP_INFO(this->get_logger(),"map_x: %d", map_x);
                // RCLCPP_INFO(this->get_logger(),"map_y: %d", map_y);
                // publishNewFrame(this->map_x, this->map_y);
                if (this->x!=0.0 && this->y!=0.0 && this->distance_between_clusters < 1.0)
                {
                    RCLCPP_INFO(this->get_logger(),"Table Found");
                    // controlLoop();
                    // std::this_thread::sleep_for(std::chrono::seconds(5));
                    // auto message = std_msgs::msg::String();
                    // message.data = "";
                    // elevator_pub_->publish(message);
                    // std::this_thread::sleep_for(std::chrono::seconds(5));
                    publishTransform(this->centroid0_x, this->centroid0_y, this->centroid1_x, this->centroid1_y);
                    res->found = true;
                }
                else 
                {
                    RCLCPP_INFO(this->get_logger(),"No Table Found");
                    res->found=false;
                }
            }
            else 
            {
                // publishStaticTransform(x, y,table_number);
                res->found = true;
            }
        }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FindTableService>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);

    rclcpp::shutdown();
}















        // // Simple P controller to follow the newly created tf between the legs
        // void controlLoop() 
        // {   
        //     bool final_goal = false;
        //     bool should_set_final_goal = false;
        //     // bool should_set_yaw_goal = false;
        //     // bool yaw_after_stopping =false;
        //     double maxAllowedDifference = 1.0;  // Adjust this threshold based on your requirement
        //     double lastPublishedX = this->x;
        //     double lastPublishedY = this->y;
        //     rclcpp::Rate loop_rate(100);
        //     while(!final_goal)
        //     {   
        //         // Check if the difference is not significant compared to the last published values
        //         if (std::abs(this->x - lastPublishedX) <= maxAllowedDifference && std::abs(this->y - lastPublishedY) <= maxAllowedDifference) {
        //             publishStaticTransform(this->x, this->y, this->table_number);
        //             // Update last published positions immediately after publishing
        //             lastPublishedX = this->x;
        //             lastPublishedY = this->y;
        //         }
        //         // publishStaticTransform(this->x, this->y);
        //         // final_goal = true;
        //         if ((tf_buffer_->canTransform("robot_base_link", "new_frame", tf2::TimePoint(), tf2::durationFromSec(0.5)))&& (!std::isinf(this->x)|| !std::isinf(this->y)) && !should_set_final_goal)
        //         {   
        //             RCLCPP_DEBUG(this->get_logger(), "I have Entered if ");
        //             try {
        //                 RCLCPP_DEBUG(this->get_logger(), "I have Entered try ");
        //                 geometry_msgs::msg::TransformStamped transform;
        //                 transform = tf_buffer_->lookupTransform("robot_base_link", "new_frame", tf2::TimePoint(), tf2::durationFromSec(1.0));

        //                 double error_distance = calculateDistanceError(transform);
        //                 double error_yaw = calculateYawError(transform);
        //                 double error_orientation = calculateOrientationError(transform);
        //                 RCLCPP_DEBUG(this->get_logger(),"error_d : %f", error_distance);
        //                 RCLCPP_DEBUG(this->get_logger(),"error_y : %f", error_yaw);
        //                 RCLCPP_DEBUG(this->get_logger(),"error_orient : %f", error_orientation);
        //                 geometry_msgs::msg::Twist twist;
        //                 if (error_distance < 0.5 && std::abs(error_yaw < 0.05)){
        //                 //  if (error_yaw < 0.05){
        //                     RCLCPP_DEBUG(this->get_logger(),"Early stage");
        //                     if (std::abs(error_orientation)>0.01)
        //                     {   
        //                         // RCLCPP_INFO(this->get_logger(),"error_orient : %f", error_orientation);
        //                         RCLCPP_DEBUG(this->get_logger(),"error_orient : %f", error_orientation);
        //                         twist.linear.x = 0.0; 
        //                         twist.angular.z = kp_orient * error_orientation;
        //                         robot_cmd_vel_publisher->publish(twist);
        //                     }
        //                     else{
        //                         should_set_final_goal = true;
        //                     }
        //                 }
        //                 // else if (should_set_yaw_goal)
        //                 // {
        //                 //     if (error_yaw>0.05) {
        //                 //         RCLCPP_INFO(this->get_logger(),"Correcting stage");
        //                 //         twist.linear.x = 0.0; //kp_distance * error_distance; 
        //                 //         twist.angular.z = kp_yaw * error_yaw;
        //                 //         robot_cmd_vel_publisher->publish(twist);
        //                 //     }
        //                 //     else {
        //                 //         should_set_final_goal = true;
        //                 //     }
        //                 // }
        //                 else{
        //                     RCLCPP_DEBUG(this->get_logger(), "I have Entered pub ");
        //                     twist.linear.x = 0.10; //kp_distance * error_distance; 
        //                     twist.angular.z = kp_yaw * error_yaw;
        //                     robot_cmd_vel_publisher->publish(twist);
        //                 }


        //             } catch (tf2::TransformException &ex) {
        //                 RCLCPP_INFO(this->get_logger(), "TF Exception: %s", ex.what());
        //             }
        //         }
        //         else if (should_set_final_goal)
        //         {   
        //             // final_goal=true;
                    
        //             rclcpp::Time start_time = this->now();
        //             std::chrono::seconds duration(13); // Move for 4 seconds
        //             rclcpp::Rate small(100);
        //             while (rclcpp::ok() && (this->now() - start_time) < duration) {
        //                 RCLCPP_DEBUG(this->get_logger(),"TIME: %f", (this->now() - start_time).seconds());
        //                 // Your control logic here
        //                 geometry_msgs::msg::Twist default_twist;
        //                 default_twist.linear.x = 0.10;  
        //                 default_twist.angular.z = 0.0;  
        //                 robot_cmd_vel_publisher->publish(default_twist);
        //                 small.sleep();
        //             }

        //             // Stop the robot after 3 seconds
        //             geometry_msgs::msg::Twist stop_twist;
        //             stop_twist.linear.x = 0.0;
        //             stop_twist.angular.z = 0.0;
        //             robot_cmd_vel_publisher->publish(stop_twist);
        //             final_goal = true;
        //         }   
        //         loop_rate.sleep();
        //     }
        // }

        // double calculateDistanceError(const geometry_msgs::msg::TransformStamped &transform) {
        //     return sqrt(((transform.transform.translation.x)) * ((transform.transform.translation.x)) +
        //                 transform.transform.translation.y * transform.transform.translation.y);
        // }

        // double calculateYawError(const geometry_msgs::msg::TransformStamped &transform) {
        //     return atan2(transform.transform.translation.y, transform.transform.translation.x);
        // }

        // double calculateOrientationError(const geometry_msgs::msg::TransformStamped &transform) {
        //     // Extract yaw (Z-axis rotation) from the quaternion
        //     double target_yaw = tf2::getYaw(transform.transform.rotation);

        //     // Compute yaw error
        //     return target_yaw;
        // }

