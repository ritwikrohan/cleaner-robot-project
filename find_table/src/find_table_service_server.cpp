#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
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
#include <deque>
#include <algorithm> // For std::any_of



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
            table_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options_table;
            options_table.callback_group = table_callback_group_;
            table_detector_sub_ = this->create_subscription<std_msgs::msg::Bool>("/detect", 10, std::bind(&FindTableService::tableCallback, this, std::placeholders::_1), options_table);
            // elevator_pub_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 1);
            // robot_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 1);
            robot_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            centroid_history = std::vector<std::vector<double>>(3, std::vector<double>(ROLLING_WINDOW_SIZE, std::numeric_limits<double>::max()));

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
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr table_detector_sub_;
        rclcpp::CallbackGroup::SharedPtr table_callback_group_;
        double x;
        double y;
        std::vector<float> range;
        std::vector<float> intensity;
        double centroid0_x = 0.0;
        double centroid0_y = 0.0;
        double centroid1_x = 0.0;
        double centroid1_y = 0.0;
        double centroid2_x = 0.0;
        double centroid2_y = 0.0;
        double id;
        double map_x = 0.0;
        double map_y = 0.0;
        bool table;
        int table_number = 0;
        double distance_between_clusters = 0.0;
        geometry_msgs::msg::Quaternion orientation_quaternion;
        // const int ROLLING_WINDOW_SIZE = 50;
        // std::vector<std::vector<double>> centroid_history(3, std::vector<double>(ROLLING_WINDOW_SIZE, std::numeric_limits<double>::max()));
        static const int ROLLING_WINDOW_SIZE = 100;
        // std::vector<std::vector<double>> centroid_history{3, std::vector<double>(ROLLING_WINDOW_SIZE, std::numeric_limits<double>::max())};
        std::vector<std::vector<double>> centroid_history;
        double min_sum_squares[3] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
        double selected_x[3] = {0.0, 0.0, 0.0};
        double selected_y[3] = {0.0, 0.0, 0.0};
        size_t count_;
        bool foundCentroid = false;

        // Circular buffer to store the last 10 messages
        std::deque<bool> recentMessages;

        // Function to update the circular buffer with the latest message
        void updateRecentMessages(bool message) {
            recentMessages.push_back(message);
            if (recentMessages.size() > 1000) {
                recentMessages.pop_front();
            }
        }

        // Function to check if at least one of the last 10 messages is true
        bool areLast10MessagesTrue() const {
            return std::any_of(recentMessages.begin(), recentMessages.end(), [](bool value) { return value; });
        }
        
        // Using centroid of the legs to calculate a middle point of two legs to create a static tf
        void tableCallback(const std::shared_ptr<std_msgs::msg::Bool> msg)
        {   
            this->table = msg->data;
        }


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
            //     else if (current_centroid.z == 2.0) 
            //     {
            //         this->centroid2_x = current_centroid.x;
            //         this->centroid2_y = current_centroid.y;
            //     }
            // }
            // this->x = (this->centroid0_x + this->centroid1_x)/2.0;
            // this->y = (this->centroid0_y + this->centroid1_y)/2.0;
            // this->distance_between_clusters =  sqrt(((this->centroid0_x - this->centroid1_x) * (this->centroid0_x - this->centroid1_x)) + ((this->centroid0_y - this->centroid1_y) * (this->centroid0_y - this->centroid1_y)));
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

            if ((distSq(selected_x[0], selected_x[1],selected_y[0],selected_y[1])) == (0.485*0.485))
            {   
                foundCentroid = true;
                // Use the selected centroids for further processing
                this->centroid0_x = selected_x[0];
                this->centroid0_y = selected_y[0];
                // Do further processing with the selected centroid for ID 0
                // ...

                this->centroid1_x = selected_x[1];
                this->centroid1_y = selected_y[1];
            }
            else {
                foundCentroid = false;
            }
        }

        double distSq(double x0, double x1, double y0, double y1)
        {
            double distsq =  (pow(x0 - x1, 2) + pow(y0 - y1, 2)); 
            if (distsq >= (0.48*0.48) && distsq <= (0.499*0.499))
            {
                return (0.485*0.485);
            }
            else if (distsq >= (0.48*0.48*2) && distsq <=(0.499*0.499*2)) {
                return (0.485*0.485*2);
            }
            else {
                return distsq;
            }
        }
        // void centroidCallback(const slg_msgs::msg::Centroids::SharedPtr msg)
        // {
        //     // Process centroids and update selected_x, selected_y
        //     for (size_t i = 0; i < msg->centroids.size(); ++i) {
        //         const auto& current_centroid = msg->centroids[i];
        //         size_t id = static_cast<size_t>(current_centroid.z);

        //         // Update the centroid history for the corresponding ID
        //         double sum_squares = current_centroid.x * current_centroid.x + current_centroid.y * current_centroid.y;

        //         centroid_history[id].push_back(sum_squares);

        //         // Keep the history size constant
        //         if (centroid_history[id].size() > ROLLING_WINDOW_SIZE) {
        //             centroid_history[id].erase(centroid_history[id].begin());
        //         }

        //         // Find the minimum sum of squared values for each ID
        //         min_sum_squares[id] = *std::min_element(centroid_history[id].begin(), centroid_history[id].end());

        //         // Select the centroid with the minimum sum of squared values
        //         if (sum_squares == min_sum_squares[id]) {
        //             selected_x[id] = current_centroid.x;
        //             selected_y[id] = current_centroid.y;
        //         }
        //     }

        //     // Increment the count
        //     count_++;

        //     // Check if we've received enough messages
        //     if (count_ >= ROLLING_WINDOW_SIZE) {
        //         processingFunction();
        //         // Reset the count after processing
        //         count_ = 0;
        //     }
        // }

        // void processingFunction()
        // {
        //     // Process the selected centroids
        //     this->centroid0_x = selected_x[0];
        //     this->centroid0_y = selected_y[0];

        //     // Do further processing with the selected centroid for ID 0
        //     // ...

        //     this->centroid1_x = selected_x[1];
        //     this->centroid1_y = selected_y[1];

        //     // Calculate distance or perform other processing here
        //     // double distance_between_clusters = sqrt(
        //     //     ((this->centroid0_x - this->centroid1_x) * (this->centroid0_x - this->centroid1_x)) +
        //     //     ((this->centroid0_y - this->centroid1_y) * (this->centroid0_y - this->centroid1_y))
        //     // );

        //     // Do further processing with the calculated distance
        //     // ...
        // }


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

            std::this_thread::sleep_for(std::chrono::seconds(1));

                 // Wait for TF2 buffer to be ready
            tf2::TimePoint time_point = tf2::TimePoint(std::chrono::seconds(0));  // Use zero time for the latest available transform

            while (rclcpp::ok()) {
                if (tf_buffer_->canTransform("map", "pre_loading_frame", time_point)) {
                    // Transform the coordinates from "table_front_frame" to "map"
                    geometry_msgs::msg::TransformStamped map_transform;
                    try {
                        map_transform = tf_buffer_->lookupTransform("map", "pre_loading_frame", time_point);
                    } catch (tf2::TransformException& ex) {
                        RCLCPP_INFO(this->get_logger(), "Failed to transform coordinates: %s", ex.what());
                        continue;  // Retry if the transformation fails
                    }

                    this->map_x = map_transform.transform.translation.x;
                    this->map_y = map_transform.transform.translation.y;
                    // this->orientation_quaternion = map_transform.transform.rotation;

                    // Create a quaternion representing a 180-degree rotation around the y-axis
                    // Create a quaternion representing a 180-degree rotation around the y-axis
                    // tf2::Quaternion rotation_quaternion(
                    //     map_transform.transform.rotation.x,
                    //     map_transform.transform.rotation.y,
                    //     map_transform.transform.rotation.z,
                    //     map_transform.transform.rotation.w
                    // );

                    // Create a quaternion representing a 180-degree rotation around the y-axis
                    // tf2::Quaternion rotation_quaternion;
                    // rotation_quaternion.setRPY(0.0, M_PI, 0.0);  // Rotate 180 degrees around the y-axis

                    // // Multiply the original quaternion by the rotation quaternion
                    // tf2::Quaternion rotated_quaternion = rotation_quaternion * tf2::Quaternion(
                    //     map_transform.transform.rotation.x,
                    //     map_transform.transform.rotation.y,
                    //     map_transform.transform.rotation.z,
                    //     map_transform.transform.rotation.w
                    // );

                    // // Assign the rotated quaternion to orientation_quaternion
                    // this->orientation_quaternion = tf2::toMsg(rotated_quaternion);




                    
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
            publishNewFrame(this->map_x, this->map_y, this->table_number);
        }

        

        void publishNewFrame(double map_x, double map_y, int table) {
            // Create a transform message for the new frame
            geometry_msgs::msg::TransformStamped new_frame_transformStamped;
            new_frame_transformStamped.header.stamp = this->now();
            new_frame_transformStamped.header.frame_id = "map";  // Set the parent frame ID
            new_frame_transformStamped.child_frame_id = "map_pre_loading_frame";  // Set the child frame ID

            // Set the translation (x, y, z)
            new_frame_transformStamped.transform.translation.x = map_x;
            new_frame_transformStamped.transform.translation.y = map_y;
            new_frame_transformStamped.transform.translation.z = 0.0;  // Z is usually 0 for 2D transforms
            // new_frame_transformStamped.transform.rotation = orientation;
            // // Set the rotation for table_number 1
            new_frame_transformStamped.transform.rotation.x = 0.0;
            new_frame_transformStamped.transform.rotation.y = 0.0;
            if (table == 1)
            {
                new_frame_transformStamped.transform.rotation.z = 0.0;
                new_frame_transformStamped.transform.rotation.w = 1.0;
            }
            else if (table ==2)
            {
                new_frame_transformStamped.transform.rotation.z = -0.707;
                new_frame_transformStamped.transform.rotation.w = 0.707;
            }
            else if (table ==3)
            {
                new_frame_transformStamped.transform.rotation.z = 0.707;
                new_frame_transformStamped.transform.rotation.w = 0.707;
            }
            

            // Publish the new frame transform
            tf_static_broadcaster_->sendTransform(new_frame_transformStamped);
            RCLCPP_DEBUG(this->get_logger(), "Published new frame");
        }


        // void find_callback(const std::shared_ptr<find_table::srv::FindTable::Request> req, const std::shared_ptr<find_table::srv::FindTable::Response> res) 
        // {
        //     bool find_action = req->look_for_table;
        //     // this->table_number = req->table_number;
        //     if (find_action)
        //     {
        //         // publishStaticTransform(this->x, this->y);
        //         // std::this_thread::sleep_for(std::chrono::seconds(5));
        //         // RCLCPP_INFO(this->get_logger(),"map_x: %d", map_x);
        //         // RCLCPP_INFO(this->get_logger(),"map_y: %d", map_y);
        //         // publishNewFrame(this->map_x, this->map_y);
        //         if (this->x!=0.0 && this->y!=0.0 && this->distance_between_clusters < 0.5)
        //         {
        //             RCLCPP_INFO(this->get_logger(),"Table Found");
        //             // controlLoop();
        //             // std::this_thread::sleep_for(std::chrono::seconds(5));
        //             // auto message = std_msgs::msg::String();
        //             // message.data = "";
        //             // elevator_pub_->publish(message);
        //             // std::this_thread::sleep_for(std::chrono::seconds(5));
        //             publishTransform(this->centroid0_x, this->centroid0_y, this->centroid1_x, this->centroid1_y);
        //             res->found = true;
        //         }
        //         else 
        //         {
        //             RCLCPP_INFO(this->get_logger(),"No Table Found");
        //             res->found=false;
        //         }
        //     }
        //     else 
        //     {
        //         // publishStaticTransform(x, y,table_number);
        //         res->found = true;
        //     }
        // }
        void find_callback(const std::shared_ptr<find_table::srv::FindTable::Request> req, const std::shared_ptr<find_table::srv::FindTable::Response> res) 
        {
            bool find_action = req->look_for_table;
            this->table_number = req->table_number;

            if (find_action)
            {
                // Assuming you have a boolean variable named "tableFound" representing the last received message
                bool tableFound = this->table;/* Obtain the value from the other node's message */

                // Update the circular buffer with the latest message
                updateRecentMessages(tableFound);

                // Check if at least one of the last 10 messages is true
                if (areLast10MessagesTrue() && foundCentroid) {
                    RCLCPP_INFO(this->get_logger(), "Table Found");
                    publishTransform(this->centroid0_x, this->centroid0_y, this->centroid1_x, this->centroid1_y);
                    res->found = true;
                } else {
                    RCLCPP_INFO(this->get_logger(), "No Table Found (All recent messages are false)");
                    res->found = false;
                }
            }
            else 
            {
                // Handle the case when find_action is false
                res->found = true;
            }
        }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    // auto node = std::make_shared<FindTableService>();
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node);
    // executor.spin();
    // // rclcpp::spin(node);

    rclcpp::executors::MultiThreadedExecutor executor;

    // Create a single instance of FindTableService
    auto node = std::make_shared<FindTableService>();
    executor.add_node(node);

    while (rclcpp::ok()) {
        executor.spin_some();  // Process any pending callbacks

        // If there are no more pending callbacks, break out of the loop
        if (executor.get_number_of_threads() == 0) {
            break;
        }
    }
    rclcpp::shutdown();
}







