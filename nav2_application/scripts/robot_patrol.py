import rclpy
from rclpy.node import Node
from attach_table.srv import GoToLoading
from geometry_msgs.msg import Twist
import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Polygon, Point32, TransformStamped
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from math import cos, sin, pi
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Empty
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs 
import math
import numpy as np

class RobotStateMachine(Node):


    def __init__(self):
        super().__init__('statemachine_node')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.client = self.create_client(GoToLoading, "/approach_table", callback_group=client_cb_group)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GoToLoading.Request()
        attach_link_cb_group = MutuallyExclusiveCallbackGroup()
        detach_link_cb_group = MutuallyExclusiveCallbackGroup()
        self.attach_link_client = self.create_client(
            AttachLink, '/ATTACHLINK', callback_group=attach_link_cb_group
        )
        while not self.attach_link_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req1 = AttachLink.Request()
        self.detach_link_client = self.create_client(
            DetachLink, '/DETACHLINK', callback_group=detach_link_cb_group
        )
        while not self.detach_link_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req2 = DetachLink.Request()
        # self.publisher_ = self.create_publisher(Twist, "/diffbot_base_controller/cmd_vel_unstamped", 10)
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.local_table_footprint_publisher = self.create_publisher(Polygon,"/local_costmap/footprint", 10)
        self.global_table_footprint_publisher = self.create_publisher(Polygon,"/global_costmap/footprint", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pub_table_down = self.create_publisher(String,'/elevator_down',10)
        self.req_to_attach = True
        self.nav = BasicNavigator()
        self.speed_msg = Twist() 
        self.initial_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.footprint = Polygon()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.robot_stage = {
            "initial_stage": [0.0, 0.0, 0.0 , 1.0],
            "loading_stage_1": [4.07681, -1.51506, 0.0, 1.0],
            "loading_stage_2": [1.2, -0.5, 0.707,0.707],
            "door_stage": [5.5, -0.55, 0.0, 1.0],
            "last_stage_1": [10.0, -0.55, 0.0, 1.0],
            "last_stage_2": [10.0, -2.55, 0.0, 1.0],
            "back_to_initial": [-0.190,0.142 ,-0.1246747,0.9921977]}
        self.stage_number = 1
        self.goal_reached = False
        # Frame:map, Position(1.02507, -0.261939, 0), Orientation(0, 0, 0.708792, 0.705417) = Angle: 1.57557

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # this function publishes a new footprint after the robot picks the table up according to table size.
    def publish_footprint_table(self):

        footprint = Polygon()
        point1 = Point32()
        point1.x = 0.35
        point1.y = 0.35

        point2 = Point32()
        point2.x = 0.35
        point2.y = -0.35

        point3 = Point32()
        point3.x = -0.35
        point3.y = -0.35

        point4 = Point32()
        point4.x = -0.35
        point4.y = 0.35

        footprint.points = [point1, point2, point3, point4]
            
        self.local_table_footprint_publisher.publish(footprint)
        self.global_table_footprint_publisher.publish(footprint)


    #this function publishes a circle shape as footprint when the robot drops the table and gets out of it
    def publish_footprint_robot(self):
        footprint = Polygon()
        points = []
        for angle in range(0, 360, 10):
            point = Point32()
            point.x = 0.25 * cos(angle * pi / 180)  
            point.y = 0.25 * sin(angle * pi / 180)  
            points.append(point)

            footprint.points = points
            
            self.local_table_footprint_publisher.publish(footprint)
            self.global_table_footprint_publisher.publish(footprint)

    # elevator down function
    def down_table(self):
        print('Down table')
        msg_pub = String()
        msg_pub.data = ""
        self.pub_table_down.publish(msg_pub)
        
        duration = Duration(seconds=5)
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            rate.sleep

    # getting out of table after dropping it
    def exit_under_the_table(self):
        msg_vel = Twist()
        msg_vel.linear.x = 0.1
        duration = Duration(seconds=7)
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg = Twist()
            msg.linear.x = 0.1
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            # print('moving forward')
            rate.sleep
        # print('stop')
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    # attach table service request
    def send_request(self, a, table_number):
        self.req.attach_to_table = a
        self.req.table_number = table_number
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    def call_attach_link_service_1(self):
        print("call_attach_link_service called")
        
        self.req1 = AttachLink.Request()
        self.req1.model1_name = 'rb1_robot'
        self.req1.link1_name = 'robot_evelator_platform_link'
        # self.req1.link1_name = 'robot_base_footprint'
        self.req1.model2_name = 'rubish_table_1'
        self.req1.link2_name = 'rubish_table_1::rubish_table_1::link'

        self.future1 = self.attach_link_client.call_async(self.req1)
        rclpy.spin_until_future_complete(self, self.future1)
        if self.future1.result() is not None:
            self.get_logger().info('Service call completed.')
        else:
            self.get_logger().info('Service call failed.')
        return self.future1.result()

    def call_detach_link_service_1(self):
        self.req2 = DetachLink.Request()
        self.req2.model1_name = 'rb1_robot'
        self.req2.link1_name = 'robot_evelator_platform_link'
        # self.req2.link1_name = 'robot_base_footprint'
        self.req2.model2_name = 'rubish_table_1'
        self.req2.link2_name = 'rubish_table_1::rubish_table_1::link'
        self.future2 = self.detach_link_client.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future2)
        if self.future2.result() is not None:
            self.get_logger().info('Service call completed.')
        else:
            self.get_logger().info('Service call failed.')
        return self.future2.result()

    def call_attach_link_service_2(self):
        print("call_attach_link_service called")
        
        self.req1 = AttachLink.Request()
        self.req1.model1_name = 'rb1_robot'
        self.req1.link1_name = 'robot_evelator_platform_link'
        # self.req1.link1_name = 'robot_base_footprint'
        self.req1.model2_name = 'rubish_table_2'
        self.req1.link2_name = 'rubish_table_2::rubish_table_2::link'

        self.future1 = self.attach_link_client.call_async(self.req1)
        rclpy.spin_until_future_complete(self, self.future1)
        if self.future1.result() is not None:
            self.get_logger().info('Service call completed.')
        else:
            self.get_logger().info('Service call failed.')
        return self.future1.result()

    def call_detach_link_service_2(self):
        self.req2 = DetachLink.Request()
        self.req2.model1_name = 'rb1_robot'
        self.req2.link1_name = 'robot_evelator_platform_link'
        # self.req2.link1_name = 'robot_base_footprint'
        self.req2.model2_name = 'rubish_table_2'
        self.req2.link2_name = 'rubish_table_2::rubish_table_2::link'
        self.future2 = self.detach_link_client.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future2)
        if self.future2.result() is not None:
            self.get_logger().info('Service call completed.')
        else:
            self.get_logger().info('Service call failed.')
        return self.future2.result()

    #setting initial position of robot for localisation.
    def set_init_pose(self):
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = self.robot_stage["initial_stage"][0]
        self.initial_pose.pose.position.y = self.robot_stage["initial_stage"][1]
        self.initial_pose.pose.orientation.z = self.robot_stage["initial_stage"][2]
        self.initial_pose.pose.orientation.w = self.robot_stage["initial_stage"][3]
        self.nav.setInitialPose(self.initial_pose)
        print('Initial Position Set')
        # Wait for navigation to activate fully
        self.nav.waitUntilNav2Active()

    # Goes to the position of the given stage
    def go_to_pose(self, stages, stage_name):
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = stages[stage_name][0]
        self.goal_pose.pose.position.y = stages[stage_name][1]
        self.goal_pose.pose.orientation.z = stages[stage_name][2]
        self.goal_pose.pose.orientation.w = stages[stage_name][3]
        self.nav.goToPose(self.goal_pose)

        # Do something during your route
        # (e.x. queue up future tasks or detect person for fine-tuned positioning)
        # Print information for workers on the robot's ETA for the demonstration
        i = 0
        while not self.nav.isTaskComplete():
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 10 == 0:
                print('Time left to reach ' + stage_name + ' : ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
        
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Reached (' + stage_name + ')...')


        elif result == TaskResult.CANCELED:
            print('Task at ' + stage_name  +
                ' was canceled. Returning to staging point...')
            exit(-1)

        elif result == TaskResult.FAILED:
            print('Task at ' + stage_name + ' failed!')
            exit(-1)

        while not self.nav.isTaskComplete():
            pass
    

    def with_table_backup_1(self, turn=True):
        print("in with_table_backup")
        msg = Twist()
        duration = Duration(seconds=15) # setting the time decides the backup distance
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg.linear.x = -0.25
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            # print('moving backward with table')
            rate.sleep
        print('stop')
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        if turn:
            #turn the table to thte direction of back room
            duration = Duration(seconds=2,nanoseconds=670000000)
            start_time = self.get_clock().now()
            while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = -1.0
                self.publisher_.publish(msg)
                print('turn with table')
                rate.sleep
        print('stop')
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
    

    def with_table_backup_2(self, turn=True):
        print("in with_table_backup")
        msg = Twist()
        duration = Duration(seconds=15) # setting the time decides the backup distance
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg.linear.x = -0.25
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            # print('moving backward with table')
            rate.sleep
        print('stop')
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        if turn:
            #turn the table to thte direction of back room
            duration = Duration(seconds=2,nanoseconds=670000000)
            start_time = self.get_clock().now()
            while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = -1.0
                self.publisher_.publish(msg)
                print('turn with table')
                rate.sleep
        print('stop')
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def table_forward(self):
        msg = Twist()
        duration = Duration(seconds=15) # setting the time decides the backup distance
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg.linear.x = 0.25
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            rate.sleep

    def table_backward(self):
        print("Backing up")
        msg = Twist()
        duration = Duration(seconds=7) # setting the time decides the backup distance
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg.linear.x = -0.25
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            rate.sleep
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def timer_callback(self):
        self.timer.cancel()
        self.control_loop()

    def door_transform(self, x, y):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "door_frame"
        static_transformStamped.transform.translation.x = x
        static_transformStamped.transform.translation.y = y
        static_transformStamped.transform.translation.z = 0.0
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0 
        static_transformStamped.transform.rotation.z = 0.0  
        static_transformStamped.transform.rotation.w = 1.0  

        self.static_tf_broadcaster.sendTransform(static_transformStamped)

    def door_controller(self, gain_p=0.1, gain_yaw=0.1):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                "robot_base_link", "door_frame", rclpy.time.Time()
            )
            self.get_logger().info(f'I HAVE ENTERED')

            distance_error = math.sqrt(
                transform_stamped.transform.translation.x ** 2 +
                transform_stamped.transform.translation.y ** 2
            )
            _, _, error_yaw = self.euler_from_quaternion(transform_stamped.transform.rotation)
            self.get_logger().info(f'Dist Error {distance_error}')
            self.get_logger().info(f'Yaw Error {error_yaw}')
            cmd_vel = Twist()
            while distance_error > 0.05 and error_yaw > 0.05:
                self.get_logger().info(f'Dist Error {distance_error}')
                self.get_logger().info(f'Yaw Error {error_yaw}')
                cmd_vel.linear.x = gain_p * distance_error
                cmd_vel.angular.z = gain_yaw * error_yaw
                self.publisher_.publish(cmd_vel)

        except Exception as e:
            self.get_logger().error(f"Error in P controller: {str(e)}")

    # Heart of the state machine which decides the flow of robot work
    def control_loop(self):
        # rate = self.node.create_rate(1)
        while rclpy.ok() and not self.goal_reached:
            # response = self.send_request(True)
            # if (response):
            #     self.goal_reached = True
            
            if self.stage_number == 1:
            #     self.set_init_pose()
                self.door_transform(5.5,-0.5489)
                self.stage_number = 2
                # self.goal_reached=True
            elif self.stage_number == 2:
                self.go_to_pose(self.robot_stage, "loading_stage_1")
                # self.goal_reached = True
                self.stage_number = 3
            elif self.stage_number==3:
                print("Loading stage reached and proceeding to attach the table slowly...")
                response = self.send_request(True,1)
                # print(response)
                self.publish_footprint_table()
                # self.goal_reached = True
                self.stage_number=4
            elif self.stage_number==4:
                print("table attached, Backing up slowly and Proceeding to final stage ...")
                response2 = self.call_attach_link_service_1()
                self.with_table_backup_1(turn=False)
                time.sleep(2)
                # self.go_to_pose(self.robot_stage, "new_stage")
                self.stage_number=5
                # self.goal_reached = True
            elif self.stage_number==5:
                print("Proceeding to final stage...")
                time.sleep(5)
                self.go_to_pose(self.robot_stage, "door_stage")
                self.stage_number=6
            elif self.stage_number==6:
                self.door_controller()
                print("going forward")
                # self.table_forward()
                self.stage_number=7
            elif self.stage_number==7:
                print("Stage 7")
                self.go_to_pose(self.robot_stage, "last_stage_1")
                self.down_table()
                time.sleep(2)
                self.call_detach_link_service_1()
                # self.call_detach_link_service()
                time.sleep(2)
                self.table_backward()
                self.publish_footprint_robot()
                self.stage_number=8
                # self.goal_reached = True  
            #############################################################################################################    
            elif self.stage_number == 8:
                self.go_to_pose(self.robot_stage, "loading_stage_2")
                # self.goal_reached = True
                self.stage_number = 9
            elif self.stage_number==9:
                print("Loading stage reached and proceeding to attach the table slowly...")
                response = self.send_request(True,2)
                # print(response)
                self.publish_footprint_table()
                # self.goal_reached = True
                self.stage_number=10
            elif self.stage_number==10:
                print("table attached, Backing up slowly and Proceeding to final stage ...")
                response2 = self.call_attach_link_service_2()
                self.with_table_backup_2(turn=False)
                time.sleep(2)
                # self.go_to_pose(self.robot_stage, "new_stage")
                self.stage_number=11
                # self.goal_reached = True
            elif self.stage_number==11:
                print("Proceeding to final stage...")
                time.sleep(5)
                self.go_to_pose(self.robot_stage, "door_stage")
                self.stage_number=12
                # self.goal_reached = True
            elif self.stage_number==12:
                self.door_controller()
                print("going forward")
                # self.table_forward()
                self.stage_number=13
                # self.goal_reached = True
            elif self.stage_number==13:
                print("Stage 8")
                self.go_to_pose(self.robot_stage, "last_stage_2")
                self.down_table()
                time.sleep(2)
                self.call_detach_link_service_2()
                # self.call_detach_link_service()
                time.sleep(2)
                self.table_backward()
                self.publish_footprint_robot()
                self.stage_number=14
                self.goal_reached = True            
            #     print("table detached and waiting for a few seconds...")
            #     self.down_table()
            #     self.publish_footprint_robot()
            #     print("Proceeding back to initial stage ...")
            #     self.exit_under_the_table()
            #     self.go_to_pose(self.robot_stage,"back_to_initial")
            #     print("Waiting for next pickup. Robot on standby...")
            #     self.goal_reached=True

            
def main(args=None):
    rclpy.init(args=args)
    state_machine = RobotStateMachine()
    executor = MultiThreadedExecutor()
    executor.add_node(state_machine)
    executor.spin()
    # rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




#############################################################################

