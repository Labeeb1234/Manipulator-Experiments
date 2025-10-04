#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
import time
import pydobot 
import threading
from serial.tools import list_ports


'''
home joint configuration:
[ 0.0,
 0.0,
 0.0,
 0.0
]



'''

class DobotStatePublisher(Node):
    def __init__(self, port_id):
        super().__init__("dobot_state_publisher")
        # Dobot Device Object Creattion
        self.dobot_mag = pydobot.Dobot(port=port_id, verbose=False)
        self.dobot_lock = threading.Lock() # declaring a threading lock for dobot data to prevent race condition just in case
        self.callback_group = ReentrantCallbackGroup() # callbackgroup for concurrency in callbacks if required
        
        
        # self.sensor_qos_profile = QoSProfile(
        #     QoSReliabilityPolicy.RELIABLE,
        #     QoSHistoryPolicy.KEEP_LAST,
        #     10,
        #     QoSDurabilityPolicy.VOLATILE,
        #     QoSLivelinessPolicy.AUTOMATIC
        # ) # use and modify only if necessary

        self.joint_states_pub_ = self.create_publisher(JointState, "joint_states", 10) # currently using the default QoS profile
        self.eef_states_pub_ = self.create_publisher(PoseStamped, "end_effector_pos", 10)
        self.timer_period = 0.1 #[s] 10 Hz
        self.create_timer(self.timer_period, callback=self.joint_states_publisher, callback_group=self.callback_group)
        self.create_timer(self.timer_period, callback=self.eef_states_publisher, callback_group=self.callback_group)

    def joint_states_publisher(self):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'world' # parent frame id to the joints/subsequent links either 'world' or 'base_link';
        joint_msg.name = ['magician_joint_1', 'magician_joint_2', 'magician_joint_3', 'magician_joint_4', 'magician_joint_5']
        with self.dobot_lock:
            _, _, _, r, j1, j2, j3, j4 = self.dobot_mag.pose()
        j1 = self.deg_to_rad(j1)
        j2 = self.deg_to_rad(j2)
        j3 = self.deg_to_rad(j3)
        j4 = self.deg_to_rad(j4)
        r = self.deg_to_rad(r)
        self.get_logger().info(f"[j1: {j1}, j2: {j2}, j3: {j3}, j4: {j4}, eef_angle: {r}]")
        joint_msg.position = [j1, j2, j3, j4, r]
        
        self.joint_states_pub_.publish(joint_msg)
    
    def eef_states_publisher(self):
        pos_msg = PoseStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = "world"
        with self.dobot_lock:
            x, y, z, r, _, _, _, _ = self.dobot_mag.pose()
        pos_msg.pose.position.x = self.mm_to_m(x)
        pos_msg.pose.position.y = self.mm_to_m(y)
        pos_msg.pose.position.z = self.mm_to_m(z)

        # self.get_logger().info(f"x, y, z: [{x}, {y}, {z}]")

        self.eef_states_pub_.publish(pos_msg)
    
    @staticmethod
    def deg_to_rad(deg_angle):
        return (3.14*deg_angle)/(180)
    
    @staticmethod
    def mm_to_m(val):
        return (val/1000)


def main(args=None):
    available_ports = list_ports.comports()
    required_port = available_ports[-1].device
    print(f"[INFO]: Dobot Serial Port ID: {required_port}")

    rclpy.init(args=args)
    node = DobotStatePublisher(port_id=required_port)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt as e:
        print(f"Exception: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()


