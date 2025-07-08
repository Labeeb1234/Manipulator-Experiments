#! /usr/bin/python3

import rclpy
from rclpy.node import Node
import rclpy.time
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
[ -0.036941178427802195,
 0.728729781680637,
 -0.029671078072653876,
 1.3933671476576064
]
'''

class DobotStatePublisher(Node):
    def __init__(self, port_id):
        super().__init__("dobot_state_publisher")
        # Dobot Construction
        self.dobot_mag = pydobot.Dobot(port=port_id, verbose=False)

        self.callback_group = ReentrantCallbackGroup()
        
        # self.custom_qos = QoSProfile(
        #     QoSReliabilityPolicy.RELIABLE,
        #     QoSHistoryPolicy.KEEP_LAST,
        #     10,
        #     QoSDurabilityPolicy.VOLATILE,
        #     QoSLivelinessPolicy.AUTOMATIC
        # ) # use and modify only if necessary

        self.joint_states_pub_ = self.create_publisher(JointState, "joint_states", qos_profile=10)
        self.timer_period = 0.1 #[s] 10 Hz
        self.create_timer(self.timer_period, callback=self.joint_states_publisher, callback_group=self.callback_group)
        self.dobot_lock = threading.Lock()

    def joint_states_publisher(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.name = ['magician_joint_1', 'magician_joint_2', 'magician_joint_3', 'magician_joint_4']
        with self.dobot_lock:
            x, y, z, r, j1, j2, j3, j4 = self.dobot_mag.pose()
        j1 = self.deg_to_rad(j1)
        j2 = self.deg_to_rad(j2)
        j3 = self.deg_to_rad(j3)
        j4 = self.deg_to_rad(j4)
        # sself.get_logger().info(f"[j1: {j1}, j2: {j2}, j3: {j3}, j4: {j4}]")
        msg.position = [j1, j2, j3, j4]
        
        self.joint_states_pub_.publish(msg)
    
    # Optional Not required
    def end_effector_state_publisher(self):
        msg = Pose()
        pass

    def deg_to_rad(self, deg_angle):
        rad_angle = (3.14*deg_angle)/180
        return rad_angle

    def move_bot(self):
        count = 0
        with self.dobot_lock:
            x, y, z, r, j1, j2, j3, j4 = self.dobot_mag.pose()            
            self.dobot_mag.move_to(236.71022033691406,-8.752776145935059, 105.04280853271484, r, wait=False)
            self.dobot_mag.move_to(x+10, y+10, z, r, wait=False)
            self.dobot_mag.move_to(x+20, y+20, z, r, wait=False)
        count += 0.01
        time.sleep(0.1)

        

def main(args=None):
    available_ports = list_ports.comports()
    required_port = available_ports[-1].device
    print(f"[INFO]: Dobot Serial Port ID: {required_port}")

    rclpy.init(args=args)
    node = DobotStatePublisher(port_id=required_port)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    move_bot_thread = threading.Thread(target=node.move_bot, daemon=True)
    move_bot_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt as e:
        print(f"Exception: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
            move_bot_thread.join()
            node.dobot_mag._set_queued_cmd_clear()


if __name__ == '__main__':
    main()


