import rclpy
from rclpy.node import Node
import rclpy.qos
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup

import time
import pydobot
from pydobot.enums import PTPMode
import threading
from serial.tools import list_ports
from typing import Union, Optional
from enum import Enum
import numpy as np


class DobotInterface(Node):
    def __init__(self, port_id, verbose: Optional[bool], ptpMode: Optional[Union[Enum, PTPMode.MOVJ_ANGLE]]):
        super().__init__("dobot_interface_node")
        # Dobot Device Object Creation
        try:
            self.dobot_mag = pydobot.Dobot(port=port_id, verbose=verbose)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
        self.dobot_lock = threading.Lock() # declaring a threading lock for dobot data to prevent race condition just in case
        
        # creating reentrant callback group object
        self.callback_group = ReentrantCallbackGroup() # callbackgroup for concurrency in callbacks if required
    
        # publishers
        self.joint_states_pub_ = self.create_publisher(JointState, "joint_states", 10) # currently using the default QoS profile
        self.eef_states_pub_ = self.create_publisher(PoseStamped, "end_effector_pos", 10)
        
        # timers
        self.timer_period = 0.1 #[s] 10 Hz
        self.create_timer(self.timer_period, callback=self.joint_states_publisher, callback_group=self.callback_group)
        self.create_timer(self.timer_period, callback=self.eef_states_publisher, callback_group=self.callback_group)

        # subscribers
        self.get_logger().info("-------------- Joint Command Mode Active --------------\n")
        self.joint_command_sub_ = self.create_subscription(
            JointState, 
            "joint_command", 
            callback=self.joint_commander, 
            qos_profile=rclpy.qos.qos_profile_system_default, 
            callback_group=self.callback_group
        )
        self.get_logger().info("-------------- EEF Command Mode Active --------------\n")
        self.eef_command_sub_ = self.create_subscription(
            PoseStamped,
            "eff_pos_command",
            callback=self.eef_commander,
            qos_profile=rclpy.qos.qos_profile_system_default,
            callback_group=self.callback_group
        )

    # --------------------------- COMMANDER CALLBACKS ---------------------------------------------------------
    def joint_commander(self, cmd):
        cmd_time_stamp = cmd.header.stamp
        [j1, j2, j3, j4, r] = cmd.position # [j1, j2, j3, j4, r]
        self.get_logger().info(f"Sending stamped joint commands: [{j1}, {j2}, {j3}, {j4}], @[{cmd_time_stamp}]")
        try:
            with self.dobot_lock:
                self.dobot_mag.move_to(x=j1, y=j2, z=j3, r=r, wait=False)
        except Exception as e:
            self.get_logger().error(f"Joint command failed: {e}")
        
    def eef_commander(self, cmd):
        cmd_time_stamp = cmd.header.stamp
        x, y, z = cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z
        self.get_logger().info(f"Sending EEF targets: [{x}, {y}, {z}, {z}], @[{cmd_time_stamp}]")
        
        try:
            with self.dobot_lock:
                self.dobot_mag.move_to(x=x, y=y, z=z, wait=True)
        except Exception as e:
            self.get_logger().error(f"Move command failed: {e}")
    # -----------------------------------------------------------------------------------------------------------

    # --------------------------- FEEDBACK STATES CALLBACKS ---------------------------------------------------------
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
        # self.get_logger().info(f"[j1: {j1}, j2: {j2}, j3: {j3}, j4: {j4}, eef_angle: {r}]")

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
    # ---------------------------------------------------------------------------------------------------------------

    def destroy_node(self):
        with self.dobot_lock:
            if self.dobot_mag and self.dobot_mag.ser.isOpen():
                self.dobot_mag.ser.close()
        super().destroy_node()


    # class static utils
    @staticmethod
    def deg_to_rad(deg_angle):
        return (np.pi*deg_angle)/(180)
    
    @staticmethod
    def rad_to_deg(rad_angle):
        return (rad_angle*180)/(np.pi)
    
    @staticmethod
    def mm_to_m(val):
        return (val/1000)
    



    



