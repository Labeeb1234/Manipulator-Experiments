#! /usr/bin/python3

import rclpy

import time
from serial.tools import list_ports
from pydobot.enums import PTPMode

from dobot_rclpy_interface import DobotInterface
import rclpy.executors

def main(args=None):

    port_id = list_ports.comports()[-1].device
    print(f"[INFO]: Dobot Serial Port ID: {port_id}")

    rclpy.init(args=args)
    node = DobotInterface(port_id=port_id, verbose=False, ptpMode=PTPMode.MOVJ_XYZ)
    executors = rclpy.executors.MultiThreadedExecutor(4)
    executors.add_node(node)

    try:
        executors.spin()
    except Exception as e:
        print(f"[INFO]: Exception occured: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    
    exit(0)

if __name__ == '__main__':
    main()