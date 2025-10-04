from serial.tools import list_ports
import pydobot
from collections import deque
import time
import threading
import matplotlib.pyplot as plt


x_pos, y_pos, z_pos = deque(maxlen=300), deque(maxlen=300), deque(maxlen=300)
time_stamps = deque(maxlen=300)
mutex_lock = threading.Lock()

def arm_control(device):
    (sx, sy, sz, sr, sj1, sj2, sj3, sj4) = device.pose()
    print(f'x:{sx} y:{sy} z:{sz} j1:{sj1} j2:{sj2} j3:{sj3} j4:{sj4}')

    t0 = time.time()
    # loop running at 100Hz
    while device.ser.isOpen():
        try:
            (x, y, z, r, j1, j2, j3, j4) = device.pose()
            print(f"[INFO]: current eef pos: ({x}, {y}, {z})")
            print(f"[INFO]: joint states: [{j1}, {j2}, {j3}, {j4}]" )
            with mutex_lock:
                x_pos.append(x)
                y_pos.append(y)
                z_pos.append(z)
                time_stamps.append(time.time()-t0)
            device.move_to(x=sx+10, y=sy+30, z=sz, r=r, wait=False)
            device.move_to(x=sx, y=sy, z=sz, r=r, wait=False)
            # time.sleep(0.01)

        except KeyboardInterrupt:
            print("[INFO]: Ctrl+C received. Attempting emergency stop...")
            device._set_queued_cmd_stop_exec()
            time.sleep(0.1)
            device._set_queued_cmd_clear()

    device.close()


def visualize_data():
    plt.ion()
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 10))  # Larger figure
    while True:
        with mutex_lock:
            x_list = list(x_pos)
            y_list = list(y_pos)
            z_list = list(z_pos)
            t_list = list(time_stamps)

            ax1.clear()
            ax2.clear()
            ax3.clear()
            ax4.clear()
            
            # Plot original data with low opacity
            ax1.plot(t_list, x_list, color='blue', label='x original')
            ax2.plot(t_list, y_list, color='green', label='y original')
            ax3.plot(t_list, z_list, color='red', label='z original')
            ax4.plot(x_list, y_list, color='yellow', label='x-y plane trajectory')

            ax1.set_ylabel('x')
            ax2.set_ylabel('y')
            ax3.set_ylabel('z')
            ax3.set_xlabel('Time (s)')
            ax4.set_ylabel('y')
            ax4.set_xlabel('x')
            
            ax1.legend()
            ax2.legend()
            ax3.legend()
            ax4.legend()
        plt.pause(0.01) # 100Hz



def main():
    available_ports = list_ports.comports()
    port = available_ports[-1].device
    device = pydobot.Dobot(port=port, verbose=False)

    arm_control_thread = threading.Thread(target=arm_control, args=(device,), daemon=True)
    arm_control_thread.start()
    visualize_data()

if __name__ == '__main__':
    main()


