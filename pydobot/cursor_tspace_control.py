#!/usr/bin/env python3
from serial.tools import list_ports
import pydobot
import pyautogui
import time
import threading


available_ports = list_ports.comports()
port = available_ports[-1].device
device = pydobot.Dobot(port=port,verbose=False)

mutex_lock = threading.Lock()
freq = 10 # in [Hz]

prev_x = pyautogui.position().x
prev_y = pyautogui.position().y
previous_time = time.time()
mx = []
my = []
def cursor_motion():
    global prev_x, prev_y, previous_time  # Declare globals to modify them
    current_time = time.time()
    if(current_time-previous_time > 1/freq):
        x, y = pyautogui.position()
        dx = x-prev_x
        dy = y-prev_y

        with mutex_lock:
            mx.append(dx)
            my.append(dy)

        prev_x = x
        prev_y = y
        previous_time = current_time

def display_data():
    with mutex_lock:
        if mx and my:
            dx = mx[-1]
            dy = my[-1]

            print(f"{len(mx)}")
            
            if len(mx) > 10:
                mx.pop(0)
            if len(my) > 10:
                my.pop(0)

            (x, y, z, r, j1, j2, j3, j4) = device.pose()
            print(f'x:{x} y:{y} z:{z} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')
            device.move_to(x+dx, y+dy, z, r, wait=True)

def main():
    try:
        # Start the cursor motion thread
        while True:
            # cursor_motion()
            # display_data()
            (x, y, z, r, j1, j2, j3, j4) = device.pose()
            print(f'x:{x} y:{y} z:{z} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')


    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        device.close()  # Ensure the device is closed on exit

if __name__ == '__main__':
    main()