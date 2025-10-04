from serial.tools import list_ports
import pydobot
import time
import threading
import keyboard

# default home pose for test :(166.51953125, -8.546928405761719, 15.977615356445312, -2.938235282897949, -2.938235282897949, 27.588985443115234, 44.849822998046875, 0.0)

def main():
    available_ports = list_ports.comports()
    port = available_ports[-1].device
    device = pydobot.Dobot(port=port, verbose=False)
    
    x, y, z, r, j1, j2, j3, j4 = device.pose()
    

    for epi in range(1000):
        
        while device.ser.isOpen():
            if epi > 100:
                break
            
            start = time.perf_counter()
            x, y, z, r, j1, j2, j3, j4 = device.pose()
            print(f"joint pos: [{j1}, {j2}, {j3}, {j4}]")
            device.move_to(x=0,y=0,z=0,r=0, wait=False)

            end = time.perf_counter()
            print(f"dt: {end-start}")

        print(f"\n")

    
    device.ser.close()
    print(f"{device.ser.isOpen()}")


if __name__ == '__main__':
    main()