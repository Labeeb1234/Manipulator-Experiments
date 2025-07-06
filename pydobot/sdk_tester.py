from serial.tools import list_ports

import pydobot
from pydobot.enums.ptpMode import PTPMode
import time

available_ports = list_ports.comports()
# print(f'available ports: {[x.device for x in available_ports]}')
port = available_ports[-1].device

device = pydobot.Dobot(port=port, verbose=False)

(x, y, z, r, j1, j2, j3, j4) = device.pose()
print(f'x:{x} y:{y} z:{z} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')

poses = []
start = time.time()
while device.ser.isOpen():
    (x, y, z, r, j1, j2, j3, j4) = device.pose()
    poses.append((x, y, z, r, j1, j2, j3, j4))
    device.move_to(x=x, y=y, z=z, r=r, wait=False)

    if time.time()-start > 5.0:
        device._set_queued_cmd_clear()
        break

    time.sleep(0.01)

print(len(poses))

device.close()
