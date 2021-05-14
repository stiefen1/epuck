# Good Parameters
# Ki : 1.6
# Kp : 890
# Kd : 700
import matplotlib.pyplot as plt
import numpy as np

import serial
import sys

import struct

from matplotlib.widgets import Slider, Button

import time
from threading import Thread

NB_SAMPLES = 1024
PORT = "COM13"

def readFloatSerial(port):

    state = 0

    while(state != 5):

        #reads 1 byte
        c1 = port.read(1)
        #timeout condition
        if(c1 == b''):
            print('Timout one...')
            return [];

        if(state == 0):
            if(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'T'):
                state = 2
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 3):
            if(c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 4):
            if(c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    #reads the size
    #converts as short int in little endian the two bytes read
    size = struct.unpack('<h',port.read(2)) 
    #removes the second element which is void
    size = size[0]  

    #reads the data
    rcv_buffer = port.read(size*4)
    data = []

    #if we receive the good amount of data, we convert them in float32
    if(len(rcv_buffer) == 4*size):
        i = 0
        while(i < size):
            decoded, = struct.unpack_from('<f',rcv_buffer, i*4)
            data.append(decoded)
            i = i+1

        # print('received !')
        return data
    else:
        print('Timout...')
        return []

def sendFloatSerial(port, data):
    send_buffer = bytearray([])

    for i in range(len(data)):
        send_buffer += struct.pack('<f',data[i])

    port.write(b'START')
    port.write(struct.pack('<h',4*len(data)))
    port.write(send_buffer)
    port.flush()


try:
  conn = serial.Serial(PORT, timeout=1, baudrate=115200)
except Exception as inst:
  print(inst)
  sys.exit(0)

print("epuck connected!")

t = np.linspace(0, 1, NB_SAMPLES)
acc = np.zeros(NB_SAMPLES)

class Reader(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.updated = False
        self.alive = True
        pass
    def run(self):
        global acc, conn, ax, t
        global skp, skd
        while self.alive:
            if conn.inWaiting() > 0:
                acc = np.array(readFloatSerial(conn))
                print(np.mean(acc))
                line.set_ydata(acc)
                
                self.updated = True
                
                # @compute_response_time
                # @send_new_regulator_parameters
            
            time.sleep(0.1)
    def stop(self):
        self.alive = False
        self.join()

reader = Reader()
reader.start()


fig, ax = plt.subplots()
line, = ax.plot(t, acc)
ax.set_xlim(0, 1)
ax.set_ylim(-np.pi/2, np.pi/2)

plt.subplots_adjust(left=0.1, bottom=0.3)

axcolor = 'lightgoldenrodyellow'
axkp = plt.axes([0.1, 0.15, 0.65, 0.03], facecolor=axcolor)
axkd = plt.axes([0.1, 0.1, 0.65, 0.03], facecolor=axcolor)

skp = Slider(axkp, 'kp',0.0, 10000.0, valinit=0.0, valstep=10.0)
skd = Slider(axkd, 'kd',0.0, 10000.0, valinit=0.0, valstep=10.0)

axki = plt.axes([0.1, 0.2, 0.65, 0.03], facecolor=axcolor)
ski = Slider(axki, 'ki',0.0, 10.0, valinit=0.0, valstep=0.1)

axth = plt.axes([0.1, 0.05, 0.65, 0.03], facecolor=axcolor)
sth = Slider(axth, 'angle', -2, 2, valinit=0.0, valstep=0.1)


def send(event):
    print("send")
    param = [-skp.val, -skd.val, -ski.val]
    sendFloatSerial(conn, param)
    

buttonax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(buttonax, 'Send', color=axcolor, hovercolor='0.975')
button.on_clicked(send)

def on_close(event):
    global stop_loop, reader
    reader.stop()
    
    if(conn.isOpen()):
        while(conn.inWaiting() > 0):
            conn.read(conn.inWaiting())
            time.sleep(0.01)
        conn.close()

fig.canvas.mpl_connect('close_event', on_close)


def update_plot():
    global reader, fig
    if reader.updated:
        fig.canvas.draw_idle()
        reader.updated = False

timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_plot)
timer.start()


plt.show()

