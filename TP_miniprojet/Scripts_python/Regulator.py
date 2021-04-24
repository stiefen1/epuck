import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import serial
import struct
import sys
import signal
import time
from threading import Thread

READ_SLEEP = 0.1
READ_MAX = 10

class SerialThread(Thread):
  def __init__(self, port):
    Thread.__init__(self)
    self._buffer = []
    # Non-blocking read
    try:
      self._conn = serial.Serial(port, timeout=0.5)
    except Exception as inst:
      print(inst)
      sys.exit(0)

  def run(self):
    received = bytearray([])
    self._buf = []
    while True:
      received += self._conn.read(size=4*READ_MAX)
      if len(received) >= 4*READ_MAX:
        for i in range(0, 4*READ_MAX, 4):
          self._buf.append(struct.unpack_from('<f', received, i))
        received = received[4*READ_MAX:]

read_th = SerialThread("COM13")
read_th.start()
