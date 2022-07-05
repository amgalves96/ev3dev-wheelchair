#!/usr/bin/env python3

import smbus
import time
# I2C-Adresse des Arduino
address = 0x40
# Erzeugen einer I2C-Instanz und Oeffnen des Busses
# scheinbar PortNr +2
ardu = smbus.SMBus(6)#Port 4
anz=200
# Lesen
while anz>0:
  daten=[]
  reg= 0x01
  anz-=1
  for i in range(8):
      d= ardu.read_byte_data(0x04,0x00+i)
      daten.append(d)
  print (daten)
  time.sleep(0.01)
