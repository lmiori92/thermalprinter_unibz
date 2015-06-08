'''
programma python:

- allo start fa un comando "go to sx home"
- prendi stato
 Se non IDLE, attendi
Se IDLE:
    comando x: trasferisci dati
    seguono i 3 bytes
    attendi IDLE -> TODO deve andare in stato trasfer o simile
    comando p: print
    comando l: avanza linea
    
 
'''

import serial
import time

dati = [[1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1],
        [1,1,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1],
        [1,0,0,0,0,0,0],
        [1,1,0,0,0,0,0],
        [1,1,1,0,0,0,0],
        [1,1,1,1,0,0,0],
        [1,1,1,1,1,0,0],
        [1,1,1,1,1,1,0],
        [1,1,1,1,1,1,1],
       ]

port = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=10.0)

#while True:
#    port.write("\r\nSay something:")
#    rcv = port.read(10)
#    port.write("\r\nYou sent:" + repr(rcv))

def wait():
    time.sleep(0.1)
#    port.write("o")
#    while port.read() != "0":
#        time.sleep(0.01)

#port.write("s") # go to SX
#time.sleep(3)

for i in range(len(dati)):
    port.write("x") # WARNING DO NOT START A WAIT AFTER X !!!
    wait()
    port.write(chr(int("".join(map(str, dati[i])), 2)))
    wait()
    port.write(chr(int("".join(map(str, dati[i])), 2)))
    wait()
    port.write(chr(int("".join(map(str, dati[i])), 2)))
    wait()
    port.write("p")
    wait()
    port.write("l")
    wait()
