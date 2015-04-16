"""Receives UDP broadcasts from trackbot GPS"""

import socket, select
from sh import nc

UDP_Port = 61557  # port trackbot writes to
bufferSize = 1024 

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('<broadcast>', UDP_Port))
s.setblocking(0)

while True:
    result = select.select([s],[],[])
    msg = result[0][0].recv(bufferSize) 
    print msg
