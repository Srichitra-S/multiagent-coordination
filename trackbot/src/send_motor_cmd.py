import socket
import time

def send_udp():
    UDP_IP = "192.168.17.150"
    UDP_PORT = 7070
    MESSAGE = "0%0"
    
    print "UDP target IP:", UDP_IP
    print "UDP target port:", UDP_PORT
    print "message:", MESSAGE
    
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


while True:
    send_udp()
    time.sleep(5)
