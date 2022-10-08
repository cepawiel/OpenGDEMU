import socket
import sys
import time

# UDP Server details
UDP_IP = ''
UDP_PORT = 6666

# create and bind socket to specified IP and port
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(300)
except (socket.error, socket.timeout) as msg:
    print 'Error - ',msg
    sock.close()
    sock = None
    sys.exit(1)

# receive data from WINC3400 UDP client 
data, addr = sock.recvfrom(1460)
print "received message:", data

# Send UDP packets.
while True:
    try:
        sock.sendto(time.asctime(),addr)
        print 'Data sent from server -> client'
        sock.settimeout(5.0)
        data, addr = sock.recvfrom(1460)
        print 'Data received from client : ', len(data), 'bytes sent to client'
        time.sleep(3)

    except socket.timeout: # fail after 1 second of no activity
        sock.sendto(time.asctime(),addr)
        time.sleep(3)
