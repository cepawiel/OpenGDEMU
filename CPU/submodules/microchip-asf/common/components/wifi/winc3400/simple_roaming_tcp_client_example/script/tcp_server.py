import socket
import time
import sys

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('', 6666)
print 'starting up on', server_address

try:
    sock.bind(server_address)

    # Listen for incoming connections
    sock.listen(1)
    
    # Wait for a connection
    print 'waiting for a connection'

    connection, client_address = sock.accept()
except (socket.error, socket.timeout) as msg:
    print 'Error - ',msg
    sock.close()
    sock = None
    sys.exit(1)

try:
    connection.settimeout(60)
    print 'connection from', client_address

    # Continuously transmit and receive data 
    while True:
        connection.sendall(time.asctime())
        print 'Data sent from server -> client'
        time.sleep(3)
        
        data_packet_size = connection.recv(1460)
        print 'Data received from client : ', len(data_packet_size), 'bytes sent to client'
        time.sleep(2)
