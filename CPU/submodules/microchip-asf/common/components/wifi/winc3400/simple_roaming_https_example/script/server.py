from flask import Flask, render_template, request,jsonify, flash, redirect,request, url_for
import random, socket, threading
import time

cache = {}

cache['pm10'] = ''

#tcp server
TCP_IP = '192.168.1.4'
TCP_PORT = 7005
BUFFER_SIZE  = 20

def launchServer():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
      

    print('waiting for connection')
    conn, addr = s.accept()
    print ('Connection address:', addr)
    
    while True:
        conn.sendall(time.asctime())
        print ('Data sent from server -> client')
        time.sleep(0.5)
        num = conn.recv(40)
        num = num.rstrip('\x00')
        cache['pm10'] = num
        print('Server received data on', str(cache['pm10']) + time.asctime())
        time.sleep(0.5)


#flask app
app = Flask(__name__)

@app.route('/', methods=['GET'])
def index():
    return render_template('main.html')

@app.route('/pay', methods=['GET'])
def pay_calc():
    return jsonify({'data1': cache['pm10']})

if __name__ == "__main__":
    t = threading.Thread(target=launchServer)
    t.daemon = True
    t.start()
    app.run(ssl_context=('cert.pem', 'key.pem'))
