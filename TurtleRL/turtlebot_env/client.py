# client.py
import socket                   # Import socket module
from datetime import datetime as dt
from zipfile import ZipFile 
import time as t
import struct
import pickle
import json

class gazebo_client(object):
    def __init__(self, host = '10.24.250.228', port = 60000):
        self.s = socket.socket()             # Create a socket object
        #host = socket.gethostname()     # Get local machine name
        self.host = host
        self.port = port                   # Reserve a port for your service.
        
        self.s.connect((self.host, self.port))

        print("Initiated connection")

    def send_msg(self,msg):
        msg = struct.pack('>I',len(msg)) + msg
        self.s.sendall(msg)
    def recv_msg(self):
        # Read message length and unpack it into an integer
        raw_msglen = self.recvall(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        return self.recvall(msglen)
    def recvall(self, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = bytearray()
        while len(data) < n:
            packet = self.s.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data
    def step(self,actions,folder='data'): #send actions over to gazebo and receive observations from remote host gazebo
        #actions should be a comma seperated string for x,y and r movement of turtlebot
        self.send_msg(actions.encode()) #Thing sent must be a string otherwise broken pipe error
        #blocksize= int(self.s.recv(1024))
        #print("Got file blocksize ",blocksize)
        self.byte_data = self.recv_msg()
        #self.byte_data = self.byte_data.decsode()
        self.data = pickle.loads(self.byte_data, encoding="bytes")
        #self.data = json.loads(self.byte_data)
        lidar = self.data[b'lidar']
        print("Got lidar data shape",lidar.shape)
        print("Got data")
        return self.data[b'camera'],self.data[b'lidar'],self.data[b'position']
    def stop(self):
        self.s.close()
        print('connection closed')
        

if __name__ == '__main__':
    gc = gazebo_client()
    print(dt.now())
    gc.step('0.5,0.0,0.0',folder='1') #passed values are global positions wrt start
    print(dt.now())
    gc.step('-2.0,0.0,1.57',folder='2')
    print(dt.now())
    gc.step('0.0,-3.0,-1.57',folder='3')
    print(dt.now())
    gc.step('0.5,-3.0,-1.57',folder='3')
    print(dt.now())
    gc.step('-1.0,-2.0,-1.57',folder='3')
    print(dt.now())

    #t.sleep(5) #do whatever other things 
    gc.stop() #After this the server code would stop with an error

#NOTE ! - def recvall(self, n): is the slowest function taking avg 0.45 secs everytime to read. Maybe try parallelize