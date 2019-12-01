# server.py
from Helpers.camera import Camera
from Helpers.laser_scan import lscan
from Helpers.motion import mover
from Helpers.pointcloud import pointcloud
#import cPickle as pickle
import pickle
import rospy
import time as t
import cv2
from zipfile import ZipFile 
from sendfile import sendfile #pip install pysendfile
import os
import struct
import json



def write_file(fname,data):
    file1 = open(fname, 'w')
    pickle.dump(data,file1)
    file1.close()

def gazebo_work(actions):
    m.teleport(actions[0],actions[1],actions[2]) #we are using absolute movement wrt one starting coord
    #m.get_model_pos() #resets the origin of the robot to the current location
def record_obs(actions):
    #pickle way of sending increses the sent file size many times and is slow
    #pos = [float(m.x_sim),float(m.y_sim),float(m.r_sim)]
    pos = [actions[0],actions[1],actions[2]]
    obs_cam = c.see()
    obs_depth = p.see()
    print("sending pos ",pos)
    obs = {'camera':obs_cam, 'lidar':obs_depth, 'position':pos}
    #write_file('obs.obj',obs)
    return pickle.dumps(obs)
    #return json.dumps(obs)
    

def parse_client(x):
    result = [float(i.strip()) for i in x.split(',')]
    return result

def send_msg(sock, msg):
    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    sock.sendall(msg)

def recv_msg(sock):
    # Read message length and unpack it into an integer
    raw_msglen = recvall(sock, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    return recvall(sock, msglen)

def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

rospy.init_node('env_observers')
c = Camera("raw")
l = lscan()
m = mover(1000)
m.get_model_pos()
p = pointcloud(viz_save = True)
t.sleep(1)

import socket                   # Import socket module
port = 60000                    # Reserve a port for your service.
s = socket.socket()             # Create a socket object
#s.setblocking(0)
#host = socket.gethostname()     # Get local machine name
#s.bind((host, port))            # Bind to the port
s.bind(('', port))            # Bind to the port
s.listen(5)                     # Now wait for client connection.

print 'Server listening....'

conn, addr = s.accept()     # Establish connection with client.
print 'Got connection from', addr

while True:
    data = recv_msg(conn)
    moves = parse_client(data.decode())
    print('Server received', moves)
    gazebo_work(moves)
    obs_data = record_obs(moves)

    print("sending file")
    send_msg(conn,obs_data)
    print('Done sending')

conn.close()

