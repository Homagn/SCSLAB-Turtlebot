# server.py
from Helpers.camera import Camera
from Helpers.laser_scan import lscan
from Helpers.motion import mover
from Helpers.pointcloud import pointcloud
import pickle
import rospy
import time as t
import cv2
from zipfile import ZipFile 
from sendfile import sendfile #pip install pysendfile
import os



def write_file(fname,data):
    file1 = open(fname, 'w')
    pickle.dump(data,file1)
    file1.close()

def gazebo_work(actions):
    m.teleport(actions[0],actions[1],actions[2])
    m.get_model_pos()
    
def record_obs():
    image = cv2.cvtColor(c.see(), cv2.COLOR_BGR2GRAY)
    cv2.imwrite('rgb_image.jpg', image)
    image = p.see()*200.0
    cv2.imwrite('depth_image.png',image)
    # zip observations
    zipObj = ZipFile('images.zip', 'w')
    # Add multiple files to the zip
    zipObj.write('rgb_image.jpg')
    zipObj.write('depth_image.png')
    zipObj.close()
    #t.sleep(1)
    '''
    #pickle way of sending increses the sent file size many times and is slow
    pos = [float(m.x_sim),float(m.y_sim),float(m.r_sim)]
    obs_cam = c.see()
    obs_depth = p.see()
    obs = {'camera':obs_cam, 'lidar':obs_depth, 'position':pos}
    write_file('obs.obj',obs)
    '''

def parse_client(x):
    result = [float(i.strip()) for i in x.split(',')]
    return result

rospy.init_node('env_observers')
c = Camera("raw")
l = lscan()
m = mover(1000)
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
    data = conn.recv(1024)
    moves = parse_client(data)
    print('Server received', moves)
    gazebo_work(moves)
    record_obs()

    #faster way of sending
    filename='images.zip'
    f = open(filename,'rb')
    blocksize = os.path.getsize("images.zip")
    offset = 0
    print("sending file")
    conn.send(repr(blocksize))
    while True:
        sent = sendfile(conn.fileno(), f.fileno(), offset, blocksize)
        if sent == 0:
            break  # EOF
        offset += sent
    f.close()
    print('Done sending')
    #conn.send("")
    #print('Done sending')
    #conn.close()
    #break
    '''
    #slightly slower way of sending
    #filename='current_depth_image.png'
    filename='images.zip'
    f = open(filename,'rb')
    l = f.read(1024*8)
    while (l):
       conn.send(l)
       #print('Sent ',repr(l))
       l = f.read(1024*8)
    f.close()
    '''
    

#conn.send('Thank you for connecting')
conn.close()

