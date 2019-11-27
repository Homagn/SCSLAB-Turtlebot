# client.py
import socket                   # Import socket module
from datetime import datetime as dt
from zipfile import ZipFile 
import time as t

class gazebo_client(object):
    def __init__(self, host = '10.24.250.228', port = 60000):
        self.s = socket.socket()             # Create a socket object
        #host = socket.gethostname()     # Get local machine name
        self.host = host
        self.port = port                   # Reserve a port for your service.
        print(dt.now())
        self.s.connect((self.host, self.port))

        print("Initiated connection")

    def step(self,actions): #send actions over to gazebo and receive observations from remote host gazebo
        #actions should be a comma seperated string for x,y and r movement of turtlebot
        
        self.s.send('-0.1,-0.5,1.57') #Thing sent must be a string otherwise broken pipe error
        blocksize= int(self.s.recv(1024))
        print("Got file blocksize ",blocksize)
        with open('images.zip', 'wb') as f:
            print 'file opened'
            data = self.s.recv(blocksize+1)
            f.write(data)
        f.close()
        '''
        with open('images.zip', 'wb') as f:
            print 'file opened'
            while True:
                #print('receiving data...')
                data = self.s.recv(1024*8)
                #print('data=%s', (data))
                if not data:
                    break
                # write data to a file
                f.write(data)
        f.close()
        '''
        print('Successfully get the file')

        with ZipFile('images.zip', 'r') as zip: 
            # printing all the contents of the zip file 
            zip.printdir() 
          
            # extracting all the files 
            print('Extracting all the files now...') 
            zip.extractall() 
            print('Done!') 
        #self.s.setblocking(0)
        #self.s.send('received')
        t.sleep(0.33) #wait for depth map to get updated
    def stop(self):
        self.s.close()
        print('connection closed')
        print(dt.now())

if __name__ == '__main__':
    gc = gazebo_client()
    gc.step('-0.1,-0.5,1.57')
    gc.step('0.1,-0.5,1.57')
    gc.step('-0.1,0.5,1.57')

    t.sleep(5) #do whatever other things 
    gc.stop() #After this the server code would stop with an error
    #t.sleep(0.5)
    #gc.step('0.1,-0.5,1.57')
    #gc.stop()
    #gc.step('0.1,-0.5,1.57')
    #gc.step('-0.1,0.5,1.57')


'''
s = socket.socket()             # Create a socket object
#host = socket.gethostname()     # Get local machine name
host = '10.24.250.228'
port = 60000                    # Reserve a port for your service.
print(dt.now())
s.connect((host, port))
s.send('-0.1,-0.5,1.57') #Thing sent must be a string otherwise broken pipe error

with open('images.zip', 'wb') as f:
    print 'file opened'
    while True:
        #print('receiving data...')
        data = s.recv(1024*8)
        #print('data=%s', (data))
        if not data:
            break
        # write data to a file
        f.write(data)
f.close()
print('Successfully get the file')

with ZipFile('images.zip', 'r') as zip: 
    # printing all the contents of the zip file 
    zip.printdir() 
  
    # extracting all the files 
    print('Extracting all the files now...') 
    zip.extractall() 
    print('Done!') 


s.close()
print('connection closed')
print(dt.now())
'''