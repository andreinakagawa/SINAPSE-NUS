import socket
import threading
import SocketServer
from mutex import mutex
from collections import deque
from threading import Lock
from threadhandler import ThreadHandler

m = Lock()
q = deque()

class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):

    def handle(self):
        global m,q
        data = self.request.recv(2)
        data = bytearray(data)        
        cur_thread = threading.current_thread()
        #response = "{}: {}".format(cur_thread.name, data)
        m.acquire()
        q.append((data[0]<<8 | data[1]))
        m.release()
        #print (data[0]<<8 | data[1])
        #print data
        #self.request.sendall(response)

def updateFile():
    global m,q
    f = open('temp4.data','a')
    m.acquire()
    n = len(q)    
    for k in range(n):
        d = q.pop()
        f.write(str(d) + '\n')
        #f.write(d.replace(',','.') + '\n')
    m.release()
    f.close()
    

if __name__ == "__main__":    
    # Port 0 means to select an arbitrary unused port
    HOST, PORT = "localhost", 8888

    server = SocketServer.TCPServer((HOST, PORT), ThreadedTCPRequestHandler)
    ip, port = server.server_address

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    server_thread = threading.Thread(target=server.serve_forever)
    # Exit the server thread when the main thread terminates
    server_thread.daemon = True
    server_thread.start()
    print "Server loop running in thread:", server_thread.name

    #client(ip, port, "Hello World 1")
    #client(ip, port, "Hello World 2")
    #client(ip, port, "Hello World 3")

    fileThread = ThreadHandler(updateFile)
    fileThread.start()

    x = raw_input('type any key to finish....\n')

    fileThread.kill()

    server.shutdown()
    server.server_close()