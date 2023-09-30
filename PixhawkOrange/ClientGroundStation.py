import socket
import base64
import json
import base64
import cv2
class ClientConn():
    def __init__(self,host,port):
        """
        host : (Name of computer or ip adress computer)("localhost")
        port : (Port used by the server)("65432")
        """
        self.host = host
        self.port = port
        self.data = {}
        self.isConnected=True
        self.isSended=True
        self.isFrontCamSended=True
        self.isUnderCamSended=True
        self.isPixhawkSended=True
        self.isDistanceSended=True
        self.isHydrophoneSended=True
        self.connect()

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
           self.sock.settimeout(0.5)
           self.sock.connect((self.host, self.port))
           print("Connected.")
           self.isConnected=True
        except Exception:
            if(self.isConnected):
                print("Connection close.")
            self.isConnected=False
    
    def codeImage(self,frame):
        _, encoded_frame = cv2.imencode('.jpg', frame)
        framebase64 = base64.b64encode(encoded_frame.tobytes()).decode() 
        return framebase64
    
    def addData2Json(self,key,value):
        self.data[key] = value
    
    def setHydrophoneData(self,hydrodata):
        try:
            json_data = json.dumps(hydrodata)
            self.addData2Json("hydrophone", json_data)
            self.isHydrophoneSended=True
        except:
            if self.isHydrophoneSended:
                print("Error: Hydrophone data not added.Check hydrophone sensor.")
                self.isHydrophoneSended=False
    
    def setDistanceData(self,disdata):
        try:
            json_data = json.dumps(disdata)
            self.addData2Json("distance", json_data)
            self.isDistanceSended=True
        except:
            if self.isDistanceSended:
                print("Error: Distance data not added.Check distance sensor.")
                self.isDistanceSended=False
                
        
    def setPixhawkData(self,pixhawkdata):
        try:
            self.addData2Json("pixhawkdata", pixhawkdata)
            self.isPixhawkSended=True
        except:
            if self.isPixhawkSended:
                print("Error: Pixhawk data not added.Check pixhawk.")
                self.isPixhawkSended=False
    
    
    def setFrontCam(self,frame):
        try:
            self.addData2Json("cam1",self.codeImage(frame))
            self.isFrontCamSended=True
        except:
            if self.isFrontCamSended:
                print("Error: Front Cam not added.Check camera.")
                self.isFrontCamSended=False
    
    def setUnderCam(self,frame):
        try:
            self.addData2Json("cam2",self.codeImage(frame))
            self.isUnderCamSended=True
        except:
            if self.isUnderCamSended:
                print("Error: Under Cam not added.Check camera.")
                self.isUnderCamSended=False
        
    def sendAllData(self):
        json_data = json.dumps(self.data)
        try:
            self.sock.send(json_data.encode())
            self.recieveData()
            self.isSended=True
        except (BrokenPipeError, ConnectionResetError):
            if(self.isSended):
                print("Server closed.")
                self.isConnected=False
                self.isSended=False
            else:
                self.connect()
        except Exception as e:
                print(e)
                self.connect() 

    def recieveData(self):
        response = self.sock.recv(1024)
        if(response.decode()=="recieved"):
            pass
            
    def close(self):
        self.sock.close()