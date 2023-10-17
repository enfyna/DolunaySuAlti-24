# pip install --user bluerobotics-ping --upgrade
from brping import Ping1D

class Distance():
    def __init__(self):
        try:
            self.rightDistance = Ping1D()
            self.rightDistance.connect_serial("/dev/ttyACM0", 115200)
            self.initilazeRightDisSensor()
        except:
            pass
        try:
            self.leftDistance = Ping1D()
            self.leftDistance.connect_serial("/dev/ttyACM0", 115200)
            self.initilazeRightDisSensor()
        except:
            pass
    
    def initilazeRightDisSensor(self):
        #self.rightDistance.set_speed_of_sound(1450000)
        if self.rightDistance.initialize() is False:
            print("Failed to initialize Ping Right!")
        
    def initilazeLeftDisSensor(self):
        #self.leftDistance.set_speed_of_sound(1450000)
        if self.leftDistance.initialize() is False:
            print("Failed to initialize Ping Left!")
        
    def getRightDistance(self):
        data = self.rightDistance.get_distance()
        return data["distance"],data["confidence"]
    
    def getLeftDistance(self):
        data = self.leftDistance.get_distance()
        return data["distance"],data["confidence"]
    
    def getDistance(self):
        datal = self.leftDistance.get_distance()
        datar = self.rightDistance.get_distance()
        return datal["distance"],datar["distance"]
    
    def getDiffDis(self):
        """
        (-) değer sol sensor daha uzak
        (+) değer sağ sensor daha uzak
        """
        left,right = self.getDistance()
        diff = right-left
        return diff