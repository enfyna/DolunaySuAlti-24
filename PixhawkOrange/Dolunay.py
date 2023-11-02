from PixhawkOrange import PixhawkOrange
from DistanceSensor import Distance
from Camera import Camera

class Dolunay():
    def __init__(self):
        self.Pixhawk = PixhawkOrange()
        self.Distance = Distance()
        self.Camera = Camera()