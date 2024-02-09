from cv2 import VideoCapture, UMat

class Camera():
    SUCCESS = 0
    def __init__(self):
        try:
            self.front_cap = VideoCapture(0)
        except Exception as e:
            self.front_cap = None
            print(f'Hata: {e}')
            print('-> Ön kamera bulunamadı.')
        
        try:
            self.bottom_cap = VideoCapture(1)
        except Exception as e:
            self.bottom_cap = None
            print(f'Hata: {e}')
            print('-> Alt kamera bulunamadı.')
        
        return

    def is_front_cam_open(self) -> bool:
        return self.front_cap.isOpened() if self.front_cap != None else False
    
    def is_bottom_cam_open(self) -> bool:
        return self.bottom_cap.isOpened() if self.bottom_cap != None else False

    def get_front_cam(self) -> tuple[bool, UMat]:
        return self.front_cap.read() # Raise error if we cant read

    def get_bottom_cam(self) -> tuple[bool, UMat]:
        return self.bottom_cap.read() # Raise error if we cant read

    def release_cams(self) -> int:
        self.front_cap.release() if self.front_cap != None else False
        self.bottom_cap.release() if self.bottom_cap != None else False
        return self.SUCCESS