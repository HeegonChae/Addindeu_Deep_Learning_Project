import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal

# 영상 경로 관련 파라미터
camera_dirs = {
        "A" : ['./data/output/detection_A'],
        "B" : ['./data/output/detection_B', './data/output/lstm_B'],
        "C" : ['./data/output/detection_C',  './data/output/lstm_C']
    }

class Detection_Screen(QDialog):   
    closed = pyqtSignal()  # Signal emitted when the window is closed
    
    def __init__(self, camera_id, parent=None):  # 부모 window 설정        
        super(Detection_Screen, self).__init__(parent)       
        detection_screen_ui = './src/SR_detection_screen.ui'            # ./src/gui/SR_detection_screen.ui  
        uic.loadUi(detection_screen_ui, self)
        
        self.label_detected = self.findChild(QLabel, 'labelDetected')  # QLabel 객체를 가져옴
        self.detection_img_dir = None
        if camera_id == 'A':
            self.detection_img_dir = camera_dirs['A'][0]
        elif camera_id == 'B':
            self.detection_img_dir = camera_dirs['B'][0]
        elif camera_id == 'C':
            self.detection_img_dir = camera_dirs['C'][0]          
        self.Loadlastimage()

    def Loadlastimage(self):
        detection_img_dir = self.detection_img_dir
           
        # detection_img_dir 디렉토리에서 모든 파일을 가져옵니다.
        image_files = [f for f in os.listdir(detection_img_dir) if os.path.isfile(os.path.join(detection_img_dir, f))]
        
        if not image_files:
            print("No images found in the directory.")
            return
        
        # 마지막 파일을 가져옵니다.
        last_image_file = image_files[-1]
        last_image_path = os.path.join(detection_img_dir, last_image_file)
        
        # 이미지를 로드하여 QLabel에 표시합니다.
        pixmap = QPixmap(last_image_path)
        self.label_detected.setPixmap(pixmap)
        self.label_detected.setScaledContents(True)  # 이미지가 QLabel 크기에 맞게 조정되도록 설정합니다.

    def closeEvent(self, event):
        self.closed.emit()  # Emit the signal when the window is closed
        event.accept()  # Ensure the event is accepted and the window is closed

if __name__ == '__main__':
    # print(os.getcwd())

    app = QApplication(sys.argv)
    detection_screen = Detection_Screen('C')
    detection_screen.show()
    sys.exit(app.exec_())

