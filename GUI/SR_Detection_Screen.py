import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal

class Detection_Screen(QDialog):   
    closed = pyqtSignal()  # Signal emitted when the window is closed
    
    def __init__(self, parent=None):  # 부모 window 설정        
        super(Detection_Screen, self).__init__(parent)       
        detection_screen_ui = './src/gui/SR_detection_screen.ui'     
        uic.loadUi(detection_screen_ui, self)

        self.label_detected = self.findChild(QLabel, 'labelDetected')  # QLabel 객체를 가져옵니다. 'imageLabel'은 UI 파일에 정의된 QLabel의 이름입니다.
        self.Loadlastimage()

    def Loadlastimage(self):
        detection_img_dir = './data/received_files/detection'
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
    detection_screen = Detection_Screen()
    detection_screen.show()
    sys.exit(app.exec_())