import sys
import os
import cv2
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

class Pose_Estimation_Screen(QDialog):   
    closed = pyqtSignal()  # Signal emitted when the window is closed
    
    def __init__(self, camera_id, parent=None):  # 부모 window 설정        
        super(Pose_Estimation_Screen, self).__init__(parent)       
        detection_screen_ui = './src/SR_estimation_screen.ui'           # ./src/gui/SR_estimation_screen.ui 
        uic.loadUi(detection_screen_ui, self)

        self.label_estimated = self.findChild(QLabel, 'labelEstimated')
        self.estimation_video_dir = None
        if camera_id == 'B':
            self.estimation_video_dir = camera_dirs['B'][-1]
            self.Loadlastvideo()
        elif camera_id == 'C':
            self.estimation_video_dir = camera_dirs['C'][1]         
            self.Loadlastvideo()

    def Loadlastvideo(self):
        estimation_video_dir = self.estimation_video_dir
        # lstm 디렉토리에서 모든 .mp4 파일을 가져옵니다.
        video_files = [f for f in os.listdir(estimation_video_dir) if f.endswith('.mp4') and os.path.isfile(os.path.join(estimation_video_dir, f))]
        
        if not video_files:
            print("No videos found in the directory.")
            return
        
        # 마지막 파일을 가져옵니다.
        last_video_file = video_files[-1]
        last_video_path = os.path.join(estimation_video_dir, last_video_file)
        #print(last_video_path)

        # 동영상을 불러옵니다.
        self.cap = cv2.VideoCapture(last_video_path)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.display_video_stream)
        self.timer.start(20)  # 20ms마다 프레임을 갱신합니다.

    def display_video_stream(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (320, 320))
            h, w, ch = frame.shape
            # print(h, w, ch)
            # print('--------------------------------')
            bytes_per_line = ch * w
            q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img)
            self.label_estimated.setPixmap(pixmap)
        else:
            self.timer.stop()
            self.cap.release()

    def closeEvent(self, event):
        self.closed.emit()  # Emit the signal when the window is closed
        event.accept()  # Ensure the event is accepted and the window is closed

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    camera_id = 'A'  # or 'B' or 'C'
    if camera_id != 'B':
        pose_estimation_screen = Pose_Estimation_Screen(camera_id)
        pose_estimation_screen.show()
    else:
        print("Camera ID is 'A'. Pose_Estimation_Screen will not be shown.")
    
    sys.exit(app.exec_())