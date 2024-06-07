import sys 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

import requests
import json
import resources_rc
import random

from_class = uic.loadUiType("dl_ui.ui")[0]

class WindowClass(QMainWindow, from_class) :
    def __init__(self) :
        super().__init__()
        self.setupUi(self)
        self.Setup()

        self.image_paths = [
            "./data/good.png",
            "./data/soso.png",
            "./data/full.png"
        ]
        self.current_index = 0
        self.detected_img = "./data/detected.png"
        self.label_emoji.setPixmap(QPixmap(self.image_paths[self.current_index]))

        self.btn_a.clicked.connect(self.buttonClicked)
        self.btn_b.clicked.connect(self.buttonClicked)
        self.btn_c.clicked.connect(self.buttonClicked)
        self.hideAllButtons()
        self.hideAllLabels()

    def Setup(self):
        # 이미지 타이머 
        self.image_timer = QTimer(self)
        self.image_timer.timeout.connect(self.update_image)
        self.image_timer.start(5000) 

        # 버튼 타이머
        self.btn_timer = QTimer(self)
        self.btn_timer.timeout.connect(self.showRandomButton)
        self.btn_timer.start(3000)  # 3초마다 업데이트

        # 클릭 타이머
        self.click_timer = QTimer(self)
        self.click_timer.setSingleShot(True)
        self.click_timer.timeout.connect(self.showDetectedImage)

        # 시계 타이머 관련 위젯
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.Showtime)
        self.lcdTimer.display('')
        self.lcdTimer.setDigitCount(8)

        # 날씨 관련 파라미터
        self.forecastApi = "6e357d11903a3df81c1cff95bca0f2af"
        self.location = "Seoul" 
        self.lang = 'kr'
        self.units = 'metric'
        self.timer.start()
        
    def Showtime(self):
        # 날짜
        datetime = QDateTime.currentDateTime()
        self.labelDate.setText(datetime.toString('yyyy-MM-dd'))
        # 시간
        sender = self.sender()
        currentTime = QTime.currentTime().toString("hh:mm:ss")
        if id(sender) == id(self.timer):
            self.lcdTimer.display(currentTime)
        # 날씨
        self.ShowForecast()

    def ShowForecast(self):
        api = f"https://api.openweathermap.org/data/2.5/weather?q={self.location}&appid={self.forecastApi}&lang={self.lang}&units={self.units}"

        result = requests.get(api)
        result = json.loads(result.text)
        weather = result['weather'][0]['main']
        degree = result['main']['temp']
        self.labelWeather.setText(str(degree) + '\u2103'+'\n'+weather)

    def update_image(self):
        # 다음 이미지로 업데이트
        self.current_index = (self.current_index + 1) % len(self.image_paths)
        self.label_emoji.setPixmap(QPixmap(self.image_paths[self.current_index]))

    def hideAllButtons(self):
        self.btn_a.hide()
        self.btn_b.hide()
        self.btn_c.hide()

    def hideAllLabels(self):
        self.label_a.clear()
        self.label_b.clear()
        self.label_c.clear()
        self.label_a.hide()
        self.label_b.hide()
        self.label_c.hide()

    def showRandomButton(self):
        self.hideAllButtons()
        self.hideAllLabels()
        self.random_button = random.choice([self.btn_a, self.btn_b, self.btn_c])
        self.random_button.show()
        self.showCorrespondingLabel(self.random_button)
        self.click_timer.start(3000)

    def showCorrespondingLabel(self, button):
        if button == self.btn_a:
            self.label_a.show()
        elif button == self.btn_b:
            self.label_b.show()
        elif button == self.btn_c:
            self.label_c.show()

    def buttonClicked(self):
        sender = self.sender()
        sender.hide()
        self.click_timer.stop()
        self.showDetectedImage()

    def showDetectedImage(self):
        if self.random_button == self.btn_a:
            self.label_a.setPixmap(QPixmap(self.detected_img))
        elif self.random_button == self.btn_b:
            self.label_b.setPixmap(QPixmap(self.detected_img))
        elif self.random_button == self.btn_c:
            self.label_c.setPixmap(QPixmap(self.detected_img))
        
        QTimer.singleShot(3000, self.resetUI)

    def resetUI(self):
        self.hideAllButtons()
        self.hideAllLabels()
        self.showRandomButton()

if __name__ == "__main__" :
    app = QApplication(sys.argv) 
    myWindows = WindowClass()
    myWindows.show() 
    sys.exit(app.exec_())
