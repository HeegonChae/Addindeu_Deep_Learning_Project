import sys
import os
import random
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
from Connect import Connect
from SR_Detection_Screen import Detection_Screen
from SR_Pose_Estimation_Screen import Pose_Estimation_Screen
import requests
import json
# 이미지 바꿀 때마다 실행   
import resources_rc     #  pyrcc5 resources.qrc -o resources_rc.py

from_class = uic.loadUiType("./SR_monitor_screen.ui")[0]        # "./src/gui/SR_monitor_screen.ui"

class getApiData():
    def __init__(self):
        # 날씨 관련 파라미터
        self.forecastApi = "6e357d11903a3df81c1cff95bca0f2af"
        self.location = "Seoul"
        self.lang = 'kr'
        self.units = 'metric'

        # 긴급 신고(경찰서, 소방서) 관련 파라미터
        self.emergencyApi = "6bed6ba9baa8446598071e945e89dcec"
        self.pIndex = 1
        self.pSize = 10
        self.SIGUN_CD = 41290

    def ShowForecast(self):
        api = f"https://api.openweathermap.org/data/2.5/weather?q={self.location}&appid={self.forecastApi}&lang={self.lang}&units={self.units}"
        result = requests.get(api)
        result = json.loads(result.text)
        weather = result['weather'][0]['main']
        degree = result['main']['temp']
        self.labelWeather.setText(str(degree) + '\u2103' + '\n' + weather)

    def ShowEmergencyCenter(self):
        api = f"https://openapi.gg.go.kr/FiresttnPolcsttnM?Key={self.emergencyApi}&pIndex={self.pIndex}&pSize={self.pSize}&SIGUN_CD={self.SIGUN_CD}&Type=json"
        result = requests.get(api)
        data = result.json()

        emergency_centers = []
        rows = data.get("FiresttnPolcsttnM", [{}])[1].get("row", [])
        for row in rows:
            inst_nm = row.get("INST_NM")
            if inst_nm:
                emergency_centers.append(inst_nm)

        return emergency_centers
    
    def GetPoliceStation(self, emergency_data):
        for station in emergency_data:
            if '경찰' in station or '경찰서'in station:
                return station
        return None
    
    def GetAmbulanceStation(self, emergency_data):
        for station in emergency_data:
            if '소방' in station or '소방서' in station:
                return station
        return None
    
class WindowClass(QMainWindow, from_class, getApiData):
    def __init__(self, db_instance):
        super().__init__()
        # getApiData 클래스 인스턴스 초기화
        getApiData.__init__(self)
        # Detection_Screen 클래스 인스턴스 초기화
        self.detection_screen = Detection_Screen(self)  
        # Detection_Screen 클래스 인스턴스 초기화
        self.estimation_screen = Pose_Estimation_Screen(self)  

        self.detection_screen.closed.connect(self.stopBlinking)  # Connect the signal
        self.estimation_screen.closed.connect(self.onEstimationScreenClosed)  # Connect the signal

        self.cursor = db_instance.cursor
        self.conn = db_instance.conn
        self.setupUi(self)
        self.Setup()

        # 경찰 이미지 btn 관련 설정 및 이벤트 처리
        self.btn_police.setStyleSheet("background-image: url(:/newPrefix/police.png);") 
        self.btn_police.clicked.connect(self.PoliceButtonClick)
        # 엠블런스 이미지 btn 관련 설정 및 이벤트 처리
        self.btn_ambulance.setStyleSheet("background-image: url(:/newPrefix/ambulance.png);") 
        self.btn_ambulance.clicked.connect(self.AmbulanceButtonClick)

        # 깜빡임 기능을 활성화
        self.blink_enabled = True 

        # Person count 업데이트 타이머
        # self.person_count_timer = QTimer()
        # self.person_count_timer.setInterval(1000)  # 1초 간격
        # self.person_count_timer.timeout.connect(self.UpdatePersonCount)
        # self.person_count_timer.start()

        # 위치별 카메라 관련 설정 및 이벤트처리
        self.camera_id = { 'A' : 'A', 'B': 'B', 'C': 'C' }
        self.btn_cameraA.setStyleSheet("border-radius : 30px; border : 1px solid black; font: bold 20px;")
        self.btn_cameraB.setStyleSheet("border-radius : 30px; border : 1px solid black; font: bold 20px;")
        self.btn_cameraC.setStyleSheet("border-radius : 30px; border : 1px solid black; font: bold 20px;")
        self.btn_cameraA.clicked.connect(self.ButtonClick)
        self.btn_cameraB.clicked.connect(self.ButtonClick)
        self.btn_cameraC.clicked.connect(self.ButtonClick)
        self.hideAllButtons()

        # QFrame 초기화
        self.frame_cameraA.setStyleSheet(f"background-color: {self.level_colors['Default']};")
        self.frame_cameraB.setStyleSheet(f"background-color: {self.level_colors['Default']};")
        self.frame_cameraC.setStyleSheet(f"background-color: {self.level_colors['Default']};")

        # 화면 시작 시 랜덤 버튼 표시
        self.showRandomButton() 

        # 5초마다 checkButtonClick 실행
        self.check_timer = QTimer(self)
        self.check_timer.setInterval(5000)  # 5초 간격
        self.check_timer.timeout.connect(self.checkButtonClick)
        self.check_timer.start()

        # Detection_Screen 타이머 추가
        self.detection_screen_timer = QTimer()
        self.detection_screen_timer.setInterval(500)  # 0.5초 간격
        self.detection_screen_timer.setSingleShot(True)
        self.detection_screen_timer.timeout.connect(self.showDetectionScreen)
        # Detection_Screen 인스턴스 관리 플래그
        self.detection_screen_shown = False  

    def Setup(self):
        # DB 관련 파라미터
        self.case_num = 0 
        self.person_num = 0

        # 시계 타이머 관련 위젯
        self.timer = QTimer(self)
        self.timer.setInterval(1000)    # 1초 간격    
        self.timer.timeout.connect(self.Showtime)
        self.lcdTimer.display('')
        self.lcdTimer.setDigitCount(8)
        self.timer.start()

        # 혼잡도 관련 파라미터
        self.person_threshold = 10
        self.image_paths = {
            "good" : ":/newPrefix/good.png",
            "soso" : ":/newPrefix/soso.png",
            "bad" : ":/newPrefix/bad.png",
        }
        self.person_colors = {
            "good": "#7DB249",
            "soso": "#FFCD4A",
            "bad": "#D94925"
        }

        # 위험 레벨 색상 관련 파라미터
        self.level_colors = {
            "Default" : 'transparent',
            "Red" : 'rgba(255, 0, 0, 128)',         # 위험 물품 감지
            "Yellow" : 'rgba(255, 255, 0, 128)'     # 교통 약자 감지
        }
    
    def Showtime(self):
        # 날짜
        datetime = QDateTime.currentDateTime()
        self.labelDate.setText(datetime.toString('yyyy-MM-dd'))
        # 시간
        sender = self.sender()
        currentTime = QTime.currentTime().toString("hh:mm:ss")
        if id(sender) == id(self.timer):
            self.lcdTimer.display(currentTime)
        # 색상 깜빡임
        if self.blink_enabled:
            self.BlinkFrame()
        # 날씨
        self.ShowForecast()

    def BlinkFrame(self):
        current_seconds = QTime.currentTime().second()
        if current_seconds % 2 == 0:  # 짝수 초일 때
            background_color = self.level_colors['Red']
        else:  # 홀수 초일 때
            background_color = self.level_colors['Default']
        
        if hasattr(self, 'random_button'):
            if self.random_button == self.btn_cameraA:
                self.frame_cameraA.setStyleSheet(f"background-color: {background_color}; border-radius : 30px")
            elif self.random_button == self.btn_cameraB:
                self.frame_cameraB.setStyleSheet(f"background-color: {background_color}; border-radius : 30px")
            elif self.random_button == self.btn_cameraC:
                self.frame_cameraC.setStyleSheet(f"background-color: {background_color}; border-radius : 30px")
    
    def ButtonClick(self):
        self.blink_enabled = False  # 클릭 시 깜빡임 중지
        self.hideAndStopBlinking()
    
    def checkButtonClick(self):
        if self.blink_enabled:  # 깜빡임이 활성화되어 있으면
            self.BlinkBackground()

    def BlinkBackground(self):
        current_seconds = QTime.currentTime().second()
        if current_seconds % 2 == 0:  # 짝수 초일 때
            background_color = 'rgba(255, 0, 0, 128)'
            if not self.detection_screen_shown:
                self.detection_screen_timer.start()  # 0.5초 후에 Detection_Screen을 표시
        else:  # 홀수 초일 때
            background_color = 'transparent'
        self.frame.setStyleSheet(f"background-color: {background_color};")

    def showDetectionScreen(self):
        self.detection_screen.show()
        self.estimation_screen.show()
        self.timer.stop()  # Pause main timer
        self.detection_screen_shown = True  # 플래그 업데이트
    
    def onEstimationScreenClosed(self):
        self.timer.start()  # Resume main timer
        self.stopBlinking()

    def stopBlinking(self):
        self.blink_enabled = False  # Stop blinking when the detection screen is closed
        self.frame.setStyleSheet(f"background-color: {self.level_colors['Default']};")
        self.hideAndStopBlinking()

    def hideAndStopBlinking(self):
        if hasattr(self, 'random_button'):
            self.random_button.hide()
            if self.random_button == self.btn_cameraA:
                self.frame_cameraA.setStyleSheet(f"background-color: {self.level_colors['Default']};")
            elif self.random_button == self.btn_cameraB:
                self.frame_cameraB.setStyleSheet(f"background-color: {self.level_colors['Default']};")
            elif self.random_button == self.btn_cameraC:
                self.frame_cameraC.setStyleSheet(f"background-color: {self.level_colors['Default']};")

    # def UpdatePersonCount(self):
    #     self.person_num += 1
    #     if self.person_num < int(self.person_threshold / 2):
    #         status = "good"
    #     elif int(self.person_threshold / 2) <= self.person_num < self.person_threshold:
    #         status = "soso"
    #     else:
    #         status = "bad"

    #     color = self.person_colors[status]
    #     self.labelCount.setStyleSheet(f"color: {color};")
    #     self.labelCount.setText(str(self.person_num))

    #     self.labelEmoji.setStyleSheet(f"background-image: url({self.image_paths[status]});")
    
    def hideAllButtons(self):
        self.btn_cameraA.hide()
        self.btn_cameraB.hide()
        self.btn_cameraC.hide()

    def showRandomButton(self):
        self.random_button = random.choice([self.btn_cameraA, self.btn_cameraB, self.btn_cameraC])
        self.random_button.show()

    def PoliceButtonClick(self):
        reply = QMessageBox.question(self, "Police Button", "Police button clicked! Do you want to proceed?", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            emergency_data = self.ShowEmergencyCenter()
            police_station = self.GetPoliceStation(emergency_data)
            QMessageBox.information(self, "Police Station", f"신고 접수: {police_station}")

            current_datetime = QDateTime.currentDateTime().toString('yyyy-MM-dd hh:mm:ss')
            self.case_num = self.GetNextCaseNum()
            query = f"""
            INSERT INTO copilot_cctv (Case_num, Camera_id, Person_num, Situation, Solution, Emergency_call, Date)
            VALUES ('{self.case_num}', '{self.camera_id['A']}', '{self.person_num}', '폭행발생', '긴급 출동 요청', '{police_station}', '{current_datetime}')
            """
            self.ExecuteQuery(query)

    def AmbulanceButtonClick(self):
        reply = QMessageBox.question(self, "Fire Button", "Fire button clicked! Do you want to proceed?", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            emergency_data = self.ShowEmergencyCenter()
            ambulance_station = self.GetAmbulanceStation(emergency_data)
            QMessageBox.information(self, "Fire Station", f"신고 접수: {ambulance_station}")

            current_datetime = QDateTime.currentDateTime().toString('yyyy-MM-dd hh:mm:ss')
            self.case_num = self.GetNextCaseNum()
            query = f"""
            INSERT INTO copilot_cctv (Case_num, Camera_id, Person_num, Situation, Solution, Emergency_call, Date)
            VALUES ('{self.case_num}', '{self.camera_id['A']}', '{self.person_num}', '쓰러진승객발생', '긴급 출동 요청', '{ambulance_station}', '{current_datetime}')
            """
            self.ExecuteQuery(query)

    def GetNextCaseNum(self):
        query = "SELECT IFNULL(MAX(Case_num), 0) + 1 FROM copilot_cctv"
        self.cursor.execute(query)
        result = self.cursor.fetchone()
        return result[0]

    def ExecuteQuery(self, query):
        try:
            self.cursor.execute(query)
            self.conn.commit()
            #QMessageBox.information(self, "Success", "Data inserted successfully!")
            print("Data inserted successfully!")
            print('-----------------------------------')
        except Exception as e:
            #QMessageBox.critical(self, "Error", f"An error occurred: {e}")
            print(f"An error occurred: {e}")
            print('-----------------------------------')

if __name__ == "__main__":
    #print(os.getcwd())
    db_instance = Connect("driver", "0603")

    app = QApplication(sys.argv)
    myWindows = WindowClass(db_instance)
    myWindows.show()
    app.exec_()

    # 애플리케이션 종료 시 DB 연결 종료
    db_instance.disConnection()
