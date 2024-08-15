# 🚍버스 운전자 보조 목적 실시간 CCTV 모니터링 시스템
  <div align=center> 
      <br/>
      <img src="https://github.com/addinedu-ros-5th/deeplearning-repo-2/assets/163790408/48c3854f-63bf-4c3e-bc36-22cb1c9a3077" width ="400">
  </div>
  (기간: 2024년 05월 27일 ~ 2024년 06월 13일)
    
# 1. 👨‍🏫 프로젝트 소개 👨‍🏫
## 1.1 프로젝트 선정배경 📍
  <div align=center> 
      <br/>
      <img src="https://github.com/addinedu-ros-5th/deeplearning-repo-2/assets/163790408/1e1a52d3-9463-4e2f-a236-5dc1d5c2178e" width ="400">
  </div>
      
## 1.2 프로젝트 목표 🎯 
- 이미지 인식 딥러닝 모델 설계 과정에 대한 이해 
- 자율주행 버스 내 CCTV를 활용하여, 탑승객의 행동과 위치를 실시간으로 추적하고 분석하는 딥러닝 모델 개발
- 탑승객의 안전과 편의를 향상시키기 위해 사용자 친화적인 인터페이스와 서비스를 제공하는 시스템 설계

## 1.3 기술스택
  <div align=center>
    
   |**Category**|**Details**|
  |:----------:|:----------:|
  |**개발환경**|<img src="https://img.shields.io/badge/Ubuntu22.04-E95420?style=for-the-badge&logo=Ubuntu22.04&logoColor=white"> <img src="https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=Linux&logoColor=white"> |
  |**소프트웨어 프로그램**|<img src="https://img.shields.io/badge/python-3776AB?style=for-the-badge&logo=python&logoColor=white">|
  |**딥러닝 및 영상처리**|<img src="https://img.shields.io/badge/YOLO-512BD4?style=for-the-badge&logoColor=white"> <img src="https://img.shields.io/badge/LSTM-ECD53F?style=for-the-badge&logoColor=white">|
  |**데이터베이스 관리 시스템**|<img src="https://img.shields.io/badge/mysql-4479A1?style=for-the-badge&logo=mysql&logoColor=white"> <img src="https://img.shields.io/badge/Amazon RDS-527FFF?style=for-the-badge&logo=Amazon RDS&logoColor=white">|
  |**GUI**|<img src="https://img.shields.io/badge/PyQt5-41CD52?style=for-the-badge&logo=Qt&logoColor=white">|
  |**협업 툴**|<img src="https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=GitHub&logoColor=white"> <img src="https://img.shields.io/badge/Git-F05032?style=for-the-badge&logo=Git&logoColor=white"> <img src="https://img.shields.io/badge/Confluence-172B4D?style=for-the-badge&logo=Confluence&logoColor=white"> <br> <img src="https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=Jira&logoColor=white"> <img src="https://img.shields.io/badge/Slack-4A154B?style=for-the-badge&logo=Slack&logoColor=white"> <img src="https://img.shields.io/badge/Figma-F24E1E?style=for-the-badge&logo=Figma&logoColor=white"> |
</div> 

## 1.4 팀구성  
  <div align=center> 
  
|**구분**|**이름**|**역할**|
|:------:|:------:|:-------|
|팀장|홍권호|<ul><li>Image Classification 모델 학습</li><li> ROS2 통신 </li></ul>|
|팀원|채희곤|<ul><li> Image Classification 모델 학습</li><li> GUI 설계 및 디자인 </li><li> 데이터베이스 구축  관리 </li></ul>|
|팀원|현혜지|<ul><li> Action Recognition 모델 학습 </li><li> ROS2 통신 </li></ul>|
|팀원|이유민|<ul><li> Action Recognition 모델 학습 </li></ul>|
</div> 

# 2. 👨🏻‍💻 프로젝트 설계 👨🏻‍💻
## 2.1 기능리스트
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/3aad47e1-d3fe-4894-a38b-3ba50b76efe2" width ="600">
      <img src="https://github.com/user-attachments/assets/b68e03e3-0abd-4949-a0b2-1bb3ef51a952" width ="600">
  </div>
  
## 2.2 HW 구성도
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/358b24b4-f95b-4b21-8763-497c9a92f4be" width ="600">
  </div>
  
## 2.2 SW 구성도
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/c1cfcbed-1ac8-4847-a941-5f716838b877" width ="600">
  </div>
  
## 2.3 시나리오 구성
#### 1) 사고예방 - 거동 불편자 승객 감지 시나리오
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/36a7fa4f-eccf-4322-9288-5ad364ad47d6" width ="600">
  </div>

#### 2) 사고대응 - 쓰러진 승객 감지 시나리오
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/904ed562-00b4-493f-a7bc-ac14c31c71d9" width ="600">
  </div>

# 3. 👨🏻‍💻 이미지 및 영상 데이터 수집 & 전처리 👨🏻‍💻
## 3.1.1 이미지 데이터 정보 ℹ️
- knife , gun 이미지 각 약 **5000장**
  <div align=center> 
      <br/>
      <img src="https://github.com/addinedu-ros-5th/deeplearning-repo-2/assets/163790408/cce3dcb9-f161-401b-8703-5f586ca8947c" width ="800">
  </div>
  
- wheelchair, crutches 각 약 **5000장**
  <div align=center> 
    <img src="https://github.com/user-attachments/assets/7c2b0bf3-7dcb-440e-8fab-d48454260971" width ="800">
  </div>
## 3.1.2 이미지 데이터 전처리 ℹ️
- 과적합 방지 해결방안(1) **타겟 클래스 Segmentation 라벨링**
  - 'knife' 클래스 
    <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/f78e1d47-0112-4022-9788-805b7b71b25f" width ="800">
  </div>

 - 과적합 방지 해결방안(2) **다양한 Background Image 추가**
   - ex) 우산, 소화기, 선풍기 등
    <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/c04d4ee8-0b4e-42ab-8785-14eb264f1a94" width ="800">
  </div>

## 3.2.1 영상 데이터 정보 ℹ️
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/abc6ef1a-d74d-448e-ba1b-6806e63ff021" width ="800">
  </div>

### 1분 이내 촬영 영상
- 사고 대응_ 비정상 상황1(**fall_down 클래스**)
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/4c311293-6371-46e9-812b-9d07808c2f39" width ="600">
  </div>
  
- 사고 대응_ 비정상 상황2(**violence 클래스**)
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/5ad222b9-74fb-4466-9f2a-e7fd64d76b89" width ="600">
  </div>
## 3.2.2 영상 데이터 전처리 ℹ️
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/b2110795-b16b-4486-849c-a13f45111535" width ="800">
  </div>

# 4 🧠딥러닝 모델학습🧠
## 4.1 학습결과 
### Train: 4000장 & Valid: 1000장 
* 사고 예방_위험물품(**knife, gun model 클래스**)
  - **YOLOv8m 모델**
    - Results
 <img width="1172" alt="스크린샷 2024-06-18 오후 9 19 24" src="https://github.com/addinedu-ros-5th/deeplearning-repo-2/assets/163790408/07470267-8bd1-4251-ba16-9c7fd0d6ea9b">

* 사고 예방_거동 불편자(**wheelchair, crutches 클래스**)
  - **YOLOv8l-worldv2 모델**
    - Confusion Matrix(Normalized)
      <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/92e5c0cd-5bae-4a3d-a55d-4cfceb1f2786" width ="700">
      
        (1) YOLOv8l 학습 결과 <br>
            [ 2-Classes ->  3-Classes(**stick_user, person_no, wheelchair_user**) ]
      </div>
      <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/14752f67-efc1-49ff-aeee-51da4d3009a2" width ="700">
        
        (2) YOLOv8l-worldv2 학습 결과 <br>
            [ 2-Classes ->  5-Classes(**person-crutches, person-no, person-rollator, person-stick, person-wheelchair**) ]
      </div>

    - Results
      <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/f90ada09-1bab-45f9-8cfa-5c7fc8b256fb" width ="700">
      </div>
      
* 사고 대응_쓰러진 승객 감지(**fall down 클래스**)
  - **YOLOv8n-pose & LSTM**
    - Results
      <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/5caf3fe8-7ac5-4123-90f1-6d585d342d3b" width ="500">
      </div>
      
* 사고 대응_폭력 발생 감지(**violate 클래스**)
  - **YOLOv8n-pose & LSTM**
    - Results
      <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/d43d3074-fbfa-48cf-b58d-5d8d90a83912" width ="500">
      </div>

# 5 🔎기능 구현🔎
## 5.1 거동불편자 감지
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/9f67c1cc-68a4-4c8c-a7f2-9fc9f30c42d1" width ="800">
  </div>
  
## 5.2 위험물 및 폭력 발생 감지  
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/992f42db-ea2c-45ad-a07c-46ea09599b0d" width ="800">
  </div>
  
## 5.３ 쓰러진 승객 감지
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/072988c2-fc27-4150-aaed-45d2316a4a75" width ="800">
  </div>

## 5.4 혼잡도 계산 및 긴급 신고
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/ac549f99-02de-4c98-90c9-159597ccc4bc" width ="450">
      <img src="https://github.com/user-attachments/assets/6984c1d4-d150-452d-a89b-bc6e46a33794" width ="450">
  </div>
