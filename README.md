# 🚍버스 운전자 보조 목적의 실시간 CCTV 모니터링 시스템🚍
![bus_cctv](https://github.com/addinedu-ros-5th/deeplearning-repo-2/assets/163790408/48c3854f-63bf-4c3e-bc36-22cb1c9a3077)


# 1. 👨‍🏫 프로젝트 소게 👨‍🏫
## 1.1 프로젝트 선정배경 📍
![bus_cctv2](https://github.com/addinedu-ros-5th/deeplearning-repo-2/assets/163790408/1e1a52d3-9463-4e2f-a236-5dc1d5c2178e)

## 1.2 프로젝트 목표 🎯 
- 이미지 인식 딥러닝 모델 설계 과정에 대한 이해 
- 자율주행 버스 내 CCTV를 활용하여, 탑승객의 행동과 위치를 실시간으로 추적하고 분석하는 딥러닝 모델 개발
- 탑승객의 안전과 편의를 향상시키기 위해 사용자 친화적인 인터페이스와 서비스를 제공하는 시스템 설계

## 1.2 프로젝트 진행과정 🏃🏻 


## 1.3 팀구성  
- 채희곤 :
- 현혜지 : 
- 이유민 : 
- 홍권호(팀장) : 

## 1.4 프로젝트 개발환경 💻


## 1.5 프로젝트 적용기술 💻



# 2. 👨🏻‍💻 이미지 데이터 및 영상 데이터 수집 👨🏻‍💻
## 2.1 이미지 데이터 정보 ℹ️
- knife , gun 이미지 각 약(5000장)
<img width="887" alt="스크린샷 2024-06-18 오후 9 04 44" src="https://github.com/addinedu-ros-5th/deeplearning-repo-2/assets/163790408/cce3dcb9-f161-401b-8703-5f586ca8947c">

- wheelchair, crutches
  



## 2.2 영상 데이터 정보 ℹ️
- 사고 대응_ 비정상 상황1(**fall_down 클래스**)

- 사고 대응_ 비정상 상황2(**violence 클래스**)


# 3 🧠딥러닝 모델학습🧠
## 3.1 모델 학습결과 
* 사고 예방_위험물품(**knife, gun model 클래스**) 
 <img width="1172" alt="스크린샷 2024-06-18 오후 9 19 24" src="https://github.com/addinedu-ros-5th/deeplearning-repo-2/assets/163790408/07470267-8bd1-4251-ba16-9c7fd0d6ea9b">

* 사고 예방_거동 불편자(**wheelchair, crutches 클래스**)
  - **YOLOv8l 모델 vs YOLOv8l-worldv2 모델**
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
  


