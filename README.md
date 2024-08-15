# 🚍버스 운전자 보조 목적의 실시간 CCTV 모니터링 시스템🚍
  <div align=center> 
      <br/>
      <img src="https://github.com/addinedu-ros-5th/deeplearning-repo-2/assets/163790408/48c3854f-63bf-4c3e-bc36-22cb1c9a3077" width ="400">
  </div>
      
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

## 1.2 프로젝트 진행과정 🏃🏻 


## 1.3 팀구성  
- 채희곤 :
- 현혜지 : 
- 이유민 : 
- 홍권호(팀장) : 

## 1.4 프로젝트 개발환경 💻


## 1.5 프로젝트 적용기술 💻



# 2. 👨🏻‍💻 이미지 및 영상 데이터 수집 & 전처리 👨🏻‍💻
## 2.1.1 이미지 데이터 정보 ℹ️
- knife , gun 이미지 각 약 **5000장**
  <div align=center> 
      <br/>
      <img src="https://github.com/addinedu-ros-5th/deeplearning-repo-2/assets/163790408/cce3dcb9-f161-401b-8703-5f586ca8947c" width ="800">
  </div>
  
- wheelchair, crutches 각 약 **5000장**
  <div align=center> 
    <img src="https://github.com/user-attachments/assets/7c2b0bf3-7dcb-440e-8fab-d48454260971" width ="800">
  </div>
## 2.1.2 이미지 데이터 전처리 ℹ️
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

## 2.2.1 영상 데이터 정보 ℹ️
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
## 2.2.2 영상 데이터 전처리 ℹ️
  <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/b2110795-b16b-4486-849c-a13f45111535" width ="800">
  </div>

# 3 🧠딥러닝 모델학습🧠
## 3.1 학습결과 
### Train: 4000장 & Valid: 1000장 
* 사고 예방_위험물품(**knife, gun model 클래스**)
  - **YOLOv8m 모델**
    - Results
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

    - Results
      <div align=center> 
      <br/>
      <img src="https://github.com/user-attachments/assets/f90ada09-1bab-45f9-8cfa-5c7fc8b256fb" width ="700">
      </div>
## 3.2 추론결과 
