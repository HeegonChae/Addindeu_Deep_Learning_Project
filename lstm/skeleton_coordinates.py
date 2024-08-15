import cv2
import numpy as np
import os
from ultralytics import YOLO

# YOLOv8 Pose 모델 로드
model = YOLO('yolov8n-pose.pt')

# 입력 영상 경로와 출력 영상 경로 설정
input_video_path = './fall_down/fall_down_train.mp4'
output_video_path = './fall_down_result/fall_down_train_result.mp4'
skeleton_data_txt_path = './fall_down_result/skeleton_fall_down_train.txt'
skeleton_data_npy_path = './fall_down_result/skeleton_fall_down_train.npy'

# 비디오 캡처 객체 생성
cap = cv2.VideoCapture(input_video_path)

# 비디오 작성 객체 생성 (프레임 크기, FPS 설정)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = int(cap.get(cv2.CAP_PROP_FPS))
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
out = cv2.VideoWriter(output_video_path, fourcc, fps, (frame_width, frame_height))

# 스켈레톤 데이터를 저장할 리스트 초기화
skeleton_sequences = []

# 이전 프레임의 키포인트 초기화
prev_keypoints = np.zeros((17, 2))

# 텍스트 파일에 스켈레톤 좌표 저장
with open(skeleton_data_txt_path, 'w') as f:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 모델을 사용하여 프레임에서 사람의 스켈레톤 추출
        results = model(frame)

        # 스켈레톤 좌표를 화면에 출력 및 리스트에 저장
        if results:
            for result in results:
                keypoints = result.keypoints.cpu().numpy()  # 감지된 사람의 스켈레톤 좌표를 numpy 배열로 변환
                for keypoints_person in keypoints.xy:
                    if keypoints_person.shape[0] < 17:  # 감지된 키포인트가 17개 미만인 경우
                        filled_keypoints = prev_keypoints.copy()  # 이전 프레임의 키포인트로 채우기
                        filled_keypoints[:keypoints_person.shape[0], :] = keypoints_person  # 감지된 키포인트 채우기
                    else:
                        filled_keypoints = keypoints_person
                    prev_keypoints = filled_keypoints  # 현재 프레임의 키포인트를 이전 프레임으로 저장

                    # 얼굴 부분(0번부터 4번까지)을 제외하고 나머지 키포인트만 사용
                    body_keypoints = np.delete(filled_keypoints, [0, 1, 2, 3, 4], axis=0)
                    flattened_keypoints = body_keypoints.flatten()  # x, y 좌표만 추출하여 일차원 배열로 변환
                    skeleton_sequences.append(flattened_keypoints)
                    f.write(' '.join(map(str, flattened_keypoints)) + '\n')  # 텍스트 파일에 좌표 저장
                    print(flattened_keypoints)  # x, y 좌표만 출력
                    print('....................')
        else:
            # 객체가 감지되지 않은 경우 이전 프레임의 키포인트 사용
            body_keypoints = np.delete(prev_keypoints, [0, 1, 2, 3, 4], axis=0)
            flattened_keypoints = body_keypoints.flatten()
            skeleton_sequences.append(flattened_keypoints)
            f.write(' '.join(map(str, flattened_keypoints)) + '\n')  # 텍스트 파일에 좌표 저장
            print(flattened_keypoints)  # x, y 좌표만 출력
            print('....................')

# 모든 작업이 끝나면 리소스 해제
cap.release()
out.release()
cv2.destroyAllWindows()

# 스켈레톤 데이터를 numpy 배열로 변환
skeleton_sequences = np.array(skeleton_sequences)

# 데이터 저장
np.save(skeleton_data_npy_path, skeleton_sequences)

print(f'Skeleton data shape: {skeleton_sequences.shape}')
