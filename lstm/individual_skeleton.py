import cv2
import numpy as np
import os
from ultralytics import YOLO

# YOLOv8 Pose 모델 로드
model = YOLO('yolov8s-pose.pt')

num_people = 2

# 입력 영상 경로와 출력 영상 경로 설정
input_video_path = './data/C_violate.mp4'
output_video_path = './data/violate_result.mp4'
skeleton_data_dir = './data/skeleton_data'

# 디렉토리가 없으면 생성
if not os.path.exists(skeleton_data_dir):
    os.makedirs(skeleton_data_dir)

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
prev_keypoints = np.full((17, 2), -1)

frame_idx = 0

# 텍스트 파일 초기화 (쓰기 모드로 열고 닫아 파일을 생성함)
for person_idx in range(num_people):  # 최대 20명까지 감지한다고 가정
    open(os.path.join(skeleton_data_dir, f'person_{person_idx}.txt'), 'w').close()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    #resized_frame = cv2.resize(frame, (640, 640))

    # 모델을 사용하여 프레임에서 사람의 스켈레톤 추출
    results = model(frame)

    detected_people = []

    # 스켈레톤 좌표를 화면에 출력 및 리스트에 저장
    if results:
        person_keypoints = []

        for result in results:
            keypoints = result.keypoints.cpu().numpy()  # 감지된 사람의 스켈레톤 좌표를 numpy 배열로 변환
            for keypoints_person in keypoints.xy:
                person_keypoints.append(keypoints_person)

        # 사람을 중심점 x 좌표 기준으로 정렬
        person_keypoints.sort(key=lambda k: np.mean(k[5, 0]))

        person_idx = 0
        for keypoints_person in person_keypoints:
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

            # 해당 사람의 텍스트 파일에 좌표 저장
            with open(os.path.join(skeleton_data_dir, f'person_{person_idx}.txt'), 'a') as f:
                f.write(' '.join(map(str, flattened_keypoints)) + '\n')
            print(flattened_keypoints)  # x, y 좌표만 출력
            print('....................')
            detected_people.append(person_idx)
            person_idx += 1

    # 검출되지 않은 사람들에 대해 -1로 채우기
    for person_idx in range(num_people):
        if person_idx not in detected_people:
            flattened_keypoints = np.full((12 * 2,), -1)
            with open(os.path.join(skeleton_data_dir, f'person_{person_idx}.txt'), 'a') as f:
                f.write(' '.join(map(str, flattened_keypoints)) + '\n')
            print(flattened_keypoints)  # x, y 좌표만 출력
            print('....................')

    frame_idx += 1

# 모든 작업이 끝나면 리소스 해제
cap.release()
out.release()

# 스켈레톤 데이터를 numpy 배열로 변환
skeleton_sequences = np.array(skeleton_sequences)

# 데이터 저장
np.save(os.path.join(skeleton_data_dir, 'violate.npy'), skeleton_sequences)

print(f'Skeleton data shape: {skeleton_sequences.shape}')
