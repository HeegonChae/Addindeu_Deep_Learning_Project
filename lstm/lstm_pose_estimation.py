import cv2
import numpy as np
import os
from ultralytics import YOLO
import json
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset
from datetime import datetime

# YOLOv8 Pose 모델 로드
model = YOLO('yolov8s-pose.pt')

num_people = 2

# 입력 영상 경로와 출력 영상 경로 설정
input_video_path = './data/violate.MOV'
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
        person_keypoints.sort(key=lambda k: np.mean(k[:, 0]))

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
np.save(os.path.join(skeleton_data_dir, 'fall_down2.npy'), skeleton_sequences)

print(f'Skeleton data shape: {skeleton_sequences.shape}')

# 하이퍼파라미터
input_size = 24
seq_length = 30
hidden_size = 64
num_layers = 2
output_size = 3
batch_size = 32

# 모델 정의
class LSTMModel(nn.Module):
    def __init__(self, input_size, hidden_size, num_layers, output_size):
        super(LSTMModel, self).__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)
    
    def forward(self, x):
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        out, _ = self.lstm(x, (h0, c0))
        out = self.fc(out[:, -1, :])
        return out

# 모델 로드
model = LSTMModel(input_size, hidden_size, num_layers, output_size)
model.load_state_dict(torch.load('lstm_model_pen_V2.pth'))
model.eval()

# 데이터셋 클래스
class SkeletonDataset(Dataset):
    def __init__(self, data_file, seq_length=20):
        self.data = np.loadtxt(data_file)
        self.seq_length = seq_length
    
    def __len__(self):
        if len(self.data) < self.seq_length:
            return 0
        return len(self.data) - self.seq_length + 1
    
    def __getitem__(self, idx):
        input_data = self.data[idx:idx + self.seq_length, :24]
        label = 0  # For this example, we're not using labels
        input_sequence = torch.tensor(input_data).float()
        return input_sequence, label

def process_individual_file(file_path):
    dataset = SkeletonDataset(file_path, seq_length=seq_length)
    loader = DataLoader(dataset=dataset, batch_size=batch_size, shuffle=False)

    all_predictions = []
    with torch.no_grad():
        for inputs, _ in loader:
            inputs = inputs.float()
            outputs = model(inputs)
            probabilities = nn.Softmax(dim=1)(outputs)
            _, predictions = torch.max(probabilities, 1)
            all_predictions.extend(predictions.cpu().numpy())
    
    return all_predictions

# 20개의 텍스트 파일 처리
skeleton_data_dir = './data/skeleton_data'
all_predictions = []
for person_idx in range(num_people):
    file_path = os.path.join(skeleton_data_dir, f'person_{person_idx}.txt')
    if os.path.exists(file_path):
        predictions = process_individual_file(file_path)
        all_predictions.append(predictions)
    else:
        all_predictions.append([])  # Empty list for missing files

# 비디오 초기화
video_path = './data/violate.MOV'
cap = cv2.VideoCapture(video_path)
fps = cap.get(cv2.CAP_PROP_FPS)

# 비디오 파일 이름에서 이름 추출
video_name = os.path.splitext(os.path.basename(video_path))[0]

# 비디오 저장 설정
output_path = f'./output_{video_name}.avi'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(output_path, fourcc, fps, (int(cap.get(3)), int(cap.get(4))))

frame_idx = 0
prev_predictions = [None] * num_people
frame_count = [0] * num_people
stable_predictions = [None] * num_people
change_points = []

with torch.no_grad():
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        for person_idx in range(num_people):
            if frame_idx < len(all_predictions[person_idx]):
                prediction = all_predictions[person_idx][frame_idx]
                
                if prev_predictions[person_idx] is None:
                    prev_predictions[person_idx] = prediction
                    stable_predictions[person_idx] = prediction
                    frame_count[person_idx] = 1
                elif prediction == prev_predictions[person_idx]:
                    frame_count[person_idx] += 1
                    if frame_count[person_idx] >= 15:
                        if stable_predictions[person_idx] != prediction:
                            stable_predictions[person_idx] = prediction
                            change_points.append(frame_idx)
                else:
                    prev_predictions[person_idx] = prediction
                    frame_count[person_idx] = 1

                color = (0, 0, 255) if stable_predictions[person_idx] == 1 or stable_predictions[person_idx] == 2 else (0, 255, 0)  # Red for 'fall_down' or 'violation', Green for 'normal'
                thickness = 2

                # Assuming the skeleton coordinates are stored somewhere for visualization
                # Here, we are simply displaying the prediction result as text
                text = 'fall_down' if stable_predictions[person_idx] == 1 else 'violation' if stable_predictions[person_idx] == 2 else 'normal'
                text_position = (50, 50 + person_idx * 30)  # Adjust position for each person
                cv2.putText(frame, text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        out.write(frame)
        frame_idx += 1

# 현재 날짜와 시간 가져오기
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")

# 폴더 경로 설정
folder_path = f"./data/lstm/{current_time}"
os.makedirs(folder_path, exist_ok=True)

# Ensure end_frame is updated for the last event
if len(change_points) > 0:
    change_points.append(frame_idx)

# JSON 파일 생성
video_info = {
    "video_info": {
        "folder": folder_path,
        "video_name": f'C_{video_name}.avi',
        "json_name": f'C_{video_name}.json',
        "txt_name": f'C_{video_name}.txt',
    },
    "size": {
        "width": int(cap.get(3)),
        "height": int(cap.get(4)),
        "depth": 3
    },
    "header": {
        "duration": f"{int(cap.get(cv2.CAP_PROP_FRAME_COUNT) / fps)}",
        "fps": fps,
        "frames": int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    },
    "event": [],
    "action": []
}

# 이벤트 및 액션 추가
for i in range(len(change_points) - 1):
    start_frame = change_points[i]
    end_frame = change_points[i + 1]  # Get the end frame from the next change point
    
    # Check if any of the persons predicted 'violate' or 'fall_down' in this time window
    event_persons = []
    for j in range(num_people):
        if all_predictions[j][start_frame] in [1, 2]:
            event_persons.append(j)
    
    if event_persons:
        event_name = 'fall_down' if all_predictions[event_persons[0]][start_frame] == 1 else 'violation'
        event_description = f"Person {event_persons[0]} {event_name}"
        
        video_info["event"].append({
            "eventname": event_name,
            "starttime": f"{start_frame / fps}",
            "duration": f"{(end_frame - start_frame) / fps}"  # Calculate the duration based on start and end frames
        })
        video_info["action"].append({
            "actionname": event_name,
            "frame": {"start": start_frame, "end": end_frame},
        })



# JSON 파일 저장
output_json_path = os.path.join(folder_path, f'{video_name}.json')
with open(output_json_path, 'w', encoding='utf-8') as json_file:
    json.dump(video_info, json_file, ensure_ascii=False, indent=4)

cap.release()
out.release()
print(f"JSON 파일이 {output_json_path}에 저장되었습니다.")
