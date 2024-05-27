import numpy as np
import torch
import torch.nn as nn
import cv2
from torch.utils.data import DataLoader
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score

# 하이퍼파라미터
input_size = 24
seq_length = 181
hidden_size = 64
num_layers = 2
output_size = 1
batch_size = 32

# 모델 초기화 및 불러오기
model = LSTMModel(input_size, hidden_size, num_layers, output_size)
model.load_state_dict(torch.load('lstm_model_v3.pth'))
model.eval()

# 테스트 데이터셋 및 데이터로더 생성
test_dataset = SkeletonDataset('./fall_down_test01.txt', seq_length=seq_length)
test_loader = DataLoader(dataset=test_dataset, batch_size=batch_size, shuffle=False)

# 평가 함수
def evaluate_model(model, data_loader):
    model.eval()
    all_labels = []
    all_predictions = []
    with torch.no_grad():
        for inputs, labels in data_loader:
            inputs = inputs.float()
            labels = labels.float()
            outputs = model(inputs)
            probabilities = torch.sigmoid(outputs).squeeze()
            predictions = torch.round(probabilities).cpu().numpy()
            all_labels.extend(labels.cpu().numpy())
            all_predictions.extend(predictions)
    accuracy = accuracy_score(all_labels, all_predictions)
    precision = precision_score(all_labels, all_predictions)
    recall = recall_score(all_labels, all_predictions)
    f1 = f1_score(all_labels, all_predictions)
    return accuracy, precision, recall, f1, all_predictions

# 모델 평가 및 'fall_down' 감지 결과 기록
accuracy, precision, recall, f1, all_predictions = evaluate_model(model, test_loader)
print(f'Accuracy: {accuracy:.4f}, Precision: {precision:.4f}, Recall: {recall:.4f}, F1 Score: {f1:.4f}')

with open('./fall_down__result.txt', 'w') as f:
    for prediction in all_predictions:
        f.write(f'{int(prediction)}\n')

# 비디오 초기화
video_path = './fall_down_test01.mp4'
cap = cv2.VideoCapture(video_path)

# 비디오 저장 설정
output_path = 'output_video3.avi'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(output_path, fourcc, 20.0, (int(cap.get(3)), int(cap.get(4))))

frame_idx = 0

# 'fall_down'이 감지된 경우 스켈레톤을 그리고 사각형 프레임 생성
with open('./fall_down_test01_result.txt', 'r') as f:
    lines = f.readlines()

with torch.no_grad():
    for inputs, _ in test_loader:
        inputs = inputs.float()
        outputs = model(inputs)
        probabilities = torch.sigmoid(outputs).squeeze()
        predictions = torch.round(probabilities).cpu().numpy()

        for i in range(inputs.size(0)):
            ret, frame = cap.read()
            if not ret:
                break

            prediction = predictions[i]

            if prediction == 1:
                skeleton_coords = inputs[i, :seq_length, :].cpu().numpy()

                # 중앙 좌표 계산
                center_x = np.mean(skeleton_coords[4:6, 0])
                center_y = np.mean(skeleton_coords[4:6, 1])

                # 프레임 크기 계산
                width = np.abs(skeleton_coords[4, 0] - skeleton_coords[5, 0]) * 3
                height = np.abs(skeleton_coords[4, 1] - skeleton_coords[5, 1]) * 4

                # 프레임의 좌상단 및 우하단 좌표 계산
                start_point = (int(center_x - width / 2), int(center_y - height / 2))
                end_point = (int(center_x + width / 2), int(center_y + height / 2))

                color = (0, 0, 255)  # Red color in BGR
                thickness = 2
                frame = cv2.rectangle(frame, start_point, end_point, color, thickness)

                # 'fall_down' 텍스트 표시
                text = 'fall_down'
                text_position = (start_point[0], start_point[1] - 10)
                cv2.putText(frame, text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

                # 확률 표시
                prob_text = f'{probabilities[i]:.2f}'
                prob_position = (end_point[0] - 60, start_point[1] - 10)
                cv2.putText(frame, prob_text, prob_position, cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            out.write(frame)
            frame_idx += 1

cap.release()
out.release()
cv2.destroyAllWindows()
