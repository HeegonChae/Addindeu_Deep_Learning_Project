import numpy as np
import torch
import torch.nn as nn
import cv2
from torch.utils.data import DataLoader, Dataset
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score

# 하이퍼파라미터
input_size = 24
seq_length = 20
hidden_size = 64
num_layers = 2
output_size = 2
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

model = LSTMModel(input_size, hidden_size, num_layers, output_size)
model.load_state_dict(torch.load('lstm_model_v3.pth'))
model.eval()

# 데이터셋 클래스
class SkeletonDataset(Dataset):
    def __init__(self, data_file, seq_length=100):
        self.data = np.loadtxt(data_file)
        self.seq_length = seq_length
    
    def __len__(self):
        if len(self.data) < self.seq_length:
            return 0
        return len(self.data) - self.seq_length + 1
    
    def __getitem__(self, idx):
        input_data = self.data[idx:idx + self.seq_length, :24]
        label = int(self.data[idx + self.seq_length - 1, -1])
        input_sequence = torch.tensor(input_data).float()
        return input_sequence, label

test_dataset = SkeletonDataset('./faint_test01.txt', seq_length=seq_length)
test_loader = DataLoader(dataset=test_dataset, batch_size=batch_size, shuffle=False)

# 평가 함수
def evaluate_model(model, data_loader):
    model.eval()
    all_labels = []
    all_predictions = []
    with torch.no_grad():
        for inputs, labels in data_loader:
            inputs = inputs.float()
            labels = labels.long()
            outputs = model(inputs)
            probabilities = nn.Softmax(dim=1)(outputs)
            _, predictions = torch.max(probabilities, 1)
            all_labels.extend(labels.cpu().numpy())
            all_predictions.extend(predictions.cpu().numpy())
            print(f"Labels: {labels.cpu().numpy()}, Predictions: {predictions.cpu().numpy()}")  # 디버그 출력 추가
    accuracy = accuracy_score(all_labels, all_predictions)
    precision = precision_score(all_labels, all_predictions, average='weighted', zero_division=1)
    recall = recall_score(all_labels, all_predictions, average='weighted', zero_division=1)
    f1 = f1_score(all_labels, all_predictions, average='weighted', zero_division=1)
    return accuracy, precision, recall, f1, all_predictions

# 모델 평가 및 'fall_down' 감지 결과 기록
accuracy, precision, recall, f1, all_predictions = evaluate_model(model, test_loader)
print(f'Accuracy: {accuracy:.4f}, Precision: {precision:.4f}, Recall: {recall:.4f}, F1 Score: {f1:.4f}')

with open('./faint_result.txt', 'w') as f:
    for prediction in all_predictions:
        f.write(f'{int(prediction)}\n')

# 비디오 초기화
video_path = './faint_test01.mp4'
cap = cv2.VideoCapture(video_path)

# 비디오 저장 설정
output_path = './output_video5.avi'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(output_path, fourcc, 20.0, (int(cap.get(3)), int(cap.get(4))))

frame_idx = 0

# 'fall_down' 감지 결과 읽기
with open('./faint_result.txt', 'r') as f:
    lines = f.readlines()

with torch.no_grad():
    for inputs, _ in test_loader:
        inputs = inputs.float()
        outputs = model(inputs)
        probabilities = nn.Softmax(dim=1)(outputs)
        _, predictions = torch.max(probabilities, 1)

        for i in range(inputs.size(0)):
            ret, frame = cap.read()
            if not ret:
                break

            prediction = predictions[i]

            # 스켈레톤 좌표 계산
            skeleton_coords = inputs[i, :seq_length, :].cpu().numpy()

            # 중앙 좌표 계산
            center_x = np.mean(skeleton_coords[:, 0])
            center_y = np.mean(skeleton_coords[:, 1])

            # 프레임 크기 계산
            width = np.abs(np.max(skeleton_coords[:, 0]) - np.min(skeleton_coords[:, 0]))
            height = np.abs(np.max(skeleton_coords[:, 1]) - np.min(skeleton_coords[:, 1]))

            # 프레임의 좌상단 및 우하단 좌표 계산
            start_point = (int(center_x - width / 2), int(center_y - height / 2))
            end_point = (int(center_x + width / 2), int(center_y + height / 2))

            color = (0, 0, 255) if prediction == 1 else (0, 255, 0)  # Red for 'fall_down', Green for 'normal'
            thickness = 2
            frame = cv2.rectangle(frame, start_point, end_point, color, thickness)

            # 'fall_down' 또는 'normal' 텍스트 표시
            text = 'fall_down' if prediction == 1 else 'normal'
            text_position = (start_point[0], start_point[1] - 10)
            cv2.putText(frame, text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            # 확률 표시
            prob_text = f'{probabilities[i, 1]:.2f}' if prediction == 1 else f'{probabilities[i, 0]:.2f}'
            prob_position = (end_point[0] - 60, start_point[1] - 10)
            cv2.putText(frame, prob_text, prob_position, cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            out.write(frame)
            frame_idx += 1

cap.release()
out.release()
cv2.destroyAllWindows()
