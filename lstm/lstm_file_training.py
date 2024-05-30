import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader

# 여러 파일로부터 데이터를 로드하여 하나의 데이터셋으로 합치는 클래스 정의
class SkeletonDataset(Dataset):
    def __init__(self, data_files, seq_length=150):
        all_data = []
        for file in data_files:
            data = np.loadtxt(file)
            all_data.append(data)
        self.data = np.vstack(all_data)
        self.seq_length = seq_length
    
    def __len__(self):
        # 총 데이터 길이에서 시퀀스 길이를 뺀 후 1을 더한 값을 반환
        if len(self.data) < self.seq_length:
            return 0
        return len(self.data) - self.seq_length + 1
    
    def __getitem__(self, idx):
        input_data = self.data[idx:idx + self.seq_length, :24]  # 100개 프레임의 x, y 좌표
        label = int(self.data[idx + self.seq_length - 1, -1])

        input_sequence = torch.tensor(input_data).float()
        
        return input_sequence, label

# LSTM 모델 정의
class LSTMModel(nn.Module):
    def __init__(self, input_size, hidden_size, num_layers, output_size):
        super(LSTMModel, self).__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)
    
    def forward(self, x):
        # 입력 데이터의 배치 크기 가져오기
        batch_size = x.size(0)
        
        # 초기 은닉 상태와 셀 상태 초기화
        h0 = torch.zeros(self.num_layers, batch_size, self.hidden_size).to(x.device)
        c0 = torch.zeros(self.num_layers, batch_size, self.hidden_size).to(x.device)
        
        # LSTM 모델 실행
        out, _ = self.lstm(x, (h0, c0))
        
        # 마지막 시퀀스의 출력만 사용하여 fully connected 레이어에 전달
        out = self.fc(out[:, -1, :])
        return out

# 수정된 하이퍼파라미터
input_size = 24  # x, y 좌표가 각각 12개씩 총 24개
seq_length = 150  # 100개 프레임
hidden_size = 64
num_layers = 2
output_size = 2
num_epochs = 25
batch_size = 32
learning_rate = 0.001

# 데이터 파일 목록 생성
data_files = []
for i in range(2):
    result_folder = f'./pose/{i}/result'
    for file in os.listdir(result_folder):
        if file.endswith('.txt'):
            data_files.append(os.path.join(result_folder, file))

# 데이터셋 및 데이터로더 생성
train_dataset = SkeletonDataset(data_files, seq_length=seq_length)
train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True)

# 모델 초기화
model = LSTMModel(input_size, hidden_size, num_layers, output_size)

# 손실 함수 및 옵티마이저 정의
criterion = nn.CrossEntropyLoss()  # 이진 교차 엔트로피 손실
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

# 모델 학습
for epoch in range(num_epochs):
    print(f'Starting epoch {epoch + 1}/{num_epochs}')
    for inputs, labels in train_loader:
        inputs = inputs.float()
        labels = labels.long()
        
        # Forward pass
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        
        # Backward pass 및 경사 하강
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
    print(f'Epoch [{epoch + 1}/{num_epochs}], Loss: {loss.item():.4f}')

# 학습된 모델 저장
torch.save(model.state_dict(), 'lstm_model_v3.pth')
