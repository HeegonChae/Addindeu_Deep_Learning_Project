import os

# 경로
label_dir = 'your own directory'

# 모든 txt 파일에 대해 작업 수행
for filename in os.listdir(label_dir):
    if filename.endswith('.txt'):
        file_path = os.path.join(label_dir, filename)
        
        # 파일 읽기
        with open(file_path, 'r') as file:
            lines = file.readlines()
        
        # 첫 번째 숫자를 3에서 0으로 변경
        new_lines = []
        for line in lines:
            parts = line.split()
            if parts[0] == '0':
                parts[0] = '1'
            new_lines.append(' '.join(parts))
            
        # 파일 쓰기
        with open(file_path, 'w') as file:
            file.write('\n'.join(new_lines))

print("Files have been updated.")

