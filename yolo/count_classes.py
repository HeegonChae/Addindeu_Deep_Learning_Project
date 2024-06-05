import os

# 디렉터리 경로 설정
directory = 'diretory'

# 맨 앞 숫자가 '0'인 파일과 '2'인 파일의 개수를 세는 변수 초기화
count_0 = 0
count_1 = 0
count_2 = 0
count_none = []
# 디렉터리 내의 모든 파일에 대해 반복
for filename in os.listdir(directory):
    if filename.endswith('.txt'):
        file_path = os.path.join(directory, filename)
        with open(file_path, 'r') as file:
            first_char = file.read(1)  # 파일의 첫 번째 문자를 읽음
            if first_char == '0':
                count_0 += 1
            elif first_char == '2':
                count_2 += 1
            elif first_char == '1':
                count_1 += 1
            else:
                count_none.append(first_char)


# 결과 출력
print(f"맨 앞 숫자가 '0'인 파일 개수: {count_0}")
print(f"맨 앞 숫자가 '1'인 파일 개수: {count_1}")
print(f"맨 앞 숫자가 '2'인 파일 개수: {count_2}")
print(f"이상한 파일 개수: {count_none}")
