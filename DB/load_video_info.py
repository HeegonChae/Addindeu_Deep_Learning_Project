import os
import cv2
import numpy as np
from Connect import Connect

base_dir = './data' # 원본 영상 경로
video_dir = './data/lstm' # json 경로

directories = [d for d in os.listdir(video_dir) if os.path.isdir(os.path.join(video_dir, d))]

latest_directory = max(directories, key=lambda d: int(d.replace('input', '')))
latest_dir_path = os.path.join(video_dir, latest_directory)
print(latest_directory)

# 편집 완료된 영상 경로
result_video_dir = './data/output_video/'
os.makedirs(result_video_dir, exist_ok=True)

# 스켈레톤 연결 정의 (관절 인덱스)
SKELETON_CONNECTIONS = [
    (1, 3), (3, 5),  # 왼팔
    (0, 2), (2, 4),  # 오른팔
    (0, 1), (1, 7), (6, 7), (6, 0),  # 몸통
    (7, 9), (9, 11),  # 왼쪽 다리
    (6, 8), (8, 10)  # 오른쪽 다리
]

class CutVideo():
    def __init__(self, db_instance):
        self.cursor = db_instance.cursor

    # 데이터베이스에서 테이블 정보를 가져오는 함수 정의
    def fetchVideoDataQuery(self, query):
        self.cursor.execute(query)
        return self.cursor.fetchall()

    # 특정 파일 이름의 데이터를 가져오는 함수 정의
    def fetchVideoDataByFilename(self, filename):
        query = f"SELECT Filename, width, height, Action_name, Start_frame, End_frame FROM video_json WHERE Filename='{filename}'"
        return self.fetchVideoDataQuery(query)

    def loadDataFromDB(self, filename):
        video_data = self.fetchVideoDataByFilename(filename)
        video_dict = {}
        for filename, width, height, action_name, start, end in video_data:
            if filename not in video_dict:
                video_dict[filename] = []
            video_dict[filename].append((width, height, action_name, start, end))
        return video_dict
    
    def process_videos(self, video_dict):
        for filename, data in video_dict.items():
            video_path = os.path.join(base_dir, filename)
            cap = cv2.VideoCapture(video_path)
            if not cap.isOpened():
                print(f"Error opening video file {filename}")
                continue
            
            for width, height, action_name, start, end in data:
                cap.set(cv2.CAP_PROP_POS_FRAMES, start)
                output_filename = os.path.join(result_img_dir, f"{os.path.splitext(filename)[0]}_{start}_{end}.mp4")
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                out = cv2.VideoWriter(output_filename, fourcc, 30.0, (width, height))

                # 스켈레톤 데이터 파일 경로 설정
                skeleton_files = [os.path.join(latest_dir_path, f"person_{i}.txt") for i in range(2)]
                skeleton_data = []
                for skel_file in skeleton_files:
                    if os.path.exists(skel_file):
                        # 해당 프레임의 스켈레톤 데이터만 읽어오기
                        start_frame_idx = int(start)
                        end_frame_idx = int(end)
                        skeleton_frame_data = np.loadtxt(skel_file)[start_frame_idx:end_frame_idx + 1]
                        skeleton_data.append(skeleton_frame_data)
                    else:
                        print(f"Skeleton file {skel_file} not found")
                        skeleton_data.append(np.empty((0, 24)))

                for frame_num in range(start, end + 1):
                    ret, frame = cap.read()
                    if not ret:
                        break
                    
                    resized_frame = cv2.resize(frame, (width, height))
                    cv2.putText(resized_frame, action_name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                    for person_idx, skel_data in enumerate(skeleton_data):
                        if frame_num - start < skel_data.shape[0]:
                            skeleton_points = skel_data[frame_num - start].reshape(-1, 2)
                            for (start_pt, end_pt) in SKELETON_CONNECTIONS:
                                if start_pt < len(skeleton_points) and end_pt < len(skeleton_points):
                                    x1, y1 = int(skeleton_points[start_pt][0]), int(skeleton_points[start_pt][1])
                                    x2, y2 = int(skeleton_points[end_pt][0]), int(skeleton_points[end_pt][1])
                                    if x1 != 0 and y1 != 0 and x2 != 0 and y2 != 0:
                                        cv2.line(resized_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

                            # 각 관절을 원으로 표시
                            for idx, (x, y) in enumerate(skeleton_points):
                                if x != 0 and y != 0:
                                    cv2.circle(resized_frame, (int(x), int(y)), 5, (0, 0, 255), -1)

                    out.write(resized_frame)
                
                out.release()
            cap.release()

if __name__ == "__main__":
    db_instance = Connect("driver", "0603")
    cut_video = CutVideo(db_instance)

    # 모든 파일 이름을 가져오고, 마지막 파일 이름을 추출
    query = "SELECT Filename FROM video_json ORDER BY Case_num DESC LIMIT 1"
    filenames = cut_video.fetchVideoDataQuery(query)
    if filenames:
        last_filename = filenames[-1][0]
        video_dict = cut_video.loadDataFromDB(last_filename)
        cut_video.process_videos(video_dict)
    
    db_instance.disConnection()
