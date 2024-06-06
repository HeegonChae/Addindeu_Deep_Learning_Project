# -*- coding: utf-8 -*-
"""YOLOv8_detect_Test.ipynb
모델 추론 파트
참고) 커스텀 데이터셋 학습한 모델에 model.set_classes로 지정한 가중치('~best_set_classes.pt') 불러오기

Original file is located at
    https://colab.research.google.com/drive/1VrhfXf7pn6xAXLZt2Tv_5EgaJK_n6Nvr

설치사항) 
pip install ultralytics
"""

import cv2
import json
import numpy as np
from ultralytics import YOLO
import os
import locale
import subprocess

# YOLO 모델 로드
model = YOLO("./src/yolov8l-worldv2_best_set_classes.pt")

# 클래스 별 색상 정의
class_colors = {
    'stick_user': (0, 0, 255),    # Red
    'person_no':      (255, 0, 0),   # Blue
    'wheelchair_user': (255, 0, 255) # Pink
}

def filter_boxes(results):
    filtered_results = []
    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        labels = result.boxes.cls.cpu().numpy()
        names = result.names

        unique_labels = np.unique(labels)
        best_boxes = []
        for label in unique_labels:
            label_mask = labels == label
            label_boxes = boxes[label_mask]
            label_scores = scores[label_mask]
            max_index = np.argmax(label_scores)
            best_boxes.append((label_boxes[max_index], label_scores[max_index], label, names))

        filtered_results.append(best_boxes)
    return filtered_results

def draw_boxes(filtered_results, image):
    for result in filtered_results:
        for box, score, label, names in result:
            x1, y1, x2, y2 = map(int, box)
            class_name = names[int(label)]
            if class_name in class_colors:
                color = class_colors[class_name]
                label_text = f"{class_name} {score:.2f}"
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(image, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

def save_json(filename, detected_objects):
    data = {
        "image": {
            "filename": filename,
            "detected_objects": detected_objects
        }
    }
    json_path = filename.replace('.jpg', '.json')
    with open(json_path, 'w') as json_file:
        json.dump(data, json_file, indent=4)

if __name__ == "__main__":
    # [옵션] GPU 확인
    gpu_info = subprocess.check_output(["nvidia-smi"]).decode("utf-8")
    locale.getpreferredencoding = lambda: "UTF-8"
    if gpu_info.find('failed') >= 0:
        print('Not connected to a GPU')
    else:
        print(gpu_info)

    # 현재 경로 확인
    # print(os.getcwd())                # /home/edu/dev_ws/Project3

    # 입출력 동영상 파일 경로
    video_path = './data/bus_wheelchair_crutch.mp4'
    output_video_path = './data/predicted_bus_wheelchair_crutch2.mp4'

    # 검출된 프레임 저장 'output_frames' 디렉토리를 생성
    output_frames_dir = output_video_path.replace('.mp4', '') + '/output_frames'
    os.makedirs(output_frames_dir, exist_ok=True)

    # 검출된 json 저장 'output_frames' 디렉토리를 생성
    output_json_dir = './data/detection'
    os.makedirs(output_json_dir, exist_ok=True)

    # 동영상 캡처 객체 생성
    cap = cv2.VideoCapture(video_path)
    frame_width = 640
    frame_height = 640
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    # 동영상 파일 저장 객체 생성
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video_path, fourcc, fps, (frame_width, frame_height))

    frame_count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (640, 640))
        results = model.predict(frame, save=False, conf=0.5)
        filtered_results = filter_boxes(results)

        detected_objects = {}
        for result in filtered_results:
            for box, score, label, names in result:
                class_name = names[int(label)]
                if class_name in ['stick_user', 'person_no', 'wheelchair_user']:
                    if class_name not in detected_objects:
                        detected_objects[class_name] = {}
                    bbox_id = f"id{len(detected_objects[class_name]) + 1}"
                    detected_objects[class_name][bbox_id] = [int(box[0]), int(box[1]), int(box[2]), int(box[3])]

        if detected_objects:
            frame_filename = os.path.join(output_frames_dir, f"frame{frame_count:04d}.jpg")
            json_frame_filename = os.path.join(output_json_dir, f"frame{frame_count:04d}.jpg")
            cv2.imwrite(frame_filename, frame)
            save_json(json_frame_filename, detected_objects)

        draw_boxes(filtered_results, frame)
        out.write(frame)
        frame_count += 1

    cap.release()
    out.release()
    cv2.destroyAllWindows()

    print(f"Processed video saved to {output_video_path}")
    print(f"Processed frames saved to {output_frames_dir}")