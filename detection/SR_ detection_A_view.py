import cv2
import json
import numpy as np
from ultralytics import YOLO
import os
import locale
import subprocess
import time

#import FilePublisher
#import json_data_to_DB


# YOLO 모델 로드
wheelchair_model_path = "/home/gwonho/dev_ws/dev_ws/DeepLearing_project/model/yolov8l-worldv2_best_set_classes.pt"
wheelchair_model = YOLO(wheelchair_model_path)

# 클래스 별 색상 정의
class_colors = {
    'stick_user': (0, 0, 255),    # Red
    'wheelchair_user': (255, 0, 255) # Pink
}

# 상태를 추적하기 위한 변수
detected_class_counters = {'stick_user': 0, 'wheelchair_user': 0}
skip_counters = {'stick_user': 0, 'wheelchair_user': 0}
save_next_frames = {'stick_user': 0, 'wheelchair_user': 0}
input_folder_counter = 1
frame_count = 0

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

def save_txt(filename, triggered_class):
    txt_path = filename.replace('.jpg', '.txt')
    with open(txt_path, 'w') as txt_file:
        txt_file.write(f"Triggered by: {triggered_class}\n")

def save_frame_data(output_dir, frame, detected_objects, frame_count):
    frame_filename = os.path.join(output_dir, f"A_input1_{frame_count:04d}.jpg")
    json_frame_filename = frame_filename.replace('.jpg', '.json')
    cv2.imwrite(frame_filename, frame)
    save_json(json_frame_filename, detected_objects)

if __name__ == "__main__":
    # [옵션] GPU 확인
    gpu_info = subprocess.check_output(["nvidia-smi"]).decode("utf-8")
    locale.getpreferredencoding = lambda: "UTF-8"
    if gpu_info.find('failed') >= 0:
        print('Not connected to a GPU')
    else:
        print(gpu_info)

    # 입출력 동영상 파일 경로
    video_path = './data/bus_wheelchair_crutch.mp4'
    output_video_path = './data/predicted_bus_wheelchair_crutch_combined.mp4'

    # 검출된 프레임과 json 저장 디렉토리 설정
    base_output_dir = './data/detection_A'
    os.makedirs(base_output_dir, exist_ok=True)

    # 동영상 캡처 객체 생성
    cap = cv2.VideoCapture(video_path)
    frame_width = 640
    frame_height = 640
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    # 동영상 파일 저장 객체 생성
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video_path, fourcc, fps, (frame_width, frame_height))

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (640, 640))
        results = wheelchair_model.predict(frame, save=False, classes=[0,2], conf=0.5)
        filtered_results = filter_boxes(results)

        detected_objects = {}
        for result in filtered_results:
            for box, score, label, names in result:
                class_name = names[int(label)]
                if class_name in ['stick_user', 'wheelchair_user']:
                    if class_name not in detected_objects:
                        detected_objects[class_name] = {}
                    bbox_id = f"id{len(detected_objects[class_name]) + 1}"
                    detected_objects[class_name][bbox_id] = [int(box[0]), int(box[1]), int(box[2]), int(box[3])]

        # 감지된 객체별로 처리
        for class_name in ['stick_user', 'wheelchair_user']:
            if class_name in detected_objects and skip_counters[class_name] == 0 and save_next_frames[class_name] == 0:
                save_next_frames[class_name] = 5
                # 객체 감지 시 새로운 폴더를 생성하고 텍스트 파일을 저장
                output_json_dir = os.path.join(base_output_dir, f'input{input_folder_counter}')
                os.makedirs(output_json_dir, exist_ok=True)
                save_txt(os.path.join(output_json_dir, 'triggered_by.txt'), class_name)
                input_folder_counter += 1

            if save_next_frames[class_name] > 0:
                save_frame_data(output_json_dir, frame, detected_objects, frame_count)
                save_next_frames[class_name] -= 1
                if save_next_frames[class_name] == 0:
                    #FilePublisher.main()
                    #time.sleep(1)
                    skip_counters[class_name] = 100

            if skip_counters[class_name] > 0:
                skip_counters[class_name] -= 1

        draw_boxes(filtered_results, frame)
        out.write(frame)
        frame_count += 1

    cap.release()
    out.release()
    cv2.destroyAllWindows()

    print(f"Processed video saved to {output_video_path}")
    print(f"Processed frames and JSON files saved to {base_output_dir}")
