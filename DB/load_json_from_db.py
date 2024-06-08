import mysql.connector as con
import os 
from upload_json_to_db import *
import cv2

# 검출된 객체있는 이미지 경로
detected_img_dir = './data/detection'

# BBox 처리 완료된 이미지 경로
result_img_dir = './data/output_frames/'
os.makedirs(result_img_dir, exist_ok=True)

# 클래스 별 색상 정의
class_colors = {
    'stick_user': (0, 0, 255),    # Red
    'person': (255, 0, 0),     # Blue
    'person_no': (255, 0, 0),     # Blue
    'wheelchair_user': (255, 0, 255) # Pink
}

class Drawbbox():
    def __init__(self, db_instance):
        self.db_instance = db_instance

    # 검출 결과를 그리기 위한 함수 정의
    def draw_boxes(self, image, bbox, label, obj_id, color):
        x1, y1, x2, y2 = map(int, bbox.split(','))  # bounding box coordinates
        label_text = f"{label} {obj_id}"
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    def loadDataFromDB(self, query):
        image_data = self.db_instance.fetchImageDataQuery(query)
        image_dict = {}
        for filename, detected_object, obj_id, bbox in image_data:
            if filename not in image_dict:
                image_dict[filename] = []
            image_dict[filename].append((detected_object, obj_id, bbox))

        for filename, annotations in image_dict.items():
            img_path = os.path.join(detected_img_dir, os.path.basename(filename))
            if os.path.exists(img_path):
                image = cv2.imread(img_path)
                for detected_object, obj_id, bbox in annotations:
                    color = class_colors[detected_object]  # 클래스 색상 가져오기
                    self.draw_boxes(image, bbox, detected_object, obj_id, color)
                file_base_name = os.path.basename(filename)
                result_path = os.path.join(result_img_dir, file_base_name)
                cv2.imwrite(result_path, image)

if __name__ == "__main__" :
    print(os.getcwd())

    db_instance = Connect("driver", "0603")
    draw_bbox_instance = Drawbbox(db_instance)
    query = "SELECT Filename, Detected_objects, ID, bbox FROM image_json;"
    draw_bbox_instance.loadDataFromDB(query)
    db_instance.disConnection()