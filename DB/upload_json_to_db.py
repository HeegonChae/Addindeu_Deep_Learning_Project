import os
import json
import re
# pip install mysql-connector-python
import mysql.connector as con

# JSON 파일 경로
json_dir = './data/detection'

class Connect():
    def __init__(self, User, Password):
        self.conn = con.connect(
            host='database-1.cdigc6umyoh0.ap-northeast-2.rds.amazonaws.com',
            user=User,
            password=Password,
            database='autobuscctvdb'
        )
        self.cursor = self.conn.cursor(buffered=True)

    def disConnection(self):
        if self.conn:
            print('!!!!!!DB SHUT DOWN!!!!!!')
            self.conn.close()
            self.cursor.close()
            self.conn = None

    def createQuery(self, query):
        self.cursor.execute(query)
        self.conn.commit()

        print("테이블 생성 완료")

    # 파일명을 기준으로 정렬
    @staticmethod
    def extract_number(filename):
        match = re.search(r'frame(\d+)\.jpg', filename)
        if match:
            return int(match.group(1))
        return float('inf')

    def insertQuery(self, query):
        # JSON 파일 읽기 및 데이터 삽입
        case_num = 1  # Case_num 카운터 초기화

        # 파일 목록 가져오기
        files = [f for f in os.listdir(json_dir) if f.endswith('.json')]

        # 파일 내용을 읽어와서 정렬
        json_data_list = []
        for file_name in files:
            with open(os.path.join(json_dir, file_name), 'r') as json_file:
                data = json.load(json_file)
                json_data_list.append(data)

        # 파일명 기준으로 정렬
        json_data_list.sort(key=lambda x: self.extract_number(x["image"]["filename"]))
        # print('-----------------------------')
        # print(json_data_list[10:])

        # 데이터 삽입
        for data in json_data_list:
            filename = data["image"]["filename"]
            detected_objects = data["image"]["detected_objects"]
            for obj_class, obj_data in detected_objects.items():
                for obj_id, bbox in obj_data.items():
                    id_val = int(obj_id[2:])  # 'id1'에서 숫자 부분 추출
                    bbox_str = ','.join(map(str, bbox))

                    # 데이터 삽입 쿼리 실행
                    self.cursor.execute(query, (case_num, filename, obj_class, id_val, bbox_str))
                    self.conn.commit()
                    # Case_num 증가
                    case_num += 1

        print("데이터 삽입 완료")

    # 데이터베이스에서 테이블 정보를 가져오는 함수 정의
    def fetchImageDataQuery(self, query):
        self.cursor.execute(query)
        return self.cursor.fetchall()
    
if __name__ == "__main__":
    print(os.getcwd())

    db_instance = Connect("driver", "0603")

    # Case 0.
    query = """
            CREATE TABLE IF NOT EXISTS image_json (
                Case_num INT NOT NULL PRIMARY KEY,
                Filename VARCHAR(100),
                Detected_objects VARCHAR(50),
                ID INT,
                bbox VARCHAR(50)
            );
            """
    db_instance.createQuery(query)    

    # Case 1.
    query = """
            INSERT INTO image_json (Case_num, Filename, Detected_objects, Id, bbox)
            VALUES (%s, %s, %s, %s, %s);
            """
    db_instance.insertQuery(query)
    
    # # Case 2.
    # query = """
    #         SELECT Filename, Detected_objects, bbox FROM image_json
    #         """
    # results = db_instance.fetchImageDataQuery(query)
    
    db_instance.disConnection()
