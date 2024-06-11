import os
import json
from Connect import Connect

# 베이스 디렉토리 경로
base_dir = './data/lstm/'

class Uploadjson():
    def __init__(self, db_instance):
        self.cursor = db_instance.cursor
        self.conn = db_instance.conn

    def disConnection(self):
        if self.conn:
            print('!!!!!DB SHUT DOWN!!!!!')
            self.conn.close()
            self.cursor.close()
            self.conn = None

    def createQuery(self, query):
        self.cursor.execute(query)
        self.conn.commit()
        print("테이블 생성 완료")

    def getLastCaseNum(self):
        query = "SELECT MAX(Case_num) FROM video_json;"
        self.cursor.execute(query)
        last_case_num = self.cursor.fetchone()[0]
        return last_case_num if last_case_num is not None else 0  # None이면 0 반환

    def insertQuery(self, query):
        # 디렉토리 목록 가져오기
        directories = [d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]

        # 디렉토리 중 최신 디렉토리 선택
        if not directories:
            print('No directories found in base_dir.')
            return

        latest_directory = max(directories, key=lambda d: int(d.replace('input', '')))
        latest_dir_path = os.path.join(base_dir, latest_directory)

        # 최신 디렉토리 내 파일 목록 가져오기
        files = [f for f in os.listdir(latest_dir_path) if f.endswith('.json')]

        if not files:
            print(f'No JSON files found in {latest_directory}.')
            return

        for file_name in files:
            with open(os.path.join(latest_dir_path, file_name), 'r') as json_file:
                data = json.load(json_file)

                video_info = data["video_info"]
                size = data["size"]
                header = data["header"]
                actions = data["action"]

                filename = video_info["video_name"]
                #width = size["width"]
                width = 1280
                #height = size["height"]
                height = 720

                for action in actions:
                    action_name = action["actionname"]
                    start_frame = action["frame"]["start"]
                    end_frame = action["frame"]["end"]

                    # 데이터 삽입 쿼리 실행
                    last_case_num = self.getLastCaseNum()
                    self.cursor.execute(query, (last_case_num + 1, filename, width, height, action_name, start_frame, end_frame))
                    self.conn.commit()

        print(f'Data inserted from latest directory: {latest_directory}')
        print("데이터 삽입 완료")
    
    #def fetchImageDataQuery(self, query):
    #    self.cursor.execute(query)
    #    return self.cursor.fetchall()

if __name__ == "__main__":
    print(os.getcwd())

    db_instance = Connect("driver", "0603")
    upload_json = Uploadjson(db_instance)

    # Case 1.
    query = """
            INSERT INTO video_json (Case_num, Filename, width, height, Action_name, Start_frame, End_frame)
            VALUES (%s, %s, %s, %s, %s, %s, %s);
            """
    upload_json.insertQuery(query)

    db_instance.disConnection()
