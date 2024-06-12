# pip install mysql-connector-python
import mysql.connector as con

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